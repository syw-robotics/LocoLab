"""Download Z-RL checkpoints from experiment links without starting a logging run.

Pass a W&B/Comet/Neptune/TrackNest run URL to --checkpoint. The highest numbered
model_<iteration>.pt is selected; append #model_1000.pt to select one explicitly.
SDK authentication uses the same login/config/environment as training.
"""

import hashlib
import importlib
import re
import tempfile
from pathlib import Path
from urllib.parse import parse_qs, quote, unquote, urlsplit


def _parse_reference(reference: str):
    url = urlsplit(reference)
    hosts = {"wandb.ai": "wandb", "www.comet.com": "comet", "comet.com": "comet", "app.neptune.ai": "neptune"}
    if url.scheme == "tracknest":
        service = "tracknest"
        parts = ["", unquote(url.netloc), unquote(url.path.strip("/"))]
    elif url.scheme in {"http", "https"} and re.fullmatch(r"/(u|p)/[^/]+/?", url.path) and url.query:
        query = parse_qs(url.query)
        if "project" not in query:
            return None
        service = "tracknest"
        run = query.get("selected_run_id") or query.get("selected_run") or [""]
        parts = [url._replace(query="", fragment="").geturl().rstrip("/") + "/", query["project"][0], run[0]]
    elif url.scheme in {"wandb", "comet", "neptune"}:
        service = url.scheme
        parts = [url.netloc, *url.path.strip("/").split("/")]
    elif url.scheme in {"http", "https"} and url.hostname in hosts:
        service = hosts[url.hostname]
        parts = url.path.strip("/").split("/")
        if service == "wandb" and len(parts) >= 4 and parts[2] == "runs":
            parts = [*parts[:2], parts[3]]
        elif service == "neptune" and len(parts) >= 4 and parts[2] == "e":
            parts = [*parts[:2], parts[3]]
        elif service == "comet":
            parts = parts[:3]  # The UI may append a tab such as /assets.
        else:
            return None  # Ordinary HTTP checkpoint files still use retrieve_file_path.
    else:
        return None
    if service == "tracknest":
        if url.username or any(not part or "/" in part or part in {".", ".."} for part in parts[1:]):
            raise ValueError("Use tracknest://<project>/<run-id> or a TrackNest run-detail URL.")
    elif len(parts) != 3 or any(not re.fullmatch(r"[\w.-]+", part) or part in {".", ".."} for part in parts):
        raise ValueError(f"Expected {service}://<workspace>/<project>/<run-id>: {reference}")
    filename = unquote(url.fragment) or "latest"
    if filename != "latest" and not re.fullmatch(r"model_\d+\.pt", filename):
        raise ValueError("Select a checkpoint with #model_<iteration>.pt (or omit it for latest).")
    return service, parts, filename


def _select_checkpoint(names, filename):
    candidates = [name for name in names if re.fullmatch(r"model_\d+\.pt", name)]
    if filename != "latest":
        if filename not in candidates:
            raise FileNotFoundError(f"Remote run has no checkpoint {filename}.")
        return filename
    if not candidates:
        raise FileNotFoundError("Remote run has no model_<iteration>.pt checkpoints. Check training uploads.")
    return max(candidates, key=lambda name: int(name[6:-3]))


def _download_tracknest(sdk, parts, filename, cache_dir):
    config = sdk._read_config()
    server = parts[0] or config["server_url"]
    saved_url, requested_url = urlsplit(config["server_url"]), urlsplit(server)
    if (saved_url.scheme, saved_url.netloc) != (requested_url.scheme, requested_url.netloc):
        raise ValueError("TrackNest URL must use the logged-in server. Run python -m tracknest login for that server.")
    server = server.rstrip("/") + "/"
    project, selector = parts[1:]
    client = sdk._RemoteClient(server, write_token=config["token"], httpx_kwargs={"trust_env": False}, verbose=False)
    records = client.predict(project, api_name="/get_runs_for_project")
    matches = [run for run in records if run["id"] == selector] or [run for run in records if run["name"] == selector]
    if not matches:
        raise FileNotFoundError(f"TrackNest run not found: {project}/{selector}")
    if len(matches) != 1:
        raise ValueError(f"Multiple TrackNest runs named {selector!r}; use the run ID.")
    run = matches[0]
    artifacts = client.predict(project=project, run=run["name"], run_id=run["id"], api_name="/get_run_artifacts")
    files = {artifact["name"]: artifact for artifact in sorted(artifacts["output"], key=lambda item: item["version"])}
    filename = _select_checkpoint(files, filename)
    artifact = files[filename]
    record = client.predict(
        project=artifact["project"], name=filename, spec=f"v{artifact['version']}", api_name="/get_artifact_manifest"
    )
    entries = record["manifest"] if record else []
    entry = next((item for item in entries if item["path"] == filename and "ref" not in item), None)
    if entry is None:
        raise FileNotFoundError(f"TrackNest artifact has no checkpoint file {filename}.")
    digest = entry["digest"]
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("Invalid TrackNest checkpoint SHA-256 digest.")
    identity = hashlib.sha256(f"{server}/{project}/{run['id']}/{digest}".encode()).hexdigest()[:16]
    destination = Path(cache_dir).expanduser().resolve() / "tracknest" / identity / filename
    if destination.is_file():
        print(f"[INFO] Using cached checkpoint: {destination}")
        return str(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    print(f"[INFO] Downloading tracknest checkpoint: {project}/{run['id']}/{filename}")
    blob_url = f"{server}artifact_blob/{quote(artifact['project'], safe='')}/{digest}"
    with tempfile.TemporaryDirectory(dir=destination.parent) as staging:
        temporary = Path(staging) / filename
        hasher = hashlib.sha256()
        with sdk.httpx2.stream(
            "GET", blob_url, headers={"X-Trackio-Write-Token": config["token"]}, trust_env=False, timeout=300
        ) as response:
            response.raise_for_status()
            with temporary.open("wb") as output:
                for chunk in response.iter_bytes():
                    output.write(chunk)
                    hasher.update(chunk)
        if hasher.hexdigest() != digest or temporary.stat().st_size != entry["size"]:
            raise ValueError("TrackNest checkpoint download failed SHA-256/size verification.")
        temporary.replace(destination)
    return str(destination)


def download_remote_checkpoint(reference: str, cache_dir: str | Path) -> str | None:
    """Return a cached local checkpoint, or None for ordinary local/file URLs.

    Example: --checkpoint 'https://www.comet.com/team/project/experiment#model_1000.pt'
    Remote metadata is refreshed even on cache hits, so an ongoing run's latest
    checkpoint can advance. Downloads are published atomically after completion.
    """
    parsed = _parse_reference(reference)
    if parsed is None:
        return None
    service, parts, filename = parsed
    package = {"wandb": "wandb", "comet": "comet_ml", "neptune": "neptune", "tracknest": "tracknest"}[service]
    try:
        sdk = importlib.import_module(package)
    except ModuleNotFoundError as exc:
        if service == "tracknest":
            raise ModuleNotFoundError("Remote TrackNest checkpoints require the tracknest-client wheel.") from exc
        install = {"wandb": "wandb", "comet": "comet-ml", "neptune": "neptune<2"}[service]
        raise ModuleNotFoundError(f"Remote {service} checkpoints require: pip install '{install}'") from exc

    if service == "tracknest":
        return _download_tracknest(sdk, parts, filename, cache_dir)

    run = None
    try:
        if service == "wandb":
            run = sdk.Api().run("/".join(parts))
            files = {file.name: file for file in run.files()}
            filename = _select_checkpoint(files, filename)
        elif service == "comet":
            run = sdk.API().get_experiment(*parts)
            if run is None:
                raise FileNotFoundError(f"Comet experiment not found: {'/'.join(parts)}")
            # Keep the newest asset if a checkpoint was uploaded more than once.
            assets = sorted(run.get_asset_list(), key=lambda asset: asset.get("createdAt") or 0)
            files = {asset["fileName"]: asset for asset in assets}
            filename = _select_checkpoint(files, filename)
        else:
            run = sdk.init_run(project="/".join(parts[:2]), with_id=parts[2], mode="read-only")
            fields = run.get_structure().get("model", {})
            files = {
                f"model_{key.removeprefix('saved_model_')}.pt": f"model/{key}"
                for key in fields if re.fullmatch(r"saved_model_\d+", key)
            }
            filename = _select_checkpoint(files, filename)

        identity = hashlib.sha256("/".join(parts).encode()).hexdigest()[:16]
        destination = Path(cache_dir).expanduser().resolve() / service / identity / filename
        if destination.is_file():
            print(f"[INFO] Using cached checkpoint: {destination}")
            return str(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        print(f"[INFO] Downloading {service} checkpoint: {'/'.join(parts)}/{filename}")
        with tempfile.TemporaryDirectory(dir=destination.parent) as staging:
            temporary = Path(staging) / filename
            if service == "wandb":
                files[filename].download(root=staging, replace=True)
            elif service == "comet":
                response = run.get_asset(files[filename]["assetId"], return_type="response", stream=True)
                response.raise_for_status()
                try:
                    with temporary.open("wb") as output:
                        for chunk in response.iter_content(chunk_size=1024 * 1024):
                            output.write(chunk)
                finally:
                    response.close()
            else:
                run[files[filename]].download(str(temporary))
            temporary.replace(destination)
        return str(destination)
    finally:
        if service == "neptune" and run is not None:
            run.stop()
