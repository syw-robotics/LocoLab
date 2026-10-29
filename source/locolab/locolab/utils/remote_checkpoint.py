"""Download Z-RL checkpoints from experiment links without starting a logging run.

Pass a W&B/Comet/Neptune run URL to play's --checkpoint. The highest numbered
model_<iteration>.pt is selected; append #model_1000.pt to select one explicitly.
SDK authentication uses the same login/config/environment as training.
"""

import hashlib
import importlib
import re
import tempfile
from pathlib import Path
from urllib.parse import unquote, urlsplit


def _parse_reference(reference: str):
    url = urlsplit(reference)
    hosts = {"wandb.ai": "wandb", "www.comet.com": "comet", "comet.com": "comet", "app.neptune.ai": "neptune"}
    if url.scheme in {"wandb", "comet", "neptune"}:
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
    if len(parts) != 3 or any(not re.fullmatch(r"[\w.-]+", part) or part in {".", ".."} for part in parts):
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
    package = {"wandb": "wandb", "comet": "comet_ml", "neptune": "neptune"}[service]
    try:
        sdk = importlib.import_module(package)
    except ModuleNotFoundError as exc:
        install = {"wandb": "wandb", "comet": "comet-ml", "neptune": "neptune<2"}[service]
        raise ModuleNotFoundError(f"Remote {service} checkpoints require: pip install '{install}'") from exc

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
