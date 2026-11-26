# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Export a finished viewer recording at its original simulation timestamps."""

from queue import Empty, Queue
from subprocess import DEVNULL, PIPE, Popen
from tempfile import TemporaryFile
from uuid import uuid4

import numpy as np

VIDEO_QUALITY = {
    "Low · 480p": (480, 75, 28),
    "Medium · 720p": (720, 88, 23),
    "High · 1080p": (1080, 96, 17),
}


def export_mp4(path, frames, dt, quality, server, client, show_frame, progress):
    from viser import _messages

    height, jpeg_quality, crf = VIDEO_QUALITY[quality]
    camera = client.camera
    width = max(2, round(height * camera.aspect / 2) * 2)
    command = ["ffmpeg", "-nostdin", "-loglevel", "error", "-f", "image2pipe", "-c:v", "mjpeg",
               "-framerate", str(1 / dt), "-i", "pipe:0", "-c:v", "libx264", "-crf", str(crf),
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
    # Use Viser's render protocol directly to stream JPEGs without decoding and
    # re-encoding them in Python, and to bound the wait if the browser goes away.
    connection = client._websock_connection
    responses = Queue()
    request_id = None
    has_render_id = "render_uuid" in _messages.GetRenderRequestMessage.__dataclass_fields__

    def receive(_client_id, message):
        if getattr(message, "render_uuid", request_id) == request_id:
            responses.put(message.payload)

    with TemporaryFile() as errors, Popen(command, stdin=PIPE, stdout=DEVNULL, stderr=errors) as encoder:
        connection.register_handler(_messages.GetRenderResponseMessage, receive)
        try:
            previous_tick = frames[0].tick
            for i, frame in enumerate(frames):
                if client.client_id not in server.get_clients():
                    raise RuntimeError("Exporting browser disconnected")
                with server.atomic():
                    show_frame(frame, (frame.tick - previous_tick) * dt)
                previous_tick = frame.tick
                server.flush()
                request_id = uuid4().hex
                connection.queue_message(_messages.GetRenderRequestMessage(
                    "image/jpeg", height, width, jpeg_quality, tuple(camera.wxyz), tuple(camera.position), camera.fov,
                    **({"render_uuid": request_id} if has_render_id else {}),
                ))
                try:
                    jpeg = responses.get(timeout=20)
                except Empty as exc:
                    raise RuntimeError("Timed out waiting for a frame from the browser") from exc
                if not jpeg:
                    raise RuntimeError("Browser could not render the scene")
                ticks = frames[i + 1].tick - frame.tick if i + 1 < len(frames) else 1
                for _ in range(ticks):
                    encoder.stdin.write(jpeg)
                progress(i + 1, len(frames))
            encoder.stdin.close()
            if encoder.wait():
                errors.seek(0)
                raise RuntimeError(errors.read().decode(errors="replace").strip() or "FFmpeg encoding failed")
        finally:
            connection.unregister_handler(_messages.GetRenderResponseMessage, receive)
            if encoder.poll() is None:
                encoder.kill()


def export_trajectory(path, frames, dt, model, env_id):
    from pxr import Gf, Tf, Usd, UsdGeom, Vt

    from ._viser import SO3, merge_geoms
    from .mjcf import visual_meshes

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # Blender's USD importer keeps the scene FPS. Its default is 24.
    # Fractional time codes preserve every simulation sample at that rate.
    fps = 24
    stage.SetTimeCodesPerSecond(fps)
    stage.SetFramesPerSecond(fps)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(np.ceil((frames[-1].tick - frames[0].tick) * dt * fps))
    root = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(root.GetPrim())
    links = []
    transforms = []
    for body in range(1, model.nbody):
        name = model.body(body).name or f"body_{body}"
        link = UsdGeom.Xform.Define(stage, f"/Robot/link_{body}_{Tf.MakeValidIdentifier(name)}")
        link.GetPrim().SetDisplayName(name)
        links.append(link.GetPath())
        transforms.append((link.AddTranslateOp(), link.AddOrientOp()))

    meshes, mesh_links = visual_meshes(model)
    if not meshes:
        # Some MJCF models use collision geometry for their entire visual model.
        mesh_links = []
        for body in range(1, model.nbody):
            geoms = np.flatnonzero(model.geom_bodyid == body).tolist()
            if geoms:
                meshes.append(merge_geoms(model, geoms))
                mesh_links.append(body - 1)
    for i, (mesh, link_idx) in enumerate(zip(meshes, mesh_links)):
        link = links[link_idx]
        geom = UsdGeom.Mesh.Define(stage, link.AppendChild(f"{link.name}_mesh_{i}"))
        geom.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(mesh.vertices, dtype=np.float32)))
        geom.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(mesh.faces, dtype=np.int32).ravel()))
        geom.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(mesh.faces), 3, dtype=np.int32)))
        geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    previous = None
    for frame in frames:
        time = (frame.tick - frames[0].tick) * dt * fps
        quats = SO3.from_matrix(frame.rotations[env_id, 1:]).wxyz
        if previous is not None:
            quats[np.sum(previous * quats, axis=-1) < 0] *= -1
        previous = quats
        for (translate, orient), pos, quat in zip(transforms, frame.positions[env_id, 1:], quats):
            translate.Set(Gf.Vec3d(*pos.tolist()), time)
            orient.Set(Gf.Quatf(*quat.tolist()), time)
    stage.GetRootLayer().Save()
