# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Per-instance push opacity for Kit and Newton without changing Isaac Lab."""

import torch
import warp as wp
from isaaclab_visualizers.kit.kit_visualization_markers import KitVisualizationMarkers
from isaaclab_visualizers.newton.newton_visualization_markers import NewtonVisualizationMarkers

from isaaclab.markers import VisualizationMarkers


class PushMarkers(VisualizationMarkers):
    def _ensure_kit_backend(self, sim=None):
        if not any(isinstance(backend, _KitPushMarkers) for backend in self._backends):
            self._backends.append(_KitPushMarkers(self.cfg, self._is_visible, sim=sim))

    def _ensure_newton_backend(self):
        if not any(isinstance(backend, _NewtonPushMarkers) for backend in self._backends):
            self._backends.append(_NewtonPushMarkers(self.cfg, self._is_visible))

    def visualize(self, translations, orientations, scales, *, opacities):
        super().visualize(translations, orientations, scales)
        for backend in self._backends:
            backend.set_opacities(opacities)


class _KitPushMarkers(KitVisualizationMarkers):
    def __init__(self, cfg, visible=True, *, sim=None):
        super().__init__(cfg, visible, sim=sim)
        from pxr import Sdf, Usd, UsdGeom, UsdShade

        root = self.stage.GetPrimAtPath(self.prim_path)
        self._opacity = UsdGeom.PrimvarsAPI(root).CreatePrimvar(
            "pushOpacity", Sdf.ValueTypeNames.FloatArray, UsdGeom.Tokens.vertex
        )
        # Read the PointInstancer's per-instance opacity in the prototype's material.
        for prim in Usd.PrimRange(root):
            if prim.IsA(UsdShade.Shader):
                shader = UsdShade.Shader(prim)
                if shader.GetIdAttr().Get() == "UsdPreviewSurface":
                    reader = UsdShade.Shader.Define(
                        self.stage, prim.GetPath().GetParentPath().AppendChild("PushOpacity")
                    )
                    reader.CreateIdAttr("UsdPrimvarReader_float")
                    reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("pushOpacity")
                    reader.CreateInput("fallback", Sdf.ValueTypeNames.Float).Set(1.0)
                    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(
                        reader.CreateOutput("result", Sdf.ValueTypeNames.Float)
                    )

    def set_opacities(self, opacities):
        from pxr import Vt

        self._opacity.Set(Vt.FloatArray.FromNumpy(opacities.detach().cpu().numpy()))


class _NewtonPushMarkers(NewtonVisualizationMarkers):
    def set_opacities(self, opacities):
        self.opacities = opacities.detach()

    def render(self, viewer, visible_env_ids, num_envs):
        if self.translations is None:
            return
        spec = self._marker_specs["arrow"]
        if not self.visible or visible_env_ids == []:
            self._hide_batch(viewer, "arrow", spec)
            return
        selection = slice(None) if visible_env_ids is None else visible_env_ids
        position = self.translations[selection]
        if viewer.world_offsets is not None:
            position = position + wp.to_torch(viewer.world_offsets)[selection].to(position)
        transforms = torch.cat((position, self.orientations[selection]), dim=-1)
        scales = self.scales[selection] * self.scales.new_tensor(spec.scale)
        color = self.cfg.markers["arrow"].visual_material.diffuse_color
        colors = transforms.new_tensor(color).expand(len(position), -1).contiguous()
        mesh_name = f"{self.group_id}/meshes/arrow"
        self._ensure_mesh_registered(viewer, mesh_name, spec)
        viewer.log_instances(
            f"{self.group_id}/arrow", mesh_name,
            wp.from_torch(transforms.to(device=str(viewer.device)).contiguous(), dtype=wp.transform),
            wp.from_torch(scales.to(device=str(viewer.device)).contiguous(), dtype=wp.vec3),
            wp.from_torch(colors.to(device=str(viewer.device)), dtype=wp.vec3),
            None,
            hidden=False,
            opacities=wp.from_torch(self.opacities[selection].to(device=str(viewer.device)).contiguous()),
        )
