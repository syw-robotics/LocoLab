import isaaclab.sim as sim_utils
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.utils import configclass

from .edge_box import EdgeBox
from .edge_cylinder_cfg import GreedyconcatEdgeCylinderCfg


@configclass
class EdgeBoxCfg(GreedyconcatEdgeCylinderCfg):
    """Fill the vertical gaps between adjacent axis-aligned floating treads.

    Boxes extend from an upper lip down to the nearest lower tread sharing its xy boundary.
    ``downward_extent`` is the minimum depth, covering slab sides without a neighboring tread.
    ``margin`` expands all six box faces, including above the tread to penalize edge standing.
    Inherited ``cylinder_radius`` is unused.
    """

    class_type: type = EdgeBox
    downward_extent: float = 0.05
    margin: float = 0.03
    min_points: int = 2
    max_edge_direction_z_ratio: float | None = 0.0
    visualizer: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/edgeBoxMarkers",
        markers={
            "box": sim_utils.CuboidCfg(
                size=(1.0, 1.0, 1.0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.1, 0.1), opacity=0.4),
            ),
        },
    )
