"""Mesh assembly seams must not become virtual foot obstacles."""

import numpy as np
import pytest
import trimesh

from locolab.utils.terrains.virtual_obstacle import GreedyconcatEdgeCylinderCfg


@pytest.mark.parametrize("height", [1.0, 2.0])
@pytest.mark.parametrize("angle", [0.0, 0.37])
def test_partial_box_join_keeps_only_exposed_lips(height, angle):
    base = trimesh.creation.box((2.0, 1.0, 1.0))
    neighbor = trimesh.creation.box(
        (1.0, 1.0, height),
        trimesh.transformations.translation_matrix((0.0, 1.0, (height - 1.0) / 2)),
    )
    mesh = trimesh.util.concatenate([base, neighbor])
    rotation = trimesh.transformations.rotation_matrix(angle, (0, 0, 1))
    mesh.apply_transform(rotation)
    mesh.merge_vertices()
    cfg = GreedyconcatEdgeCylinderCfg(min_points=2, max_edge_direction_z_ratio=0.1)
    detector = cfg.class_type(cfg)
    edges = detector._filter_edges_by_direction(detector._upper_surface_edges(mesh))
    edges = detector.process_edges(edges)
    edges = trimesh.transform_points(edges.reshape(-1, 3), np.linalg.inv(rotation)).reshape(-1, 6)
    starts, ends = edges[:, :3], edges[:, 3:]
    midpoints = (starts + ends) / 2
    # The join x=[-0.5, 0.5], y=0.5, z=0.5 is flat or blocked by the taller box.
    false_seam = (
        (np.abs(midpoints[:, 0]) < 0.5 - 1e-6)
        & np.isclose(midpoints[:, 1], 0.5)
        & np.isclose(midpoints[:, 2], 0.5)
    )
    assert not false_seam.any()
    # Preserve the two exposed shoulders and every genuine horizontal lip.
    expected_length = 8.0 if height == 1.0 else 9.0
    assert np.linalg.norm(ends - starts, axis=1).sum() == pytest.approx(expected_length)


def test_separated_floating_slabs_keep_their_gap_edges():
    slabs = [
        trimesh.creation.box((1, 1, 0.05), trimesh.transformations.translation_matrix((x, 0, 1)))
        for x in (-1.0, 1.0)
    ]
    mesh = trimesh.util.concatenate(slabs)
    cfg = GreedyconcatEdgeCylinderCfg(min_points=2)
    detector = cfg.class_type(cfg)
    edges = detector.process_edges(detector._upper_surface_edges(mesh))
    assert np.linalg.norm(edges[:, 3:] - edges[:, :3], axis=1).sum() == pytest.approx(8.0)
    assert np.all(np.sign(edges[:, 0]) == np.sign(edges[:, 3]))


@pytest.mark.parametrize("rise", [-0.2, 0.2])
@pytest.mark.parametrize("angle", [0.0, 0.37])
def test_adjacent_floating_steps_keep_both_lips(rise, angle):
    slabs = [
        trimesh.creation.box(
            (1, 1, 0.05), trimesh.transformations.translation_matrix((0, y, top - 0.025))
        )
        for y, top in [(0, 1.0), (1, 1.0 + rise)]
    ]
    mesh = trimesh.util.concatenate(slabs)
    rotation = trimesh.transformations.rotation_matrix(angle, (0, 0, 1))
    mesh.apply_transform(rotation)
    mesh.merge_vertices()
    cfg = GreedyconcatEdgeCylinderCfg(min_points=2)
    detector = cfg.class_type(cfg)
    edges = detector.process_edges(detector._upper_surface_edges(mesh))
    edges = trimesh.transform_points(edges.reshape(-1, 3), np.linalg.inv(rotation)).reshape(-1, 6)
    for top in (1.0, 1.0 + rise):
        facing = edges[np.isclose(edges[:, 1], 0.5) & np.isclose(edges[:, 4], 0.5)
                       & np.isclose(edges[:, 2], top)]
        assert np.linalg.norm(facing[:, 3:] - facing[:, :3], axis=1).sum() == pytest.approx(1.0)
    assert np.linalg.norm(edges[:, 3:] - edges[:, :3], axis=1).sum() == pytest.approx(8.0)
