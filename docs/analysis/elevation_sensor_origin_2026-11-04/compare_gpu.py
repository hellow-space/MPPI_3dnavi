#!/usr/bin/env python3
"""Synthetic GPU before/after experiment, not a field accuracy benchmark."""
import argparse
import json
from pathlib import Path
import tempfile

import cupy as cp
import numpy as np
from elevation_mapping_cupy import ElevationMap, Parameter
from elevation_mapping_cupy.pointcloud_geometry import sensor_frame_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as work:
        plugin = Path(work)/'empty_plugins.yaml'
        plugin.write_text('{}\n')

        def make_map():
            p = Parameter(enable_traversability=False, use_chainer=False,
                          plugin_config_file=str(plugin), resolution=.1, map_length=4.,
                          enable_visibility_cleanup=False, enable_overlap_clearance=False,
                          mahalanobis_thresh=100000., sensor_noise_factor=.05, max_variance=100.)
            p.update()
            return ElevationMap(p)

        origin = np.array([1., 0., 0.], dtype=np.float32)
        camera = np.array([10., 0., 0.], dtype=np.float32)
        identity = np.eye(3, dtype=np.float32)
        x, y = np.meshgrid(np.arange(.45, 1.5, .1), np.arange(-.35, .4, .1))
        world = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -.5)]).astype(np.float32)
        cloud = world-camera
        points, rotation, translation = sensor_frame_points(cloud, identity, camera, identity, origin)
        corrected, reference, legacy = [make_map() for _ in range(3)]
        for _ in range(13):
            corrected.input_pointcloud(points, ['x', 'y', 'z'], rotation, translation, 0., 0.)
            reference.input_pointcloud(world-origin, ['x', 'y', 'z'], identity, origin, 0., 0.)
            legacy.input_pointcloud(cloud, ['x', 'y', 'z'], identity, camera, 0., 0.)
        cp.cuda.Stream.null.synchronize()
        h_old, h_new, h_ref = [cp.asnumpy(m.get_elevation()) for m in [legacy, corrected, reference]]
        v_old, v_new, v_ref = [cp.asnumpy(m.get_variance()) for m in [legacy, corrected, reference]]
        common = np.isfinite(h_new) & np.isfinite(h_old) & np.isfinite(h_ref)
        assert common.sum() >= 30
        np.testing.assert_array_equal(np.isfinite(h_new), np.isfinite(h_ref))
        np.testing.assert_allclose(h_new[common], h_ref[common], atol=5e-4)
        np.testing.assert_allclose(v_new[common], v_ref[common], atol=5e-4)
        metrics = dict(scope='Synthetic real-GPU fusion using unchanged kernels; no field navigation or real-world height truth.',
                       geometry=dict(physical_sensor_map_xyz=origin.tolist(), cloud_frame_map_xyz=camera.tolist(),
                                     input_points_per_frame=len(world), repeated_observations=13, plane_height_m=-.5,
                                     map_length_m=4., resolution_m=.1, visibility_cleanup=False, overlap_clearance=False,
                                     mahalanobis_thresh=100000., sensor_noise_factor=.05, max_variance=100.,
                                     other_parameters='Parameter defaults; same across all three maps'),
                       plane=dict(common_valid_cells=int(common.sum()),
                                  mean_variance_before_m2=float(v_old[common].mean()),
                                  mean_variance_after_m2=float(v_new[common].mean()),
                                  mean_variance_reference_m2=float(v_ref[common].mean()),
                                  height_mae_to_synthetic_truth_before_m=float(np.abs(h_old[common]+.5).mean()),
                                  height_mae_to_synthetic_truth_after_m=float(np.abs(h_new[common]+.5).mean()),
                                  max_height_difference_to_native_reference_before_m=float(np.abs(h_old[common]-h_ref[common]).max()),
                                  max_height_difference_to_native_reference_after_m=float(np.abs(h_new[common]-h_ref[common]).max()),
                                  max_variance_difference_to_native_reference_after_m2=float(np.abs(v_new[common]-v_ref[common]).max())))
        gates = {}
        for label, point in [('inside_minimum_distance', [1.05, 0., 0.]),
                             ('above_ramped_height_limit', [1.5, 0., .8])]:
            corrected, legacy = make_map(), make_map()
            raw = np.array([point], dtype=np.float32)-camera
            fixed, r, t = sensor_frame_points(raw, identity, camera, identity, origin)
            for _ in range(13):
                corrected.input_pointcloud(fixed, ['x', 'y', 'z'], r, t, 0., 0.)
                legacy.input_pointcloud(raw, ['x', 'y', 'z'], identity, camera, 0., 0.)
            before = int(cp.isfinite(legacy.get_elevation()).sum())
            after = int(cp.isfinite(corrected.get_elevation()).sum())
            assert before > 0 and after == 0
            gates[label] = dict(world_point_m=point, expected_retained_cells=0,
                                retained_cells_before=before, retained_cells_after=after)
        metrics['gates'] = gates
        args.output.write_text(json.dumps(metrics, indent=2)+'\n')
        print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
