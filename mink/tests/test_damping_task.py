"""Tests for damping_task.py."""

import numpy as np
from absl.testing import absltest
from robot_descriptions.loaders.mujoco import load_robot_description

from mink import Configuration
from mink.tasks import DampingTask


class TestDampingTask(absltest.TestCase):
    """Test consistency of the damping task."""

    def test_qp_objective_fixed_base(self):
        model = load_robot_description("ur5e_mj_description")
        configuration = Configuration(model)
        task = DampingTask(model, cost=1.0)
        nv = configuration.nv
        H, c = task.compute_qp_objective(configuration)
        np.testing.assert_allclose(H, np.eye(nv))
        np.testing.assert_allclose(c, np.zeros(nv))

    def test_qp_objective_floating_base(self):
        model = load_robot_description("g1_mj_description")
        configuration = Configuration(model)
        task = DampingTask(model, cost=1.0)
        nv = configuration.nv
        H, c = task.compute_qp_objective(configuration)
        # Floating base indices should not be damped.
        H_expected = np.zeros((nv, nv))
        H_expected[6:, 6:] = np.eye(nv - 6)
        np.testing.assert_allclose(H, H_expected)
        np.testing.assert_allclose(c, np.zeros(nv))


if __name__ == "__main__":
    absltest.main()
