from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from ..exceptions import InvalidMocapBody
from .base import MatrixLieGroup
from .so3 import SO3
from .utils import get_epsilon, skew

_IDENTITY_WXYZ_XYZ = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass(frozen=True)
class SE3(MatrixLieGroup):
    """Special Euclidean group for proper rigid transforms in 3D.

    Internal parameterization is (qw, qx, qy, qz, x, y, z). Tangent parameterization is
    (vx, vy, vz, omega_x, omega_y, omega_z).
    """

    wxyz_xyz: np.ndarray
    matrix_dim: int = 4
    parameters_dim: int = 7
    tangent_dim: int = 6
    space_dim: int = 3

    def __repr__(self) -> str:
        quat = np.round(self.wxyz_xyz[:4], 5)
        xyz = np.round(self.wxyz_xyz[4:], 5)
        return f"{self.__class__.__name__}(wxyz={quat}, xyz={xyz})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SE3):
            return NotImplemented
        return np.array_equal(self.wxyz_xyz, other.wxyz_xyz)

    def copy(self) -> SE3:
        return SE3(wxyz_xyz=np.array(self.wxyz_xyz))

    def parameters(self) -> np.ndarray:
        return self.wxyz_xyz

    @classmethod
    def identity(cls) -> SE3:
        return SE3(wxyz_xyz=_IDENTITY_WXYZ_XYZ)

    @classmethod
    def from_rotation_and_translation(
        cls,
        rotation: SO3,
        translation: np.ndarray,
    ) -> SE3:
        assert translation.shape == (SE3.space_dim,)
        return SE3(wxyz_xyz=np.concatenate([rotation.wxyz, translation]))

    @classmethod
    def from_rotation(cls, rotation: SO3) -> SE3:
        return SE3.from_rotation_and_translation(
            rotation=rotation, translation=np.zeros(SE3.space_dim)
        )

    @classmethod
    def from_translation(cls, translation: np.ndarray) -> SE3:
        return SE3.from_rotation_and_translation(
            rotation=SO3.identity(), translation=translation
        )

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> SE3:
        assert matrix.shape == (SE3.matrix_dim, SE3.matrix_dim)
        return SE3.from_rotation_and_translation(
            rotation=SO3.from_matrix(matrix[:3, :3]),
            translation=matrix[:3, 3],
        )

    @classmethod
    def from_mocap_id(cls, data: mujoco.MjData, mocap_id: int) -> SE3:
        return SE3.from_rotation_and_translation(
            rotation=SO3(data.mocap_quat[mocap_id]),
            translation=data.mocap_pos[mocap_id],
        )

    @classmethod
    def from_mocap_name(
        cls, model: mujoco.MjModel, data: mujoco.MjData, mocap_name: str
    ) -> SE3:
        mocap_id = model.body(mocap_name).mocapid[0]
        if mocap_id == -1:
            raise InvalidMocapBody(mocap_name, model)
        return SE3.from_mocap_id(data, mocap_id)

    @classmethod
    def sample_uniform(cls) -> SE3:
        return SE3.from_rotation_and_translation(
            rotation=SO3.sample_uniform(),
            translation=np.random.uniform(-1.0, 1.0, size=(SE3.space_dim,)),
        )

    def rotation(self) -> SO3:
        return SO3(wxyz=self.wxyz_xyz[:4])

    def translation(self) -> np.ndarray:
        return self.wxyz_xyz[4:]

    def as_matrix(self) -> np.ndarray:
        hmat = np.eye(self.matrix_dim, dtype=np.float64)
        hmat[:3, :3] = self.rotation().as_matrix()
        hmat[:3, 3] = self.translation()
        return hmat

    @classmethod
    def exp(cls, tangent: np.ndarray) -> SE3:
        assert tangent.shape == (cls.tangent_dim,)
        rotation = SO3.exp(tangent[3:])
        theta = np.float64(mujoco.mju_norm3(tangent[3:]))
        t2 = theta * theta
        if t2 < get_epsilon(t2.dtype):
            v_mat = rotation.as_matrix()
        else:
            skew_omega = skew(tangent[3:])
            v_mat = (
                np.eye(3, dtype=np.float64)
                + (1.0 - np.cos(theta)) / t2 * skew_omega
                + (theta - np.sin(theta)) / (t2 * theta) * (skew_omega @ skew_omega)
            )
        return cls.from_rotation_and_translation(
            rotation=rotation,
            translation=v_mat @ tangent[:3],
        )

    def inverse(self) -> SE3:
        inverse_wxyz_xyz = np.empty(SE3.parameters_dim, dtype=np.float64)
        mujoco.mju_negQuat(inverse_wxyz_xyz[:4], self.wxyz_xyz[:4])
        mujoco.mju_rotVecQuat(
            inverse_wxyz_xyz[4:], -1.0 * self.wxyz_xyz[4:], inverse_wxyz_xyz[:4]
        )
        return SE3(wxyz_xyz=inverse_wxyz_xyz)

    def normalize(self) -> SE3:
        normalized_wxyz_xyz = np.array(self.wxyz_xyz)
        mujoco.mju_normalize4(normalized_wxyz_xyz[:4])
        return SE3(wxyz_xyz=normalized_wxyz_xyz)

    def apply(self, target: np.ndarray) -> np.ndarray:
        assert target.shape == (SE3.space_dim,)
        rotated_target = np.empty(SE3.space_dim, dtype=np.float64)
        mujoco.mju_rotVecQuat(rotated_target, target, self.wxyz_xyz[:4])
        return rotated_target + self.wxyz_xyz[4:]

    def multiply(self, other: SE3) -> SE3:
        wxyz_xyz = np.empty(SE3.parameters_dim, dtype=np.float64)
        mujoco.mju_mulQuat(wxyz_xyz[:4], self.wxyz_xyz[:4], other.wxyz_xyz[:4])
        mujoco.mju_rotVecQuat(wxyz_xyz[4:], other.wxyz_xyz[4:], self.wxyz_xyz[:4])
        wxyz_xyz[4:] += self.wxyz_xyz[4:]
        return SE3(wxyz_xyz=wxyz_xyz)

    def log(self) -> np.ndarray:
        omega = self.rotation().log()
        theta = np.float64(mujoco.mju_norm3(omega))
        t2 = theta * theta
        skew_omega = skew(omega)
        skew_omega2 = skew_omega @ skew_omega
        if t2 < get_epsilon(t2.dtype):
            vinv_mat = (
                np.eye(3, dtype=np.float64) - 0.5 * skew_omega + skew_omega2 / 12.0
            )
        else:
            half_theta = 0.5 * theta
            vinv_mat = (
                np.eye(3, dtype=np.float64)
                - 0.5 * skew_omega
                + (1.0 - 0.5 * theta * np.cos(half_theta) / np.sin(half_theta))
                / t2
                * skew_omega2
            )
        tangent = np.empty(SE3.tangent_dim, dtype=np.float64)
        tangent[:3] = vinv_mat @ self.translation()
        tangent[3:] = omega
        return tangent

    def adjoint(self) -> np.ndarray:
        rotation = self.rotation()
        rotation_mat = rotation.as_matrix()
        tangent_mat = skew(self.translation()) @ rotation_mat
        adjoint_mat = np.zeros((SE3.tangent_dim, SE3.tangent_dim), dtype=np.float64)
        adjoint_mat[:3, :3] = rotation_mat
        adjoint_mat[:3, 3:] = tangent_mat
        adjoint_mat[3:, 3:] = rotation_mat
        return adjoint_mat

    # Jacobians.

    # Eqn 179 a)
    @classmethod
    def ljac(cls, other: np.ndarray) -> np.ndarray:
        theta_squared = np.float64(mujoco.mju_dot3(other[3:], other[3:]))
        if theta_squared < get_epsilon(theta_squared.dtype):
            return np.eye(cls.tangent_dim)
        ljac_se3 = np.zeros((cls.tangent_dim, cls.tangent_dim), dtype=np.float64)
        ljac_translation = _getQ(other)
        ljac_so3 = SO3.ljac(other[3:])
        ljac_se3[:3, :3] = ljac_so3
        ljac_se3[:3, 3:] = ljac_translation
        ljac_se3[3:, 3:] = ljac_so3
        return ljac_se3

    # Eqn 179 b)
    @classmethod
    def ljacinv(cls, other: np.ndarray) -> np.ndarray:
        theta_squared = np.float64(mujoco.mju_dot3(other[3:], other[3:]))
        if theta_squared < get_epsilon(theta_squared.dtype):
            return np.eye(cls.tangent_dim)
        ljacinv_se3 = np.zeros((cls.tangent_dim, cls.tangent_dim), dtype=np.float64)
        ljac_translation = _getQ(other)
        ljacinv_so3 = SO3.ljacinv(other[3:])
        ljacinv_se3[:3, :3] = ljacinv_so3
        ljacinv_se3[:3, 3:] = -ljacinv_so3 @ ljac_translation @ ljacinv_so3
        ljacinv_se3[3:, 3:] = ljacinv_so3
        return ljacinv_se3


# Eqn 180.
def _getQ(c) -> np.ndarray:
    theta = np.float64(mujoco.mju_norm3(c[3:]))
    t2 = theta * theta
    A = 0.5
    if t2 < get_epsilon(t2.dtype):
        B = (1.0 / 6.0) + (1.0 / 120.0) * t2
        C = -(1.0 / 24.0) + (1.0 / 720.0) * t2
        D = -(1.0 / 60.0)
    else:
        t4 = t2 * t2
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        B = (theta - sin_theta) / (t2 * theta)
        C = (1.0 - 0.5 * t2 - cos_theta) / t4
        D = (2.0 * theta - 3.0 * sin_theta + theta * cos_theta) / (2.0 * t4 * theta)
    V = skew(c[:3])
    W = skew(c[3:])
    VW = V @ W
    WV = VW.T
    WVW = WV @ W
    VWW = VW @ W
    return (
        +A * V
        + B * (WV + VW + WVW)
        - C * (VWW - VWW.T - 3.0 * WVW)
        + D * (WVW @ W + W @ WVW)
    )
