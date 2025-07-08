"""Exceptions specific to mink."""

from typing import Sequence

import mujoco


class MinkError(Exception):
    """Base class for Mink exceptions."""


class NoSolutionFound(MinkError):
    """Exception raised when the QP solver fails to find a solution."""

    def __init__(self, solver_name: str):
        super().__init__(f"QP solver {solver_name} failed to find a solution.")


class UnsupportedFrame(MinkError):
    """Exception raised when a frame type is unsupported."""

    def __init__(self, frame_type: str, supported_types: Sequence[str]):
        message = (
            f"{frame_type} is not supported."
            f"Supported frame types are: {supported_types}"
        )
        super().__init__(message)


class InvalidFrame(MinkError):
    """Exception raised when a frame name is not found in the robot model."""

    def __init__(
        self,
        frame_name: str,
        frame_type: str,
        model: mujoco.MjModel,
    ):
        if frame_type == "body":
            available_names_of_type_frame_type = [
                model.body(i).name for i in range(model.nbody)
            ]
        elif frame_type == "site":
            available_names_of_type_frame_type = [
                model.site(i).name for i in range(model.nsite)
            ]
        else:
            assert frame_type == "geom"
            available_names_of_type_frame_type = [
                model.geom(i).name for i in range(model.ngeom)
            ]

        message = (
            f"{frame_type} '{frame_name}' does not exist in the model. "
            f"Available {frame_type} names: {available_names_of_type_frame_type}"
        )

        super().__init__(message)


class InvalidKeyframe(MinkError):
    """Exception raised when a keyframe name is not found in the robot model."""

    def __init__(self, keyframe_name: str, model: mujoco.MjModel):
        available_keyframes = [model.key(i).name for i in range(model.nkey)]
        message = (
            f"Keyframe {keyframe_name} does not exist in the model. "
            f"Available keyframe names: {available_keyframes}"
        )
        super().__init__(message)


class InvalidMocapBody(MinkError):
    """Exception raised when a body is not a mocap body."""

    def __init__(self, mocap_name: str, model: mujoco.MjModel):
        available_mocap_names = [
            model.body(i).name
            for i in range(model.nbody)
            if model.body(i).mocapid[0] != -1
        ]
        message = (
            f"Body '{mocap_name}' is not a mocap body. "
            f"Available mocap bodies: {available_mocap_names}"
        )
        super().__init__(message)


class NotWithinConfigurationLimits(MinkError):
    """Exception raised when a configuration violates its limits."""

    def __init__(
        self,
        joint_id: int,
        value: float,
        lower: float,
        upper: float,
        model: mujoco.MjModel,
    ):
        joint_name = model.joint(joint_id).name
        message = (
            f"Joint {joint_id} ({joint_name}) violates configuration limits "
            f"{lower} <= {value} <= {upper}"
        )
        super().__init__(message)


class TaskDefinitionError(MinkError):
    """Exception raised when a task definition is ill-formed."""


class IntegrationTimestepNotSet(MinkError):
    """Exception raised when the integration timestep is not set."""

    def __init__(self, cls_name: str):
        message = f"No integration timestep set for {cls_name}"
        super().__init__(message)


class TargetNotSet(MinkError):
    """Exception raised when attempting to use a task with an unset target."""

    def __init__(self, cls_name: str):
        message = f"No target set for {cls_name}"
        super().__init__(message)


class InvalidTarget(MinkError):
    """Exception raised when the target is invalid."""


class InvalidGain(MinkError):
    """Exception raised when the gain is outside the valid range."""


class InvalidDamping(MinkError):
    """Exception raised when the damping is outside the valid range."""


class InvalidConstraint(MinkError):
    """Exception raised when a constraint is invalid."""


class LimitDefinitionError(MinkError):
    """Exception raised when a limit definition is ill-formed."""
