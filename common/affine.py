import numpy as np
from scipy.spatial.transform import Rotation


def rotational_affine(axis: str, angle: float, degrees=True) -> np.ndarray:
    if axis not in 'xyz':
        raise ValueError(f"Invalid axis: {axis}")
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = Rotation.from_euler(axis, angle, degrees=degrees).as_matrix()
    return rotation_matrix


def translational_affine(axis: str, displacement: float) -> np.ndarray:
    """Calculates a translational affine matrix for a specific displacement."""
    if axis not in ["tx", "ty", "tz"]:
        raise ValueError(f"Invalid axis: {axis}")

    transform = np.eye(4)
    if axis == 'tx':
        transform[0, 3] = displacement
    elif axis == 'ty':
        transform[1, 3] = displacement
    elif axis == 'tz':
        transform[2, 3] = displacement
    return transform


def affine_matrix_from_rotation_and_translation(rotation: float, translation: np.ndarray, axis='z', degrees=True) \
        -> np.ndarray:
    """Calculates the affine matrix from an axis rotation and a translation vector."""
    if 0 < len(translation) < 3:
        translation = np.array([translation, 0, 0]) if len(translation) == 1 else (
            np.array([translation[0], translation[1], 0]))
    else:
        raise ValueError("Translation vector must have 3 elements.")
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = Rotation.from_euler(axis, rotation, degrees=degrees).as_matrix()
    rotation_matrix[:3, 3] = translation
    return rotation_matrix
