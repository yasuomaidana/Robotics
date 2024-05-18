from numpy import ndarray, cross
from numpy.linalg import norm


def omega_from_v_r(v: ndarray | list | tuple, r: ndarray | list | tuple) -> float:
    if norm(v) == 0:
        return 0
    if norm(r) == 0:
        raise ValueError("The radius vector cannot be zero")
    return (lambda v_, r_: cross(r_, v_))(v, r)/norm(r) ** 2
