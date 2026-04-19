"""Inertia tensor helpers for SimReady asset authoring.

Implements Blow's tetrahedral inertia decomposition (exact inertia for a
closed triangle mesh of uniform density) and the parallel-axis shift used
when an inertia tensor is re-anchored off its centroid.

Reference:
    Jonathan Blow, "How to find the inertia tensor (or other mass properties)
    of a 3D solid body represented by a triangle mesh."
    http://number-none.com/blow/inertia/

The algorithm builds a signed-volume tetrahedron from the origin to each
triangle (v0, v1, v2) and sums per-tet contributions to mass, first moments
(for the centre of mass), and the inertia tensor. It is exact for closed
orientable meshes and works for non-convex geometry.

Used by the pipeline when PhysX auto-inertia is unreliable — tall thin parts
(doors, drawers, wheel rims), or when F59 (principal-axes misalignment) or
F60 (armature-as-stabilizer) is suspected.
"""

from __future__ import annotations

import numpy as np


__all__ = ["tetrahedral_inertia_tensor", "parallel_axis_shift"]


def tetrahedral_inertia_tensor(mesh_points, mesh_faces, density: float = 1.0) -> dict:
    """Compute mass properties of a closed triangle mesh via Blow's algorithm.

    Args:
        mesh_points: (N, 3) array-like of vertex positions in the body frame.
            Positions may be in any right-handed frame; the returned tensor
            sits in the same frame.
        mesh_faces: (M, 3) array-like of triangle vertex indices. Triangles
            must be consistently wound (CCW when viewed from outside) for the
            signed volume to be positive.
        density: Uniform mass density in kg / m^3. The mesh is assumed to be
            in metres; convert upstream if it isn't.

    Returns:
        dict with:
            mass            — total mass in kg (float)
            com             — (3,) centre of mass in the input frame
            inertia_tensor  — (3, 3) inertia tensor about the COM, axes
                              aligned with the input frame
            volume          — signed volume (float, for sanity: should be > 0
                              on a well-formed outward-winding mesh)

    Raises:
        ValueError on empty / malformed input, or when the computed volume
        is non-positive (winding reversed or non-closed mesh).
    """
    pts = np.asarray(mesh_points, dtype=float)
    faces = np.asarray(mesh_faces, dtype=np.int64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"mesh_points must be (N,3); got {pts.shape}")
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"mesh_faces must be (M,3); got {faces.shape}")
    if len(faces) == 0:
        raise ValueError("mesh has no faces")
    if density <= 0:
        raise ValueError(f"density must be positive; got {density}")

    v0 = pts[faces[:, 0]]
    v1 = pts[faces[:, 1]]
    v2 = pts[faces[:, 2]]

    # Signed volume of each tetrahedron (origin, v0, v1, v2) = (v0 . (v1 x v2)) / 6
    det = np.einsum("ij,ij->i", v0, np.cross(v1, v2))
    tet_vol = det / 6.0
    total_volume = float(tet_vol.sum())
    if total_volume <= 0:
        raise ValueError(
            f"non-positive signed volume ({total_volume:.3e}); "
            "mesh may be inside-out or non-closed"
        )

    mass = density * total_volume

    # First moment of volume: integral of r dV over the tet = (vol/4) * (v0+v1+v2)
    com_weighted = ((v0 + v1 + v2) * tet_vol[:, None]).sum(axis=0) / 4.0
    com = com_weighted / total_volume  # still in volume units

    # Second moments about the origin. For a tetrahedron (0, v0, v1, v2) with
    # signed volume V the integral of x_i x_j dV is:
    #
    #   I_ij = V/20 * ( 2 sum_k a_i^k a_j^k + sum_{k<l} (a_i^k a_j^l + a_i^l a_j^k) )
    #
    # where a^1, a^2, a^3 are the three non-origin vertices. Symmetric in (i,j).
    a = v0  # (M, 3)
    b = v1
    c = v2

    def mixed(i: int, j: int) -> float:
        return float(
            (
                tet_vol
                * (
                    2.0 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j])
                    + (a[:, i] * b[:, j] + b[:, i] * a[:, j])
                    + (a[:, i] * c[:, j] + c[:, i] * a[:, j])
                    + (b[:, i] * c[:, j] + c[:, i] * b[:, j])
                )
            ).sum()
            / 20.0
        )

    # Second-moment tensor about the origin, per unit density.
    M_origin = np.array(
        [
            [mixed(0, 0), mixed(0, 1), mixed(0, 2)],
            [mixed(0, 1), mixed(1, 1), mixed(1, 2)],
            [mixed(0, 2), mixed(1, 2), mixed(2, 2)],
        ]
    )

    # Inertia tensor about origin: I_origin = trace(M)*Id - M, scaled by density.
    trace_M = np.trace(M_origin)
    I_origin = density * (trace_M * np.eye(3) - M_origin)

    # Shift from origin to COM (inverse parallel-axis): I_C = I_O - m*(d²I - d⊗d)
    d = com
    d_sq = float(d @ d)
    I_com = I_origin - mass * (d_sq * np.eye(3) - np.outer(d, d))

    # Numerical symmetry enforcement (the analytic tensor is symmetric; fp drift
    # can leave it slightly off). Symmetrising is harmless when correct.
    I_com = 0.5 * (I_com + I_com.T)

    return {
        "mass": float(mass),
        "com": com.astype(float),
        "inertia_tensor": I_com.astype(float),
        "volume": total_volume,
    }


def parallel_axis_shift(I_C, mass: float, d) -> np.ndarray:
    """Shift an inertia tensor from the centroid to a new reference point.

    I_O = I_C + m * ( (d . d) I_3 - d (x) d )

    Args:
        I_C: (3, 3) inertia tensor at the centroid.
        mass: Mass of the body in kg.
        d: (3,) shift vector FROM the centroid TO the new reference point, in
            the same frame as I_C.

    Returns:
        (3, 3) inertia tensor about the new reference point.
    """
    I_C = np.asarray(I_C, dtype=float)
    if I_C.shape != (3, 3):
        raise ValueError(f"I_C must be (3,3); got {I_C.shape}")
    d = np.asarray(d, dtype=float).reshape(3)
    d_sq = float(d @ d)
    return I_C + mass * (d_sq * np.eye(3) - np.outer(d, d))
