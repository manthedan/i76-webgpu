#ifndef CAMERA_H
#define CAMERA_H

/*
 * Canonical world-space camera consumed by every renderer.
 *
 * nitro.exe stores the active inverse-view basis at 0x004F3708 as right,
 * up, forward columns; world-to-view is therefore dot(right/up/forward,
 * world-eye).  The basis is Y-up and right-handed: right x up = forward.
 *
 * fov_tan_half is the projection half-angle tangent (horizontal): 1.0 is
 * the binary's normal 90-degree construction; the one verified exception
 * (PRESET_VIEW_4 handler 0x436680) builds a 120-degree projection, tan(60)
 * = 1.7320508075688772. Constructors set 1.0; a zero field in a legacy or
 * manually assembled struct means "normal" and is read as 1.0 at render
 * use, so camera_view_valid deliberately does not inspect it.
 */
typedef struct {
    double eye[3];
    double right[3];
    double up[3];
    double forward[3];
    double fov_tan_half;
} CameraView;

/* Native look-at construction (nitro.exe FUN_0049BB90). Returns -1 only
 * when eye and target coincide; out is not modified on failure. */
int camera_view_look_at(CameraView *out, const double eye[3],
                        const double target[3]);
/* Adopt an existing engine transform basis without rebuilding world-up.
 * Returns -1 if it is not a finite orthonormal right-handed frame. */
int camera_view_from_basis(CameraView *out, const double eye[3],
                           const double right[3], const double up[3],
                           const double forward[3]);

/* Native fixed-direction construction (FUN_0049B170): the three angles are
 * the literal first, third, and second direction slots after converting the
 * FSM's centi-degrees to radians. The matrix is Z(second) * X(first) *
 * Y(third), using the original's row-vector convention. */
void camera_view_native_angles(CameraView *out, const double eye[3],
                               double first_rad, double second_rad,
                               double third_rad);

/* Finite, unit, orthogonal, right-handed basis check. */
int camera_view_valid(const CameraView *view);
/* Positive finite horizontal half-FOV tangent; legacy zero/invalid fields
 * resolve to the normal 90-degree value (1.0). */
double camera_view_fov_tan_half(const CameraView *view);

/* Debug/look-at compatibility only; renderers consume the basis directly. */
void camera_view_target(const CameraView *view, double target[3]);

#endif /* CAMERA_H */
