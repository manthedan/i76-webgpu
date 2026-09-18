/* camera.c -- the native-compatible world/view camera basis. */

#include "engine/camera.h"

#include <math.h>
#include <string.h>

static double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static int normalize3(double v[3])
{
    double length = sqrt(dot3(v, v));
    if (!isfinite(length) || length < 1e-12)
        return -1;
    v[0] /= length;
    v[1] /= length;
    v[2] /= length;
    return 0;
}

int camera_view_look_at(CameraView *out, const double eye[3],
                        const double target[3])
{
    if (!out || !eye || !target)
        return -1;

    CameraView view;
    memcpy(view.eye, eye, sizeof view.eye);
    view.fov_tan_half = 1.0;             /* normal 90-degree construction */
    view.forward[0] = target[0] - eye[0];
    view.forward[1] = target[1] - eye[1];
    view.forward[2] = target[2] - eye[2];
    if (normalize3(view.forward) != 0)
        return -1;

    /* FUN_0049BB90: right = normalize(Y-up x forward). */
    view.right[0] = view.forward[2];
    view.right[1] = 0.0;
    view.right[2] = -view.forward[0];
    if (normalize3(view.right) != 0) {
        /* The binary's horizontal construction is singular for a vertical
         * view. Pick +X, then preserve its handedness below. */
        view.right[0] = 1.0;
        view.right[1] = 0.0;
        view.right[2] = 0.0;
    }
    cross3(view.forward, view.right, view.up);
    if (normalize3(view.up) != 0)
        return -1;

    *out = view;
    return 0;
}

int camera_view_from_basis(CameraView *out, const double eye[3],
                           const double right[3], const double up[3],
                           const double forward[3])
{
    if (!out || !eye || !right || !up || !forward)
        return -1;
    CameraView view;
    memcpy(view.eye, eye, sizeof view.eye);
    view.fov_tan_half = 1.0;             /* normal 90-degree construction */
    memcpy(view.right, right, sizeof view.right);
    memcpy(view.up, up, sizeof view.up);
    memcpy(view.forward, forward, sizeof view.forward);
    if (!camera_view_valid(&view))
        return -1;
    *out = view;
    return 0;
}

void camera_view_native_angles(CameraView *out, const double eye[3],
                               double first_rad, double second_rad,
                               double third_rad)
{
    const double ca = cos(first_rad), sa = sin(first_rad);
    const double cb = cos(third_rad), sb = sin(third_rad);
    const double cc = cos(second_rad), sc = sin(second_rad);

    if (!out || !eye)
        return;
    memcpy(out->eye, eye, sizeof out->eye);
    out->fov_tan_half = 1.0;             /* normal 90-degree construction */

    /* Literal FUN_0049B170 composition: Z(second) * X(first) * Y(third).
     * Rows are the world-space local +X/+Y/+Z axes. */
    out->right[0] = cc * cb + sc * sa * sb;
    out->right[1] = sc * ca;
    out->right[2] = -cc * sb + sc * sa * cb;

    out->up[0] = -sc * cb + cc * sa * sb;
    out->up[1] = cc * ca;
    out->up[2] = sc * sb + cc * sa * cb;

    out->forward[0] = ca * sb;
    out->forward[1] = -sa;
    out->forward[2] = ca * cb;
}

int camera_view_valid(const CameraView *view)
{
    if (!view)
        return 0;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(view->eye[i]) || !isfinite(view->right[i]) ||
            !isfinite(view->up[i]) || !isfinite(view->forward[i]))
            return 0;
    }

    const double rr = dot3(view->right, view->right);
    const double uu = dot3(view->up, view->up);
    const double ff = dot3(view->forward, view->forward);
    if (fabs(rr - 1.0) > 1e-9 || fabs(uu - 1.0) > 1e-9 ||
        fabs(ff - 1.0) > 1e-9 || fabs(dot3(view->right, view->up)) > 1e-9 ||
        fabs(dot3(view->right, view->forward)) > 1e-9 ||
        fabs(dot3(view->up, view->forward)) > 1e-9)
        return 0;

    double cross[3];
    cross3(view->right, view->up, cross);
    return dot3(cross, view->forward) > 1.0 - 1e-9;
}

double camera_view_fov_tan_half(const CameraView *view)
{
    return view && isfinite(view->fov_tan_half) && view->fov_tan_half > 0.0
         ? view->fov_tan_half : 1.0;
}

void camera_view_target(const CameraView *view, double target[3])
{
    if (!view || !target)
        return;
    target[0] = view->eye[0] + view->forward[0];
    target[1] = view->eye[1] + view->forward[1];
    target[2] = view->eye[2] + view->forward[2];
}
