#ifndef PINHOLE_CAMERA_H
#define PINHOLE_CAMERA_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float fx;           // ½¹¾à x (ÏñËØ)
    float fy;           // ½¹¾à y (ÏñËØ)
    float cx;           // Ö÷µã x (ÏñËØ)
    float cy;           // Ö÷µã y (ÏñËØ)
} PinholeCamera;

static inline bool camera_pixel_to_vector(const PinholeCamera* cam,
                                          float x, float y,
                                          float vec_out[3]) {
    float u = (x - cam->cx) / cam->fx;
    float v = (y - cam->cy) / cam->fy;
    float norm = sqrtf(1.0f + u*u + v*v);
    vec_out[0] = u / norm;
    vec_out[1] = v / norm;
    vec_out[2] = 1.0f / norm;
    return true;
}

#endif
