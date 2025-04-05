#ifndef SFR_H
#define SFR_H

#ifdef __cplusplus
extern "C" {
#endif

//================================================
//:         PUBLIC API
//================================================

/*          pre processing settings:

))  the default values are the opposite of the define,
    e.g. functions aren't inline by default

SFR_USE_INLINE - whether functions should be defined as 'static inline' or 'static'
SFR_FUNC - SFR_USE_INLINE control this, however you can't turn off 'static'
         - buf if you define this as nothing, SFR_FUNC will be nothing

SFR_NO_STD - whether or not 'stdio.h' and 'stdlib.h' are allowed to be included
           - they're only used in 'sfr_load_mesh' and 'sfr_unload_mesh'

))  TODO the following should be changed to normal variables at some point

SFR_NO_CULLING - whether or not to cull triangles

SFR_NEAR_DIST - the near clipping plane distance
SFR_FAR_DIST  - the far clipping plane distance
*/

typedef struct sfrvec  sfrvec_t;
typedef struct sfrmat  sfrmat_t;
typedef struct sfrmesh sfrmesh_t;

// colors are currently ARGB8888, e.g. red = 0xFF0000, blue = 0x0000FF
// TODO support for other formats
typedef unsigned sfrcol_t;

// global variables below can be managed by you, however
// there is probably a function that will do what you want
extern int sfrWidth, sfrHeight;
extern sfrcol_t* sfrPixelBuf;
extern float* sfrDepthBuf;
extern int sfrRasterCount; // how many triangles have been rasterized since the last call to clear
extern sfrmat_t sfrMatModel, sfrMatView, sfrMatProj;
extern sfrvec_t sfrCamPos;
extern float sfrCamFov;


#ifndef SFR_FUNC
    #ifdef SFR_USE_INLINE
        #define SFR_FUNC static inline
    #else
        #define SFR_FUNC static
    #endif
#endif

// math functions
SFR_FUNC sfrvec_t sfr_vec_add(sfrvec_t a, sfrvec_t b);
SFR_FUNC sfrvec_t sfr_vec_sub(sfrvec_t a, sfrvec_t b);
SFR_FUNC sfrvec_t sfr_vec_mul(sfrvec_t a, float b);
SFR_FUNC sfrvec_t sfr_vec_div(sfrvec_t a, float b);
SFR_FUNC float    sfr_vec_dot(sfrvec_t a, sfrvec_t b);
SFR_FUNC float    sfr_vec_length(sfrvec_t v);
SFR_FUNC float    sfr_vec_length2(sfrvec_t v);
SFR_FUNC sfrvec_t sfr_vec_cross(sfrvec_t a, sfrvec_t b);
SFR_FUNC sfrvec_t sfr_vec_norm(sfrvec_t v);
SFR_FUNC sfrvec_t sfr_vec_normf(float a, float b, float c);
SFR_FUNC sfrvec_t sfr_vec_face_normal(sfrvec_t a, sfrvec_t b, sfrvec_t c);
SFR_FUNC sfrmat_t sfr_mat_identity();
SFR_FUNC sfrmat_t sfr_mat_rot_x(float a);
SFR_FUNC sfrmat_t sfr_mat_rot_y(float a);
SFR_FUNC sfrmat_t sfr_mat_rot_z(float a);
SFR_FUNC sfrmat_t sfr_mat_translate(float x, float y, float z);
SFR_FUNC sfrmat_t sfr_mat_scale(float x, float y, float z);
SFR_FUNC sfrmat_t sfr_mat_proj(float fovDev, float aspect, float near, float far);
SFR_FUNC sfrmat_t sfr_mat_mul(sfrmat_t a, sfrmat_t b);
SFR_FUNC sfrvec_t sfr_mat_mul_vec(sfrmat_t m, sfrvec_t v);
SFR_FUNC sfrmat_t sfr_mat_qinv(sfrmat_t m);
SFR_FUNC sfrmat_t sfr_mat_look_at(sfrvec_t pos, sfrvec_t target, sfrvec_t up);

// core functions
SFR_FUNC void sfr_init( // initialize matrices and set buffers
    sfrcol_t* pixelBuf, float* depthBuf, int w, int h, float fovDeg);

SFR_FUNC void sfr_reset(void);                          // reset model matrix to identity
SFR_FUNC void sfr_rotate_x(float theta);                // rotate model matrix about x by theta radians
SFR_FUNC void sfr_rotate_y(float theta);                // rotate model matrix about y by theta radians
SFR_FUNC void sfr_rotate_z(float theta);                // rotate model matrix about z by theta radians
SFR_FUNC void sfr_translate(float x, float y, float z); // translate model matrix by x y z
SFR_FUNC void sfr_scale(float x, float y, float z);     // scale model matrix by x y z
SFR_FUNC void sfr_look_at(float x, float y, float z);   // set view matrix to look at x y z

SFR_FUNC void sfr_clear(void); // reset depth and pixel buffers, and reset lighting to {0}
SFR_FUNC void sfr_triangle(    // using current matrices, draw specified triangle
    float ax, float ay, float az,
    float bx, float by, float bz,
    float cx, float cy, float cz,
    sfrcol_t col);
SFR_FUNC void sfr_cube(sfrcol_t col);          // using current matrices, draw a cube
SFR_FUNC void sfr_mesh(const sfrmesh_t* mesh); // draw specified mesh, changing matrices based on 'mesh'

SFR_FUNC int sfr_world_to_screen( // project the world position specified to screen coordinates
    float x, float y, float z, int* screenX, int* screenY);    

    SFR_FUNC void sfr_set_camera( // update the camera with the new position and view
    float x, float y, float z, float yaw, float pitch, float roll);
SFR_FUNC void sfr_set_fov(float fovDeg); // update projection matrix with new fov
SFR_FUNC void sfr_set_lighting(          // update internal lighting state for simple shading on triangles
    int on, sfrvec_t dir, float ambientIntensity);

#ifndef SFR_NO_STD
#include <stdio.h>
#include <stdlib.h>
SFR_FUNC sfrmesh_t* sfr_load_mesh(const char* filename);
SFR_FUNC void sfr_release_mesh(sfrmesh_t** mesh);
#endif


//================================================
//:         IMPLEMENTATION
//================================================

#define SFR_IMPL
#ifdef SFR_IMPL

#include <string.h> // for 'memset' and 'memmove'
#include <math.h>   // for various math function, e.g. 'sqrtf' and 'tanf'


//================================================
//:         PRE PROCESSING OPTIONS
//================================================

#ifndef SFR_NEAR_DIST
    #define SFR_NEAR_DIST 0.1f
#endif
#ifndef SFR_FAR_DIST
    #define SFR_FAR_DIST 100.f
#endif

//================================================
//:         TYPES
//================================================

typedef struct sfrvec {
    float x, y, z, w;
} sfrvec_t;

typedef struct sfrmat {
    float m[4][4];
} sfrmat_t;

typedef struct sfrtri {
    sfrvec_t p[3];
} sfrtri;

typedef struct sfrLightingState {
    int on;
    sfrvec_t dir;
    float ambient;
} SfrLightingState;

typedef struct sfrmesh {
    float* tris;
    int vertCount;
    sfrvec_t pos, scale, rot;
    sfrcol_t col;
} sfrmesh_t;


//================================================
//:         GLOBAL VARIABLES
//================================================

int sfrWidth, sfrHeight;
sfrcol_t* sfrPixelBuf;
float* sfrDepthBuf;
int sfrRasterCount;

sfrmat_t sfrMatModel, sfrMatView, sfrMatProj;
sfrvec_t sfrCamPos;
float sfrCamFov;

SfrLightingState sfrLightingState = {0};
sfrmat_t sfrMatNormal; // only used for shading
int sfrIsNormalDirty;  // only used for shading


//================================================
//:         MISC HELPER MACROS
//================================================

#define SFR_PI 3.14159265358979323846f
#define SFR_SWAP(a, b) { typeof(a) _swapTemp = (a); (a) = (b); (b) = _swapTemp; }
#define SFR_VEC0 ((sfrvec_t){0.f, 0.f, 0.f, 0.f})


//================================================
//:         MATH
//================================================

SFR_FUNC sfrvec_t sfr_vec_add(sfrvec_t a, sfrvec_t b) {
    sfrvec_t r;
    r.x = a.x + b.x;
    r.y = a.y + b.y;
    r.z = a.z + b.z;
    r.w = a.w + b.w;
    return r;
}

SFR_FUNC sfrvec_t sfr_vec_sub(sfrvec_t a, sfrvec_t b) {
    sfrvec_t r;
    r.x = a.x - b.x;
    r.y = a.y - b.y;
    r.z = a.z - b.z;
    r.w = a.w - b.w;
    return r;
}

SFR_FUNC sfrvec_t sfr_vec_mul(sfrvec_t a, float b) {
    sfrvec_t r;
    r.x = a.x * b;
    r.y = a.y * b;
    r.z = a.z * b;
    r.w = a.w * b;
    return r;
}


SFR_FUNC sfrvec_t sfr_vec_div(sfrvec_t a, float b) {
    sfrvec_t r;
    r.x = a.x / b;
    r.y = a.y / b;
    r.z = a.z / b;
    r.w = 1.f;
    return r;
}

SFR_FUNC float sfr_vec_dot(sfrvec_t a, sfrvec_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

SFR_FUNC float sfr_vec_length(sfrvec_t v) {
    return sqrtf(sfr_vec_dot(v, v));
}

SFR_FUNC float sfr_vec_length2(sfrvec_t v) {
    return sfr_vec_dot(v, v);
}

SFR_FUNC sfrvec_t sfr_vec_cross(sfrvec_t a, sfrvec_t b) {
    sfrvec_t r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    r.w = 1.f;
    return r;
}


SFR_FUNC sfrvec_t sfr_vec_norm(sfrvec_t v) {
    const float l = sfr_vec_length(v);
    sfrvec_t r;
    r.x = v.x / l;
    r.y = v.y / l;
    r.z = v.z / l;
    r.w = 1.f;
    return r;
}

SFR_FUNC sfrvec_t sfr_vec_normf(float x, float y, float z) {
    return sfr_vec_norm((sfrvec_t){x, y, z, 1.f});
}

SFR_FUNC sfrvec_t sfr_vec_face_normal(sfrvec_t a, sfrvec_t b, sfrvec_t c) {
    const sfrvec_t edge1 = sfr_vec_sub(b, a);
    const sfrvec_t edge2 = sfr_vec_sub(c, a);
    return sfr_vec_norm(sfr_vec_cross(edge1, edge2));
}

SFR_FUNC sfrmat_t sfr_mat_identity() {
    sfrmat_t r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = 1.f;
    r.m[2][2] = 1.f;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_rot_x(float a) {
    sfrmat_t r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = cosf(a);
    r.m[1][2] = sinf(a);
    r.m[2][1] = -sinf(a);
    r.m[2][2] = cosf(a);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_rot_y(float a) {
    sfrmat_t r = {0};
    r.m[0][0] = cosf(a);
    r.m[0][2] = sinf(a);
    r.m[1][1] = 1.f;
    r.m[2][0] = -sinf(a);
    r.m[2][2] = cosf(a);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_rot_z(float a) {
    sfrmat_t r = {0};
    r.m[0][0] = cosf(a);
    r.m[0][1] = sinf(a);
    r.m[1][0] = -sinf(a);
    r.m[1][1] = cosf(a);
    r.m[2][2] = 1.f;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_translate(float x, float y, float z) {
    sfrmat_t r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = 1.f;
    r.m[2][2] = 1.f;
    r.m[3][0] = x;
    r.m[3][1] = y;
    r.m[3][2] = z;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_scale(float x, float y, float z) {
    sfrmat_t r = {0};
    r.m[0][0] = x;
    r.m[1][1] = y;
    r.m[2][2] = z;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_proj(float fovDev, float aspect, float near, float far) {
    const float fov = 1.0f / tanf(fovDev * 0.5f / 180.0f * 3.1415926536f);
    sfrmat_t r = {0};
    r.m[0][0] = aspect * fov;
    r.m[1][1] = fov;
    r.m[2][2] = far / (far - near);
    r.m[3][2] = (-far * near) / (far - near);
    r.m[2][3] = 1;
    r.m[3][3] = 0;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_mul(sfrmat_t a, sfrmat_t b) {
    sfrmat_t matrix;
    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            matrix.m[r][c] =
                a.m[r][0] * b.m[0][c] +
                a.m[r][1] * b.m[1][c] +
                a.m[r][2] * b.m[2][c] +
                a.m[r][3] * b.m[3][c];
        }
    }
    return matrix;
}

SFR_FUNC sfrvec_t sfr_mat_mul_vec(sfrmat_t m, sfrvec_t v) {
    sfrvec_t r;
    r.x = v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + v.w * m.m[3][0];
    r.y = v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + v.w * m.m[3][1];
    r.z = v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + v.w * m.m[3][2];
    r.w = v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + v.w * m.m[3][3];
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_qinv(sfrmat_t m) {
    sfrmat_t r;
    r.m[0][0] = m.m[0][0];
    r.m[0][1] = m.m[1][0];
    r.m[0][2] = m.m[2][0];
    r.m[0][3] = 0.f;
    r.m[1][0] = m.m[0][1];
    r.m[1][1] = m.m[1][1];
    r.m[1][2] = m.m[2][1];
    r.m[1][3] = 0.f;
    r.m[2][0] = m.m[0][2];
    r.m[2][1] = m.m[1][2];
    r.m[2][2] = m.m[2][2];
    r.m[2][3] = 0.f;
    r.m[3][0] = -(m.m[3][0] * r.m[0][0] + m.m[3][1] * r.m[1][0] + m.m[3][2] * r.m[2][0]);
    r.m[3][1] = -(m.m[3][0] * r.m[0][1] + m.m[3][1] * r.m[1][1] + m.m[3][2] * r.m[2][1]);
    r.m[3][2] = -(m.m[3][0] * r.m[0][2] + m.m[3][1] * r.m[1][2] + m.m[3][2] * r.m[2][2]);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_look_at(sfrvec_t pos, sfrvec_t target, sfrvec_t up) {
    const sfrvec_t forward = sfr_vec_norm(sfr_vec_sub(target, pos));
    up = sfr_vec_norm(sfr_vec_sub(up, sfr_vec_mul(forward, sfr_vec_dot(up, forward))));
    const sfrvec_t right = sfr_vec_cross(up, forward);

    sfrmat_t r;
    r.m[0][0] = right.x;
    r.m[0][1] = right.y;
    r.m[0][2] = right.z;
    r.m[0][3] = 0.f;
    r.m[1][0] = up.x;
    r.m[1][1] = up.y;
    r.m[1][2] = up.z;
    r.m[1][3] = 0.f;
    r.m[2][0] = forward.x;
    r.m[2][1] = forward.y;
    r.m[2][2] = forward.z;
    r.m[2][3] = 0.f;
    r.m[3][0] = pos.x;
    r.m[3][1] = pos.y;
    r.m[3][2] = pos.z;
    r.m[3][3] = 1.f;
    return r;
}


//================================================
//:         RENDERING
//================================================

SFR_FUNC sfrvec_t sfr_intersect_plane(sfrvec_t plane, sfrvec_t norm, sfrvec_t start, sfrvec_t end) {
    norm = sfr_vec_norm(norm);
    const float delta = -sfr_vec_dot(norm, plane);
    const float ad = sfr_vec_dot(start, norm);
    const float bd = sfr_vec_dot(end, norm);
    const float t = (-delta - ad) / (bd - ad);
    
    const sfrvec_t startToEnd = sfr_vec_sub(end, start);
    const sfrvec_t segment = sfr_vec_mul(startToEnd, t);
    
    return sfr_vec_add(start, segment);
}

SFR_FUNC int sfr_clip_against_plane(sfrtri out[2], sfrvec_t plane, sfrvec_t norm, sfrtri in) {
    norm = sfr_vec_norm(norm);

    sfrvec_t* inside[3];
    sfrvec_t* outside[3];
    int insideCount = 0, outsideCount = 0;

    const float ndotp = sfr_vec_dot(norm, plane);
    for (int i = 0; i < 3; i += 1) {
        const float d = norm.x * in.p[i].x + norm.y * in.p[i].y + norm.z * in.p[i].z - ndotp;
        if (d >= 0.f) {
            inside[insideCount] = &in.p[i];
            insideCount += 1;
        } else {
            outside[outsideCount] = &in.p[i];
            outsideCount += 1;
        }
    }

    if (3 == insideCount) {
        out[0] = in;
        return 1;
    }

    if (1 == insideCount && 2 == outsideCount) {
        out[0].p[0] = *inside[0];
        out[0].p[1] = sfr_intersect_plane(plane, norm, *inside[0], *outside[0]);
        out[0].p[2] = sfr_intersect_plane(plane, norm, *inside[0], *outside[1]);
        return 1;
    }

    if (2 == insideCount && 1 == outsideCount) {
        out[0].p[0] = *inside[0];
        out[0].p[1] = *inside[1];
        out[0].p[2] = sfr_intersect_plane(plane, norm, *inside[0], *outside[0]);
        out[1].p[0] = *inside[1];
        out[1].p[1] = out[0].p[2];
        out[1].p[2] = sfr_intersect_plane(plane, norm, *inside[1], *outside[0]);
        return 2;
    }

    return 0;
}

SFR_FUNC void sfr_rasterize(
    float ax, float ay, float az,
    float bx, float by, float bz,
    float cx, float cy, float cz,
    sfrcol_t col
) {
    sfrRasterCount += 1;

    ax = floorf(ax);
    ay = floorf(ay);
    bx = floorf(bx);
    by = floorf(by);
    cx = floorf(cx);
    cy = floorf(cy);
    
    if (ay > by) {
        SFR_SWAP(ax, bx);
        SFR_SWAP(ay, by);
        SFR_SWAP(az, bz);
    }
    if (ay > cy) {
        SFR_SWAP(ax, cx);
        SFR_SWAP(ay, cy);
        SFR_SWAP(az, cz);
    }
    if (by > cy) {
        SFR_SWAP(bx, cx);
        SFR_SWAP(by, cy);
        SFR_SWAP(bz, cz);
    }

    float alpha = 0.f, alphaStep = (cy - ay) > 0.f ? 1.f / (cy - ay) : 0.f;
    float beta = 0.f, betaStep = (by - ay) > 0.f ? 1.f / (by - ay) : 0.f;
    for (int y = (int)ay; y < (int)by; y++, alpha += alphaStep, beta += betaStep) {
        float sx = ax + (cx - ax) * alpha;
        float sz = az + (cz - az) * alpha;
        float ex = ax + (bx - ax) * beta;
        float ez = az + (bz - az) * beta;
        if (sx > ex) {
            SFR_SWAP(sx, ex);
            SFR_SWAP(sz, ez);
        }
        
        const float depthStep = 0.f != (ex - sx) ? (ez - sz) / (ex - sx) : 0.f;
        float depth = sz;
        for (int x = (int)sx, i = y * sfrWidth + (int)sx; x < (int)ex; x += 1, i += 1, depth += depthStep) {
            if (x >= 0 && x < sfrWidth && y >= 0 && y < sfrHeight) {
                if (depth < sfrDepthBuf[i]) {
                    sfrDepthBuf[i] = depth;
                    sfrPixelBuf[i] = col;
                }
            }
        }
    }

    beta = 0.f;
    betaStep = (cy - by) > 0.f ? 1.f / (cy - by) : 0.f;
    for (int y = by; y < cy; y += 1, alpha += alphaStep, beta += betaStep) {
        float sx = ax + (cx - ax) * alpha;
        float sz = az + (cz - az) * alpha;
        float ex = bx + (cx - bx) * beta;
        float ez = bz + (cz - bz) * beta;
        if (sx > ex) {
            SFR_SWAP(sx, ex);
            SFR_SWAP(sz, ez);
        }

        const float depthStep = (ex - sx) != 0.f ? (ez - sz) / (ex - sx) : 0.f;
        float depth = sz;
        for (int x = (int)sx, i = y * sfrWidth + (int)sx; x < (int)ex; x += 1, i += 1, depth += depthStep) {
            if (x >= 0 && x < sfrWidth && y >= 0 && y < sfrHeight) {
                if (depth < sfrDepthBuf[i]) {
                    sfrDepthBuf[i] = depth;
                    sfrPixelBuf[i] = col;
                }
            }
        }
    }
}

//: PUBLIC API FUNCTIONS

SFR_FUNC void sfr_init(sfrcol_t* pixelBuf, float* depthBuf, int w, int h, float fovDeg) {
    sfrWidth = w;
    sfrHeight = h;
    
    sfrPixelBuf = pixelBuf;
    sfrDepthBuf = depthBuf;

    sfr_clear();
    sfr_reset();

    sfrMatProj = sfr_mat_proj(fovDeg, (float)h / w, SFR_NEAR_DIST, SFR_FAR_DIST);
    sfr_set_camera(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
}

SFR_FUNC void sfr_reset(void) {
    sfrMatModel = sfr_mat_identity();
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_rotate_x(float theta) {
    const sfrmat_t rot = sfr_mat_rot_x(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_rotate_y(float theta) {
    const sfrmat_t rot = sfr_mat_rot_y(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_rotate_z(float theta) {
    const sfrmat_t rot = sfr_mat_rot_z(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_translate(float x, float y, float z) {
    const sfrmat_t trans = sfr_mat_translate(x, y, z);
    sfrMatModel = sfr_mat_mul(sfrMatModel, trans);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_scale(float x, float y, float z) {
    const sfrmat_t scale = sfr_mat_scale(x, y, z);
    sfrMatModel = sfr_mat_mul(sfrMatModel, scale);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_look_at(float x, float y, float z) {
    const sfrvec_t up = {0.f, 1.f, 0.f, 1.f};
    const sfrmat_t view = sfr_mat_look_at(sfrCamPos, (sfrvec_t){x, y, z, 1.f}, up);
    sfrMatView = sfr_mat_qinv(view);
}

SFR_FUNC void sfr_clear(void) {
    memset(sfrPixelBuf, 0, sizeof(sfrcol_t) * sfrWidth * sfrHeight);
    memset(sfrDepthBuf, 0x7f, sizeof(float) * sfrWidth * sfrHeight);
    sfrRasterCount = 0;
}

// helper for when lighting is enabled
SFR_FUNC sfrcol_t sfr_adjust_color_u32(sfrcol_t col, float intensity) {
    const unsigned char r = (col >> 16) & 0xFF;
    const unsigned char g = (col >> 8)  & 0xFF;
    const unsigned char b = (col >> 0)  & 0xFF;
    return ((unsigned char)(r * intensity) << 16) |
           ((unsigned char)(g * intensity) << 8)  |
           ((unsigned char)(b * intensity) << 0);
}

// helper for updating normal mat used for shading
SFR_FUNC void sfr_update_normal_mat(void) {
    if (!sfrIsNormalDirty) {
        return;
    }

    const float* m = &sfrMatModel.m[0][0];
    
    const float a = m[0], b = m[4], c = m[8];
    const float d = m[1], e = m[5], f = m[9];
    const float g = m[2], h = m[6], i = m[10];

    const float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    const float invDet = 0 != det ? 1.f / det : 0.f;

    sfrMatNormal = (sfrmat_t){{
        { (e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet, 0.f },
        { (f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet, 0.f },
        { (d * h - e * g) * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet, 0.f },
        { 0.f, 0.f, 0.f, 1.f }
    }};

    sfrIsNormalDirty = 0;
}

SFR_FUNC void sfr_triangle(
    float ax, float ay, float az,
    float bx, float by, float bz,
    float cx, float cy, float cz,
    sfrcol_t col
) {
    if (sfrLightingState.on) {
        sfr_update_normal_mat();

        sfrvec_t normal = sfr_vec_face_normal(
            (sfrvec_t){ax, ay, az},
            (sfrvec_t){bx, by, bz},
            (sfrvec_t){cx, cy, cz});
        
        normal = sfr_mat_mul_vec(sfrMatNormal, normal);
        normal = sfr_vec_norm(normal);

        const float intensity = fmaxf(
            sfrLightingState.ambient, 
            sfr_vec_dot(normal, sfrLightingState.dir));
        
        col = sfr_adjust_color_u32(col, intensity);
    }

    sfrtri tri = {{
        {ax, ay, az, 1.f},
        {bx, by, bz, 1.f},
        {cx, cy, cz, 1.f},
    }};

    tri.p[0] = sfr_mat_mul_vec(sfrMatModel, tri.p[0]);
    tri.p[1] = sfr_mat_mul_vec(sfrMatModel, tri.p[1]);
    tri.p[2] = sfr_mat_mul_vec(sfrMatModel, tri.p[2]);

    #ifndef SFR_NO_CULLING
        const sfrvec_t line0 = sfr_vec_sub(tri.p[1], tri.p[0]);
        const sfrvec_t line1 = sfr_vec_sub(tri.p[2], tri.p[0]);
        const sfrvec_t normal = sfr_vec_cross(line0, line1);
        const sfrvec_t camRay = sfr_vec_sub(tri.p[0], sfrCamPos);
        if (sfr_vec_dot(normal, camRay) > 0.0001f) {
            return;
        }
    #endif

    tri.p[0] = sfr_mat_mul_vec(sfrMatView, tri.p[0]);
    tri.p[1] = sfr_mat_mul_vec(sfrMatView, tri.p[1]);
    tri.p[2] = sfr_mat_mul_vec(sfrMatView, tri.p[2]);

    sfrtri clipped[2];
    const int clippedCount = sfr_clip_against_plane(clipped,
        (sfrvec_t){0.f, 0.f, SFR_NEAR_DIST, 1.f},
        (sfrvec_t){0.f, 0.f, 1.f, 1.f},
        tri);

    sfrtri queue[16];
    for (int c = 0; c < clippedCount; c += 1) {
        tri.p[0] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[0]);
        tri.p[1] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[1]);
        tri.p[2] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[2]);

        tri.p[0] = sfr_vec_div(tri.p[0], tri.p[0].w);
        tri.p[1] = sfr_vec_div(tri.p[1], tri.p[1].w);
        tri.p[2] = sfr_vec_div(tri.p[2], tri.p[2].w);

        const float w2 = sfrWidth / 2.f, h2 = sfrHeight / 2.f;
        for (int i = 0; i < 3; i += 1) {
            tri.p[i].x =  (tri.p[i].x + 1.f) * w2;
            tri.p[i].y = (-tri.p[i].y + 1.f) * h2;
        }

        queue[c] = tri;
    }

    const sfrvec_t clipPlanes[4][2] = {
        {{0.f, 0.5f, 0.f, 1.f}, {0.f, 1.f, 0.f, 1.f}},              // top
        {{0.f, (float)sfrHeight, 0.f, 1.f}, {0.f, -1.f, 0.f, 1.f}}, // bottom 
        {{0.5f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 1.f}},              // left
        {{(float)sfrWidth, 0.f, 0.f, 1.f}, {-1.f, 0.f, 0.f, 1.f}},  // right
    };

    int queueCount = clippedCount, trisToClip = clippedCount;
    for (int p = 0; p < 4; p += 1) {
        int c = 0;
        while (trisToClip > 0) {
            const sfrtri test = queue[0];
            queueCount -= 1;
            trisToClip -= 1;
            memmove(queue, queue + 1, (int)sizeof(sfrtri) * queueCount);
            c = sfr_clip_against_plane(clipped, clipPlanes[p][0], clipPlanes[p][1], test);

            for (int i = 0; i < c; i += 1) {
                queue[queueCount] = clipped[i];
                queueCount += 1;
            }
        }

        trisToClip = queueCount;
    }

    for (int i = 0; i < queueCount; i += 1) {
        sfr_rasterize(
            queue[i].p[0].x, queue[i].p[0].y, queue[i].p[0].z,
            queue[i].p[1].x, queue[i].p[1].y, queue[i].p[1].z,
            queue[i].p[2].x, queue[i].p[2].y, queue[i].p[2].z,
            col
        );
    }
}

SFR_FUNC void sfr_cube(sfrcol_t col) {
    sfr_triangle(-0.5,-0.5,-0.5, -0.5, 0.5,-0.5,  0.5, 0.5,-0.5, col);
    sfr_triangle(-0.5,-0.5,-0.5,  0.5, 0.5,-0.5,  0.5,-0.5,-0.5, col);
    sfr_triangle( 0.5,-0.5,-0.5,  0.5, 0.5,-0.5,  0.5, 0.5, 0.5, col);
    sfr_triangle( 0.5,-0.5,-0.5,  0.5, 0.5, 0.5,  0.5,-0.5, 0.5, col);
    sfr_triangle( 0.5,-0.5, 0.5,  0.5, 0.5, 0.5, -0.5, 0.5, 0.5, col);
    sfr_triangle( 0.5,-0.5, 0.5, -0.5, 0.5, 0.5, -0.5,-0.5, 0.5, col);
    sfr_triangle(-0.5,-0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5,-0.5, col);
    sfr_triangle(-0.5,-0.5, 0.5, -0.5, 0.5,-0.5, -0.5,-0.5,-0.5, col);
    sfr_triangle(-0.5, 0.5,-0.5, -0.5, 0.5, 0.5,  0.5, 0.5, 0.5, col);
    sfr_triangle(-0.5, 0.5,-0.5,  0.5, 0.5, 0.5,  0.5, 0.5,-0.5, col);
    sfr_triangle( 0.5,-0.5, 0.5, -0.5,-0.5, 0.5, -0.5,-0.5,-0.5, col);
    sfr_triangle( 0.5,-0.5, 0.5, -0.5,-0.5,-0.5,  0.5,-0.5,-0.5, col);
}

SFR_FUNC void sfr_mesh(const sfrmesh_t* mesh) {
    if (0.f != mesh->rot.y) {
        sfr_rotate_y(mesh->rot.y);
    }
    if (0.f != mesh->rot.x) {
        sfr_rotate_x(mesh->rot.x);
    }
    if (0.f != mesh->rot.z) {
        sfr_rotate_z(mesh->rot.z);
    }
    sfr_scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    sfr_translate(mesh->pos.x, mesh->pos.y, mesh->pos.z);

    for (int i = 0; i < mesh->vertCount; i += 9) {
        sfr_triangle(
            mesh->tris[i + 0], mesh->tris[i + 1], mesh->tris[i + 2],
            mesh->tris[i + 3], mesh->tris[i + 4], mesh->tris[i + 5],
            mesh->tris[i + 6], mesh->tris[i + 7], mesh->tris[i + 8],
            mesh->col);
    }
}

SFR_FUNC int sfr_world_to_screen(float x, float y, float z, int* screenX, int* screenY) {
    sfrvec_t p = {x, y, z, 1.f};
    p = sfr_mat_mul_vec(sfrMatView, p);
    p = sfr_mat_mul_vec(sfrMatProj, p);
    // p = sfr_vec_div(p, p.w);

    // behind camera
    if (p.w <= 0.f) {
        return 0;
    }

    p = sfr_vec_div(p, p.w);
    p.x =  (p.x + 1.f) * sfrWidth / 2.f;
    p.y = (-p.y + 1.f) * sfrHeight / 2.f;
    *screenX = (int)(p.x + 0.5f);
    *screenY = (int)(p.y + 0.5f);

    return 1;
}

SFR_FUNC void sfr_set_camera(float x, float y, float z, float yaw, float pitch, float roll) {
    sfrCamPos = (sfrvec_t){x, y, z, 1.f};
    sfrvec_t up = {0.f, 1.f, 0.f, 1.f};
    sfrvec_t target = {0.f, 0.f, 1.f, 1.f};

    const sfrmat_t rotX = sfr_mat_rot_x(pitch);
    const sfrmat_t rotY = sfr_mat_rot_y(yaw);
    const sfrmat_t rotZ = sfr_mat_rot_z(roll);

    up = sfr_mat_mul_vec(rotZ, up);
    target = sfr_mat_mul_vec(rotX, target);
    target = sfr_mat_mul_vec(rotY, target);
    target = sfr_vec_add(sfrCamPos, target);

    sfrMatView = sfr_mat_qinv(sfr_mat_look_at(sfrCamPos, target, up));
}

SFR_FUNC void sfr_set_fov(float fovDeg) {
    const float aspect = (float)sfrHeight / sfrWidth;
    sfrMatProj = sfr_mat_proj(fovDeg, aspect, SFR_NEAR_DIST, SFR_FAR_DIST);
    sfrCamFov = fovDeg;
}

SFR_FUNC void sfr_set_lighting(int on, sfrvec_t dir, float ambientIntensity) {
    sfrLightingState.on = on;
    sfrLightingState.dir = dir;
    sfrLightingState.ambient = ambientIntensity;
}

#ifndef SFR_NO_STD
SFR_FUNC sfrmesh_t* sfr_load_mesh(const char* filename) {
    sfrmesh_t* mesh = (sfrmesh_t*)malloc(sizeof(sfrmesh_t));
    if (!mesh) {
        printf("sfr_load_mesh: failed to allocate mesh\n");
        return 0;
    }
    
    FILE* objFile = fopen(filename, "r");
    if (!objFile) {
        printf("sfr_load_mesh: failed to open file '%s'\n", filename);
        return 0;
    }


    int vi, ti = 0;
    float* verts = (float*)malloc(sizeof(float) * 3);
    char line[128];
    float x, y, z;
    while (fgets(line, 128, objFile)) {
        if ('v' == line[0] && sscanf(line, " v %f %f %f ", &x, &y, &z)) {
            verts = realloc(verts, (vi + 3) * sizeof(float));
            verts[vi++] = x;
            verts[vi++] = y;
            verts[vi++] = z;
        }
    }

    rewind(objFile);

    mesh->tris = (float*)malloc(sizeof(float) * 3);
    int a, b, c;
    while (fgets(line, 128, objFile)) {
        if ('f' != line[0]) {
            continue;
        }

        for (int i = 0; i < 128; i += 1) {
            if ('/' == line[i]) {
                while (' ' != line[i] && '\n' != line[i] && '\0' != line[i]) {
                    line[i++] = ' ';
                }
            }
        }

        if (sscanf(line, " f %d %d %d ", &a, &b, &c)) {
            a -= 1;
            b -= 1;
            c -= 1;
            mesh->tris = realloc(mesh->tris, (ti + 9) * sizeof(float));
            mesh->tris[ti++] = verts[(a * 3) + 0];
            mesh->tris[ti++] = verts[(a * 3) + 1];
            mesh->tris[ti++] = verts[(a * 3) + 2];
            mesh->tris[ti++] = verts[(b * 3) + 0];
            mesh->tris[ti++] = verts[(b * 3) + 1];
            mesh->tris[ti++] = verts[(b * 3) + 2];
            mesh->tris[ti++] = verts[(c * 3) + 0];
            mesh->tris[ti++] = verts[(c * 3) + 1];
            mesh->tris[ti++] = verts[(c * 3) + 2];
        }
    }

    free(verts);

    mesh->vertCount = ti;

    return mesh;
}

SFR_FUNC void sfr_release_mesh(sfrmesh_t** mesh) {
    if (!mesh || !(*mesh)) {
        return;
    }

    free((*mesh)->tris);
    (*mesh)->tris = NULL;
    
    free(*mesh);
    *mesh = NULL;
}
#endif

#ifdef __cplusplus
}
#endif

#endif // SFR_IMPL
#endif // SFR_H
