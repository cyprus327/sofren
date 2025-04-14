#ifndef SFR_H
#define SFR_H

#ifdef __cplusplus
extern "C" {
#endif

//================================================
//:         PUBLIC API
//================================================

/*          pre processing settings:

        the default values are the opposite of the define unless otherwise stated,
        e.g. functions aren't inline by default

SFR_USE_INLINE - whether functions should be defined as 'static inline' or 'static'

SFR_FUNC - SFR_USE_INLINE controls this, however you can't turn off 'static'
         - if you define this as nothing, SFR_FUNC will be nothing, i.e. no 'static'

SFR_NO_STD - whether or not 'stdio.h' and 'stdlib.h' are allowed to be included
           - they're only used in 'sfr_load_mesh' and 'sfr_unload_mesh'

SFR_NO_MATH - whether or not 'math.h' is allowed to be included
            - there are fallbacks for functions but they're probably worse than math.h's

SFR_NO_STRING - whether or not 'string.h' is allowed to be included
              - will use custom memset / memmove when defined

SFR_NO_STDINT - whether or not 'stdint.h' is allowed to be included
              - dictates type of 'sfrint_t'

SFR_USE_DOUBLE - whether or not to use double precision floats, default is single
               - dictates type of 'sfrflt_t'

SFR_SQRT_ACCURACY - only applicable when SFR_NO_MATH defined, defaults to 20 if not defined

SFR_TRIG_ACCURACY - only applicable when SFR_NO_MATH defined, defaults to 10 if not defined


        TODO the following should be changed to a normal variable at some point

SFR_NO_CULLING - whether or not to cull triangles
*/

// not really a point of this now but I'll probably want more types later
#ifndef SFR_NO_STDINT
    #include <stdint.h>
    typedef int32_t sfrint_t;
#else
    typedef int sfrint_t;
#endif

#ifdef SFR_USE_DOUBLE
    typedef double sfrflt_t;
#else
    typedef float sfrflt_t;
#endif

typedef struct sfrvec  sfrvec_t;
typedef struct sfrmat  sfrmat_t;
typedef struct sfrmesh sfrmesh_t;

// colors are currently ARGB8888, e.g. red = 0xFF0000, blue = 0x0000FF
// TODO support for other formats
typedef sfrint_t sfrcol_t;

#define SFR_DEPTH_SCALE 65536
typedef sfrint_t sfrfix_t; // 16.16 fixed point

// global variables below can be managed by you, however
// there is probably a function that will do what you want
extern sfrint_t sfrWidth, sfrHeight;
extern sfrcol_t* sfrPixelBuf;
extern sfrfix_t* sfrDepthBuf;
extern sfrint_t sfrRasterCount; // how many triangles have been rasterized since the last call to clear

extern sfrmat_t sfrMatModel, sfrMatView, sfrMatProj;
extern sfrvec_t sfrCamPos;
extern sfrflt_t sfrCamFov;
extern sfrflt_t sfrNearDist, sfrFarDist;

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
SFR_FUNC sfrvec_t sfr_vec_mul(sfrvec_t a, sfrflt_t b);
SFR_FUNC sfrvec_t sfr_vec_div(sfrvec_t a, sfrflt_t b);
SFR_FUNC sfrflt_t sfr_vec_dot(sfrvec_t a, sfrvec_t b);
SFR_FUNC sfrflt_t sfr_vec_length(sfrvec_t v);
SFR_FUNC sfrflt_t sfr_vec_length2(sfrvec_t v);
SFR_FUNC sfrvec_t sfr_vec_cross(sfrvec_t a, sfrvec_t b);
SFR_FUNC sfrvec_t sfr_vec_norm(sfrvec_t v);
SFR_FUNC sfrvec_t sfr_vec_normf(sfrflt_t a, sfrflt_t b, sfrflt_t c);
SFR_FUNC sfrvec_t sfr_vec_face_normal(sfrvec_t a, sfrvec_t b, sfrvec_t c);
SFR_FUNC sfrmat_t sfr_mat_identity();
SFR_FUNC sfrmat_t sfr_mat_rot_x(sfrflt_t a);
SFR_FUNC sfrmat_t sfr_mat_rot_y(sfrflt_t a);
SFR_FUNC sfrmat_t sfr_mat_rot_z(sfrflt_t a);
SFR_FUNC sfrmat_t sfr_mat_translate(sfrflt_t x, sfrflt_t y, sfrflt_t z);
SFR_FUNC sfrmat_t sfr_mat_scale(sfrflt_t x, sfrflt_t y, sfrflt_t z);
SFR_FUNC sfrmat_t sfr_mat_proj(sfrflt_t fovDev, sfrflt_t aspect, sfrflt_t near, sfrflt_t far);
SFR_FUNC sfrmat_t sfr_mat_mul(sfrmat_t a, sfrmat_t b);
SFR_FUNC sfrvec_t sfr_mat_mul_vec(sfrmat_t m, sfrvec_t v);
SFR_FUNC sfrmat_t sfr_mat_qinv(sfrmat_t m);
SFR_FUNC sfrmat_t sfr_mat_look_at(sfrvec_t pos, sfrvec_t target, sfrvec_t up);

// core functions
SFR_FUNC void sfr_init( // initialize matrices and set buffers
    sfrcol_t* pixelBuf, sfrfix_t* depthBuf, sfrint_t w, sfrint_t h, sfrflt_t fovDeg);

SFR_FUNC void sfr_reset(void);                                   // reset model matrix to identity and lighting to {0}
SFR_FUNC void sfr_rotate_x(sfrflt_t theta);                      // rotate model matrix about x by theta radians
SFR_FUNC void sfr_rotate_y(sfrflt_t theta);                      // rotate model matrix about y by theta radians
SFR_FUNC void sfr_rotate_z(sfrflt_t theta);                      // rotate model matrix about z by theta radians
SFR_FUNC void sfr_translate(sfrflt_t x, sfrflt_t y, sfrflt_t z); // translate model matrix by x y z
SFR_FUNC void sfr_scale(sfrflt_t x, sfrflt_t y, sfrflt_t z);     // scale model matrix by x y z
SFR_FUNC void sfr_look_at(sfrflt_t x, sfrflt_t y, sfrflt_t z);   // set view matrix to look at x y z

SFR_FUNC void sfr_clear(void); // reset depth and pixel buffers
SFR_FUNC void sfr_triangle(    // using current matrices, draw specified triangle
    sfrflt_t ax, sfrflt_t ay, sfrflt_t az,
    sfrflt_t bx, sfrflt_t by, sfrflt_t bz,
    sfrflt_t cx, sfrflt_t cy, sfrflt_t cz,
    sfrcol_t col);
SFR_FUNC void sfr_cube(sfrcol_t col);          // using current matrices, draw a cube
SFR_FUNC void sfr_mesh(const sfrmesh_t* mesh); // draw specified mesh, changing matrices based on 'mesh'

SFR_FUNC sfrint_t sfr_world_to_screen( // project the world position specified to screen coordinates
    sfrflt_t x, sfrflt_t y, sfrflt_t z, sfrint_t* screenX, sfrint_t* screenY);    

SFR_FUNC void sfr_set_camera( // update the camera with the new position and view
    sfrflt_t x, sfrflt_t y, sfrflt_t z, sfrflt_t yaw, sfrflt_t pitch, sfrflt_t roll);
SFR_FUNC void sfr_set_fov(sfrflt_t fovDeg); // update projection matrix with new fov
SFR_FUNC void sfr_set_lighting( // update internal lighting state for simple shading on triangles
    sfrint_t on, sfrvec_t dir, sfrflt_t ambientIntensity);

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

#ifndef SFR_NO_STRING
    #include <string.h> // for 'memset' and 'memmove'
    #define sfr_memset memset
    #define sfr_memmove memmove
#else
    SFR_FUNC void* sfr_memset(void* dest, char c, sfrint_t count) {
        char* p = (char*)dest;
        while (count--) {
            *p++ = c;
        }
        return dest;
    }
    
    SFR_FUNC void* sfr_memmove(void* dest, const void* src, sfrint_t count) {
        char* d = (char*)dest;
        const char* s = (const char*)src;
        
        if (d == s) {
            return dest;
        }
        
        // check for overlapping regions
        if (s < d && d < s + count) {
            // copy backwards from end to handle overlap
            d += count;
            s += count;
            while (count--) {
                *--d = *--s;
            }
        } else {
            // copy forwards
            while (count--) {
                *d++ = *s++;
            }
        }

        return dest;
    }
#endif

#ifndef SFR_SQRT_ACCURACY
    #define SFR_SQRT_ACCURACY 20
#endif
#ifndef SFR_TRIG_ACCURACY
    #define SFR_TRIG_ACCURACY 10
#endif


//================================================
//:         TYPES
//================================================

typedef struct sfrvec {
    sfrflt_t x, y, z, w;
} sfrvec_t;

typedef struct sfrmat {
    sfrflt_t m[4][4];
} sfrmat_t;

typedef struct sfrtri {
    sfrvec_t p[3];
} sfrtri;

typedef struct sfrLightingState {
    sfrint_t on;
    sfrvec_t dir;
    sfrflt_t ambient;
} SfrLightingState;

typedef struct sfrmesh {
    sfrflt_t* tris;
    sfrint_t vertCount;
    sfrvec_t pos, scale, rot;
    sfrcol_t col;
} sfrmesh_t;


//================================================
//:         GLOBAL VARIABLES
//================================================

sfrint_t sfrWidth, sfrHeight;
sfrcol_t* sfrPixelBuf;
sfrfix_t* sfrDepthBuf;
sfrint_t sfrRasterCount;

sfrmat_t sfrMatModel, sfrMatView, sfrMatProj;
sfrvec_t sfrCamPos;
sfrflt_t sfrCamFov;
sfrflt_t sfrNearDist = 0.1f, sfrFarDist = 100.f;

SfrLightingState sfrLightingState = {0};
sfrmat_t sfrMatNormal; // only used for shading
sfrint_t sfrIsNormalDirty;  // only used for shading


//================================================
//:         MISC HELPER MACROS
//================================================

#define SFR_PI ((sfrflt_t)3.14159265358979323846)
#define SFR_EPSILON ((sfrflt_t)1e-10)

#define SFR_SWAPF(a, b) { sfrflt_t _swapTemp = (a); (a) = (b); (b) = _swapTemp; }
#define SFR_VEC0 ((sfrvec_t){0.f, 0.f, 0.f, 0.f})


//================================================
//:         MATH
//================================================

#ifndef SFR_NO_MATH
    #include <math.h>
    #ifdef SFR_USE_DOUBLE
        #define sfr_floorf floor
        #define sfr_fmaxf fmax
        #define sfr_sqrtf sqrt
        #define sfr_cosf cos
        #define sfr_sinf sin
        #define sfr_tanf tan
    #else
        #define sfr_floorf floorf
        #define sfr_fmaxf fmaxf
        #define sfr_sqrtf sqrtf
        #define sfr_cosf cosf
        #define sfr_sinf sinf
        #define sfr_tanf tanf
    #endif
#else
    SFR_FUNC sfrflt_t sfr_floorf(sfrflt_t x) {
        const sfrint_t ix = (sfrint_t)x;
        return (x < ix) ? ix - 1 : ix;
    }

    SFR_FUNC sfrflt_t sfr_fmaxf(sfrflt_t a, sfrflt_t b) {
        return (a > b) ? a : b;
    }

    SFR_FUNC sfrflt_t sfr_sqrtf(sfrflt_t x) { // newton-raphson method
        if (x <= 0.f) {
            return 0.f;
        }

        sfrflt_t g = x;
        for (sfrint_t i = 0; i < SFR_SQRT_ACCURACY; i += 1) {
            g = 0.5f * (g + x / g);
        }
        return g;
    }

    SFR_FUNC sfrflt_t sfr_cosf(sfrflt_t x) { // taylor series approximation
        x -= (2 * SFR_PI) * sfr_floorf((x + SFR_PI) / (2.f * SFR_PI));
        const sfrflt_t x2 = x * x;
        sfrflt_t term = 1.f, sum = 1.f;
        for (sfrint_t i = 1, n = 0; i < SFR_TRIG_ACCURACY; i += 1) {
            n += 2;
            term *= -x2 / (n * (n - 1));
            sum += term;
        }
        return sum;
    }

    SFR_FUNC sfrflt_t sfr_sinf(sfrflt_t x) { // taylor series approximation
        x -= (2 * SFR_PI) * sfr_floorf((x + SFR_PI) / (2.f * SFR_PI));
        const sfrflt_t x2 = x * x;
        sfrflt_t term = x, sum = x;
        for (sfrint_t i = 1, n = 1; i < SFR_TRIG_ACCURACY; i += 1) {
            n += 2;
            term *= -x2 / (n * (n - 1));
            sum += term;
        }
        return sum;
    }

    SFR_FUNC sfrflt_t sfr_tanf(sfrflt_t x) {
        const sfrflt_t c = sfr_cosf(x);
        return (c < SFR_EPSILON && c > -SFR_EPSILON) ? 0.f : sfr_sinf(x) / c;
    }
#endif

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

SFR_FUNC sfrvec_t sfr_vec_mul(sfrvec_t a, sfrflt_t b) {
    sfrvec_t r;
    r.x = a.x * b;
    r.y = a.y * b;
    r.z = a.z * b;
    r.w = a.w * b;
    return r;
}


SFR_FUNC sfrvec_t sfr_vec_div(sfrvec_t a, sfrflt_t b) {
    sfrvec_t r;
    r.x = a.x / b;
    r.y = a.y / b;
    r.z = a.z / b;
    r.w = 1.f;
    return r;
}

SFR_FUNC sfrflt_t sfr_vec_dot(sfrvec_t a, sfrvec_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

SFR_FUNC sfrflt_t sfr_vec_length(sfrvec_t v) {
    return sfr_sqrtf(sfr_vec_dot(v, v));
}

SFR_FUNC sfrflt_t sfr_vec_length2(sfrvec_t v) {
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
    const sfrflt_t l = sfr_vec_length(v);
    sfrvec_t r;
    r.x = v.x / l;
    r.y = v.y / l;
    r.z = v.z / l;
    r.w = 1.f;
    return r;
}

SFR_FUNC sfrvec_t sfr_vec_normf(sfrflt_t x, sfrflt_t y, sfrflt_t z) {
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

SFR_FUNC sfrmat_t sfr_mat_rot_x(sfrflt_t a) {
    sfrmat_t r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = sfr_cosf(a);
    r.m[1][2] = sfr_sinf(a);
    r.m[2][1] = -sfr_sinf(a);
    r.m[2][2] = sfr_cosf(a);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_rot_y(sfrflt_t a) {
    sfrmat_t r = {0};
    r.m[0][0] = sfr_cosf(a);
    r.m[0][2] = sfr_sinf(a);
    r.m[1][1] = 1.f;
    r.m[2][0] = -sfr_sinf(a);
    r.m[2][2] = sfr_cosf(a);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_rot_z(sfrflt_t a) {
    sfrmat_t r = {0};
    r.m[0][0] = sfr_cosf(a);
    r.m[0][1] = sfr_sinf(a);
    r.m[1][0] = -sfr_sinf(a);
    r.m[1][1] = sfr_cosf(a);
    r.m[2][2] = 1.f;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_translate(sfrflt_t x, sfrflt_t y, sfrflt_t z) {
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

SFR_FUNC sfrmat_t sfr_mat_scale(sfrflt_t x, sfrflt_t y, sfrflt_t z) {
    sfrmat_t r = {0};
    r.m[0][0] = x;
    r.m[1][1] = y;
    r.m[2][2] = z;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat_t sfr_mat_proj(sfrflt_t fovDev, sfrflt_t aspect, sfrflt_t near, sfrflt_t far) {
    const sfrflt_t fov = 1.0 / sfr_tanf(fovDev * 0.5 / 180.0 * SFR_PI);
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
    for (sfrint_t c = 0; c < 4; c += 1) {
        for (sfrint_t r = 0; r < 4; r += 1) {
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
    const sfrflt_t delta = -sfr_vec_dot(norm, plane);
    const sfrflt_t ad = sfr_vec_dot(start, norm);
    const sfrflt_t bd = sfr_vec_dot(end, norm);
    const sfrflt_t t = (-delta - ad) / (bd - ad);
    
    const sfrvec_t startToEnd = sfr_vec_sub(end, start);
    const sfrvec_t segment = sfr_vec_mul(startToEnd, t);
    
    return sfr_vec_add(start, segment);
}

SFR_FUNC sfrint_t sfr_clip_against_plane(sfrtri out[2], sfrvec_t plane, sfrvec_t norm, sfrtri in) {
    norm = sfr_vec_norm(norm);

    sfrvec_t* inside[3];
    sfrvec_t* outside[3];
    sfrint_t insideCount = 0, outsideCount = 0;

    const sfrflt_t ndotp = sfr_vec_dot(norm, plane);
    for (sfrint_t i = 0; i < 3; i += 1) {
        const sfrflt_t d = norm.x * in.p[i].x + norm.y * in.p[i].y + norm.z * in.p[i].z - ndotp;
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
    sfrflt_t ax, sfrflt_t ay, sfrflt_t az,
    sfrflt_t bx, sfrflt_t by, sfrflt_t bz,
    sfrflt_t cx, sfrflt_t cy, sfrflt_t cz,
    sfrcol_t col
) {
    sfrRasterCount += 1;

    ax = (sfrflt_t)((sfrint_t)ax);
    ay = (sfrflt_t)((sfrint_t)ay);
    bx = (sfrflt_t)((sfrint_t)bx);
    by = (sfrflt_t)((sfrint_t)by);
    cx = (sfrflt_t)((sfrint_t)cx);
    cy = (sfrflt_t)((sfrint_t)cy);

    if (ay > by) {
        SFR_SWAPF(ax, bx);
        SFR_SWAPF(ay, by);
        SFR_SWAPF(az, bz);
    }
    if (ay > cy) {
        SFR_SWAPF(ax, cx);
        SFR_SWAPF(ay, cy);
        SFR_SWAPF(az, cz);
    }
    if (by > cy) {
        SFR_SWAPF(bx, cx);
        SFR_SWAPF(by, cy);
        SFR_SWAPF(bz, cz);
    }

    const sfrflt_t deltaACX = cx - ax, deltaACZ = cz - az;
    const sfrflt_t deltaABX = bx - ax, delta_ab_z = bz - az;
    const sfrflt_t invHeightAC = (cy != ay) ? 1.0 / (cy - ay) : 0.0;
    const sfrflt_t invHeightAB = (by != ay) ? 1.0 / (by - ay) : 0.0;

    sfrint_t y0 = ((sfrint_t)ay < 0) ? 0 : (sfrint_t)ay, y1 = ((sfrint_t)by >= sfrHeight) ? sfrHeight : (sfrint_t)by;
    for (sfrint_t y = y0; y < y1; y += 1) {
        const sfrflt_t dy = (sfrflt_t)y - ay;
        const sfrflt_t alpha = dy * invHeightAC;
        const sfrflt_t beta = dy * invHeightAB;

        sfrflt_t sx = ax + deltaACX * alpha, sz = az + deltaACZ * alpha;
        sfrflt_t ex = ax + deltaABX * beta, ez = az + delta_ab_z * beta;

        if (sx > ex) {
            SFR_SWAPF(sx, ex);
            SFR_SWAPF(sz, ez);
        }

        sfrint_t sxi = (sfrint_t)sx, exi = (sfrint_t)ex;
        sxi = (sxi < 0) ? 0 : ((sxi >= sfrWidth) ? sfrWidth : sxi);
        exi = (exi < 0) ? 0 : ((exi >= sfrWidth) ? sfrWidth : exi);
        if (sxi >= exi) {
            continue;
        }

        const sfrflt_t dxScan = ex - sx;
        const sfrflt_t depthStep = (0.0 != dxScan) ? (ez - sz) / dxScan : 0.0;
        sfrflt_t depth = sz + (sxi - sx) * depthStep;

        sfrint_t i = y * sfrWidth + sxi;
        for (sfrint_t x = sxi; x < exi; x += 1, i += 1, depth += depthStep) {
            const sfrfix_t depthVal = (sfrfix_t)(depth * SFR_DEPTH_SCALE);
            if (depthVal < sfrDepthBuf[i]) {
                sfrDepthBuf[i] = depthVal;
                sfrPixelBuf[i] = col;
            }
        }
    }

    const sfrflt_t deltaBCX = cx - bx, deltaBCZ = cz - bz;
    const sfrflt_t invHeightBC = (cy != by) ? 1.0 / (cy - by) : 0.0;

    y0 = ((sfrint_t)by < 0) ? 0 : (sfrint_t)by, y1 = ((sfrint_t)cy >= sfrHeight) ? sfrHeight : (sfrint_t)cy;
    for (sfrint_t y = y0; y < y1; y += 1) {
        const sfrflt_t dyAlpha = (sfrflt_t)y - ay;
        const sfrflt_t dyBeta  = (sfrflt_t)y - by;
        const sfrflt_t alpha = dyAlpha * invHeightAC;
        const sfrflt_t beta  = dyBeta * invHeightBC;

        sfrflt_t sx = ax + deltaACX * alpha, sz = az + deltaACZ * alpha;
        sfrflt_t ex = bx + deltaBCX * beta, ez = bz + deltaBCZ * beta;

        if (sx > ex) {
            SFR_SWAPF(sx, ex);
            SFR_SWAPF(sz, ez);
        }

        sfrint_t sxi = (sfrint_t)sx, exi = (sfrint_t)ex;
        sxi = (sxi < 0) ? 0 : ((sxi >= sfrWidth) ? sfrWidth : sxi);
        exi = (exi < 0) ? 0 : ((exi >= sfrWidth) ? sfrWidth : exi);
        if (sxi >= exi) {
            continue;
        }

        const sfrflt_t dxScan = ex - sx;
        const sfrflt_t depthStep = (0.0 != dxScan) ? (ez - sz) / dxScan : 0.0;
        sfrflt_t depth = sz + (sxi - sx) * depthStep;

        sfrint_t i = y * sfrWidth + sxi;
        for (sfrint_t x = sxi; x < exi; x += 1, i += 1, depth += depthStep) {
            const sfrfix_t depthVal = (sfrfix_t)(depth * SFR_DEPTH_SCALE);
            if (depthVal < sfrDepthBuf[i]) {
                sfrDepthBuf[i] = depthVal;
                sfrPixelBuf[i] = col;
            }
        }
    }
}

//: PUBLIC API FUNCTIONS

SFR_FUNC void sfr_init(sfrcol_t* pixelBuf, sfrfix_t* depthBuf, sfrint_t w, sfrint_t h, sfrflt_t fovDeg) {
    sfrWidth = w;
    sfrHeight = h;
    
    sfrPixelBuf = pixelBuf;
    sfrDepthBuf = depthBuf;

    sfr_clear();
    sfr_reset();

    sfr_set_fov(fovDeg);
    sfr_set_camera(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
}

SFR_FUNC void sfr_reset(void) {
    sfrMatModel = sfr_mat_identity();
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_rotate_x(sfrflt_t theta) {
    const sfrmat_t rot = sfr_mat_rot_x(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_rotate_y(sfrflt_t theta) {
    const sfrmat_t rot = sfr_mat_rot_y(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_rotate_z(sfrflt_t theta) {
    const sfrmat_t rot = sfr_mat_rot_z(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_translate(sfrflt_t x, sfrflt_t y, sfrflt_t z) {
    const sfrmat_t trans = sfr_mat_translate(x, y, z);
    sfrMatModel = sfr_mat_mul(sfrMatModel, trans);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_scale(sfrflt_t x, sfrflt_t y, sfrflt_t z) {
    const sfrmat_t scale = sfr_mat_scale(x, y, z);
    sfrMatModel = sfr_mat_mul(sfrMatModel, scale);
    sfrIsNormalDirty = 1;
}

SFR_FUNC void sfr_look_at(sfrflt_t x, sfrflt_t y, sfrflt_t z) {
    const sfrvec_t up = {0.f, 1.f, 0.f, 1.f};
    const sfrmat_t view = sfr_mat_look_at(sfrCamPos, (sfrvec_t){x, y, z, 1.f}, up);
    sfrMatView = sfr_mat_qinv(view);
}

SFR_FUNC void sfr_clear(void) {
    sfr_memset(sfrPixelBuf, 0, sizeof(sfrcol_t) * sfrWidth * sfrHeight);
    sfr_memset(sfrDepthBuf, 0x7f, sizeof(sfrfix_t) * sfrWidth * sfrHeight);
    sfrRasterCount = 0;
}

// helper for when lighting is enabled
SFR_FUNC sfrcol_t sfr_adjust_color_u32(sfrcol_t col, sfrflt_t intensity) {
    const unsigned char r = (col >> 16) & 0xFF;
    const unsigned char g = (col >> 8)  & 0xFF;
    const unsigned char b = (col >> 0)  & 0xFF;
    return ((unsigned char)(r * intensity) << 16) |
           ((unsigned char)(g * intensity) << 8)  |
           ((unsigned char)(b * intensity) << 0);
}

// helper for updating normal mat used for shading
SFR_FUNC void sfr_update_normal_mat(void) {
    const sfrflt_t* m = &sfrMatModel.m[0][0];
    
    const sfrflt_t a = m[0], b = m[4], c = m[8];
    const sfrflt_t d = m[1], e = m[5], f = m[9];
    const sfrflt_t g = m[2], h = m[6], i = m[10];

    const sfrflt_t det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    const sfrflt_t invDet = (0 != det) ? 1.f / det : 0.f;

    sfrMatNormal = (sfrmat_t){{
        { (e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet, 0.f },
        { (f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet, 0.f },
        { (d * h - e * g) * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet, 0.f },
        { 0.f, 0.f, 0.f, 1.f }
    }};

    sfrIsNormalDirty = 0;
}

SFR_FUNC void sfr_triangle(
    sfrflt_t ax, sfrflt_t ay, sfrflt_t az,
    sfrflt_t bx, sfrflt_t by, sfrflt_t bz,
    sfrflt_t cx, sfrflt_t cy, sfrflt_t cz,
    sfrcol_t col
) {
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
        if (sfr_vec_dot(normal, camRay) > 0.f) {
            return;
        }
    #endif

    tri.p[0] = sfr_mat_mul_vec(sfrMatView, tri.p[0]);
    tri.p[1] = sfr_mat_mul_vec(sfrMatView, tri.p[1]);
    tri.p[2] = sfr_mat_mul_vec(sfrMatView, tri.p[2]);

    sfrtri clipped[2];
    const sfrint_t clippedCount = sfr_clip_against_plane(clipped,
        (sfrvec_t){0.f, 0.f, sfrNearDist, 1.f},
        (sfrvec_t){0.f, 0.f, 1.f, 1.f},
        tri);

    const sfrflt_t w2 = sfrWidth / 2.f, h2 = sfrHeight / 2.f;
    sfrtri queue[16];
    for (sfrint_t c = 0; c < clippedCount; c += 1) {
        tri.p[0] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[0]);
        tri.p[1] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[1]);
        tri.p[2] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[2]);

        tri.p[0] = sfr_vec_div(tri.p[0], tri.p[0].w);
        tri.p[1] = sfr_vec_div(tri.p[1], tri.p[1].w);
        tri.p[2] = sfr_vec_div(tri.p[2], tri.p[2].w);

        for (sfrint_t i = 0; i < 3; i += 1) {
            tri.p[i].x =  (tri.p[i].x + 1.f) * w2;
            tri.p[i].y = (-tri.p[i].y + 1.f) * h2;
        }

        queue[c] = tri;
    }

    const sfrvec_t clipPlanes[4][2] = {
        {{0.f, 0.5f, 0.f, 1.f}, {0.f, 1.f, 0.f, 1.f}},                 // top
        {{0.f, (sfrflt_t)sfrHeight, 0.f, 1.f}, {0.f, -1.f, 0.f, 1.f}}, // bottom 
        {{0.5f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 1.f}},                 // left
        {{(sfrflt_t)sfrWidth, 0.f, 0.f, 1.f}, {-1.f, 0.f, 0.f, 1.f}},  // right
    };

    sfrtri bufferA[sizeof(queue) / sizeof(queue[0])], bufferB[sizeof(queue) / sizeof(queue[0])];
    sfrtri* inputBuffer = bufferA;
    sfrtri* outputBuffer = bufferB;
    sfrint_t inputCount = clippedCount, outputCount;

    for (sfrint_t i = 0; i < clippedCount; i += 1) {
        inputBuffer[i] = queue[i];
    }

    for (sfrint_t p = 0; p < 4; p += 1) {
        outputCount = 0;
        for (sfrint_t i = 0; i < inputCount; i += 1) {
            const sfrtri test = inputBuffer[i];
            const sfrint_t c = sfr_clip_against_plane(clipped, clipPlanes[p][0], clipPlanes[p][1], test);
            for (sfrint_t j = 0; j < c; j += 1) {
                // if (outputCount < sizeof(queue) / sizeof(queue[0])) {
                outputBuffer[outputCount] = clipped[j];
                outputCount += 1;
                // }
            }
        }
        
        sfrtri* temp = inputBuffer;
        inputBuffer = outputBuffer, outputBuffer = temp;
        inputCount = outputCount, outputCount = 0;
    }

    if (sfrLightingState.on) {
        if (sfrIsNormalDirty) {
            sfr_update_normal_mat();
        }

        sfrvec_t normal = sfr_vec_face_normal(
            (sfrvec_t){ax, ay, az},
            (sfrvec_t){bx, by, bz},
            (sfrvec_t){cx, cy, cz});
        
        normal = sfr_mat_mul_vec(sfrMatNormal, normal);
        normal = sfr_vec_norm(normal);

        const sfrflt_t intensity = sfr_fmaxf(
            sfrLightingState.ambient, 
            sfr_vec_dot(normal, sfrLightingState.dir));
        
        col = sfr_adjust_color_u32(col, intensity);
    }

    for (sfrint_t i = 0; i < inputCount; i += 1) {
        const sfrtri* tri = &inputBuffer[i];
        sfr_rasterize(
            tri->p[0].x, tri->p[0].y, tri->p[0].z,
            tri->p[1].x, tri->p[1].y, tri->p[1].z,
            tri->p[2].x, tri->p[2].y, tri->p[2].z,
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

    for (sfrint_t i = 0; i < mesh->vertCount; i += 9) {
        sfr_triangle(
            mesh->tris[i + 0], mesh->tris[i + 1], mesh->tris[i + 2],
            mesh->tris[i + 3], mesh->tris[i + 4], mesh->tris[i + 5],
            mesh->tris[i + 6], mesh->tris[i + 7], mesh->tris[i + 8],
            mesh->col);
    }
}

SFR_FUNC sfrint_t sfr_world_to_screen(sfrflt_t x, sfrflt_t y, sfrflt_t z, sfrint_t* screenX, sfrint_t* screenY) {
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
    *screenX = (sfrint_t)(p.x + 0.5f);
    *screenY = (sfrint_t)(p.y + 0.5f);

    return 1;
}

SFR_FUNC void sfr_set_camera(sfrflt_t x, sfrflt_t y, sfrflt_t z, sfrflt_t yaw, sfrflt_t pitch, sfrflt_t roll) {
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

SFR_FUNC void sfr_set_fov(sfrflt_t fovDeg) {
    const sfrflt_t aspect = (sfrflt_t)sfrHeight / sfrWidth;
    sfrMatProj = sfr_mat_proj(fovDeg, aspect, sfrNearDist, sfrFarDist);
    sfrCamFov = fovDeg;
}

SFR_FUNC void sfr_set_lighting(sfrint_t on, sfrvec_t dir, sfrflt_t ambientIntensity) {
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

    sfrint_t vi, ti = 0;
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

    mesh->tris = (sfrflt_t*)malloc(sizeof(sfrflt_t) * 3);
    sfrint_t a, b, c;
    while (fgets(line, 128, objFile)) {
        if ('f' != line[0]) {
            continue;
        }

        for (sfrint_t i = 0; i < 128; i += 1) {
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
            mesh->tris = realloc(mesh->tris, (ti + 9) * sizeof(sfrflt_t));
            mesh->tris[ti++] = (sfrflt_t)verts[(a * 3) + 0];
            mesh->tris[ti++] = (sfrflt_t)verts[(a * 3) + 1];
            mesh->tris[ti++] = (sfrflt_t)verts[(a * 3) + 2];
            mesh->tris[ti++] = (sfrflt_t)verts[(b * 3) + 0];
            mesh->tris[ti++] = (sfrflt_t)verts[(b * 3) + 1];
            mesh->tris[ti++] = (sfrflt_t)verts[(b * 3) + 2];
            mesh->tris[ti++] = (sfrflt_t)verts[(c * 3) + 0];
            mesh->tris[ti++] = (sfrflt_t)verts[(c * 3) + 1];
            mesh->tris[ti++] = (sfrflt_t)verts[(c * 3) + 2];
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

#endif // SFR_IMPL

#ifdef __cplusplus
}
#endif

#endif // SFR_H
