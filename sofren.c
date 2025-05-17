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

SFR_PREFIXED_TYPES - whether or not types start with 'sfr'
                   - e.g. 'sfrmesh_t' instead of just 'Mesh'

SFR_NO_WARNINGS - whether or not #warning is used to report small potential problems

SFR_NO_STD - whether or not 'stdio.h' and 'stdlib.h' are allowed to be included
           - they're only used in 'sfr_load_mesh' and 'sfr_unload_mesh'

SFR_NO_MATH - whether or not 'math.h' is allowed to be included
            - there are fallbacks for functions but they're probably worse than math.h's

SFR_NO_STRING - whether or not 'string.h' is allowed to be included
              - will use custom memset / memmove when defined

SFR_NO_STDINT - whether or not 'stdint.h' is allowed to be included
              - dictates type of 'i32', 'u32', etc.

SFR_SQRT_ACCURACY - only applicable when SFR_NO_MATH defined, defaults to 20 if not defined

SFR_TRIG_ACCURACY - only applicable when SFR_NO_MATH defined, defaults to 10 if not defined


        TODO the following should be changed to a normal variable at some point

SFR_NO_CULLING - whether or not to cull triangles
*/

//: types
#ifdef SFR_PREFIXED_TYPES
    #define Vec     sfrvec_t
    #define Mat     sfrmat_t
    #define Tri     sfrtri_t
    #define Mesh    sfrmesh_t
    #define Texture sfrtexture_t
    #define Font    sfrfont_t
    
    #define i8  sfri8_t
    #define u8  sfru8_t
    #define i16 sfri16_t
    #define u16 sfru16_t
    #define i32 sfri32_t
    #define u32 sfru32_t
    #define i64 sfri64_t
    #define u64 sfru64_t
    #define f32 sfrf32_t
    #define f64 sfrf64_t
#endif

#ifndef SFR_NO_STDINT
    #include <stdint.h>
    typedef int8_t   i8;
    typedef uint8_t  u8;
    typedef int16_t  i16;
    typedef uint16_t u16;
    typedef int32_t  i32;
    typedef uint32_t u32;
    typedef int64_t  i64;
    typedef uint64_t u64;
#else
    typedef signed char        i8;
    typedef unsigned char      u8;
    typedef signed short       i16;
    typedef unsigned short     u16;
    typedef signed int         i32;
    typedef unsigned int       u32;
    typedef signed long long   i64;
    typedef unsigned long long u64;
#endif
typedef float  f32;
typedef double f64;

typedef struct sfrvec     Vec;
typedef struct sfrmat     Mat;
typedef struct sfrmesh    Mesh;
typedef struct sfrtexture Texture;
typedef struct sfrfont    Font;

//: extern variables
extern i32 sfrWidth, sfrHeight;
extern u32* sfrPixelBuf; // ARGB8888 colors, TODO add more formats
extern f32* sfrDepthBuf;

// global variables below can be managed by you, however
// there is probably a function that will do what you want

extern i32 sfrRasterCount; // how many triangles have been rasterized since the last call to clear

extern Mat sfrMatModel, sfrMatView, sfrMatProj;
extern Vec sfrCamPos;
extern f32 sfrCamFov;
extern f32 sfrNearDist, sfrFarDist;

#ifdef SFR_FUNC
#ifdef SFR_USE_INLINE
    #warning "SFR WARNING: SFR_FUNC and SFR_USE_INLINE both being defined is contradictory, using SFR_FUNC"    
#endif
#endif

#ifndef SFR_FUNC
    #ifdef SFR_USE_INLINE
        #define SFR_FUNC static inline
    #else
        #define SFR_FUNC static
    #endif
#endif

#ifndef SFR_SQRT_ACCURACY
    #define SFR_SQRT_ACCURACY 20
#endif
#ifndef SFR_TRIG_ACCURACY
    #define SFR_TRIG_ACCURACY 10
#endif

#ifndef SFR_FONT_GLYPH_MAX
    #define SFR_FONT_GLYPH_MAX 512
#endif
#ifndef SFR_FONT_VERT_MAX
    // 72 verts max == 12 tris max
    #define SFR_FONT_VERT_MAX 72
#endif
#define SFR_FONT_VERT_EMPTY 1234321

//: math functions
SFR_FUNC Vec sfr_vec_add(Vec a, Vec b);
SFR_FUNC Vec sfr_vec_sub(Vec a, Vec b);
SFR_FUNC Vec sfr_vec_mul(Vec a, f32 b);
SFR_FUNC Vec sfr_vec_div(Vec a, f32 b);
SFR_FUNC f32 sfr_vec_dot(Vec a, Vec b);
SFR_FUNC f32 sfr_vec_length(Vec v);
SFR_FUNC f32 sfr_vec_length2(Vec v);
SFR_FUNC Vec sfr_vec_cross(Vec a, Vec b);
SFR_FUNC Vec sfr_vec_norm(Vec v);
SFR_FUNC Vec sfr_vec_normf(f32 a, f32 b, f32 c);
SFR_FUNC Vec sfr_vec_face_normal(Vec a, Vec b, Vec c);
SFR_FUNC Mat sfr_mat_identity();
SFR_FUNC Mat sfr_mat_rot_x(f32 a);
SFR_FUNC Mat sfr_mat_rot_y(f32 a);
SFR_FUNC Mat sfr_mat_rot_z(f32 a);
SFR_FUNC Mat sfr_mat_translate(f32 x, f32 y, f32 z);
SFR_FUNC Mat sfr_mat_scale(f32 x, f32 y, f32 z);
SFR_FUNC Mat sfr_mat_proj(f32 fovDev, f32 aspect, f32 near, f32 far);
SFR_FUNC Mat sfr_mat_mul(Mat a, Mat b);
SFR_FUNC Vec sfr_mat_mul_vec(Mat m, Vec v);
SFR_FUNC Mat sfr_mat_qinv(Mat m);
SFR_FUNC Mat sfr_mat_look_at(Vec pos, Vec target, Vec up);

//: core functions
SFR_FUNC void sfr_init( // initialize matrices and set buffers
    u32* pixelBuf, f32* depthBuf, i32 w, i32 h, f32 fovDeg);

SFR_FUNC void sfr_reset(void);                    // reset model matrix to identity and lighting to {0}
SFR_FUNC void sfr_rotate_x(f32 theta);            // rotate model matrix about x by theta radians
SFR_FUNC void sfr_rotate_y(f32 theta);            // rotate model matrix about y by theta radians
SFR_FUNC void sfr_rotate_z(f32 theta);            // rotate model matrix about z by theta radians
SFR_FUNC void sfr_translate(f32 x, f32 y, f32 z); // translate model matrix by x y z
SFR_FUNC void sfr_scale(f32 x, f32 y, f32 z);     // scale model matrix by x y z
SFR_FUNC void sfr_look_at(f32 x, f32 y, f32 z);   // set view matrix to look at x y z

// all drawing functions below other than sfr_mesh use current matrices when drawing
SFR_FUNC void sfr_clear(void); // reset depth and pixel buffers
SFR_FUNC void sfr_triangle(    // draw specified triangle
    f32 ax, f32 ay, f32 az,
    f32 bx, f32 by, f32 bz,
    f32 cx, f32 cy, f32 cz,
    u32 col);
SFR_FUNC void sfr_triangle_tex( // draw specified triangle with 'tex' applied
    f32 ax, f32 ay, f32 az, f32 au, f32 av,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv,
    const Texture* tex);
SFR_FUNC void sfr_cube(u32 col);                // draw a cube with a solid color
SFR_FUNC void sfr_cube_ex(u32 col[12]);         // draw a cube with triangles of specified colors
SFR_FUNC void sfr_cube_tex(const Texture* tex); // draw a cube with 'tex' applied
SFR_FUNC void sfr_mesh(const Mesh* mesh);       // draw specified mesh, using matrices based on 'mesh'
SFR_FUNC void sfr_mesh_tex(                     // same as 'sfr_mesh' but with 'tex' applied using mesh's uvs
    const Mesh* mesh, const Texture* tex);
SFR_FUNC void sfr_string(                       // draw a string, '\n' handled automatically
    const Font* font, const char* s, i32 sLength, u32 col);
SFR_FUNC void sfr_glyph(                        // draw single character glyph 
    const Font* font, u16 id, u32 col);

SFR_FUNC i32 sfr_world_to_screen( // project the world position specified to screen coordinates
    f32 x, f32 y, f32 z, i32* screenX, i32* screenY);    

SFR_FUNC void sfr_set_camera( // update the camera with the new position and view
    f32 x, f32 y, f32 z, f32 yaw, f32 pitch, f32 roll);
SFR_FUNC void sfr_set_fov(f32 fovDeg); // update projection matrix with new fov
SFR_FUNC void sfr_set_lighting( // update internal lighting state for simple shading on triangles
    i32 on, Vec dir, f32 ambientIntensity);

#ifndef SFR_NO_STD
    #include <stdio.h>
    #include <stdlib.h>

    SFR_FUNC Mesh* sfr_load_mesh(const char* filename); // load an obj file into a struct that sofren can use
    SFR_FUNC void sfr_release_mesh(Mesh** mesh);        // release loaded mesh's memory

    SFR_FUNC Texture* sfr_load_texture(const char* filename); // load a BMP texture
    SFR_FUNC void sfr_release_texture(Texture** texture);     // release loaded texture's memory

    SFR_FUNC Font* sfr_load_font(const char* filename); // load a .srft (sofren font type) font, see 'sfr-fontmaker'
    SFR_FUNC void sfr_release_font(Font** font);        // release loaded font's memory
#endif

SFR_FUNC void sfr_rand_seed(u32 seed);       // seed random number generator
SFR_FUNC u32 sfr_rand_next(void);            // Lehmer random number generator
SFR_FUNC i32 sfr_rand_int(i32 min, i32 max); // random int in range [min, max]
SFR_FUNC f32 sfr_rand_flt(f32 min, f32 max); // random f32 in range [min, max]


//================================================
//:         IMPLEMENTATION
//================================================

// #define SFR_IMPL
#ifdef SFR_IMPL

#ifndef SFR_NO_STRING
    #include <string.h> // for 'memset' and 'memmove'
    #define sfr_memset memset
    #define sfr_memmove memmove
#else
    SFR_FUNC void* sfr_memset(void* dest, char c, i32 count) {
        char* p = (char*)dest;
        while (count--) {
            *p++ = c;
        }
        return dest;
    }
    
    SFR_FUNC void* sfr_memmove(void* dest, const void* src, i32 count) {
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


//================================================
//:         TYPES
//================================================

typedef struct sfrvec {
    f32 x, y, z, w;
} Vec;

typedef struct sfrmat {
    f32 m[4][4];
} Mat;

typedef struct sfrtri {
    Vec p[3];
} Tri;

typedef struct sfrmesh { // see the 'sfr_mesh' for example usage
    f32* tris;           // vertex positions
    f32* uvs;            // uv coordinates
    i32 vertCount;       // total number of position floats (3 per vertex)
    Vec pos, scale, rot; // properties that will be used in 'sfr_mesh'
    u32 col;             // color to draw the mesh with if using 'sfr_mesh'
} Mesh;

typedef struct sfrtexture {
    u32* pixels;
    i32 w, h;
} Texture;

typedef struct sfrfont {
    // xy pairs [x0][y0][x1][y1][x2][...]
    f32 verts[SFR_FONT_GLYPH_MAX][SFR_FONT_VERT_MAX];
} Font;

typedef struct strState {
    i32 lightingEnabled;
    Vec lightingDir;
    f32 lightingAmbient;

    Mat matNormal;
    i32 normalMatDirty;

    u32 randState;
} SfrState;

// helper to track vertex attributes during clipping of textured triangles
typedef struct {
    Vec pos;   // position in view space
    f32 u, v;  // texture coords
    f32 viewZ; // z in view space for perspective correction
} SfrTexVert;


//================================================
//:         GLOBAL VARIABLES
//================================================

i32 sfrWidth, sfrHeight;
u32* sfrPixelBuf;
f32* sfrDepthBuf;
i32 sfrRasterCount;

Mat sfrMatModel, sfrMatView, sfrMatProj;
Vec sfrCamPos;
f32 sfrCamFov;
f32 sfrNearDist = 0.1f, sfrFarDist = 100.f;

// not extern, internal state
SfrState sfrState = {0};


//================================================
//:         MISC HELPER MACROS
//================================================

#define SFR_PI ((f32)3.14159265358979323846)
#define SFR_EPSILON ((f32)1e-10)

#define SFR_SWAPF(_a, _b) { f32 _swapTemp = (_a); (_a) = (_b); (_b) = _swapTemp; }
#define SFR_VEC0 ((Vec){0.f, 0.f, 0.f, 0.f})

#define SFR_ARRLEN(_arr)  (sizeof(_arr) / sizeof((_arr)[0]))

#define SFR_MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))
#define SFR_MAX(_a, _b) ((_a) > (_b) ? (_a) : (_b))
#define SFR_CLAMP(_x, _min, _max) ((_x) < (_min) ? (_min) : ((_x) > (_max) ? (_max) : (_x)))

#ifndef SFR_NO_STD
    #define SFR_ERR_EXIT(...) { \
        fprintf(stderr, "SFR error (%s) at line %d:\n\t", __FILE__, __LINE__); \
        fprintf(stderr, __VA_ARGS__); \
        exit(1); \
    }
    #define SFR_ERR_RET(_r, ...) { \
        fprintf(stderr, "SFR error (%s) at line %d:\n\t", __FILE__, __LINE__); \
        fprintf(stderr, __VA_ARGS__); \
        return _r; \
    }
#else
    #ifndef SFR_NO_WARNINGS
        #warning "SFR WARNING: If there is an internal error it will not be reported (SFR_NO_STD defined)"
    #endif
    #define SFR_ERR_EXIT(...) { *(int*)0 = 0 } // crash the program to exit
    #define SFR_ERR_RET(_r, ...) return (_r)
#endif


//================================================
//:         MATH
//================================================

#ifndef SFR_NO_MATH
    #include <math.h>
    #define sfr_floorf floorf
    #define sfr_fmaxf fmaxf
    #define sfr_sqrtf sqrtf
    #define sfr_cosf cosf
    #define sfr_sinf sinf
    #define sfr_tanf tanf
#else
    SFR_FUNC f32 sfr_floorf(f32 x) {
        const i32 ix = (i32)x;
        return (x < ix) ? ix - 1 : ix;
    }

    SFR_FUNC f32 sfr_fmaxf(f32 a, f32 b) {
        return (a > b) ? a : b;
    }

    SFR_FUNC f32 sfr_sqrtf(f32 x) { // newton-raphson method
        if (x <= 0.f) {
            return 0.f;
        }

        f32 g = x;
        for (i32 i = 0; i < SFR_SQRT_ACCURACY; i += 1) {
            g = 0.5f * (g + x / g);
        }
        return g;
    }

    SFR_FUNC f32 sfr_cosf(f32 x) { // taylor series approximation
        x -= (2 * SFR_PI) * sfr_floorf((x + SFR_PI) / (2.f * SFR_PI));
        const f32 x2 = x * x;
        f32 term = 1.f, sum = 1.f;
        for (i32 i = 1, n = 0; i < SFR_TRIG_ACCURACY; i += 1) {
            n += 2;
            term *= -x2 / (n * (n - 1));
            sum += term;
        }
        return sum;
    }

    SFR_FUNC f32 sfr_sinf(f32 x) { // taylor series approximation
        x -= (2 * SFR_PI) * sfr_floorf((x + SFR_PI) / (2.f * SFR_PI));
        const f32 x2 = x * x;
        f32 term = x, sum = x;
        for (i32 i = 1, n = 1; i < SFR_TRIG_ACCURACY; i += 1) {
            n += 2;
            term *= -x2 / (n * (n - 1));
            sum += term;
        }
        return sum;
    }

    SFR_FUNC f32 sfr_tanf(f32 x) {
        const f32 c = sfr_cosf(x);
        return (c < SFR_EPSILON && c > -SFR_EPSILON) ? 0.f : sfr_sinf(x) / c;
    }
#endif

SFR_FUNC Vec sfr_vec_add(Vec a, Vec b) {
    Vec r;
    r.x = a.x + b.x;
    r.y = a.y + b.y;
    r.z = a.z + b.z;
    r.w = a.w + b.w;
    return r;
}

SFR_FUNC Vec sfr_vec_sub(Vec a, Vec b) {
    Vec r;
    r.x = a.x - b.x;
    r.y = a.y - b.y;
    r.z = a.z - b.z;
    r.w = a.w - b.w;
    return r;
}

SFR_FUNC Vec sfr_vec_mul(Vec a, f32 b) {
    Vec r;
    r.x = a.x * b;
    r.y = a.y * b;
    r.z = a.z * b;
    r.w = a.w * b;
    return r;
}

SFR_FUNC Vec sfr_vec_div(Vec a, f32 b) {
    Vec r;
    r.x = a.x / b;
    r.y = a.y / b;
    r.z = a.z / b;
    r.w = 1.f;
    return r;
}

SFR_FUNC f32 sfr_vec_dot(Vec a, Vec b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

SFR_FUNC f32 sfr_vec_length(Vec v) {
    return sfr_sqrtf(sfr_vec_dot(v, v));
}

SFR_FUNC f32 sfr_vec_length2(Vec v) {
    return sfr_vec_dot(v, v);
}

SFR_FUNC Vec sfr_vec_cross(Vec a, Vec b) {
    Vec r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    r.w = 1.f;
    return r;
}

SFR_FUNC Vec sfr_vec_norm(Vec v) {
    const f32 l = sfr_vec_length(v);
    Vec r;
    r.x = v.x / l;
    r.y = v.y / l;
    r.z = v.z / l;
    r.w = 1.f;
    return r;
}

SFR_FUNC Vec sfr_vec_normf(f32 x, f32 y, f32 z) {
    return sfr_vec_norm((Vec){x, y, z, 1.f});
}

SFR_FUNC Vec sfr_vec_face_normal(Vec a, Vec b, Vec c) {
    const Vec edge1 = sfr_vec_sub(b, a);
    const Vec edge2 = sfr_vec_sub(c, a);
    return sfr_vec_norm(sfr_vec_cross(edge1, edge2));
}

SFR_FUNC Mat sfr_mat_identity() {
    Mat r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = 1.f;
    r.m[2][2] = 1.f;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC Mat sfr_mat_rot_x(f32 a) {
    Mat r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = sfr_cosf(a);
    r.m[1][2] = sfr_sinf(a);
    r.m[2][1] = -sfr_sinf(a);
    r.m[2][2] = sfr_cosf(a);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC Mat sfr_mat_rot_y(f32 a) {
    Mat r = {0};
    r.m[0][0] = sfr_cosf(a);
    r.m[0][2] = sfr_sinf(a);
    r.m[1][1] = 1.f;
    r.m[2][0] = -sfr_sinf(a);
    r.m[2][2] = sfr_cosf(a);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC Mat sfr_mat_rot_z(f32 a) {
    Mat r = {0};
    r.m[0][0] = sfr_cosf(a);
    r.m[0][1] = sfr_sinf(a);
    r.m[1][0] = -sfr_sinf(a);
    r.m[1][1] = sfr_cosf(a);
    r.m[2][2] = 1.f;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC Mat sfr_mat_translate(f32 x, f32 y, f32 z) {
    Mat r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = 1.f;
    r.m[2][2] = 1.f;
    r.m[3][0] = x;
    r.m[3][1] = y;
    r.m[3][2] = z;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC Mat sfr_mat_scale(f32 x, f32 y, f32 z) {
    Mat r = {0};
    r.m[0][0] = x;
    r.m[1][1] = y;
    r.m[2][2] = z;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC Mat sfr_mat_proj(f32 fovDev, f32 aspect, f32 near, f32 far) {
    const f32 fov = 1.f / sfr_tanf(fovDev * 0.5f / 180.f * SFR_PI);
    Mat r = {0};
    r.m[0][0] = aspect * fov;
    r.m[1][1] = fov;
    r.m[2][2] = far / (far - near);
    r.m[3][2] = (-far * near) / (far - near);
    r.m[2][3] = 1.f;
    r.m[3][3] = 0.f;
    return r;
}

SFR_FUNC Mat sfr_mat_mul(Mat a, Mat b) {
    Mat matrix;
    for (i32 c = 0; c < 4; c += 1) {
        for (i32 r = 0; r < 4; r += 1) {
            matrix.m[r][c] =
                a.m[r][0] * b.m[0][c] +
                a.m[r][1] * b.m[1][c] +
                a.m[r][2] * b.m[2][c] +
                a.m[r][3] * b.m[3][c];
        }
    }
    return matrix;
}

SFR_FUNC Vec sfr_mat_mul_vec(Mat m, Vec v) {
    Vec r;
    r.x = v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + v.w * m.m[3][0];
    r.y = v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + v.w * m.m[3][1];
    r.z = v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + v.w * m.m[3][2];
    r.w = v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + v.w * m.m[3][3];
    return r;
}

SFR_FUNC Mat sfr_mat_qinv(Mat m) {
    Mat r;
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

SFR_FUNC Mat sfr_mat_look_at(Vec pos, Vec target, Vec up) {
    const Vec forward = sfr_vec_norm(sfr_vec_sub(target, pos));
    up = sfr_vec_norm(sfr_vec_sub(up, sfr_vec_mul(forward, sfr_vec_dot(up, forward))));
    const Vec right = sfr_vec_cross(up, forward);

    Mat r;
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

SFR_FUNC Vec sfr_intersect_plane(Vec plane, Vec norm, Vec start, Vec end) {
    norm = sfr_vec_norm(norm);
    const f32 delta = -sfr_vec_dot(norm, plane);
    const f32 ad = sfr_vec_dot(start, norm);
    const f32 bd = sfr_vec_dot(end, norm);
    const f32 t = (-delta - ad) / (bd - ad);
    
    const Vec startToEnd = sfr_vec_sub(end, start);
    const Vec segment = sfr_vec_mul(startToEnd, t);
    
    return sfr_vec_add(start, segment);
}

SFR_FUNC i32 sfr_clip_against_plane(Tri out[2], Vec plane, Vec norm, Tri in) {
    norm = sfr_vec_norm(norm);

    Vec* inside[3];
    Vec* outside[3];
    i32 insideCount = 0, outsideCount = 0;

    const f32 ndotp = sfr_vec_dot(norm, plane);
    for (i32 i = 0; i < 3; i += 1) {
        const f32 d = norm.x * in.p[i].x + norm.y * in.p[i].y + norm.z * in.p[i].z - ndotp;
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

SFR_FUNC i32 sfr_clip_tex_tri_against_plane(SfrTexVert out[2][3], Vec plane, Vec norm, const SfrTexVert in[3]) {
    norm = sfr_vec_norm(norm);
    
    SfrTexVert inside[3];
    SfrTexVert outside[3];
    i32 insideCount = 0, outsideCount = 0;
    const f32 ndotp = sfr_vec_dot(norm, plane);

    // classify points
    for (i32 i = 0; i < 3; i += 1) {
        const f32 dist = sfr_vec_dot(norm, in[i].pos) - ndotp;
        if (dist >= 0) {
            inside[insideCount] = in[i];
            insideCount += 1;
        } else {
            outside[outsideCount] = in[i];
            outsideCount += 1;
        }
    }

    if (3 == insideCount) {
        out[0][0] = in[0];
        out[0][1] = in[1];
        out[0][2] = in[2];
        return 1;
    }

    if (1 == insideCount && 2 == outsideCount) {
        // interpolate between inside outside pairs
        const SfrTexVert A = inside[0];
        const SfrTexVert B = outside[0];
        const SfrTexVert C = outside[1];

        // intersection values
        const f32 tAB = (-ndotp - sfr_vec_dot(norm, A.pos)) / 
                       (sfr_vec_dot(norm, sfr_vec_sub(B.pos, A.pos)));
        const f32 tAC = (-ndotp - sfr_vec_dot(norm, A.pos)) / 
                       (sfr_vec_dot(norm, sfr_vec_sub(C.pos, A.pos)));

        // interpolate vertices
        SfrTexVert newB = {
            .pos = sfr_intersect_plane(plane, norm, A.pos, B.pos),
            .u = A.u + tAB * (B.u - A.u),
            .v = A.v + tAB * (B.v - A.v),
            .viewZ = A.viewZ + tAB * (B.viewZ - A.viewZ)
        };
        
        SfrTexVert newC = {
            .pos = sfr_intersect_plane(plane, norm, A.pos, C.pos),
            .u = A.u + tAC * (C.u - A.u),
            .v = A.v + tAC * (C.v - A.v),
            .viewZ = A.viewZ + tAC * (C.viewZ - A.viewZ)
        };

        out[0][0] = A;
        out[0][1] = newB;
        out[0][2] = newC;
        return 1;
    }

    if (2 == insideCount && 1 == outsideCount) {
        const SfrTexVert A = inside[0];
        const SfrTexVert B = inside[1];
        const SfrTexVert C = outside[0];

        const f32 tAC = (-ndotp - sfr_vec_dot(norm, A.pos)) / 
                       (sfr_vec_dot(norm, sfr_vec_sub(C.pos, A.pos)));
        const f32 tBC = (-ndotp - sfr_vec_dot(norm, B.pos)) / 
                       (sfr_vec_dot(norm, sfr_vec_sub(C.pos, B.pos)));

        SfrTexVert newAC = {
            .pos = sfr_intersect_plane(plane, norm, A.pos, C.pos),
            .u = A.u + tAC * (C.u - A.u),
            .v = A.v + tAC * (C.v - A.v),
            .viewZ = A.viewZ + tAC * (C.viewZ - A.viewZ)
        };

        SfrTexVert newBC = {
            .pos = sfr_intersect_plane(plane, norm, B.pos, C.pos),
            .u = B.u + tBC * (C.u - B.u),
            .v = B.v + tBC * (C.v - B.v),
            .viewZ = B.viewZ + tBC * (C.viewZ - B.viewZ)
        };

        out[0][0] = A;
        out[0][1] = B;
        out[0][2] = newAC;
        
        out[1][0] = B;
        out[1][1] = newAC;
        out[1][2] = newBC;
        
        return 2;
    }

    return 0;
}

SFR_FUNC void sfr_rasterize(
    f32 ax, f32 ay, f32 az,
    f32 bx, f32 by, f32 bz,
    f32 cx, f32 cy, f32 cz,
    u32 col
) {
    sfrRasterCount += 1;

    ax = (f32)((i32)ax);
    ay = (f32)((i32)ay);
    bx = (f32)((i32)bx);
    by = (f32)((i32)by);
    cx = (f32)((i32)cx);
    cy = (f32)((i32)cy);

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

    const f32 deltaACX = cx - ax, deltaACZ = cz - az;
    const f32 deltaABX = bx - ax, delta_ab_z = bz - az;
    const f32 invHeightAC = (cy != ay) ? 1.0 / (cy - ay) : 0.0;
    const f32 invHeightAB = (by != ay) ? 1.0 / (by - ay) : 0.0;

    i32 y0 = ((i32)ay < 0) ? 0 : (i32)ay, y1 = ((i32)by >= sfrHeight) ? sfrHeight : (i32)by;
    for (i32 y = y0; y < y1; y += 1) {
        const f32 dy = (f32)y - ay;
        const f32 alpha = dy * invHeightAC;
        const f32 beta = dy * invHeightAB;

        f32 sx = ax + deltaACX * alpha, sz = az + deltaACZ * alpha;
        f32 ex = ax + deltaABX * beta, ez = az + delta_ab_z * beta;

        if (sx > ex) {
            SFR_SWAPF(sx, ex);
            SFR_SWAPF(sz, ez);
        }

        i32 sxi = (i32)sx, exi = (i32)ex;
        sxi = (sxi < 0) ? 0 : ((sxi >= sfrWidth) ? sfrWidth : sxi);
        exi = (exi < 0) ? 0 : ((exi >= sfrWidth) ? sfrWidth : exi);
        if (sxi >= exi) {
            continue;
        }

        const f32 dxScan = ex - sx;
        const f32 depthStep = (0.0 != dxScan) ? (ez - sz) / dxScan : 0.0;
        f32 depth = sz + (sxi - sx) * depthStep;

        i32 i = y * sfrWidth + sxi;
        for (i32 x = sxi; x < exi; x += 1, i += 1, depth += depthStep) {
            if (depth < sfrDepthBuf[i]) {
                sfrDepthBuf[i] = depth;
                sfrPixelBuf[i] = col;
            }
        }
    }

    const f32 deltaBCX = cx - bx, deltaBCZ = cz - bz;
    const f32 invHeightBC = (cy != by) ? 1.0 / (cy - by) : 0.0;

    y0 = ((i32)by < 0) ? 0 : (i32)by, y1 = ((i32)cy >= sfrHeight) ? sfrHeight : (i32)cy;
    for (i32 y = y0; y < y1; y += 1) {
        const f32 dyAlpha = (f32)y - ay;
        const f32 dyBeta  = (f32)y - by;
        const f32 alpha = dyAlpha * invHeightAC;
        const f32 beta  = dyBeta * invHeightBC;

        f32 sx = ax + deltaACX * alpha, sz = az + deltaACZ * alpha;
        f32 ex = bx + deltaBCX * beta, ez = bz + deltaBCZ * beta;

        if (sx > ex) {
            SFR_SWAPF(sx, ex);
            SFR_SWAPF(sz, ez);
        }

        i32 sxi = (i32)sx, exi = (i32)ex;
        sxi = (sxi < 0) ? 0 : ((sxi >= sfrWidth) ? sfrWidth : sxi);
        exi = (exi < 0) ? 0 : ((exi >= sfrWidth) ? sfrWidth : exi);
        if (sxi >= exi) {
            continue;
        }

        const f32 dxScan = ex - sx;
        const f32 depthStep = (0.0 != dxScan) ? (ez - sz) / dxScan : 0.0;
        f32 depth = sz + (sxi - sx) * depthStep;

        i32 i = y * sfrWidth + sxi;
        for (i32 x = sxi; x < exi; x += 1, i += 1, depth += depthStep) {
            if (depth < sfrDepthBuf[i]) {
                sfrDepthBuf[i] = depth;
                sfrPixelBuf[i] = col;
            }
        }
    }
}

SFR_FUNC void sfr_rasterize_tex(
    f32 ax, f32 ay, f32 az, f32 aInvZ, f32 auoz, f32 avoz,
    f32 bx, f32 by, f32 bz, f32 bInvZ, f32 buoz, f32 bvoz,
    f32 cx, f32 cy, f32 cz, f32 cInvZ, f32 cuoz, f32 cvoz,
    const Texture* tex
) {
    sfrRasterCount += 1;

    ax = (f32)((i32)ax);
    ay = (f32)((i32)ay);
    bx = (f32)((i32)bx);
    by = (f32)((i32)by);
    cx = (f32)((i32)cx);
    cy = (f32)((i32)cy);

    // sort vertices by y coord
    if (ay > by) {
        SFR_SWAPF(ax, bx); SFR_SWAPF(ay, by); SFR_SWAPF(az, bz);
        SFR_SWAPF(aInvZ, bInvZ); SFR_SWAPF(auoz, buoz); SFR_SWAPF(avoz, bvoz);
    }
    if (ay > cy) {
        SFR_SWAPF(ax, cx); SFR_SWAPF(ay, cy); SFR_SWAPF(az, cz);
        SFR_SWAPF(aInvZ, cInvZ); SFR_SWAPF(auoz, cuoz); SFR_SWAPF(avoz, cvoz);
    }
    if (by > cy) {
        SFR_SWAPF(bx, cx); SFR_SWAPF(by, cy); SFR_SWAPF(bz, cz);
        SFR_SWAPF(bInvZ, cInvZ); SFR_SWAPF(buoz, cuoz); SFR_SWAPF(bvoz, cvoz);
    }

    // edge deltas for triangle
    const f32 deltaACX = cx - ax;
    const f32 deltaACZ = cz - az;
    const f32 deltaACinvZ = cInvZ - aInvZ;
    const f32 deltaACuoz = cuoz - auoz;
    const f32 deltaACvoz = cvoz - avoz;
    const f32 invHeightAC = (cy != ay) ? 1.0f / (cy - ay) : 0;

    { // lower triangle (A to B)
        const f32 deltaABX = bx - ax;
        const f32 deltaABZ = bz - az;
        const f32 deltaABinvZ = bInvZ - aInvZ;
        const f32 deltaAB_u_ov_z = buoz - auoz;
        const f32 deltaAB_v_ov_z = bvoz - avoz;
        const f32 invHeightAB = (by != ay) ? 1.0f / (by - ay) : 0;

        const i32 yStart = (ay < 0) ? 0 : (i32)ay;
        const i32 yEnd = (by >= sfrHeight) ? sfrHeight : (i32)by;
        for (i32 y = yStart; y < yEnd; y += 1) {
            const f32 dy = y - ay;
            const f32 alpha = dy * invHeightAC;
            const f32 beta = dy * invHeightAB;
    
            // interpolate along AC edge (start)
            f32 sx = ax + deltaACX * alpha;
            f32 sz = az + deltaACZ * alpha;
            f32 sInvZ = aInvZ + deltaACinvZ * alpha;
            f32 su = auoz + deltaACuoz * alpha;
            f32 sv = avoz + deltaACvoz * alpha;
    
            // interpolate along BC edge (end)
            f32 ex = ax + deltaABX * beta;
            f32 ez = az + deltaABZ * beta;
            f32 eInvZ = aInvZ + deltaABinvZ * beta;
            f32 eu = auoz + deltaAB_u_ov_z * beta;
            f32 ev = avoz + deltaAB_v_ov_z * beta;
    
            // swap start/end if needed
            if (sx > ex) {
                SFR_SWAPF(sx, ex); SFR_SWAPF(sz, ez);
                SFR_SWAPF(sInvZ, eInvZ); SFR_SWAPF(su, eu); SFR_SWAPF(sv, ev);
            }
    
            // clamp x coordinates
            const i32 xStart = (sx < 0) ? 0 : (i32)sx;
            const i32 xEnd = (ex >= sfrWidth) ? sfrWidth : (i32)ex;
            if (xStart >= xEnd) {
                continue;
            }
    
            // scanline interpolation parameters
            const f32 dx = ex - sx;
            const f32 tStep = (0.f != dx) ? 1.f / dx : 0.f;
            const f32 invZStep = (eInvZ - sInvZ) * tStep;
            const f32 uStep = (eu - su) * tStep;
            const f32 vStep = (ev - sv) * tStep;
    
            // initialize current values
            f32 invZ = sInvZ + (xStart - sx) * invZStep;
            f32 uoz = su + (xStart - sx) * uStep;
            f32 voz = sv + (xStart - sx) * vStep;
    
            // rasterize scanline
            for (i32 x = xStart, i = y * sfrWidth + xStart; x < xEnd; x += 1, i += 1) {
                // perspective correction
                const f32 z = 1.f / invZ;
                if (z < sfrDepthBuf[i]) {
                    sfrDepthBuf[i] = z;

                    // recover texture coords
                    const f32 u = uoz * z;
                    const f32 v = voz * z;
                    
                    // wrap texture coords and clamp
                    i32 tx = (i32)(u * (tex->w - 1));
                    i32 ty = (i32)(v * (tex->h - 1));
                    tx = (tx < 0) ? 0 : ((tx >= tex->w) ? tex->w - 1 : tx);
                    ty = (ty < 0) ? 0 : ((ty >= tex->h) ? tex->h - 1 : ty);
                    
                    sfrPixelBuf[i] = tex->pixels[ty * tex->w + tx];
                }
    
                // increment interpolants
                invZ += invZStep;
                uoz += uStep;
                voz += vStep;
            }
        }
    }

    { // upper triangle (B to C)
        const f32 deltaBCX = cx - bx;
        const f32 deltaBCZ = cz - bz;
        const f32 deltaBCinvZ = cInvZ - bInvZ;
        const f32 deltaBCuoz = cuoz - buoz;
        const f32 deltaBCvoz = cvoz - bvoz;
        const f32 invHeightBC = (cy != by) ? 1.f / (cy - by) : 0.f;
    
        const i32 yStart = (by < 0.f) ? 0.f : (i32)by;
        const i32 yEnd = (cy >= sfrHeight) ? sfrHeight : (i32)cy;
        for (i32 y = yStart; y < yEnd; y += 1) {
            const f32 alpha = (f32)(y - ay) * invHeightAC;
            const f32 beta = (f32)(y - by) * invHeightBC;
    
            // interpolate along BC edge (start)
            f32 sx = bx + deltaBCX * beta;
            f32 sz = bz + deltaBCZ * beta;
            f32 sInvZ = bInvZ + deltaBCinvZ * beta;
            f32 suoz = buoz + deltaBCuoz * beta;
            f32 svoz = bvoz + deltaBCvoz * beta;
    
            // interpolate along AC edge (end)
            f32 ex = ax + deltaACX * alpha;
            f32 ez = az + deltaACZ * alpha;
            f32 eInvZ = aInvZ + deltaACinvZ * alpha;
            f32 euoz = auoz + deltaACuoz * alpha;
            f32 evoz = avoz + deltaACvoz * alpha;
    
            // swap start/end if needed
            if (sx > ex) {
                SFR_SWAPF(sx, ex);
                SFR_SWAPF(sz, ez);
                SFR_SWAPF(sInvZ, eInvZ);
                SFR_SWAPF(suoz, euoz);
                SFR_SWAPF(svoz, evoz);
            }
    
            // clamp x coordinates
            const i32 xStart = (sx < 0) ? 0 : (i32)sx;
            const i32 xEnd = (ex >= sfrWidth) ? sfrWidth : (i32)ex;
            if (xStart >= xEnd) {
                continue;
            }
    
            // scanline interpolation parameters
            const f32 dx = ex - sx;
            const f32 tStep = (0.f != dx) ? 1.f / dx : 0.f;
            const f32 invZStep = (eInvZ - sInvZ) * tStep;
            const f32 uStep = (euoz - suoz) * tStep;
            const f32 vStep = (evoz - svoz) * tStep;
    
            // initialize current values
            f32 invZ = sInvZ + (xStart - sx) * invZStep;
            f32 uoz = suoz + (xStart - sx) * uStep;
            f32 voz = svoz + (xStart - sx) * vStep;
    
            // rasterize scanline
            for (i32 x = xStart, i = y * sfrWidth + xStart; x < xEnd; x += 1, i += 1) {
                // perspective correction
                const f32 z = 1.f / invZ;
                if (z < sfrDepthBuf[i]) {
                    sfrDepthBuf[i] = z;
                    
                    // recover texture coords
                    const f32 u = uoz * z;
                    const f32 v = voz * z;
                    
                    // wrap texture coords and clamp
                    i32 tx = (i32)(u * (tex->w - 1));
                    i32 ty = (i32)(v * (tex->h - 1));
                    tx = (tx < 0) ? 0 : ((tx >= tex->w) ? tex->w - 1 : tx);
                    ty = (ty < 0) ? 0 : ((ty >= tex->h) ? tex->h - 1 : ty);
                    
                    sfrPixelBuf[i] = tex->pixels[ty * tex->w + tx];
                }

                // increment interpolants
                invZ += invZStep;
                uoz += uStep;
                voz += vStep;
            }
        }
    }
}

// helper for when lighting is enabled
SFR_FUNC i32 sfr_adjust_color_u32(i32 col, f32 intensity) {
    const unsigned char r = (col >> 16) & 0xFF;
    const unsigned char g = (col >> 8)  & 0xFF;
    const unsigned char b = (col >> 0)  & 0xFF;
    return ((unsigned char)(r * intensity) << 16) |
           ((unsigned char)(g * intensity) << 8)  |
           ((unsigned char)(b * intensity) << 0);
}

// helper for updating normal mat used for shading
SFR_FUNC void sfr_update_normal_mat(void) {
    const f32* m = &sfrMatModel.m[0][0];
    
    const f32 a = m[0], b = m[4], c = m[8];
    const f32 d = m[1], e = m[5], f = m[9];
    const f32 g = m[2], h = m[6], i = m[10];

    const f32 det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    const f32 invDet = (0 != det) ? 1.f / det : 0.f;

    sfrState.matNormal = (Mat){{
        { (e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet, 0.f },
        { (f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet, 0.f },
        { (d * h - e * g) * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet, 0.f },
        { 0.f, 0.f, 0.f, 1.f }
    }};

    sfrState.normalMatDirty = 0;
}


//================================================
//:         PUBLIC API FUNCTION DEFINITIONS
//================================================

SFR_FUNC void sfr_init(u32* pixelBuf, f32* depthBuf, i32 w, i32 h, f32 fovDeg) {
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
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_rotate_x(f32 theta) {
    const Mat rot = sfr_mat_rot_x(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_rotate_y(f32 theta) {
    const Mat rot = sfr_mat_rot_y(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_rotate_z(f32 theta) {
    const Mat rot = sfr_mat_rot_z(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_translate(f32 x, f32 y, f32 z) {
    const Mat trans = sfr_mat_translate(x, y, z);
    sfrMatModel = sfr_mat_mul(sfrMatModel, trans);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_scale(f32 x, f32 y, f32 z) {
    const Mat scale = sfr_mat_scale(x, y, z);
    sfrMatModel = sfr_mat_mul(sfrMatModel, scale);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_look_at(f32 x, f32 y, f32 z) {
    const Vec up = {0.f, 1.f, 0.f, 1.f};
    const Mat view = sfr_mat_look_at(sfrCamPos, (Vec){x, y, z, 1.f}, up);
    sfrMatView = sfr_mat_qinv(view);
}

SFR_FUNC void sfr_clear(void) {
    sfr_memset(sfrPixelBuf, 0, sizeof(i32) * sfrWidth * sfrHeight);
    sfr_memset(sfrDepthBuf, 0x7f, sizeof(i32) * sfrWidth * sfrHeight);
    sfrRasterCount = 0;
}

SFR_FUNC void sfr_triangle(
    f32 ax, f32 ay, f32 az,
    f32 bx, f32 by, f32 bz,
    f32 cx, f32 cy, f32 cz,
    u32 col
) {
    Tri tri = {{
        {ax, ay, az, 1.f},
        {bx, by, bz, 1.f},
        {cx, cy, cz, 1.f},
    }};

    tri.p[0] = sfr_mat_mul_vec(sfrMatModel, tri.p[0]);
    tri.p[1] = sfr_mat_mul_vec(sfrMatModel, tri.p[1]);
    tri.p[2] = sfr_mat_mul_vec(sfrMatModel, tri.p[2]);

    #ifndef SFR_NO_CULLING
        const Vec line0 = sfr_vec_sub(tri.p[1], tri.p[0]);
        const Vec line1 = sfr_vec_sub(tri.p[2], tri.p[0]);
        const Vec normal = sfr_vec_cross(line0, line1);
        const Vec camRay = sfr_vec_sub(tri.p[0], sfrCamPos);
        if (sfr_vec_dot(normal, camRay) > 0.f) {
            return;
        }
    #endif

    tri.p[0] = sfr_mat_mul_vec(sfrMatView, tri.p[0]);
    tri.p[1] = sfr_mat_mul_vec(sfrMatView, tri.p[1]);
    tri.p[2] = sfr_mat_mul_vec(sfrMatView, tri.p[2]);

    Tri clipped[2];
    const i32 clippedCount = sfr_clip_against_plane(clipped,
        (Vec){0.f, 0.f, sfrNearDist, 1.f},
        (Vec){0.f, 0.f, 1.f, 1.f},
        tri);

    const f32 w2 = sfrWidth / 2.f, h2 = sfrHeight / 2.f;
    Tri queue[16];
    for (i32 c = 0; c < clippedCount; c += 1) {
        tri.p[0] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[0]);
        tri.p[1] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[1]);
        tri.p[2] = sfr_mat_mul_vec(sfrMatProj, clipped[c].p[2]);

        tri.p[0] = sfr_vec_div(tri.p[0], tri.p[0].w);
        tri.p[1] = sfr_vec_div(tri.p[1], tri.p[1].w);
        tri.p[2] = sfr_vec_div(tri.p[2], tri.p[2].w);

        for (i32 i = 0; i < 3; i += 1) {
            tri.p[i].x =  (tri.p[i].x + 1.f) * w2;
            tri.p[i].y = (-tri.p[i].y + 1.f) * h2;
        }

        queue[c] = tri;
    }

    const Vec clipPlanes[4][2] = {
        {{0.f, 0.5f, 0.f, 1.f}, {0.f, 1.f, 0.f, 1.f}},            // top
        {{0.f, (f32)sfrHeight, 0.f, 1.f}, {0.f, -1.f, 0.f, 1.f}}, // bottom 
        {{0.5f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 1.f}},            // left
        {{(f32)sfrWidth, 0.f, 0.f, 1.f}, {-1.f, 0.f, 0.f, 1.f}},  // right
    };

    Tri bufferA[SFR_ARRLEN(queue)], bufferB[SFR_ARRLEN(queue)];
    Tri* inputBuffer = bufferA;
    Tri* outputBuffer = bufferB;
    i32 inputCount = clippedCount, outputCount;

    for (i32 i = 0; i < clippedCount; i += 1) {
        inputBuffer[i] = queue[i];
    }

    for (i32 p = 0; p < 4; p += 1) {
        outputCount = 0;
        for (i32 i = 0; i < inputCount; i += 1) {
            const Tri test = inputBuffer[i];
            const i32 c = sfr_clip_against_plane(clipped, clipPlanes[p][0], clipPlanes[p][1], test);
            for (i32 j = 0; j < c; j += 1) {
                // if (outputCount < SFR_ARRLEN(queue)) {
                outputBuffer[outputCount] = clipped[j];
                outputCount += 1;
                // }
            }
        }
        
        Tri* temp = inputBuffer;
        inputBuffer = outputBuffer, outputBuffer = temp;
        inputCount = outputCount, outputCount = 0;
    }

    if (sfrState.lightingEnabled) {
        if (sfrState.normalMatDirty) {
            sfr_update_normal_mat();
        }

        Vec normal = sfr_vec_face_normal(
            (Vec){ax, ay, az},
            (Vec){bx, by, bz},
            (Vec){cx, cy, cz});
        
        normal = sfr_mat_mul_vec(sfrState.matNormal, normal);
        normal = sfr_vec_norm(normal);

        const f32 intensity = sfr_fmaxf(
            sfrState.lightingAmbient, 
            sfr_vec_dot(normal, sfrState.lightingDir));
        
        col = sfr_adjust_color_u32(col, intensity);
    }

    for (i32 i = 0; i < inputCount; i += 1) {
        const Tri* tri = &inputBuffer[i];
        sfr_rasterize(
            tri->p[0].x, tri->p[0].y, tri->p[0].z,
            tri->p[1].x, tri->p[1].y, tri->p[1].z,
            tri->p[2].x, tri->p[2].y, tri->p[2].z,
            col
        );
    }
}

SFR_FUNC void sfr_triangle_tex(
    f32 ax, f32 ay, f32 az, f32 au, f32 av,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv,
    const Texture* tex
) {
    // transform vertices to world space for culling
    Vec aModel = sfr_mat_mul_vec(sfrMatModel, (Vec){ax, ay, az, 1.f});
    Vec bModel = sfr_mat_mul_vec(sfrMatModel, (Vec){bx, by, bz, 1.f});
    Vec cModel = sfr_mat_mul_vec(sfrMatModel, (Vec){cx, cy, cz, 1.f});

    // backface culling
    #ifndef SFR_NO_CULLING
        Vec line0 = sfr_vec_sub(bModel, aModel);
        Vec line1 = sfr_vec_sub(cModel, aModel);
        Vec normal = sfr_vec_cross(line0, line1);
        Vec camRay = sfr_vec_sub(aModel, sfrCamPos);
        if (sfr_vec_dot(normal, camRay) > 0.f) {
            return;
        }
    #endif

    // to view space
    SfrTexVert viewTri[3] = {
        {sfr_mat_mul_vec(sfrMatView, aModel), au, av, 0},
        {sfr_mat_mul_vec(sfrMatView, bModel), bu, bv, 0},
        {sfr_mat_mul_vec(sfrMatView, cModel), cu, cv, 0}
    };
    for (int i = 0; i < 3; i += 1) {
        viewTri[i].viewZ = viewTri[i].pos.z; // original z for perspective
    }

    // clip against near plane
    SfrTexVert clipped[2][3];
    const i32 clippedCount = sfr_clip_tex_tri_against_plane(
        clipped,
        (Vec){0.f, 0.f, sfrNearDist, 1.f},
        (Vec){0.f, 0.f, 1.f, 1.f},
        viewTri
    );

    // process clipped triangles
    for (i32 c = 0; c < clippedCount; c += 1) {
        // project to screen space
        SfrTexVert* tri = clipped[c];
        Vec aProj = sfr_mat_mul_vec(sfrMatProj, tri[0].pos);
        Vec bProj = sfr_mat_mul_vec(sfrMatProj, tri[1].pos);
        Vec cProj = sfr_mat_mul_vec(sfrMatProj, tri[2].pos);

        // perspective divide and screen space conversion
        aProj = sfr_vec_div(aProj, aProj.w);
        bProj = sfr_vec_div(bProj, bProj.w);
        cProj = sfr_vec_div(cProj, cProj.w);

        const f32 w2 = sfrWidth / 2.f, h2 = sfrHeight / 2.f;
        aProj.x = (aProj.x + 1.f) * w2;
        aProj.y = (-aProj.y + 1.f) * h2;
        bProj.x = (bProj.x + 1.f) * w2;
        bProj.y = (-bProj.y + 1.f) * h2;
        cProj.x = (cProj.x + 1.f) * w2;
        cProj.y = (-cProj.y + 1.f) * h2;

        // screen space vertices with perspective attributes
        struct {
            f32 x, y, z;
            f32 invZ, uoz, voz;
        } screen[3] =
        { {
            aProj.x, aProj.y, aProj.z, 
            1.f / tri[0].viewZ, 
            tri[0].u / tri[0].viewZ, 
            tri[0].v / tri[0].viewZ
        }, {
            bProj.x, bProj.y, bProj.z, 
            1.f/tri[1].viewZ, 
            tri[1].u / tri[1].viewZ, 
            tri[1].v / tri[1].viewZ
        }, {
            cProj.x, cProj.y, cProj.z, 
            1.f / tri[2].viewZ, 
            tri[2].u / tri[2].viewZ, 
            tri[2].v / tri[2].viewZ
        }};

        // TODO clip against screen edges

        sfr_rasterize_tex(
            screen[0].x, screen[0].y, screen[0].z, screen[0].invZ, screen[0].uoz, screen[0].voz,
            screen[1].x, screen[1].y, screen[1].z, screen[1].invZ, screen[1].uoz, screen[1].voz,
            screen[2].x, screen[2].y, screen[2].z, screen[2].invZ, screen[2].uoz, screen[2].voz,
            tex
        );
    }
}

SFR_FUNC void sfr_cube(u32 col) {
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

SFR_FUNC void sfr_cube_ex(u32 col[12]) {
    sfr_triangle(-0.5,-0.5,-0.5, -0.5, 0.5,-0.5,  0.5, 0.5,-0.5, col[0]);
    sfr_triangle(-0.5,-0.5,-0.5,  0.5, 0.5,-0.5,  0.5,-0.5,-0.5, col[1]);
    sfr_triangle( 0.5,-0.5,-0.5,  0.5, 0.5,-0.5,  0.5, 0.5, 0.5, col[2]);
    sfr_triangle( 0.5,-0.5,-0.5,  0.5, 0.5, 0.5,  0.5,-0.5, 0.5, col[3]);
    sfr_triangle( 0.5,-0.5, 0.5,  0.5, 0.5, 0.5, -0.5, 0.5, 0.5, col[4]);
    sfr_triangle( 0.5,-0.5, 0.5, -0.5, 0.5, 0.5, -0.5,-0.5, 0.5, col[5]);
    sfr_triangle(-0.5,-0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5,-0.5, col[6]);
    sfr_triangle(-0.5,-0.5, 0.5, -0.5, 0.5,-0.5, -0.5,-0.5,-0.5, col[7]);
    sfr_triangle(-0.5, 0.5,-0.5, -0.5, 0.5, 0.5,  0.5, 0.5, 0.5, col[8]);
    sfr_triangle(-0.5, 0.5,-0.5,  0.5, 0.5, 0.5,  0.5, 0.5,-0.5, col[9]);
    sfr_triangle( 0.5,-0.5, 0.5, -0.5,-0.5, 0.5, -0.5,-0.5,-0.5, col[10]);
    sfr_triangle( 0.5,-0.5, 0.5, -0.5,-0.5,-0.5,  0.5,-0.5,-0.5, col[11]);
}

SFR_FUNC void sfr_cube_tex(const Texture* tex) {
    // front face
    sfr_triangle_tex(
        -0.5f,-0.5f,-0.5f, 0.f,0.f, 
        -0.5f, 0.5f,-0.5f, 0.f,1.f,
         0.5f, 0.5f,-0.5f, 1.f,1.f, tex);
    sfr_triangle_tex(
        -0.5f,-0.5f,-0.5f, 0.f,0.f,
         0.5f, 0.5f,-0.5f, 1.f,1.f,
         0.5f,-0.5f,-0.5f, 1.f,0.f, tex);

    // right face
    sfr_triangle_tex(
        0.5f,-0.5f,-0.5f, 0.f,0.f,
        0.5f, 0.5f,-0.5f, 0.f,1.f,
        0.5f, 0.5f, 0.5f, 1.f,1.f, tex);
    sfr_triangle_tex(
        0.5f,-0.5f,-0.5f, 0.f,0.f,
        0.5f, 0.5f, 0.5f, 1.f,1.f,
        0.5f,-0.5f, 0.5f, 1.f,0.f, tex);

    // back face
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 0.f,0.f,
         0.5f, 0.5f, 0.5f, 0.f,1.f,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, tex);
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 0.f,0.f,
        -0.5f, 0.5f, 0.5f, 1.f,1.f,
        -0.5f,-0.5f, 0.5f, 1.f,0.f, tex);

    // left face
    sfr_triangle_tex(
        -0.5f,-0.5f, 0.5f, 0.f,0.f,
        -0.5f, 0.5f, 0.5f, 0.f,1.f,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, tex);
    sfr_triangle_tex(
        -0.5f,-0.5f, 0.5f, 0.f,0.f,
        -0.5f, 0.5f,-0.5f, 1.f,1.f,
        -0.5f,-0.5f,-0.5f, 1.f,0.f, tex);

    // top face
    sfr_triangle_tex(
        -0.5f, 0.5f,-0.5f, 0.f,0.f,
        -0.5f, 0.5f, 0.5f, 0.f,1.f,
         0.5f, 0.5f, 0.5f, 1.f,1.f, tex);
    sfr_triangle_tex(
        -0.5f, 0.5f,-0.5f, 0.f,0.f,
         0.5f, 0.5f, 0.5f, 1.f,1.f,
         0.5f, 0.5f,-0.5f, 1.f,0.f, tex);

    // bottom face
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 0.f,0.f,
        -0.5f,-0.5f, 0.5f, 0.f,1.f,
        -0.5f,-0.5f,-0.5f, 1.f,1.f, tex);
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 0.f,0.f,
        -0.5f,-0.5f,-0.5f, 1.f,1.f,
         0.5f,-0.5f,-0.5f, 1.f,0.f, tex);
}

SFR_FUNC void sfr_mesh(const Mesh* mesh) {
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

    for (i32 i = 0; i < mesh->vertCount; i += 9) {
        sfr_triangle(
            mesh->tris[i + 0], mesh->tris[i + 1], mesh->tris[i + 2],
            mesh->tris[i + 3], mesh->tris[i + 4], mesh->tris[i + 5],
            mesh->tris[i + 6], mesh->tris[i + 7], mesh->tris[i + 8],
            mesh->col);
    }
}

SFR_FUNC void sfr_mesh_tex(const Mesh* mesh, const Texture* tex) {
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

    for (i32 i = 0; i < mesh->vertCount; i += 9) {
        const i32 uv = (i / 9) * 6;
        const f32 ax = mesh->tris[i + 0];
        const f32 ay = mesh->tris[i + 1];
        const f32 az = mesh->tris[i + 2];
        const f32 au = mesh->uvs[uv + 0];
        const f32 av = mesh->uvs[uv + 1];
        const f32 bx = mesh->tris[i + 3];
        const f32 by = mesh->tris[i + 4];
        const f32 bz = mesh->tris[i + 5];
        const f32 bu = mesh->uvs[uv + 2];
        const f32 bv = mesh->uvs[uv + 3];
        const f32 cx = mesh->tris[i + 6];
        const f32 cy = mesh->tris[i + 7];
        const f32 cz = mesh->tris[i + 8];
        const f32 cu = mesh->uvs[uv + 4];
        const f32 cv = mesh->uvs[uv + 5];
        sfr_triangle_tex(
            ax, ay, az, au, av,
            bx, by, bz, bu, bv,
            cx, cy, cz, cu, cv,
            tex
        );
    }
}

SFR_FUNC void sfr_string(const Font* font, const char* s, i32 sLength, u32 col) {
    SFR_ERR_RET(, "sfr_string: TODO not implemented, 'sfr_glyph' is implemented\n");
}

SFR_FUNC void sfr_glyph(const Font* font, u16 id, u32 col) {
    if (id >= SFR_FONT_GLYPH_MAX) {
        SFR_ERR_RET(, "sfr_glyph: invalid id (%d >= %d)\n", id, SFR_FONT_GLYPH_MAX);
    }

    // printf("V0: %f\n", font->verts[id][0]);

    const f32* t = font->verts[id];
    for (i32 i = 0; i < SFR_FONT_VERT_MAX; i += 6) {
        if (SFR_FONT_VERT_EMPTY == font->verts[id][i]) {
            break;
        }

        sfr_triangle(
            t[i + 0], t[i + 1], 0.f,
            t[i + 2], t[i + 3], 0.f,
            t[i + 4], t[i + 5], 0.f, col);
    }
}

SFR_FUNC i32 sfr_world_to_screen(f32 x, f32 y, f32 z, i32* screenX, i32* screenY) {
    Vec p = {x, y, z, 1.f};
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
    *screenX = (i32)(p.x + 0.5f);
    *screenY = (i32)(p.y + 0.5f);

    return 1;
}

SFR_FUNC void sfr_set_camera(f32 x, f32 y, f32 z, f32 yaw, f32 pitch, f32 roll) {
    sfrCamPos = (Vec){x, y, z, 1.f};
    Vec up = {0.f, 1.f, 0.f, 1.f};
    Vec target = {0.f, 0.f, 1.f, 1.f};

    const Mat rotX = sfr_mat_rot_x(pitch);
    const Mat rotY = sfr_mat_rot_y(yaw);
    const Mat rotZ = sfr_mat_rot_z(roll);

    up = sfr_mat_mul_vec(rotZ, up);
    target = sfr_mat_mul_vec(rotX, target);
    target = sfr_mat_mul_vec(rotY, target);
    target = sfr_vec_add(sfrCamPos, target);

    sfrMatView = sfr_mat_qinv(sfr_mat_look_at(sfrCamPos, target, up));
}

SFR_FUNC void sfr_set_fov(f32 fovDeg) {
    const f32 aspect = (f32)sfrHeight / sfrWidth;
    sfrMatProj = sfr_mat_proj(fovDeg, aspect, sfrNearDist, sfrFarDist);
    sfrCamFov = fovDeg;
}

SFR_FUNC void sfr_set_lighting(i32 on, Vec dir, f32 ambientIntensity) {
    sfrState.lightingEnabled = on;
    sfrState.lightingDir = dir;
    sfrState.lightingAmbient = ambientIntensity;
}

#ifndef SFR_NO_STD
SFR_FUNC Mesh* sfr_load_mesh(const char* filename) {
    Mesh* mesh = (Mesh*)malloc(sizeof(Mesh));
    if (!mesh) {
        SFR_ERR_RET(0, "sfr_load_mesh: failed to allocate mesh\n");
    }

    mesh->tris = NULL;
    mesh->uvs = NULL;
    mesh->vertCount = 0;

    FILE* objFile = fopen(filename, "r");
    if (!objFile) {
        free(mesh);
        SFR_ERR_RET(0, "sfr_load_mesh: failed to open file '%s'\n", filename);
    }

    // Read vertices and UVs
    f32* verts = NULL;
    f32* uvCoords = NULL;
    i32 vi = 0, uvi = 0;
    char line[128];
    f32 x, y, z, u, v;

    while (fgets(line, sizeof(line), objFile)) {
        if ('v' != line[0]) {
            continue;
        }

        if (' ' == line[1] && 3 == sscanf(line, "v %f %f %f", &x, &y, &z)) {
            verts = realloc(verts, (vi + 3) * sizeof(f32));
            if (!verts) {
                fclose(objFile);
                free(mesh);
                SFR_ERR_RET(0, "sfr_load_mesh: failed to allocate vertices\n");
            }
            verts[vi + 0] = x;
            verts[vi + 1] = y;
            verts[vi + 2] = z;
            vi += 3;
        } else if ('t' == line[1] && 2 == sscanf(line, "vt %f %f", &u, &v)) {
            uvCoords = realloc(uvCoords, (uvi + 2) * sizeof(f32));
            if (!uvCoords) {
                fclose(objFile);
                free(verts);
                free(mesh);
                SFR_ERR_RET(0, "sfr_load_mesh: failed to allocate UVs\n");
            }
            uvCoords[uvi + 0] = u;
            uvCoords[uvi + 1] = v;
            uvi += 2;
        }
    }

    rewind(objFile);

    // process faces
    i32 triInd = 0, uvInd = 0;
    u8 meshHasUVs = 0;

    while (fgets(line, sizeof(line), objFile)) {
        if ('f' != line[0]) {
            continue;
        }

        char* tokens[4];
        i32 tokenCount = 0;
        char* token = strtok(line, " \t\n");
        while (NULL != (token = strtok(NULL, " \t\n")) && tokenCount < 3) {
            tokens[tokenCount] = token;
            tokenCount += 1;
        }
        if (3 != tokenCount) {
            continue;
        }

        i32 vIndices[3], uvIndices[3];
        u8 currentFaceHasUV = 0;

        for (i32 i = 0; i < 3; i += 1) {
            vIndices[i] = uvIndices[i] = -1;
            char* parts[3] = {0};
            char* part = strtok(tokens[i], "/");
            for (i32 j = 0; part && j < 3; j += 1) {
                parts[j] = part;
                part = strtok(NULL, "/");
            }
            if (parts[0]) vIndices[i]  = atoi(parts[0]) - 1;
            if (parts[1]) uvIndices[i] = atoi(parts[1]) - 1;

            if (-1 != uvIndices[i]) {
                currentFaceHasUV = 1;
            }
        }

        if (currentFaceHasUV) {
            if (!meshHasUVs) {
                meshHasUVs = 1;
                for (i32 i = 0; i < 3; i += 1) {
                    if (-1 == uvIndices[i]) {
                        fclose(objFile);
                        free(verts);
                        free(uvCoords);
                        free(mesh->tris);
                        free(mesh);
                        SFR_ERR_RET(0, "sfr_load_mesh: inconsistent UV indices\n");
                    }
                }
                if (!uvCoords) {
                    fclose(objFile);
                    free(verts);
                    free(mesh);
                    SFR_ERR_RET(0, "sfr_load_mesh: UVs missing\n");
                }
            }

            for (i32 i = 0; i < 3; i += 1) {
                if (uvIndices[i] < 0 || uvIndices[i] >= uvi / 2) {
                    fclose(objFile);
                    free(verts);
                    free(uvCoords);
                    free(mesh->uvs);
                    free(mesh->tris);
                    free(mesh);
                    SFR_ERR_RET(0, "sfr_load_mesh: invalid UV index\n");
                }

                const f32 u    = uvCoords[uvIndices[i] * 2 + 0];
                const f32 uVal = uvCoords[uvIndices[i] * 2 + 1];

                mesh->uvs = realloc(mesh->uvs, (uvInd + 2) * sizeof(f32));
                if (!mesh->uvs) {
                    fclose(objFile);
                    free(verts);
                    free(uvCoords);
                    free(mesh->tris);
                    free(mesh);
                    SFR_ERR_RET(0, "sfr_load_mesh: failed to allocate UVs\n");
                }
                mesh->uvs[uvInd + 0] = u;
                mesh->uvs[uvInd + 1] = uVal;
                uvInd += 2;
            }
        } else if (meshHasUVs) {
            fclose(objFile);
            free(verts);
            free(uvCoords);
            free(mesh->uvs);
            free(mesh->tris);
            free(mesh);
            SFR_ERR_RET(0, "sfr_load_mesh: face missing UVs\n");
        }

        // add vertices to mesh->tris
        for (i32 i = 0; i < 3; i += 1) {
            if (vIndices[i] < 0 || vIndices[i] >= vi / 3) {
                fclose(objFile);
                free(verts);
                free(uvCoords);
                free(mesh->uvs);
                free(mesh->tris);
                free(mesh);
                SFR_ERR_RET(0, "sfr_load_mesh: invalid vertex index\n");
            }

            f32* v = &verts[vIndices[i] * 3];
            mesh->tris = realloc(mesh->tris, (triInd + 3) * sizeof(f32));
            if (!mesh->tris) {
                fclose(objFile);
                free(verts);
                free(uvCoords);
                free(mesh->uvs);
                free(mesh);
                SFR_ERR_RET(0, "sfr_load_mesh: failed to allocate vertices\n");
            }

            mesh->tris[triInd + 0] = v[0];
            mesh->tris[triInd + 1] = v[1];
            mesh->tris[triInd + 2] = v[2];
            triInd += 3;
        }
    }

    free(verts);
    free(uvCoords);
    fclose(objFile);
    mesh->vertCount = triInd;

    return mesh;
}

SFR_FUNC void sfr_release_mesh(Mesh** mesh) {
    if (!mesh || !(*mesh)) {
        return;
    }

    if ((*mesh)->tris) {
        free((*mesh)->tris);
        (*mesh)->tris = NULL;
    }

    if ((*mesh)->uvs) {
        free((*mesh)->uvs);
        (*mesh)->uvs = NULL;
    }

    free(*mesh);
    *mesh = NULL;
}

SFR_FUNC Texture* sfr_load_texture(const char* filename) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        SFR_ERR_RET(NULL, "sfr_load_texture: failed to open file '%s'\n", filename);
    }

    // read bmp headers
    u8 header[54];
    if (fread(header, 1, 54, file) != 54) {
        fclose(file);
        SFR_ERR_RET(NULL, "sfr_load_texture: not a valid BMP file ('%s')\n", filename);
    }

    if (header[0] != 'B' || header[1] != 'M') {
        fclose(file);
        SFR_ERR_RET(NULL, "sfr_load_texture: not a BMP file ('%s')\n", filename);
    }

    // parse header information
    const i32 dataOffset = *(i32*)(header + 0x0A);
    const i32 infoSize   = *(i32*)(header + 0x0E);
    const i32 width      = *(i32*)(header + 0x12);
    const i32 rawHeight  = *(i32*)(header + 0x16);
    const u16 bpp        = *(u16*)(header + 0x1C);
    const u32 comp       = *(u32*)(header + 0x1E);

    // verify supported format
    if (comp > 3) {
        fclose(file);
        SFR_ERR_RET(NULL, "sfr_load_texture: unsupported BMP compression\n");
    }

    // handle height direction
    const u8 isTopDown = rawHeight < 0;
    const i32 height = isTopDown ? -rawHeight : rawHeight;

    // extended header information
    u32 masks[4] = {0};
    u32 palette[256] = {0};
    i32 paletteSize = 0;

    if (bpp <= 8) {
        const i32 colorsUsed = *(i32*)(header + 0x2E);
        paletteSize = (colorsUsed ? colorsUsed : 1 << bpp) * 4;
        if (infoSize >= 40 && fread(palette, 1, paletteSize, file) != paletteSize) {
            fclose(file);
            SFR_ERR_RET(NULL, "sfr_load_texture: failed to read color palette\n");
        }
    } else if (3 == comp && 4 != fread(masks, 4, 4, file)) { // BI_BITFIELDS
        fclose(file);
        SFR_ERR_RET(NULL, "sfr_load_texture: failed to read bit masks\n");
    }

    // stride and data size
    const i32 stride = ((width * bpp + 31) / 32) * 4;
    const i32 dataSize = stride * height;

    // read pixel data
    u8* pixelData = (u8*)malloc(dataSize);
    if (!pixelData) {
        fclose(file);
        SFR_ERR_RET(NULL, "sfr_load_texture: failed to allocate pixel data\n");
    }

    fseek(file, dataOffset, SEEK_SET);
    if (fread(pixelData, 1, dataSize, file) != dataSize) {
        free(pixelData);
        fclose(file);
        SFR_ERR_RET(NULL, "sfr_load_texture: failed to read pixel data\n");
    }
    fclose(file);

    // create texture
    Texture* tex = (Texture*)malloc(sizeof(Texture));
    if (!tex) {
        free(pixelData);
        SFR_ERR_RET(NULL, "sfr_load_texture: failed to allocate texture struct\n");
    }

    tex->w = width;
    tex->h = height;
    tex->pixels = (u32*)malloc(width * height * sizeof(u32));
    if (!tex->pixels) {
        free(pixelData);
        free(tex);
        SFR_ERR_RET(NULL, "sfr_load_texture: failed to allocate texture pixels\n");
    }

    // process pixels
    for (i32 y = 0, i = 0; y < height; y += 1) {
        const i32 srcY = isTopDown ? y : height - 1 - y;
        const u8* row = pixelData + srcY * stride;
        for (i32 x = 0; x < width; x += 1, i += 1) {
            u32 col = 0;
            switch (bpp) {
                case 32: {
                    const u8* px = row + x * 4;
                    col = (px[3] << 24) | (px[2] << 16) | (px[1] << 8) | px[0];
                } break;
                case 24: {
                    const u8* px = row + x * 3;
                    col |= (px[2] << 16) | (px[1] << 8) | px[0];
                } break;
                case 16: {
                    const u16 px = *((u16*)(row + x * 2));
                    u32 r_mask = masks[0] ? masks[0] : 0x7C00;
                    u32 g_mask = masks[1] ? masks[1] : 0x03E0;
                    u32 b_mask = masks[2] ? masks[2] : 0x001F;
                    
                    col |= ((px & r_mask) * 0xFF / r_mask) << 16;
                    col |= ((px & g_mask) * 0xFF / g_mask) << 8;
                    col |= ((px & b_mask) * 0xFF / b_mask);
                } break;
                case 8:
                case 4:
                case 1: {
                    const u8 index = row[x / (8 / bpp)];
                    const u8 shift = (7 - (x % (8 / bpp))) * bpp;
                    const u8 pal_idx = (index >> shift) & ((1 << bpp) - 1);
                    const u8* entry = (u8*)(palette + pal_idx);
                    col |= (entry[2] << 16) | (entry[1] << 8) | entry[0];
                } break;
                default: {
                    free(pixelData);
                    free(tex->pixels);
                    free(tex);
                    SFR_ERR_RET(NULL, "sfr_load_texture: unsupported bit depth: %d\n", bpp);
                }
            }
            
            tex->pixels[i] = col;
        }
    }

    free(pixelData);

    return tex;
}

SFR_FUNC void sfr_release_texture(Texture** tex) {
    if (!tex || !(*tex)) {
        return;
    }

    if ((*tex)->pixels) {
        free((*tex)->pixels);
        (*tex)->pixels = NULL;
    }

    free(*tex);
    *tex = NULL;
}

SFR_FUNC Font* sfr_load_font(const char* filename) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        SFR_ERR_RET(NULL, "sfr_load_font: failed to open file '%s'\n", filename);
    }

    /* .srft format

    [0..4] header:
    - [0..4] = ['s']['r']['f']['t'] = 1936877172

    [5...] data:
    - {
        vert count, u16, 2 bytes, max 
        vert data,  f32, 4 bytes, [x0][y0][x1][y1][x2][...]
    }
    */

    // validate file as .srft (sofren font type)
    const u32 code = ('s' << 24) | ('r' << 16) | ('f' << 8)  | 't';
    u32 codeCheck;
    if (1 != fread(&codeCheck, 4, 1, file) || code != codeCheck) {
        fclose(file);
        SFR_ERR_RET(NULL, "sfr_load_font: not a valid .srft file ('%s')\n", filename);
    }

    // allocate space for font
    Font* font = (Font*)malloc(sizeof(Font));
    if (!font) {
        fclose(file);
        SFR_ERR_RET(NULL, "sfr_load_font: failed to allocate struct\n");
    }

    // load font
    for (i32 i = 0; i < SFR_FONT_GLYPH_MAX; i += 1) {
        for (i32 j = 0; j < SFR_FONT_VERT_MAX; j += 1) {
            font->verts[i][j] = SFR_FONT_VERT_EMPTY;
        }
        
        u8 vertCount;
        if (1 != fread(&vertCount, 1, 1, file)) {
            fclose(file);
            free(font);
            SFR_ERR_RET(NULL, "sfr_load_font: error reading from file\n");
        }
        if (vertCount >= SFR_FONT_VERT_MAX) {
            fclose(file);
            free(font);
            SFR_ERR_RET(NULL, 
                "sfr_load_font: vert count out of bounds for glyph '%c' (%d) (%d >= %d)\n",
                i, i, vertCount, SFR_FONT_VERT_MAX);
        }

        if (vertCount && vertCount != fread(font->verts[i], 4, vertCount, file)) {
            fclose(file);
            free(font);
            SFR_ERR_RET(NULL, "sfr_load_font: error reading vertices from file\n");
        }
    }
    
    return font;
}

SFR_FUNC void sfr_release_font(Font** font) {
    if (!font || !(*font)) {
        return;
    }

    free(*font);
    *font = NULL;
}

#endif // !SFR_NO_STD

SFR_FUNC void sfr_rand_seed(u32 seed) {
    sfrState.randState = seed;
}

SFR_FUNC u32 sfr_rand_next(void) {
    sfrState.randState += 0xE120FC15;
    u64 temp = (u64)sfrState.randState * 0x4A39B70D;
    u32 m1 = (u32)((temp >> 32) ^ temp);
    temp = (u64)m1 * 0x12FAD5C9;
    u32 m2 = (u32)((temp >> 32) ^ temp);
    return m2;
}

SFR_FUNC i32 sfr_rand_int(i32 min, i32 max) {
    if (min >= max) {
        SFR_ERR_RET(min, "sfr_rand_int: min >= max (%d >= %d)", min, max);
    }
    return (i32)(sfr_rand_next() % (max - min)) + min;
}

SFR_FUNC f32 sfr_rand_flt(f32 min, f32 max) {
    if (min >= max) {
        SFR_ERR_RET(min, "sfr_rand_flt: min >= max (%f >= %f)", min, max);
    }
    return (f32)(min + sfr_rand_next() / (f64)0xFFFFFFFF * ((f64)max - (f64)min));
}

#endif // SFR_IMPL

#ifdef SFR_PREFIXED_TYPES
    #undef Vec
    #undef Mat
    #undef Tri
    #undef Mesh
    #undef Texture
    #undef Font

    #undef i8
    #undef u8
    #undef i16
    #undef u16
    #undef i32
    #undef u32
    #undef i64
    #undef u64
    #undef f32
    #undef f64
#endif

#ifdef __cplusplus
}
#endif

#endif // SFR_H
