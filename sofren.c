#ifndef SFR_H
#define SFR_H

#ifdef __cplusplus
extern "C" {
#endif

//================================================
//:         PUBLIC API
//================================================

#ifndef SFR_MAX_WIDTH
    #define SFR_MAX_WIDTH 1920
#endif
#ifndef SFR_MAX_HEIGHT
    #define SFR_MAX_HEIGHT 1080
#endif

//: threading config
#ifdef SFR_THREAD_COUNT
    #if SFR_THREAD_COUNT < 1 || SFR_THREAD_COUNT > 32
        #error "SFR_ERROR: SFR_THREAD_COUNT must be between 1 and 32 (32 arbitrary)"
    #endif
#else
    #define SFR_THREAD_COUNT 1
#endif

#if SFR_THREAD_COUNT > 1
    #define SFR_MULTITHREADED
    #ifndef SFR_TILE_WIDTH
        #define SFR_TILE_WIDTH 64
    #endif
    #ifndef SFR_TILE_HEIGHT
        #define SFR_TILE_HEIGHT 64
    #endif
    #ifndef SFR_MAX_BINS_PER_TILE
        // max triangles binned to a single tile
        #define SFR_MAX_BINS_PER_TILE (4096)
    #endif
    #ifndef SFR_MAX_BINNED_TRIS
        // max triangles binned per frame
        #define SFR_MAX_BINNED_TRIS (1024 * 256)
    #endif
#endif

#ifdef SFR_MULTITHREADED
    #ifdef _WIN32
        #ifndef _WIN32_WINNT
            // for CreateSemaphoreEx
            #define _WIN32_WINNT 0x0600
        #endif
        #define NOGDI
        #define NOUSER
        #define NOMINMAX
        #define WIN32_LEAN_AND_MEAN
        #include <windows.h>
        #ifdef near
            #undef near
        #endif
        #ifdef far
            #undef far
        #endif
        #include <process.h> // for _beginthreadex
        typedef HANDLE SfrThread;
        typedef HANDLE SfrSemaphore;
        typedef CRITICAL_SECTION SfrMutex;
        typedef volatile LONG SfrAtomic32;
        typedef volatile LONG64 SfrAtomic64;
    #else
        #include <pthread.h>
        #include <semaphore.h>
        #include <sched.h> // for sched_yield
        #include <time.h> // for timespec
        #include <errno.h>
        typedef pthread_t SfrThread;
        typedef sem_t SfrSemaphore;
        typedef pthread_mutex_t SfrMutex;
        typedef volatile int SfrAtomic32;
        typedef volatile long long SfrAtomic64;
    #endif
#else
    #define SfrAtomic32 i32
    #define SfrAtomic64 i64
#endif

//: types
#ifdef SFR_PREFIXED_TYPES
    #define Vec     sfrvec_t
    #define Mat     sfrmat_t
    #define Tri     sfrtri_t
    #define Mesh    sfrmesh_t
    #define Texture sfrtexture_t
    #define Font    sfrfont_t

    #define Particle       sfrparticle_t;
    #define ParticleSystem sfrparticles_t;
    
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

typedef struct sfrvec  Vec;
typedef struct sfrmat  Mat;
typedef struct sfrmesh Mesh;
typedef struct sfrtex  Texture;
typedef struct sfrfont Font;

typedef struct sfrparticle  Particle;
typedef struct sfrparticles ParticleSystem;

typedef struct sfrBuffers SfrBuffers;

typedef struct sfrTriangleBin SfrTriangleBin;
typedef struct sfrTile SfrTile;
#ifdef SFR_MULTITHREADED
    typedef struct sfrThreadData SfrThreadData;
#endif

//: extern variables
extern i32 sfrWidth, sfrHeight;
extern SfrBuffers* sfrBuffers;

// global variables below can be managed by you, however
// there is probably a function that will do what you want

extern SfrAtomic32 sfrRasterCount; // how many triangles have been rasterized since the last call to clear

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
SFR_FUNC Vec sfr_vec_normf(f32 x, f32 y, f32 z);
SFR_FUNC Vec sfr_vec_face_normal(Vec a, Vec b, Vec c);
SFR_FUNC Mat sfr_mat_identity();
SFR_FUNC Mat sfr_mat_rot_x(f32 a);
SFR_FUNC Mat sfr_mat_rot_y(f32 a);
SFR_FUNC Mat sfr_mat_rot_z(f32 a);
SFR_FUNC Mat sfr_mat_translate(f32 x, f32 y, f32 z);
SFR_FUNC Mat sfr_mat_scale(f32 x, f32 y, f32 z);
SFR_FUNC Mat sfr_mat_proj(f32 fovDeg, f32 aspect, f32 near, f32 far);
SFR_FUNC Mat sfr_mat_mul(Mat a, Mat b);
SFR_FUNC Vec sfr_mat_mul_vec(Mat m, Vec v);
SFR_FUNC Mat sfr_mat_qinv(Mat m);
SFR_FUNC Mat sfr_mat_look_at(Vec pos, Vec target, Vec up);

//: core functions
SFR_FUNC void sfr_init(SfrBuffers* buffers, i32 w, i32 h, f32 fovDeg);

#ifdef SFR_MULTITHREADED
    SFR_FUNC void sfr_shutdown(void);
    SFR_FUNC void sfr_flush_and_wait(void);
#endif

#ifndef SFR_NO_ALPHA
    SFR_FUNC void sfr_present_alpha(void); // draw transparent parts of scene last
#endif

SFR_FUNC void sfr_resize(i32 width, i32 height);

SFR_FUNC void sfr_reset(void);                    // reset model matrix to identity and lighting to {0}
SFR_FUNC void sfr_rotate_x(f32 theta);            // rotate model matrix about x by theta radians
SFR_FUNC void sfr_rotate_y(f32 theta);            // rotate model matrix about y by theta radians
SFR_FUNC void sfr_rotate_z(f32 theta);            // rotate model matrix about z by theta radians
SFR_FUNC void sfr_translate(f32 x, f32 y, f32 z); // translate model matrix by x y z
SFR_FUNC void sfr_scale(f32 x, f32 y, f32 z);     // scale model matrix by x y z
SFR_FUNC void sfr_look_at(f32 x, f32 y, f32 z);   // set view matrix to look at x y z

// clear all buffers to default states
SFR_FUNC void sfr_clear(u32 clearCol);
SFR_FUNC void sfr_clear_depth(void);

// triangle drawing functions
SFR_FUNC void sfr_triangle(
    f32 ax, f32 ay, f32 az,
    f32 bx, f32 by, f32 bz,
    f32 cx, f32 cy, f32 cz,
    u32 col);
SFR_FUNC void sfr_triangle_tex(
    f32 ax, f32 ay, f32 az, f32 au, f32 av,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv,
    u32 col, const Texture* tex);

// other drawing functions, if tex is null sfrState.baseTex (white 1x1 texture) will be used
SFR_FUNC void sfr_billboard(u32 col, const Texture* tex);
SFR_FUNC void sfr_cube(u32 col, const Texture* tex);
SFR_FUNC void sfr_cube_ex(u32 col[12]);
SFR_FUNC void sfr_mesh(const Mesh* mesh, u32 col, const Texture* tex);
SFR_FUNC void sfr_string(const Font* font, const char* s, i32 sLength, u32 col); // not yet implemented
SFR_FUNC void sfr_glyph(const Font* font, u16 id, u32 col); // draw a single character

// project the world position specified to screen coordinates
SFR_FUNC i32 sfr_world_to_screen(f32 x, f32 y, f32 z, i32* screenX, i32* screenY);    

// update the camera with the new position and view
SFR_FUNC void sfr_set_camera(f32 x, f32 y, f32 z, f32 yaw, f32 pitch, f32 roll);
SFR_FUNC void sfr_set_fov(f32 fovDeg); // update projection matrix with new fov
SFR_FUNC void sfr_set_lighting( // update internal lighting state for simple shading on triangles
    i32 on, Vec dir, f32 ambientIntensity);

// things requiring malloc / stdio
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

// all particle related functions
SFR_FUNC ParticleSystem sfr_particles_create(Particle* buffer, i32 count, const Texture* tex);
SFR_FUNC void sfr_particles_update(ParticleSystem* sys, f32 frameTime);
SFR_FUNC void sfr_particles_draw(const ParticleSystem* sys);
SFR_FUNC void sfr_particles_emit(ParticleSystem* sys,
    f32 px, f32 py, f32 pz,
    f32 vx, f32 vy, f32 vz,
    f32 ax, f32 ay, f32 az,
    f32 startSize, f32 endSize,
    u32 startCol, u32 endCol,
    f32 lifetime);

SFR_FUNC void sfr_rand_seed(u32 seed);       // seed random number generator
SFR_FUNC u32 sfr_rand_next(void);            // Lehmer random number generator
SFR_FUNC i32 sfr_rand_int(i32 min, i32 max); // random int in range [min, max]
SFR_FUNC f32 sfr_rand_flt(f32 min, f32 max); // random f32 in range [min, max]

// threading related functions
#ifdef SFR_MULTITHREADED
    #ifdef _WIN32
        static unsigned __stdcall sfr__worker_thread_func(void* arg);
    #else
        static void* sfr__worker_thread_func(void* arg);
    #endif

    SFR_FUNC i32 sfr_atomic_add(SfrAtomic32* a, i32 val);
    SFR_FUNC i32 sfr_atomic_get(SfrAtomic32* a);
    SFR_FUNC void sfr_atomic_set(SfrAtomic32* a, i32 val);
    SFR_FUNC i32 sfr_atomic_cas(SfrAtomic32* ptr, i32 oldVal, i32 newVal);
    SFR_FUNC i64 sfr_atomic_add64(SfrAtomic64* a, i64 val);
    SFR_FUNC i64 sfr_atomic_get64(SfrAtomic64* a);
    SFR_FUNC void sfr_atomic_set64(SfrAtomic64* a, i64 val);
    SFR_FUNC i64 sfr_atomic_cas64(SfrAtomic64* ptr, i64 oldVal, i64 newVal);

    SFR_FUNC void sfr_mutex_init(SfrMutex* m);
    SFR_FUNC void sfr_mutex_destroy(SfrMutex* m);
    SFR_FUNC void sfr_mutex_lock(SfrMutex* m);
    SFR_FUNC void sfr_mutex_unlock(SfrMutex* m);

    SFR_FUNC i32 sfr_semaphore_init(SfrSemaphore* s, i32 initialCount);
    SFR_FUNC void sfr_semaphore_destroy(SfrSemaphore* s);
    SFR_FUNC void sfr_semaphore_wait(SfrSemaphore* s);
    SFR_FUNC void sfr_semaphore_post(SfrSemaphore* s, i32 n);
#endif


//================================================
//:         IMPLEMENTATION
//================================================

#define SFR_IMPL
#ifdef SFR_IMPL

#ifndef SFR_NO_STRING
    #include <string.h>
    #define sfr_memset memset
    #define sfr_memmove memmove
    #define sfr_strtok strtok
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

    SFR_FUNC char* sfr_strtok(char* str, const char* delim) {
        static char* next = 0; // persistent pointer for subsequent calls
        if (str) {
            next = str; // initialize on first call
        }

        if (!next || !*next) {
            return 0; // no tokens left
        }

        // skip leading delimiters
        while (*next) {
            const char* d = delim;
            u8 found = 0;
            while (*d) {
                if (*next == *d++) {
                    found = 1;
                    break;
                }
            }
            if (!found) {
                break;
            }
            next++;
        }

        char* token = next; // start of current token
        if (!*next) {
            return next = 0; // end of string
        }

        // find token end
        while (*next) {
            const char* d = delim;
            u8 found = 0;
            while (*d) {
                if (*next ==* d) {
                    found = 1;
                    break;
                }
                d++;
            }
            if (found) {
                break;
            }
            next++;
        }

        // terminate token and update next
        if (*next) {
            *next++ = '\0'; // split and move past delimiter
        } else {
            next = 0; // last token, reset next
        }
        return token;
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

typedef struct sfrmesh {
    f32* tris;     // vertex positions
    f32* uvs;      // uv coordinates
    i32 vertCount; // total number of floats (3 per vertex)
} Mesh;

typedef struct sfrtex {
    u32* pixels;
    i32 w, h;
} Texture;

typedef struct sfrfont {
    // xy pairs [x0, y0, x1, y1, x2, ...]
    f32 verts[SFR_FONT_GLYPH_MAX][SFR_FONT_VERT_MAX];
} Font;

typedef struct sfrparticle {
    f32 px, py, pz; // position
    f32 vx, vy, vz; // velocity
    f32 ax, ay, az; // acceleration
    f32 startSize, endSize;
    u32 startCol, endCol;
    f32 lifetime, age;
} Particle;

typedef struct sfrparticles {
    Particle* particles;
    i32 total, active;
    const Texture* tex;
} ParticleSystem;

typedef struct sfrTriangleBin {
    f32 ax, ay, az, aInvZ, auoz, avoz;
    f32 bx, by, bz, bInvZ, buoz, bvoz;
    f32 cx, cy, cz, cInvZ, cuoz, cvoz;
    u32 col;
    const Texture* tex;
} SfrTriangleBin;

typedef struct sfrTile {
    i32 minX, minY, maxX, maxY;
    #ifdef SFR_MULTITHREADED
        SfrTriangleBin* bins[SFR_MAX_BINS_PER_TILE];
        SfrAtomic32 binCount;
    #endif
} SfrTile;

#ifdef SFR_MULTITHREADED
    typedef struct sfrThreadData {
        SfrThread handle;
        i32 threadInd;
    } SfrThreadData;
#endif

typedef struct sfrBuffers {
    u32 pixel[SFR_MAX_WIDTH * SFR_MAX_HEIGHT];
    f32 depth[SFR_MAX_WIDTH * SFR_MAX_HEIGHT];
    #ifndef SFR_NO_ALPHA
        struct sfrAccumCol {
            u16 a, r, g, b;
            f32 depth;
        } accum[SFR_MAX_WIDTH * SFR_MAX_HEIGHT];
    #endif
    #ifdef SFR_MULTITHREADED
        // tiling system data
        SfrTile tiles[
            ((SFR_MAX_HEIGHT + SFR_TILE_WIDTH - 1) / SFR_TILE_WIDTH) *
            ((SFR_MAX_WIDTH + SFR_TILE_WIDTH - 1) / SFR_TILE_WIDTH)];
        i32 tileCols, tileRows, tileCount;
        SfrTriangleBin triangleBinPool[SFR_MAX_BINNED_TRIS];
        SfrAtomic32 triangleBinAllocator;
        
        // work dispatch data
        i32 workQueue[(SFR_MAX_WIDTH / SFR_TILE_WIDTH + 1) * (SFR_MAX_HEIGHT / SFR_TILE_HEIGHT + 1)];
        SfrAtomic32 workQueueCount;
        SfrAtomic32 workQueueHead;

        // thread management data
        struct sfrThreadData threads[SFR_THREAD_COUNT];
        SfrSemaphore workStartSem; // wakes threads to start a batch of work
        SfrSemaphore workDoneSem;  // signaled by threads when they finish a batch
    #endif
} SfrBuffers;

typedef struct sfrState {
    i32 lightingEnabled;
    Vec lightingDir;
    f32 lightingAmbient;

    Mat matNormal;
    i32 normalMatDirty;

    u32 randState;

    f32 halfWidth, halfHeight;

    Vec clipPlanes[4][2];

    u32 baseTexPixels[1];
    Texture baseTex;

    #ifdef SFR_MULTITHREADED
        i32 shutdown;
    #endif
} SfrState;

// helper to track vertex attributes during clipping of textured triangles
typedef struct sfrTexVert {
    Vec pos;   // position in view space
    f32 u, v;  // texture coords
    f32 viewZ; // z in view space for perspective correction
} SfrTexVert;


//================================================
//:         GLOBAL VARIABLES
//================================================

i32 sfrWidth, sfrHeight;
SfrBuffers* sfrBuffers;
SfrAtomic32 sfrRasterCount;

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

#define SFR_SWAPF(_a, _b) { const f32 _swapTemp = (_a); (_a) = (_b); (_b) = _swapTemp; }
#define SFR_LERPF(_a, _b, _t) ((_a) + (_t) * ((_b) - (_a)))
#define SFR_VEC0 ((Vec){0.f, 0.f, 0.f, 0.f})

#define SFR_DIV255(_r, _a, _b) \
    const u32 _##_r##Temp = (u32)(_a) * (u32)(_b) + 128; \
    const u8 _r = (_##_r##Temp + (_##_r##Temp >> 8)) >> 8;

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
    #define SFR_ERR_EXIT(...) { *(int*)0 = 0; } // crash the program to exit
    #define SFR_ERR_RET(_r, ...) return _r
#endif


//================================================
//:         MATH
//================================================

SFR_FUNC f32 sfr_lerp(f32 a, f32 b, f32 t) {
    return a + t * (b - a);
}

SFR_FUNC u32 sfr_lerp_col(u32 c1, u32 c2, f32 t) {
    const u8 a1 = (c1 >> 24) & 0xFF;
    const u8 r1 = (c1 >> 16) & 0xFF;
    const u8 g1 = (c1 >> 8)  & 0xFF;
    const u8 b1 = (c1 >> 0)  & 0xFF;
    
    const u8 a2 = (c2 >> 24) & 0xFF;
    const u8 r2 = (c2 >> 16) & 0xFF;
    const u8 g2 = (c2 >> 8)  & 0xFF;
    const u8 b2 = (c2 >> 0)  & 0xFF;
    
    const u8 a = (u8)(a1 + (a2 - a1) * t);
    const u8 r = (u8)(r1 + (r2 - r1) * t);
    const u8 g = (u8)(g1 + (g2 - g1) * t);
    const u8 b = (u8)(b1 + (b2 - b1) * t);
    
    return (a << 24) | (r << 16) | (g << 8) | b;
}

#ifndef SFR_NO_MATH
    #include <math.h>
    #define sfr_floorf floorf
    #define sfr_fmaxf fmaxf
    #define sfr_fabsf fabsf
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

    SFR_FUNC f32 sfr_fabsf(f32 x) {
        return x < 0.f ? -x : x;
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
    Mat r;
    for (i32 i = 0; i < 4; i += 1) {
        const f32 a0 = a.m[i][0], a1 = a.m[i][1], a2 = a.m[i][2], a3 = a.m[i][3];
        r.m[i][0] = a0 * b.m[0][0] + a1 * b.m[1][0] + a2 * b.m[2][0] + a3 * b.m[3][0];
        r.m[i][1] = a0 * b.m[0][1] + a1 * b.m[1][1] + a2 * b.m[2][1] + a3 * b.m[3][1];
        r.m[i][2] = a0 * b.m[0][2] + a1 * b.m[1][2] + a2 * b.m[2][2] + a3 * b.m[3][2];
        r.m[i][3] = a0 * b.m[0][3] + a1 * b.m[1][3] + a2 * b.m[2][3] + a3 * b.m[3][3];
    }
    return r;
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

SFR_FUNC i32 sfr_clip_tri_homogeneous(SfrTexVert out[2][3], Vec plane, const SfrTexVert in[3]) {
    SfrTexVert inside[3], outside[3];
    f32 insideDists[3], outsideDists[3];
    i32 insideCount = 0, outsideCount = 0;

    // precompute distances during classification
    for (i32 i = 0; i < 3; i += 1) {
        const Vec v = in[i].pos;
        const f32 dist = plane.x * v.x + plane.y * v.y + plane.z * v.z + plane.w * v.w;
        if (dist >= 0.f) {
            inside[insideCount] = in[i];
            insideDists[insideCount] = dist;
            insideCount += 1;
        } else {
            outside[outsideCount] = in[i];
            outsideDists[outsideCount] = dist;
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
        const SfrTexVert a = inside[0];
        const SfrTexVert b = outside[0];
        const SfrTexVert c = outside[1];

        // use precomputed distances
        const f32 dA = insideDists[0];
        const f32 dB = outsideDists[0];
        const f32 dC = outsideDists[1];
        
        const f32 tAB = dA / (dA - dB);
        const f32 tAC = dA / (dA - dC);

        // interpolate vertex b
        SfrTexVert newB = {
            .pos = {
                .x = a.pos.x + tAB * (b.pos.x - a.pos.x),
                .y = a.pos.y + tAB * (b.pos.y - a.pos.y),
                .z = a.pos.z + tAB * (b.pos.z - a.pos.z),
                .w = a.pos.w + tAB * (b.pos.w - a.pos.w)
            },
            .u = a.u + tAB * (b.u - a.u),
            .v = a.v + tAB * (b.v - a.v),
            .viewZ = a.viewZ + tAB * (b.viewZ - a.viewZ)
        };
        
        // interpolate vertex c
        SfrTexVert newC = {
            .pos = {
                .x = a.pos.x + tAC * (c.pos.x - a.pos.x),
                .y = a.pos.y + tAC * (c.pos.y - a.pos.y),
                .z = a.pos.z + tAC * (c.pos.z - a.pos.z),
                .w = a.pos.w + tAC * (c.pos.w - a.pos.w)
            },
            .u = a.u + tAC * (c.u - a.u),
            .v = a.v + tAC * (c.v - a.v),
            .viewZ = a.viewZ + tAC * (c.viewZ - a.viewZ)
        };

        out[0][0] = a;
        out[0][1] = newB;
        out[0][2] = newC;
        return 1;
    }

    if (2 == insideCount && 1 == outsideCount) {
        const SfrTexVert a = inside[0];
        const SfrTexVert b = inside[1];
        const SfrTexVert c = outside[0];

        // use precomputed distances
        const f32 dA = insideDists[0];
        const f32 dB = insideDists[1];
        const f32 dC = outsideDists[0];
        
        const f32 tAC = dA / (dA - dC);
        const f32 tBC = dB / (dB - dC);

        // interpolate vertices
        SfrTexVert newAC = {
            .pos = {
                .x = a.pos.x + tAC * (c.pos.x - a.pos.x),
                .y = a.pos.y + tAC * (c.pos.y - a.pos.y),
                .z = a.pos.z + tAC * (c.pos.z - a.pos.z),
                .w = a.pos.w + tAC * (c.pos.w - a.pos.w)
            },
            .u = a.u + tAC * (c.u - a.u),
            .v = a.v + tAC * (c.v - a.v),
            .viewZ = a.viewZ + tAC * (c.viewZ - a.viewZ)
        };

        SfrTexVert newBC = {
            .pos = {
                .x = b.pos.x + tBC * (c.pos.x - b.pos.x),
                .y = b.pos.y + tBC * (c.pos.y - b.pos.y),
                .z = b.pos.z + tBC * (c.pos.z - b.pos.z),
                .w = b.pos.w + tBC * (c.pos.w - b.pos.w)
            },
            .u = b.u + tBC * (c.u - b.u),
            .v = b.v + tBC * (c.v - b.v),
            .viewZ = b.viewZ + tBC * (c.viewZ - b.viewZ)
        };

        out[0][0] = a;
        out[0][1] = b;
        out[0][2] = newAC;
        
        out[1][0] = b;
        out[1][1] = newAC;
        out[1][2] = newBC;
        
        return 2;
    }

    return 0;
}

SFR_FUNC void sfr_rasterize_bin(const SfrTriangleBin* bin, const SfrTile* tile) {
    // skip if fully transparent
    const u32 col = bin->col;
    #ifndef SFR_NO_ALPHA
        const u8 ca = (col >> 24) & 0xFF;
        if (0 == ca) {
            return;
        }
    #endif
    const u8 cr = (col >> 16) & 0xFF;
    const u8 cg = (col >> 8)  & 0xFF;
    const u8 cb = (col >> 0)  & 0xFF;

    f32 ax = (f32)((i32)bin->ax), ay = (f32)((i32)bin->ay), az = bin->az;
    f32 bx = (f32)((i32)bin->bx), by = (f32)((i32)bin->by), bz = bin->bz;
    f32 cx = (f32)((i32)bin->cx), cy = (f32)((i32)bin->cy), cz = bin->cz;
    
    f32 aInvZ = bin->aInvZ, auoz = bin->auoz, avoz = bin->avoz;
    f32 bInvZ = bin->bInvZ, buoz = bin->buoz, bvoz = bin->bvoz;
    f32 cInvZ = bin->cInvZ, cuoz = bin->cuoz, cvoz = bin->cvoz;

    const Texture* tex = bin->tex;
    
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

    const i32 texW = tex->w, texH = tex->h;
    const i32 texW1 = tex->w - 1, texH1 = tex->h - 1;

    // edge deltas for triangle
    const f32 deltaACX = cx - ax;
    const f32 deltaACZ = cz - az;
    const f32 deltaACinvZ = cInvZ - aInvZ;
    const f32 deltaACuoz = cuoz - auoz;
    const f32 deltaACvoz = cvoz - avoz;
    const f32 invHeightAC = (cy != ay) ? 1.f / (cy - ay) : 0.f;

    { // lower triangle (A to B)
        const f32 deltaABX = bx - ax;
        const f32 deltaABZ = bz - az;
        const f32 deltaABinvZ = bInvZ - aInvZ;
        const f32 deltaABuoz = buoz - auoz;
        const f32 deltaABvoz = bvoz - avoz;
        const f32 invHeightAB = (by != ay) ? 1.f / (by - ay) : 0.f;

        const i32 yStart = SFR_MAX(tile->minY, ay);
        const i32 yEnd   = SFR_MIN(tile->maxY, by);
        for (i32 y = yStart; y < yEnd; y += 1) {
            const f32 dy = y - ay;
            const f32 alpha = dy * invHeightAC;
            const f32 beta = dy * invHeightAB;
    
            // interpolate along AC edge (start)
            f32 sx = ax + deltaACX * alpha;
            f32 sz = az + deltaACZ * alpha; // NDC z for depth
            f32 sInvZ = aInvZ + deltaACinvZ * alpha;
            f32 su = auoz + deltaACuoz * alpha;
            f32 sv = avoz + deltaACvoz * alpha;
    
            // interpolate along AB edge (end)
            f32 ex = ax + deltaABX * beta;
            f32 ez = az + deltaABZ * beta; // NDC z for depth
            f32 eInvZ = aInvZ + deltaABinvZ * beta;
            f32 eu = auoz + deltaABuoz * beta;
            f32 ev = avoz + deltaABvoz * beta;
    
            // swap start/end if needed
            if (sx > ex) {
                SFR_SWAPF(sx, ex); SFR_SWAPF(sz, ez);
                SFR_SWAPF(sInvZ, eInvZ); SFR_SWAPF(su, eu); SFR_SWAPF(sv, ev);
            }
    
            // clamp x coordinates
            const i32 xStart = SFR_MAX(tile->minX, sx);
            const i32 xEnd   = SFR_MIN(tile->maxX, ex);
            if (xStart >= xEnd) {
                continue;
            }
    
            // scanline interpolation parameters
            const f32 dx = ex - sx;
            const f32 tStep = (0.f != dx) ? 1.f / dx : 0.f;
            const f32 depthStep = (ez - sz) * tStep; // NDC z step
            const f32 invZStep = (eInvZ - sInvZ) * tStep;
            const f32 uStep = (eu - su) * tStep;
            const f32 vStep = (ev - sv) * tStep;
    
            // initialize current values
            f32 depth = sz + (xStart - sx) * depthStep; // NDC z for depth testing
            f32 invZ = sInvZ + (xStart - sx) * invZStep;
            f32 uoz = su + (xStart - sx) * uStep;
            f32 voz = sv + (xStart - sx) * vStep;
    
            // rasterize scanline
            for (i32 x = xStart, i = y * sfrWidth + xStart; x < xEnd; x += 1, i += 1,
                invZ += invZStep, uoz += uStep, voz += vStep, depth += depthStep
            ) {
                if (depth > sfrBuffers->depth[i]) {
                    continue;
                }

                // perspective correction for texture
                const f32 zView = 1.f / invZ;

                // recover texture coords
                const f32 u = uoz * zView;
                const f32 v = voz * zView;
                
                // wrap texture coords and clamp
                i32 tx = (i32)(u * texW1);
                i32 ty = (i32)(v * texH1);
                tx = (tx < 0) ? 0 : ((tx >= texW) ? texW1 : tx);
                ty = (ty < 0) ? 0 : ((ty >= texH) ? texH1 : ty);
                
                const u32 texCol = tex->pixels[ty * texW + tx];
                const u8 tr = (texCol >> 16) & 0xFF;
                const u8 tg = (texCol >> 8)  & 0xFF;
                const u8 tb = (texCol >> 0)  & 0xFF;
                SFR_DIV255(fr, tr, cr);
                SFR_DIV255(fg, tg, cg);
                SFR_DIV255(fb, tb, cb);

                #ifdef SFR_NO_ALPHA
                    sfrBuffers->pixel[i] = (fr << 16) | (fg << 8) | fb;
                    sfrBuffers->depth[i] = depth;
                #else
                    const u8 ta = (texCol >> 24) & 0xFF;
                    SFR_DIV255(fa, ta, ca);
                    if (0xFF != fa) {
                        const u8 currAlpha = sfrBuffers->accum[i].a;
                        SFR_DIV255(contribution, 255 - currAlpha, fa);

                        // accumulate premultiplied color components
                        SFR_DIV255(accumR, fr, contribution);
                        SFR_DIV255(accumG, fg, contribution);
                        SFR_DIV255(accumB, fb, contribution);
                        sfrBuffers->accum[i].r += accumR;
                        sfrBuffers->accum[i].g += accumG;
                        sfrBuffers->accum[i].b += accumB;

                        // update accumulated alpha
                        sfrBuffers->accum[i].a = currAlpha + contribution;
                        if (depth < sfrBuffers->accum[i].depth) {
                            sfrBuffers->accum[i].depth = depth;
                        }
                    } else {
                        sfrBuffers->pixel[i] = (fr << 16) | (fg << 8) | fb;
                        sfrBuffers->depth[i] = depth;
                    }
                #endif
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
    
        const i32 yStart = SFR_MAX(tile->minY, by);
        const i32 yEnd   = SFR_MIN(tile->maxY, cy);
        for (i32 y = yStart; y < yEnd; y += 1) {
            const f32 alpha = (f32)(y - ay) * invHeightAC;
            const f32 beta = (f32)(y - by) * invHeightBC;
    
            // interpolate along BC edge (start)
            f32 sx = bx + deltaBCX * beta;
            f32 sz = bz + deltaBCZ * beta; // NDC z for depth
            f32 sInvZ = bInvZ + deltaBCinvZ * beta;
            f32 suoz = buoz + deltaBCuoz * beta;
            f32 svoz = bvoz + deltaBCvoz * beta;
    
            // interpolate along AC edge (end)
            f32 ex = ax + deltaACX * alpha;
            f32 ez = az + deltaACZ * alpha; // NDC z for depth
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
            const i32 xStart = SFR_MAX(tile->minX, sx);
            const i32 xEnd   = SFR_MIN(tile->maxX, ex);
            if (xStart >= xEnd) {
                continue;
            }
    
            // scanline interpolation parameters
            const f32 dx = ex - sx;
            const f32 tStep = (0.f != dx) ? 1.f / dx : 0.f;
            const f32 depthStep = (ez - sz) * tStep; // NDC z step
            const f32 invZStep = (eInvZ - sInvZ) * tStep;
            const f32 uStep = (euoz - suoz) * tStep;
            const f32 vStep = (evoz - svoz) * tStep;
    
            // initialize current values
            f32 depth = sz + (xStart - sx) * depthStep; // NDC z for depth testing
            f32 invZ = sInvZ + (xStart - sx) * invZStep;
            f32 uoz = suoz + (xStart - sx) * uStep;
            f32 voz = svoz + (xStart - sx) * vStep;
    
            // rasterize scanline
            for (i32 x = xStart, i = y * sfrWidth + xStart; x < xEnd; x += 1, i += 1,
                invZ += invZStep, uoz += uStep, voz += vStep, depth += depthStep
            ) {
                if (depth > sfrBuffers->depth[i]) {
                    continue;
                }

                // perspective correction for texture
                const f32 zView = 1.f / invZ;

                // recover texture coords
                const f32 u = uoz * zView;
                const f32 v = voz * zView;
                
                // wrap texture coords and clamp
                i32 tx = (i32)(u * texW1);
                i32 ty = (i32)(v * texH1);
                tx = (tx < 0) ? 0 : ((tx >= texW) ? texW1 : tx);
                ty = (ty < 0) ? 0 : ((ty >= texH) ? texH1 : ty);
                
                const u32 texCol = tex->pixels[ty * texW + tx];
                const u8 tr = (texCol >> 16) & 0xFF;
                const u8 tg = (texCol >> 8)  & 0xFF;
                const u8 tb = (texCol >> 0)  & 0xFF;
                SFR_DIV255(fr, tr, cr);
                SFR_DIV255(fg, tg, cg);
                SFR_DIV255(fb, tb, cb);

                #ifdef SFR_NO_ALPHA
                    sfrBuffers->pixel[i] = (fr << 16) | (fg << 8) | fb;
                    sfrBuffers->depth[i] = depth;
                #else
                    const u8 ta = (texCol >> 24) & 0xFF;
                    SFR_DIV255(fa, ta, ca);
                    if (0xFF != fa) {
                        const u8 currAlpha = sfrBuffers->accum[i].a;
                        SFR_DIV255(contribution, 255 - currAlpha, fa);

                        // accumulate premultiplied color components
                        SFR_DIV255(accumR, fr, contribution);
                        SFR_DIV255(accumG, fg, contribution);
                        SFR_DIV255(accumB, fb, contribution);
                        sfrBuffers->accum[i].r += accumR;
                        sfrBuffers->accum[i].g += accumG;
                        sfrBuffers->accum[i].b += accumB;

                        // update accumulated alpha
                        sfrBuffers->accum[i].a = currAlpha + contribution;
                        if (depth < sfrBuffers->accum[i].depth) {
                            sfrBuffers->accum[i].depth = depth;
                        }
                    } else {
                        sfrBuffers->pixel[i] = (fr << 16) | (fg << 8) | fb;
                        sfrBuffers->depth[i] = depth;
                    }
                #endif
            }
        }
    }
}

SFR_FUNC void sfr__bin_triangle(
    f32 ax, f32 ay, f32 az, f32 aInvZ, f32 auoz, f32 avoz,
    f32 bx, f32 by, f32 bz, f32 bInvZ, f32 buoz, f32 bvoz,
    f32 cx, f32 cy, f32 cz, f32 cInvZ, f32 cuoz, f32 cvoz,
    u32 col, const Texture* tex
) {
    #ifdef SFR_MULTITHREADED
        // get a new triangle bin from the global pool
        const i32 binInd = sfr_atomic_add(&sfrBuffers->triangleBinAllocator, 1) - 1;
        if (binInd >= SFR_MAX_BINNED_TRIS) {
            return;
        }

        SfrTriangleBin* bin = &sfrBuffers->triangleBinPool[binInd];
        *bin = (SfrTriangleBin){
            ax, ay, az, aInvZ, auoz, avoz,
            bx, by, bz, bInvZ, buoz, bvoz,
            cx, cy, cz, cInvZ, cuoz, cvoz,
            col, tex
        };
        sfr_atomic_add(&sfrRasterCount, 1);

        // calculate screen space bounding box of the triangle
        const f32 minX = SFR_MAX(0.f,             SFR_MIN(ax, SFR_MIN(bx, cx)));
        const f32 maxX = SFR_MIN(sfrWidth - 1.f,  SFR_MAX(ax, SFR_MAX(bx, cx)));
        const f32 minY = SFR_MAX(0.f,             SFR_MIN(ay, SFR_MIN(by, cy)));
        const f32 maxY = SFR_MIN(sfrHeight - 1.f, SFR_MAX(ay, SFR_MAX(by, cy)));

        // determine which tiles the triangle overlaps
        const i32 xStart = (i32)minX / SFR_TILE_WIDTH;
        const i32 xEnd   = (i32)maxX / SFR_TILE_WIDTH;
        const i32 yStart = (i32)minY / SFR_TILE_HEIGHT;
        const i32 yEnd   = (i32)maxY / SFR_TILE_HEIGHT;

        // add the bin to all overlapped tiles
        for (i32 ty = yStart; ty <= yEnd; ty += 1) {
            for (i32 tx = xStart; tx <= xEnd; tx += 1) {
                const i32 tileInd = ty * sfrBuffers->tileCols + tx;
                SfrTile* tile = &sfrBuffers->tiles[tileInd];
                
                const i32 tileBinInd = sfr_atomic_add(&tile->binCount, 1) - 1;
                if (tileBinInd >= SFR_MAX_BINS_PER_TILE) {
                    continue;
                }

                if (tileBinInd == 0) {
                    const i32 workInd = sfr_atomic_add(&sfrBuffers->workQueueCount, 1) - 1;
                    if (workInd < SFR_ARRLEN(sfrBuffers->workQueue)) {
                        sfrBuffers->workQueue[workInd] = tileInd;
                    }
                }
                tile->bins[tileBinInd] = bin;
            }
        }
    #else
        sfrRasterCount += 1;
        const SfrTile fullTile = {0, 0, sfrWidth, sfrHeight};
        const SfrTriangleBin bin = {
            ax, ay, az, aInvZ, auoz, avoz,
            bx, by, bz, bInvZ, buoz, bvoz,
            cx, cy, cz, cInvZ, cuoz, cvoz,
            col, tex
        };
        sfr_rasterize_bin(&bin, &fullTile);
    #endif
}

// helper for when lighting is enabled
SFR_FUNC i32 sfr_adjust_color_u32(u32 col, f32 intensity) {
    const u8 r = (col >> 16) & 0xFF;
    const u8 g = (col >> 8)  & 0xFF;
    const u8 b = (col >> 0)  & 0xFF;
    return (col & 0xFF000000)          | 
           ((u8)(r * intensity) << 16) |
           ((u8)(g * intensity) << 8)  |
           ((u8)(b * intensity) << 0);
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

SFR_FUNC void sfr_init(SfrBuffers* buffers, i32 w, i32 h, f32 fovDeg) {    
    sfrBuffers = buffers;

    #ifdef SFR_MULTITHREADED
        sfrState.shutdown = 0;
        sfr_atomic_set(&sfrBuffers->triangleBinAllocator, 0);
        sfr_atomic_set(&sfrBuffers->workQueueCount, 0);
        sfr_atomic_set(&sfrBuffers->workQueueHead, 0);

        sfr_semaphore_init(&sfrBuffers->workStartSem, 0);
        sfr_semaphore_init(&sfrBuffers->workDoneSem, 0);

        for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
            sfrBuffers->threads[i].threadInd = i;
            #ifdef _WIN32
                const u64 handle = _beginthreadex(NULL, 0, sfr__worker_thread_func, &sfrBuffers->threads[i], 0, NULL);
                sfrBuffers->threads[i].handle = (SfrThread)handle;
            #else
                pthread_create(&sfrBuffers->threads[i].handle, NULL, sfr__worker_thread_func, &sfrBuffers->threads[i]);
            #endif
        }
    #endif

    sfr_resize(w, h); // call resize to setup tiling system
    sfr_clear(0xFF000000);
    sfr_reset();
    sfr_set_fov(fovDeg);
    sfr_set_camera(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

    sfrState.baseTexPixels[0] = 0xFFFFFFFF;
    sfrState.baseTex = (Texture){ .w = 1, .h = 1, .pixels = sfrState.baseTexPixels };
}

#ifdef SFR_MULTITHREADED

SFR_FUNC void sfr_shutdown(void) {
    sfr_flush_and_wait();

    sfrState.shutdown = 1;
    sfr_semaphore_post(&sfrBuffers->workStartSem, SFR_THREAD_COUNT);

    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        #ifdef _WIN32
            WaitForSingleObject(sfrBuffers->threads[i].handle, INFINITE);
            CloseHandle(sfrBuffers->threads[i].handle);
        #else
            pthread_join(sfrBuffers->threads[i].handle, NULL);
        #endif
    }

    sfr_semaphore_destroy(&sfrBuffers->workStartSem);
    sfr_semaphore_destroy(&sfrBuffers->workDoneSem);
}

// dispatches the binned triangles to workers and waits for them
SFR_FUNC void sfr_flush_and_wait(void) {
    if (0 == sfr_atomic_get(&sfrBuffers->workQueueCount)) {
        sfr_atomic_set(&sfrBuffers->triangleBinAllocator, 0);
        return; // no work to do
    }

    // wake up all workers
    sfr_semaphore_post(&sfrBuffers->workStartSem, SFR_THREAD_COUNT);

    // wait for all workers to signal they're done
    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        sfr_semaphore_wait(&sfrBuffers->workDoneSem);
    }

    // reset for the next frame
    sfr_atomic_set(&sfrBuffers->triangleBinAllocator, 0);
    sfr_atomic_set(&sfrBuffers->workQueueCount, 0);
    sfr_atomic_set(&sfrBuffers->workQueueHead, 0);
}

#endif // SFR_MULTITHREADED

#ifndef SFR_NO_ALPHA

SFR_FUNC void sfr_present_alpha(void) {
    #ifdef SFR_MULTITHREADED
        sfr_flush_and_wait();
    #endif

    for (i32 i = sfrWidth * sfrHeight - 1; i >= 0; i -= 1) {
        const u8 a = sfrBuffers->accum[i].a;
        if (0 == a || sfrBuffers->depth[i] < sfrBuffers->accum[i].depth) {
            continue;
        }

        const u32 bgCol = sfrBuffers->pixel[i];
        const u8 bgr = (bgCol >> 16) & 0xFF;
        const u8 bgg = (bgCol >> 8)  & 0xFF;
        const u8 bgb = (bgCol >> 0)  & 0xFF;

        // blend accumulated color with background
        const u8 r = sfrBuffers->accum[i].r + (bgr * (255 - a)) / 255;
        const u8 g = sfrBuffers->accum[i].g + (bgg * (255 - a)) / 255;
        const u8 b = sfrBuffers->accum[i].b + (bgb * (255 - a)) / 255;

        sfrBuffers->pixel[i] = (0xFF << 24) | (r << 16) | (g << 8) | b;
    }
}

#endif // !SFR_NO_ALPHA

SFR_FUNC void sfr_resize(i32 width, i32 height) {
    if (width > SFR_MAX_WIDTH || height > SFR_MAX_HEIGHT) {
        SFR_ERR_RET(, "sfr_resize: dimensions out of range (%d, %d) > (%d, %d)\n",
            width, height, SFR_MAX_WIDTH, SFR_MAX_HEIGHT);
    }

    #ifdef SFR_MULTITHREADED
        sfr_flush_and_wait();
    #endif

    sfrWidth = width;
    sfrHeight = height;
    sfrState.halfWidth = width / 2.f;
    sfrState.halfHeight = height / 2.f;

    // top
    sfrState.clipPlanes[0][0] = (Vec){0.f, 0.5f, 0.f, 1.f};
    sfrState.clipPlanes[0][1] = (Vec){0.f, 1.f,  0.f, 1.f};
    
    // bottom
    sfrState.clipPlanes[1][0] = (Vec){0.f, height, 0.f, 1.f};
    sfrState.clipPlanes[1][1] = (Vec){0.f, -1.f,   0.f, 1.f};

    // left
    sfrState.clipPlanes[2][0] = (Vec){0.5f, 0.f, 0.f, 1.f};
    sfrState.clipPlanes[2][1] = (Vec){1.f,  0.f, 0.f, 1.f};
    
    // right
    sfrState.clipPlanes[3][0] = (Vec){width, 0.f, 0.f, 1.f};
    sfrState.clipPlanes[3][1] = (Vec){-1.f,  0.f, 0.f, 1.f};

    #ifdef SFR_MULTITHREADED
        sfrBuffers->tileCols = (width + SFR_TILE_WIDTH - 1) / SFR_TILE_WIDTH;
        sfrBuffers->tileRows = (height + SFR_TILE_HEIGHT - 1) / SFR_TILE_HEIGHT;
        sfrBuffers->tileCount = sfrBuffers->tileCols * sfrBuffers->tileRows;

        for (i32 y = 0; y < sfrBuffers->tileRows; y += 1) {
            for (i32 x = 0; x < sfrBuffers->tileCols; x += 1) {
                SfrTile* tile = &sfrBuffers->tiles[y * sfrBuffers->tileCols + x];
                tile->minX = x * SFR_TILE_WIDTH;
                tile->minY = y * SFR_TILE_HEIGHT;
                tile->maxX = SFR_MIN((x + 1) * SFR_TILE_WIDTH, width);
                tile->maxY = SFR_MIN((y + 1) * SFR_TILE_HEIGHT, height);
                sfr_atomic_set(&tile->binCount, 0);
            }
        }
    #endif
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

SFR_FUNC void sfr_clear(u32 clearCol) {
    #ifdef SFR_MULTITHREADED
        sfr_flush_and_wait();
    #endif
    
    for (i32 i = sfrWidth * sfrHeight - 1; i >= 0; i -= 1) {
        sfrBuffers->pixel[i] = clearCol;
        sfrBuffers->depth[i] = sfrFarDist;
        #ifndef SFR_NO_ALPHA
            sfr_memset(&sfrBuffers->accum[i], 0, sizeof(sfrBuffers->accum[0]));
            sfrBuffers->accum[i].depth = sfrFarDist;
        #endif
    }
    
    #ifdef SFR_MULTITHREADED
        // reset tile bin counts
        for (i32 i = 0; i < sfrBuffers->tileCount; i += 1) {
            sfr_atomic_set(&sfrBuffers->tiles[i].binCount, 0);
        }

        sfr_atomic_set(&sfrRasterCount, 0);
    #else
        sfrRasterCount = 0;
    #endif
}

SFR_FUNC void sfr_clear_depth(void) {
    #ifdef SFR_MULTITHREADED
        sfr_flush_and_wait();
    #endif

    for (i32 i = sfrWidth * sfrHeight - 1; i >= 0; i -= 1) {
        sfrBuffers->depth[i] = sfrFarDist;
        #ifndef SFR_NO_ALPHA
            sfrBuffers->accum[i].depth = sfrFarDist;
        #endif
    }
}

SFR_FUNC void sfr_triangle(
    f32 ax, f32 ay, f32 az,
    f32 bx, f32 by, f32 bz,
    f32 cx, f32 cy, f32 cz,
    u32 col
) {
    sfr_triangle_tex(
        ax, ay, az, 0.f, 0.f,
        bx, by, bz, 0.f, 0.f,
        cx, cy, cz, 0.f, 0.f,
        col, &sfrState.baseTex);
}

SFR_FUNC void sfr_triangle_tex(
    f32 ax, f32 ay, f32 az, f32 au, f32 av,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv,
    u32 col, const Texture* tex
) {
    // transform vertices to world space for culling
    const Vec aModel = sfr_mat_mul_vec(sfrMatModel, (Vec){ax, ay, az, 1.f});
    const Vec bModel = sfr_mat_mul_vec(sfrMatModel, (Vec){bx, by, bz, 1.f});
    const Vec cModel = sfr_mat_mul_vec(sfrMatModel, (Vec){cx, cy, cz, 1.f});

    // backface culling
    Vec triNormal;
    #ifndef SFR_NO_CULLING
        const Vec line0 = sfr_vec_sub(bModel, aModel);
        const Vec line1 = sfr_vec_sub(cModel, aModel);
        triNormal = sfr_vec_cross(line0, line1);
        const Vec camRay = sfr_vec_sub(aModel, sfrCamPos);
        if (sfr_vec_dot(triNormal, camRay) > 0.f) {
            return;
        }
    #endif

    // lighting calculation
    if (sfrState.lightingEnabled) {
        if (sfrState.normalMatDirty) {
            sfr_update_normal_mat();
        }

        #ifdef SFR_NO_CULLING
            const Vec line0 = sfr_vec_sub(bModel, aModel);
            const Vec line1 = sfr_vec_sub(cModel, aModel);
            triNormal = sfr_vec_cross(line0, line1);
        #endif

        triNormal = sfr_mat_mul_vec(sfrState.matNormal, triNormal);
        triNormal = sfr_vec_norm(triNormal);

        const f32 intensity = sfr_fmaxf(
            sfrState.lightingAmbient, 
            sfr_vec_dot(triNormal, sfrState.lightingDir));
        
        col = sfr_adjust_color_u32(col, intensity);
    }

    // to view space
    SfrTexVert viewTri[3] = {
        {sfr_mat_mul_vec(sfrMatView, aModel), au, av, 0},
        {sfr_mat_mul_vec(sfrMatView, bModel), bu, bv, 0},
        {sfr_mat_mul_vec(sfrMatView, cModel), cu, cv, 0}
    };
    for (int i = 0; i < 3; i += 1) {
        viewTri[i].viewZ = viewTri[i].pos.z;
    }

    // prepare clip space verts and transform to clip space
    SfrTexVert clipTris[16][3];
    SfrTexVert (*input)[3] = clipTris; i32 inputCount = 1; // used in rasterization
    for (i32 i = 0; i < 3; i += 1) {
        clipTris[0][i].pos = sfr_mat_mul_vec(sfrMatProj, viewTri[i].pos);
        clipTris[0][i].u = viewTri[i].u;
        clipTris[0][i].v = viewTri[i].v;
        clipTris[0][i].viewZ = viewTri[i].viewZ;
    }

    // check if clipping is needed at all
    const f32 guardBand = 1.2f; // 20 percent    
    u8 needsClipping = 0;
    for (i32 i = 0; i < 3; i += 1) {
        const Vec p = clipTris[0][i].pos;
        const f32 wGuard = p.w * guardBand;
        if (p.x < -wGuard || p.x > wGuard || 
            p.y < -wGuard || p.y > wGuard ||
            p.z < -p.w || p.z > p.w
        ) {
            needsClipping = 1;
            break;
        }
    }
    if (!needsClipping) {
        goto SFR_TRI_TEX_BIN;
    }

    // frustum planes in homogeneous clip space
    const Vec frustumPlanes[6] = {
        {0.f, 0.f, 1.f, 1.f},  // near:   z + w >= 0
        {0.f, 0.f, -1.f, 1.f}, // far:   -z + w >= 0
        {1.f, 0.f, 0.f, 1.f},  // left:   x + w >= 0
        {-1.f, 0.f, 0.f, 1.f}, // right: -x + w >= 0
        {0.f, 1.f, 0.f, 1.f},  // bottom: y + w >= 0
        {0.f, -1.f, 0.f, 1.f}  // top:   -y + w >= 0
    };

    // clip against frustum planes
    SfrTexVert buffer[SFR_ARRLEN(clipTris)][3];
    SfrTexVert (*output)[3] = buffer;
    
    // process each clipping plane
    for (i32 p = 0; p < 6; p += 1) {
        i32 outputCount = 0;
        for (i32 i = 0; i < inputCount; i += 1) {
            SfrTexVert clipped[2][3];
            const i32 count = sfr_clip_tri_homogeneous(
                clipped,
                frustumPlanes[p],
                input[i]
            );
            
            // add clipped triangles to output
            for (i32 j = 0; j < count; j += 1) {
                output[outputCount][0] = clipped[j][0];
                output[outputCount][1] = clipped[j][1];
                output[outputCount][2] = clipped[j][2];
                outputCount += 1;
            }
        }
        
        // swap buffers
        SfrTexVert (*temp)[3] = input;
        input = output;
        output = temp;
        inputCount = outputCount;
    }

    // bin all final triangles
    SFR_TRI_TEX_BIN:;
    const Texture* texToUse = tex ? tex : &sfrState.baseTex;
    for (i32 i = 0; i < inputCount; i += 1) {
        SfrTexVert* tri = input[i];
        SfrTexVert screen[3];
        
        // perspective divide and screen space conversion
        u8 skip = 0;
        for (i32 j = 0; j < 3; j += 1) {
            if (tri[j].pos.w <= SFR_EPSILON) {
                skip = 1;
                break;
            }
            const Vec ndc = sfr_vec_div(tri[j].pos, tri[j].pos.w);
            screen[j].pos.x =  (ndc.x + 1.f) * sfrState.halfWidth;
            screen[j].pos.y = (-ndc.y + 1.f) * sfrState.halfHeight;
            screen[j].pos.z = ndc.z;
            screen[j].u = tri[j].u;
            screen[j].v = tri[j].v;
            screen[j].viewZ = tri[j].viewZ;
        }

        if (skip ||
            screen[0].viewZ <= SFR_EPSILON ||
            screen[1].viewZ <= SFR_EPSILON ||
            screen[2].viewZ <= SFR_EPSILON
        ) {
            continue;
        }

        const f32 aInvZ = 1.f / screen[0].viewZ;
        const f32 bInvZ = 1.f / screen[1].viewZ;
        const f32 cInvZ = 1.f / screen[2].viewZ;
        
        const f32 auoz = screen[0].u * aInvZ;
        const f32 avoz = screen[0].v * aInvZ;
        const f32 buoz = screen[1].u * bInvZ;
        const f32 bvoz = screen[1].v * bInvZ;
        const f32 cuoz = screen[2].u * cInvZ;
        const f32 cvoz = screen[2].v * cInvZ;
        
        sfr__bin_triangle(
            screen[0].pos.x, screen[0].pos.y, screen[0].pos.z, aInvZ, auoz, avoz,
            screen[1].pos.x, screen[1].pos.y, screen[1].pos.z, bInvZ, buoz, bvoz,
            screen[2].pos.x, screen[2].pos.y, screen[2].pos.z, cInvZ, cuoz, cvoz,
            col, texToUse
        );
    }
}

SFR_FUNC void sfr_billboard(u32 col, const Texture* tex) {
    const Mat savedModel = sfrMatModel;
    sfrMatModel = sfr_mat_identity();

    const Vec center = { savedModel.m[3][0], savedModel.m[3][1], savedModel.m[3][2], 1.f };

    const f32 sx = 0.5f * sfr_sqrtf(
        savedModel.m[0][0] * savedModel.m[0][0] + 
        savedModel.m[0][1] * savedModel.m[0][1] + 
        savedModel.m[0][2] * savedModel.m[0][2]);
    const f32 sy = 0.5f * sfr_sqrtf(
        savedModel.m[1][0] * savedModel.m[1][0] + 
        savedModel.m[1][1] * savedModel.m[1][1] + 
        savedModel.m[1][2] * savedModel.m[1][2]);

    const Vec right = { sx * sfrMatView.m[0][0], sx * sfrMatView.m[1][0], sx * sfrMatView.m[2][0], 0.f };
    const Vec up    = { sy * sfrMatView.m[0][1], sy * sfrMatView.m[1][1], sy * sfrMatView.m[2][1], 0.f };

    const Vec a = sfr_vec_add(center, sfr_vec_add(sfr_vec_mul(right, -1.f), sfr_vec_mul(up, -1.f)));
    const Vec b = sfr_vec_add(center, sfr_vec_add(sfr_vec_mul(right,  1.f), sfr_vec_mul(up, -1.f)));
    const Vec c = sfr_vec_add(center, sfr_vec_add(sfr_vec_mul(right,  1.f), sfr_vec_mul(up,  1.f)));
    const Vec d = sfr_vec_add(center, sfr_vec_add(sfr_vec_mul(right, -1.f), sfr_vec_mul(up,  1.f)));

    sfr_triangle_tex(
        a.x, a.y, a.z, 0.f, 1.f,
        c.x, c.y, c.z, 1.f, 0.f,
        b.x, b.y, b.z, 1.f, 1.f,
        col, tex);
    sfr_triangle_tex(
        a.x, a.y, a.z, 0.f, 1.f,
        d.x, d.y, d.z, 0.f, 0.f,
        c.x, c.y, c.z, 1.f, 0.f,
        col, tex);

    sfrMatModel = savedModel;
}

SFR_FUNC void sfr_cube(u32 col, const Texture* tex) {
    // front face
    sfr_triangle_tex(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 
        -0.5f, 0.5f,-0.5f, 1.f,1.f,
         0.5f, 0.5f,-0.5f, 0.f,1.f, col, tex);
    sfr_triangle_tex(
        -0.5f,-0.5f,-0.5f, 1.f,0.f,
         0.5f, 0.5f,-0.5f, 0.f,1.f,
         0.5f,-0.5f,-0.5f, 0.f,0.f, col, tex);

    // right face
    sfr_triangle_tex(
        0.5f,-0.5f,-0.5f, 1.f,0.f,
        0.5f, 0.5f,-0.5f, 1.f,1.f,
        0.5f, 0.5f, 0.5f, 0.f,1.f, col, tex);
    sfr_triangle_tex(
        0.5f,-0.5f,-0.5f, 1.f,0.f,
        0.5f, 0.5f, 0.5f, 0.f,1.f,
        0.5f,-0.5f, 0.5f, 0.f,0.f, col, tex);

    // back face
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f,
         0.5f, 0.5f, 0.5f, 1.f,1.f,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, col, tex);
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f,
        -0.5f, 0.5f, 0.5f, 0.f,1.f,
        -0.5f,-0.5f, 0.5f, 0.f,0.f, col, tex);

    // left face
    sfr_triangle_tex(
        -0.5f,-0.5f, 0.5f, 1.f,0.f,
        -0.5f, 0.5f, 0.5f, 1.f,1.f,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, col, tex);
    sfr_triangle_tex(
        -0.5f,-0.5f, 0.5f, 1.f,0.f,
        -0.5f, 0.5f,-0.5f, 0.f,1.f,
        -0.5f,-0.5f,-0.5f, 0.f,0.f, col, tex);

    // top face
    sfr_triangle_tex(
        -0.5f, 0.5f,-0.5f, 1.f,0.f,
        -0.5f, 0.5f, 0.5f, 1.f,1.f,
         0.5f, 0.5f, 0.5f, 0.f,1.f, col, tex);
    sfr_triangle_tex(
        -0.5f, 0.5f,-0.5f, 1.f,0.f,
         0.5f, 0.5f, 0.5f, 0.f,1.f,
         0.5f, 0.5f,-0.5f, 0.f,0.f, col, tex);

    // bottom face
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f,
        -0.5f,-0.5f, 0.5f, 1.f,1.f,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, col, tex);
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f,
        -0.5f,-0.5f,-0.5f, 0.f,1.f,
         0.5f,-0.5f,-0.5f, 0.f,0.f, col, tex);
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

SFR_FUNC void sfr_mesh(const Mesh* mesh, u32 col, const Texture* tex) {
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
            col, tex
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

    // behind camera
    if (p.w <= 0.f) {
        return 0;
    }

    p = sfr_vec_div(p, p.w);
    p.x = (1.f + p.x) * sfrState.halfWidth;
    p.y = (1.f - p.y) * sfrState.halfHeight;

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
        SFR_ERR_RET(0, "sfr_load_mesh: failed to allocate Mesh struct\n");
    }

    mesh->tris = NULL;
    mesh->uvs = NULL;
    mesh->vertCount = 0;

    FILE* objFile = fopen(filename, "r");
    if (!objFile) {
        free(mesh);
        SFR_ERR_RET(0, "sfr_load_mesh: failed to open file '%s'\n", filename);
    }

    // read verts and uvs
    f32* verts = NULL;
    f32* uvCoords = NULL;
    i32 vi = 0, uvi = 0;
    char line[128];

    while (fgets(line, sizeof(line), objFile)) {
        if ('v' != line[0]) {
            continue;
        }

        f32 x, y, z;
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
        } else if ('t' == line[1] && 2 == sscanf(line, "vt %f %f", &x, &y)) {
            uvCoords = realloc(uvCoords, (uvi + 2) * sizeof(f32));
            if (!uvCoords) {
                fclose(objFile);
                free(verts);
                free(mesh);
                SFR_ERR_RET(0, "sfr_load_mesh: failed to allocate UVs\n");
            }
            uvCoords[uvi + 0] = x;
            uvCoords[uvi + 1] = y;
            uvi += 2;
        }
    }

    rewind(objFile);

    // process faces
    i32 triInd = 0, uvInd = 0;
    u8 meshHasUVs = 0;

    while (fgets(line, sizeof(line), objFile)) {
        if (line[0] != 'f') {
            continue;
        }

        char* tokens[4];
        i32 tokenCount = 0;
        char* token = sfr_strtok(line, " \t\n"); // skip 'f'
        while ((token = sfr_strtok(NULL, " \t\n")) != NULL && tokenCount < 4) {
            tokens[tokenCount] = token;
            tokenCount += 1;
        }
        
        // skip lines with less than 3 or more than 4 vertices
        if (tokenCount < 3 || tokenCount > 4) {
            continue;
        }

        i32 vIndices[4], uvIndices[4];
        u8 currentFaceHasUV = 0;

        for (i32 i = 0; i < tokenCount; i += 1) {
            vIndices[i] = uvIndices[i] = -1;
            char* parts[3] = {0};
            char* part = sfr_strtok(tokens[i], "/");
            for (i32 j = 0; part && j < 3; j += 1) {
                parts[j] = part;
                part = sfr_strtok(NULL, "/");
            }
            if (parts[0]) vIndices[i]  = atoi(parts[0]) - 1;
            if (parts[1]) uvIndices[i] = atoi(parts[1]) - 1;

            if (-1 != uvIndices[i]) {
                currentFaceHasUV = 1;
            }
        }

        // determine the number of triangles and their vertex inds
        i32 numTriangles = (tokenCount == 3) ? 1 : 2;
        i32 triVertices[2][3];
        if (tokenCount == 3) {
            triVertices[0][0] = 0;
            triVertices[0][1] = 1;
            triVertices[0][2] = 2;
        } else {
            // quad => split into two triangles (0,1,2) and (0,2,3)
            triVertices[0][0] = 0;
            triVertices[0][1] = 1;
            triVertices[0][2] = 2;
            triVertices[1][0] = 0;
            triVertices[1][1] = 2;
            triVertices[1][2] = 3;
        }

        // process each triangle
        for (i32 t = 0; t < numTriangles; t += 1) {
            // check uv consistency
            if (currentFaceHasUV) {
                for (i32 i = 0; i < 3; i += 1) {
                    const i32 ind = triVertices[t][i];
                    if (-1 == uvIndices[ind]) {
                        fclose(objFile);
                        free(verts);
                        free(uvCoords);
                        free(mesh->tris);
                        free(mesh->uvs);
                        free(mesh);
                        SFR_ERR_RET(0, "sfr_load_mesh: inconsistent UV indices\n");
                    }
                }
            }

            if (currentFaceHasUV) {
                if (!meshHasUVs) {
                    meshHasUVs = 1;
                    if (!uvCoords) {
                        fclose(objFile);
                        free(verts);
                        free(mesh->tris);
                        free(mesh);
                        SFR_ERR_RET(0, "sfr_load_mesh: UVs missing\n");
                    }
                }

                // add uvs for this triangle
                for (i32 i = 0; i < 3; i += 1) {
                    const i32 ind = triVertices[t][i];
                    const i32 currUVInd = uvIndices[ind];
                    if (currUVInd < 0 || currUVInd >= uvi / 2) {
                        fclose(objFile);
                        free(verts);
                        free(uvCoords);
                        free(mesh->uvs);
                        free(mesh->tris);
                        free(mesh);
                        SFR_ERR_RET(0, "sfr_load_mesh: invalid UV index\n");
                    }

                    const f32 u =       uvCoords[currUVInd * 2 + 0];
                    const f32 v = 1.f - uvCoords[currUVInd * 2 + 1];

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
                    mesh->uvs[uvInd + 1] = v;
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
                const i32 ind = triVertices[t][i];
                const i32 vInd = vIndices[ind];
                if (vInd < 0 || vInd >= vi / 3) {
                    fclose(objFile);
                    free(verts);
                    free(uvCoords);
                    free(mesh->uvs);
                    free(mesh->tris);
                    free(mesh);
                    SFR_ERR_RET(0, "sfr_load_mesh: invalid vertex index\n");
                }

                mesh->tris = realloc(mesh->tris, (triInd + 3) * sizeof(f32));
                if (!mesh->tris) {
                    fclose(objFile);
                    free(verts);
                    free(uvCoords);
                    free(mesh->uvs);
                    free(mesh);
                    SFR_ERR_RET(0, "sfr_load_mesh: failed to allocate vertices\n");
                }

                const f32* v = &verts[vInd * 3];
                mesh->tris[triInd + 0] = v[0];
                mesh->tris[triInd + 1] = v[1];
                mesh->tris[triInd + 2] = v[2];
                triInd += 3;
            }
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
                    col |= 0xFF000000 | (px[2] << 16) | (px[1] << 8) | px[0];
                } break;
                case 16: {
                    const u16 px = *((u16*)(row + x * 2));
                    const u32 rMask = masks[0] ? masks[0] : 0x7C00;
                    const u32 gMask = masks[1] ? masks[1] : 0x03E0;
                    const u32 bMask = masks[2] ? masks[2] : 0x001F;
                    col |= 0xFF000000;
                    col |= ((px & rMask) * 0xFF / rMask) << 16;
                    col |= ((px & gMask) * 0xFF / gMask) << 8;
                    col |= ((px & bMask) * 0xFF / bMask);
                } break;
                case 8:
                case 4:
                case 1: {
                    const u8 index = row[x / (8 / bpp)];
                    const u8 shift = (7 - (x % (8 / bpp))) * bpp;
                    const u8 palInd = (index >> shift) & ((1 << bpp) - 1);
                    const u8* entry = (u8*)(palette + palInd);
                    col |= 0xFF000000 | (entry[2] << 16) | (entry[1] << 8) | entry[0];
                } break;
                default: {
                    free(pixelData);
                    free(tex->pixels);
                    free(tex);
                    SFR_ERR_RET(NULL, "sfr_load_texture: unsupported bit depth: %d\n", bpp);
                } break;
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

SFR_FUNC ParticleSystem sfr_particles_create(Particle* buffer, i32 count, const Texture* tex) {
    return (ParticleSystem){
        .particles = buffer,
        .active = 0, .total = count,
        .tex = tex
    };
}

SFR_FUNC void sfr_particles_update(ParticleSystem* sys, f32 frameTime) {
    for (i32 i = 0; i < sys->active;) {
        Particle* p = &sys->particles[i];
        p->age += frameTime;

        // remove dead particles
        if (p->age >= p->lifetime) {
            sys->particles[i] = sys->particles[sys->active - 1];
            sys->active -= 1;
            continue;
        }

        // update particle
        p->vx += p->ax * frameTime;
        p->vy += p->ay * frameTime;
        p->vz += p->az * frameTime;
        p->px += p->vx * frameTime;
        p->py += p->vy * frameTime;
        p->pz += p->vz * frameTime;

        i += 1;
    }
}

SFR_FUNC void sfr_particles_draw(const ParticleSystem* sys) {
    for (i32 i = 0; i < sys->active; i += 1) {
        Particle* p = &sys->particles[i];

        // interpolate properties
        const f32 t = p->age / p->lifetime;
        const f32 size = SFR_LERPF(p->startSize, p->endSize, t);
        const u32 col = sfr_lerp_col(p->startCol, p->endCol, t);

        // set transform and draw
        sfr_reset();
        sfr_scale(size, size, size);
        sfr_translate(p->px, p->py, p->pz);
        sfr_billboard(col, sys->tex);
    }
}

SFR_FUNC void sfr_particles_emit(ParticleSystem* sys,
    f32 px, f32 py, f32 pz,
    f32 vx, f32 vy, f32 vz,
    f32 ax, f32 ay, f32 az,
    f32 startSize, f32 endSize,
    u32 startCol, u32 endCol,
    f32 lifetime
) {
    if (sys->active >= sys->total) {
        return;
    }

    // create new particle
    Particle* p = &sys->particles[sys->active++];
    p->px = px, p->py = py, p->pz = pz;
    p->vx = vx, p->vy = vy, p->vz = vz;
    p->ax = ax, p->ay = ay, p->az = az;
    p->startSize = startSize, p->endSize = endSize;
    p->startCol = startCol, p->endCol = endCol,
    p->age = 0.f, p->lifetime = lifetime;
}

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

#ifdef SFR_MULTITHREADED

#ifdef _WIN32
static unsigned __stdcall sfr__worker_thread_func(void* arg) {
#else
static void* sfr__worker_thread_func(void* arg) {
#endif
    // const SfrThreadData* self = (SfrThreadData*)arg;
    // const i32 selfInd = self->threadInd;

    while (1) {
        // wait for the main thread to signal a batch of work is ready
        sfr_semaphore_wait(&sfrBuffers->workStartSem);
        if (sfrState.shutdown) {
            break;
        }
        
        while (1) {
            // get the next tile index from the work queue
            const i32 head = sfr_atomic_add(&sfrBuffers->workQueueHead, 1) - 1;

            if (head >= sfr_atomic_get(&sfrBuffers->workQueueCount)) {
                break; // no more work in the queue
            }

            // get the tile and rasterize its contents
            SfrTile* tile = &sfrBuffers->tiles[sfrBuffers->workQueue[head]];
            i32 binCount = sfr_atomic_get(&tile->binCount);
            binCount = SFR_MIN(binCount, SFR_MAX_BINS_PER_TILE);
            for (i32 i = 0; i < binCount; i += 1) {
                sfr_rasterize_bin(tile->bins[i], tile);
            }

            sfr_atomic_set(&tile->binCount, 0);
        }
        
        // signal this thread finished its work
        sfr_semaphore_post(&sfrBuffers->workDoneSem, 1);
    }

    #ifdef _WIN32
        return 0;
    #else
        return NULL;
    #endif
}

SFR_FUNC i32 sfr_atomic_add(SfrAtomic32* a, i32 val) {
#ifdef _WIN32
    return InterlockedExchangeAdd(a, val) + val;
#else
    return __sync_add_and_fetch(a, val);
#endif
}

SFR_FUNC i32 sfr_atomic_get(SfrAtomic32* a) {
#ifdef _WIN32
    return _InterlockedCompareExchange(a, 0, 0);
#else
    return __sync_fetch_and_add(a, 0);
#endif
}

SFR_FUNC void sfr_atomic_set(SfrAtomic32* a, i32 val) {
#ifdef _WIN32
    _InterlockedExchange(a, val);
#else
    __sync_lock_test_and_set(a, val);
#endif
}

SFR_FUNC i32 sfr_atomic_cas(SfrAtomic32* ptr, i32 oldVal, i32 newVal) {
#ifdef _WIN32
    return (i32)_InterlockedCompareExchange(ptr, (LONG)newVal, (LONG)oldVal);
#else
    return (i32)__sync_val_compare_and_swap(ptr, oldVal, newVal);
#endif
}

SFR_FUNC i64 sfr_atomic_add64(SfrAtomic64* a, i64 val) {
#ifdef _WIN32
    return _InterlockedExchangeAdd64(a, val) + val;
#else
    return __sync_add_and_fetch(a, val);
#endif
}

SFR_FUNC i64 sfr_atomic_get64(SfrAtomic64* a) {
#ifdef _WIN32
    return _InterlockedCompareExchange64(a, 0, 0);
#else
    return __sync_fetch_and_add(a, 0);
#endif
}

SFR_FUNC void sfr_atomic_set64(SfrAtomic64* a, i64 val) {
#ifdef _WIN32
    _InterlockedExchange64(a, val);
#else
    __sync_lock_test_and_set(a, val);
#endif
}

SFR_FUNC i64 sfr_atomic_cas64(SfrAtomic64* ptr, i64 oldVal, i64 newVal) {
#ifdef _WIN32
    return _InterlockedCompareExchange64(ptr, newVal, oldVal);
#else
    return __sync_val_compare_and_swap(ptr, oldVal, newVal);
#endif
}

SFR_FUNC void sfr_mutex_init(SfrMutex* m) {
#ifdef _WIN32
    InitializeCriticalSection(m);
#else
    pthread_mutex_init(m, NULL);
#endif
}

SFR_FUNC void sfr_mutex_destroy(SfrMutex* m) {
#ifdef _WIN32
    DeleteCriticalSection(m);
#else
    pthread_mutex_destroy(m);
#endif
}

SFR_FUNC void sfr_mutex_lock(SfrMutex* m) {
#ifdef _WIN32
    EnterCriticalSection(m);
#else
    pthread_mutex_lock(m);
#endif
}

SFR_FUNC void sfr_mutex_unlock(SfrMutex* m) {
#ifdef _WIN32
    LeaveCriticalSection(m);
#else
    pthread_mutex_unlock(m);
#endif
}

SFR_FUNC i32 sfr_semaphore_init(SfrSemaphore* s, i32 initial_count) {
#ifdef _WIN32
    *s = CreateSemaphoreExW(NULL, initial_count, SFR_THREAD_COUNT * 2, NULL, 0, SEMAPHORE_ALL_ACCESS);
    return (NULL != *s) ? 0 : -1;
#else
    return sem_init(s, 0, initial_count);
#endif
}

SFR_FUNC void sfr_semaphore_destroy(SfrSemaphore* s) {
#ifdef _WIN32
    CloseHandle(*s);
#else
    sem_destroy(s);
#endif
}

SFR_FUNC void sfr_semaphore_wait(SfrSemaphore* s) {
#ifdef _WIN32
    WaitForSingleObjectEx(*s, INFINITE, 0);
#else
    // loop on sem_wait to handle EINTR (interrupts from signals)
    while (-1 == sem_wait(s) && errno == EINTR) {
        continue;
    }
#endif
}

SFR_FUNC void sfr_semaphore_post(SfrSemaphore* s, i32 n) {
#ifdef _WIN32
    ReleaseSemaphore(*s, n, NULL);
#else
    for (i32 i = 0; i < n; i += 1) {
        sem_post(s);
    }
#endif
}

#endif // SFR_MULTITHREADED

#endif // SFR_IMPL

#ifdef SFR_PREFIXED_TYPES
    #undef Vec
    #undef Mat
    #undef Tri
    #undef Mesh
    #undef Texture
    #undef Font

    #undef Particle
    #undef ParticleSystem

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
