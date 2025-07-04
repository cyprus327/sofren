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
        #define SFR_MAX_BINS_PER_TILE (1024 * 8)
    #endif
    #ifndef SFR_MAX_BINS_PER_FRAME
        // max triangles binned per frame
        #define SFR_MAX_BINS_PER_FRAME (1024 * 256)
    #endif
    #ifndef SFR_MAX_GEOMETRY_JOBS
        // max mesh chunks to be processed per frame
        #define SFR_MAX_GEOMETRY_JOBS (1024 * 1)
    #endif
    #ifndef SFR_GEOMETRY_JOB_SIZE
        // number of triangles per geometry job
        #define SFR_GEOMETRY_JOB_SIZE 64
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

#ifdef SFR_USE_SIMD
    #include <smmintrin.h> // SSE 4.1
#endif

//: types
#ifdef SFR_PREFIXED_TYPES
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

#ifdef SFR_USE_SIMD
    typedef union sfrvec sfrvec;
#else
    typedef struct sfrvec sfrvec;
#endif
typedef union sfrmat sfrmat;

typedef struct sfrmesh SfrMesh;
typedef struct sfrtex  SfrTexture;
typedef struct sfrfont SfrFont;

typedef struct sfrparticle  SfrParticle;
typedef struct sfrparticles SfrParticleSystem;

typedef struct sfrBuffers SfrBuffers;

typedef struct sfrTriangleBin SfrTriangleBin;
typedef struct sfrTile SfrTile;
#ifdef SFR_MULTITHREADED
    typedef struct sfrThreadData SfrThreadData;
    typedef struct sfrMeshChunkJob SfrMeshChunkJob;
#endif

//: extern variables
extern i32 sfrWidth, sfrHeight;
extern SfrBuffers* sfrBuffers;

// global variables below can be managed by you, however
// there is probably a function that will do what you want

extern SfrAtomic32 sfrRasterCount; // how many triangles have been rasterized since the last call to clear

extern sfrmat sfrMatModel, sfrMatView, sfrMatProj;
extern sfrvec sfrCamPos;
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
SFR_FUNC sfrvec sfr_vec_add(sfrvec a, sfrvec b);
SFR_FUNC sfrvec sfr_vec_sub(sfrvec a, sfrvec b);
SFR_FUNC sfrvec sfr_vec_mul(sfrvec a, f32 b);
SFR_FUNC sfrvec sfr_vec_div(sfrvec a, f32 b);
SFR_FUNC f32 sfr_vec_dot(sfrvec a, sfrvec b);
SFR_FUNC f32 sfr_vec_length(sfrvec v);
SFR_FUNC f32 sfr_vec_length2(sfrvec v);
SFR_FUNC sfrvec sfr_vec_cross(sfrvec a, sfrvec b);
SFR_FUNC sfrvec sfr_vec_norm(sfrvec v);
SFR_FUNC sfrvec sfr_vec_normf(f32 x, f32 y, f32 z);
SFR_FUNC sfrvec sfr_vec_face_normal(sfrvec a, sfrvec b, sfrvec c);
SFR_FUNC sfrmat sfr_mat_identity();
SFR_FUNC sfrmat sfr_mat_rot_x(f32 a);
SFR_FUNC sfrmat sfr_mat_rot_y(f32 a);
SFR_FUNC sfrmat sfr_mat_rot_z(f32 a);
SFR_FUNC sfrmat sfr_mat_translate(f32 x, f32 y, f32 z);
SFR_FUNC sfrmat sfr_mat_scale(f32 x, f32 y, f32 z);
SFR_FUNC sfrmat sfr_mat_proj(f32 fovDeg, f32 aspect, f32 near, f32 far);
SFR_FUNC sfrmat sfr_mat_mul(sfrmat a, sfrmat b);
SFR_FUNC sfrvec sfr_mat_mul_vec(sfrmat m, sfrvec v);
SFR_FUNC sfrmat sfr_mat_qinv(sfrmat m);
SFR_FUNC sfrmat sfr_mat_look_at(sfrvec pos, sfrvec target, sfrvec up);

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
    f32 ax, f32 ay, f32 az, f32 anx, f32 any, f32 anz,
    f32 bx, f32 by, f32 bz, f32 bnx, f32 bny, f32 bnz,
    f32 cx, f32 cy, f32 cz, f32 cnx, f32 cny, f32 cnz,
    u32 col);
SFR_FUNC void sfr_triangle_tex(
    f32 ax, f32 ay, f32 az, f32 au, f32 av, f32 anx, f32 any, f32 anz,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv, f32 bnx, f32 bny, f32 bnz,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv, f32 cnx, f32 cny, f32 cnz,
    u32 col, const SfrTexture* tex);

// other drawing functions, if tex is null sfrState.baseTex (white 1x1 texture) will be used
SFR_FUNC void sfr_billboard(u32 col, const SfrTexture* tex);
SFR_FUNC void sfr_cube(u32 col, const SfrTexture* tex);
SFR_FUNC void sfr_cube_ex(u32 col[12]);
SFR_FUNC void sfr_mesh(const SfrMesh* mesh, u32 col, const SfrTexture* tex);
SFR_FUNC void sfr_string(const SfrFont* font, const char* s, i32 sLength, u32 col); // not yet implemented
SFR_FUNC void sfr_glyph(const SfrFont* font, u16 id, u32 col); // draw a single character

// project the world position specified to screen coordinates
SFR_FUNC i32 sfr_world_to_screen(f32 x, f32 y, f32 z, i32* screenX, i32* screenY);    

// update the camera with the new position and view
SFR_FUNC void sfr_set_camera(f32 x, f32 y, f32 z, f32 yaw, f32 pitch, f32 roll);
SFR_FUNC void sfr_set_fov(f32 fovDeg); // update projection matrix with new fov
SFR_FUNC void sfr_set_lighting( // update internal lighting state for simple shading on triangles
    i32 on, sfrvec dir, f32 ambientIntensity);

// things requiring malloc / stdio
#ifndef SFR_NO_STD
    #include <stdio.h>
    #include <stdlib.h>

    SFR_FUNC SfrMesh* sfr_load_mesh(const char* filename); // load an obj file into a struct that sofren can use
    SFR_FUNC void sfr_release_mesh(SfrMesh** mesh);        // release loaded mesh's memory

    SFR_FUNC SfrTexture* sfr_load_texture(const char* filename); // load a BMP texture
    SFR_FUNC void sfr_release_texture(SfrTexture** texture);     // release loaded texture's memory

    SFR_FUNC SfrFont* sfr_load_font(const char* filename); // load a .srft (sofren font type) font, see 'sfr-fontmaker'
    SFR_FUNC void sfr_release_font(SfrFont** font);        // release loaded font's memory
#endif

// all particle related functions
SFR_FUNC SfrParticleSystem sfr_particles_create(SfrParticle* buffer, i32 count, const SfrTexture* tex);
SFR_FUNC void sfr_particles_update(SfrParticleSystem* sys, f32 frameTime);
SFR_FUNC void sfr_particles_draw(const SfrParticleSystem* sys);
SFR_FUNC void sfr_particles_emit(SfrParticleSystem* sys,
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

#ifdef SFR_USE_SIMD
    #ifdef _MSC_VER
        typedef union __declspec(align(16)) sfrvec {
            struct { f32 x, y, z, w; };
            __m128 v;
        } sfrvec;
    #else
        typedef union __attribute__((aligned(16))) sfrvec {
            struct { f32 x, y, z, w; };
            __m128 v;
        } sfrvec;
    #endif
#else
    typedef struct sfrvec { f32 x, y, z, w; } sfrvec;
#endif

typedef union sfrmat {
    struct { f32 m[4][4]; };
    sfrvec rows[4];
} sfrmat;

typedef struct sfrmesh {
    f32* tris;     // vertex positions
    f32* uvs;      // uv coordinates
    f32* normals;  // vertex normals
    i32 vertCount; // total number of floats (3 per vertex)
} SfrMesh;

typedef struct sfrtex {
    u32* pixels;
    i32 w, h;
} SfrTexture;

typedef struct sfrfont {
    // xy pairs [x0, y0, x1, y1, x2, ...]
    f32 verts[SFR_FONT_GLYPH_MAX][SFR_FONT_VERT_MAX];
} SfrFont;

typedef struct sfrparticle {
    f32 px, py, pz; // position
    f32 vx, vy, vz; // velocity
    f32 ax, ay, az; // acceleration
    f32 startSize, endSize;
    u32 startCol, endCol;
    f32 lifetime, age;
} SfrParticle;

typedef struct sfrparticles {
    SfrParticle* particles;
    i32 total, active;
    const SfrTexture* tex;
} SfrParticleSystem;

typedef struct sfrTriangleBin {
    f32 ax, ay, az, aInvZ, auoz, avoz, aIntensity;
    f32 bx, by, bz, bInvZ, buoz, bvoz, bIntensity;
    f32 cx, cy, cz, cInvZ, cuoz, cvoz, cIntensity;
    u32 col;
    const SfrTexture* tex;
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
    
    typedef struct sfrMeshChunkJob {
        const SfrMesh* mesh;
        sfrmat matModel;
        sfrmat matNormal;
        u32 col;
        const SfrTexture* tex;
        i32 startTriangle;
        i32 triangleCount;
    } SfrMeshChunkJob;
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
        SfrTriangleBin triangleBinPool[SFR_MAX_BINS_PER_FRAME];
        SfrAtomic32 triangleBinAllocator;
        
        // rasterizer work dispatch data
        i32 rasterWorkQueue[(SFR_MAX_WIDTH / SFR_TILE_WIDTH + 1) * (SFR_MAX_HEIGHT / SFR_TILE_HEIGHT + 1)];
        SfrAtomic32 rasterWorkQueueCount;
        SfrAtomic32 rasterWorkQueueHead;

        // geometry job system data
        SfrMeshChunkJob meshJobPool[SFR_MAX_GEOMETRY_JOBS];
        SfrAtomic32 meshJobAllocator;
        i32 geometryWorkQueue[SFR_MAX_GEOMETRY_JOBS];
        SfrAtomic32 geometryWorkQueueCount;
        SfrAtomic32 geometryWorkQueueHead;

        // thread management data
        struct sfrThreadData threads[SFR_THREAD_COUNT];
        SfrSemaphore geometryStartSem;
        SfrSemaphore geometryDoneSem;
        SfrSemaphore rasterStartSem;
        SfrSemaphore rasterDoneSem;
    #endif
} SfrBuffers;

typedef struct sfrState {
    i32 lightingEnabled;
    sfrvec lightingDir;
    f32 lightingAmbient;

    sfrmat matNormal;
    i32 normalMatDirty;

    u32 randState;

    f32 halfWidth, halfHeight;

    sfrvec clipPlanes[4][2];

    u32 baseTexPixels[1];
    SfrTexture baseTex;

    #ifdef SFR_MULTITHREADED
        i32 shutdown;
    #endif
} SfrState;

// helper to track vertex attributes during clipping of textured triangles
typedef struct sfrTexVert {
    sfrvec pos;       // position in view space
    f32 u, v;      // texture coords
    f32 viewZ;     // z in view space for perspective correction
    f32 intensity; // lighting intensity
} SfrTexVert;


//================================================
//:         GLOBAL VARIABLES
//================================================

i32 sfrWidth, sfrHeight;
SfrBuffers* sfrBuffers;
SfrAtomic32 sfrRasterCount;

sfrmat sfrMatModel, sfrMatView, sfrMatProj;
sfrvec sfrCamPos;
f32 sfrCamFov;
f32 sfrNearDist = 0.1f, sfrFarDist = 100.f;

// not extern, internal state
SfrState sfrState = {0};


//================================================
//:         MISC HELPER MACROS
//================================================

#define SFR_PI (3.14159265358979323846)
#define SFR_EPSILON (1e-10)

#define SFR_SWAPF(_a, _b) { const f32 _swapTemp = (_a); (_a) = (_b); (_b) = _swapTemp; }
#define SFR_LERPF(_a, _b, _t) ((_a) + (_t) * ((_b) - (_a)))

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

SFR_FUNC sfrvec sfr_vec_add(sfrvec a, sfrvec b) {
    sfrvec r;
    #ifdef SFR_USE_SIMD
        r.v = _mm_add_ps(a.v, b.v);
    #else
        r.x = a.x + b.x;
        r.y = a.y + b.y;
        r.z = a.z + b.z;
        r.w = a.w + b.w;
    #endif
    return r;
}

SFR_FUNC sfrvec sfr_vec_sub(sfrvec a, sfrvec b) {
    sfrvec r;
    #ifdef SFR_USE_SIMD
        r.v = _mm_sub_ps(a.v, b.v);
    #else
        r.x = a.x - b.x;
        r.y = a.y - b.y;
        r.z = a.z - b.z;
        r.w = a.w - b.w;
    #endif
    return r;
}

SFR_FUNC sfrvec sfr_vec_mul(sfrvec a, f32 b) {
    sfrvec r;
    #ifdef SFR_USE_SIMD
        const __m128 t = _mm_set1_ps(b);
        r.v = _mm_mul_ps(a.v, t);
    #else
        r.x = a.x * b;
        r.y = a.y * b;
        r.z = a.z * b;
        r.w = a.w * b;
    #endif
    return r;
}

SFR_FUNC sfrvec sfr_vec_div(sfrvec a, f32 b) {
    sfrvec r;
    #ifdef SFR_USE_SIMD
        const __m128 t = _mm_set1_ps(b);
        r.v = _mm_div_ps(a.v, t);
        r.w = 1.f;
    #else
        r.x = a.x / b;
        r.y = a.y / b;
        r.z = a.z / b;
        r.w = 1.f;
    #endif
    return r;
}

SFR_FUNC f32 sfr_vec_dot(sfrvec a, sfrvec b) {
    #ifdef SFR_USE_SIMD
        // 0x71 => multiply first 3 components (0111), sum them,
        // store result in first component of output (0001)
        const __m128 r = _mm_dp_ps(a.v, b.v, 0x71);
        return _mm_cvtss_f32(r);
    #else
        return a.x * b.x + a.y * b.y + a.z * b.z;
    #endif
}

SFR_FUNC f32 sfr_vec_length(sfrvec v) {
    return sfr_sqrtf(sfr_vec_dot(v, v));
}

SFR_FUNC f32 sfr_vec_length2(sfrvec v) {
    return sfr_vec_dot(v, v);
}

SFR_FUNC sfrvec sfr_vec_cross(sfrvec a, sfrvec b) {
    sfrvec r;
    #ifdef SFR_USE_SIMD
        const __m128 as1 = _mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(3, 0, 2, 1)); // {a.y, a.z, a.x, a.w}
        const __m128 bs1 = _mm_shuffle_ps(b.v, b.v, _MM_SHUFFLE(3, 1, 0, 2)); // {b.z, b.x, b.y, b.w}
        const __m128 mul1 = _mm_mul_ps(as1, bs1);
        const __m128 as2 = _mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(3, 1, 0, 2)); // {a.z, a.x, a.y, a.w}
        const __m128 bs2 = _mm_shuffle_ps(b.v, b.v, _MM_SHUFFLE(3, 0, 2, 1)); // {b.y, b.z, b.x, b.w}
        const __m128 mul2 = _mm_mul_ps(as2, bs2);
        r.v = _mm_sub_ps(mul1, mul2);
        r.w = 1.f;
    #else
        r.x = a.y * b.z - a.z * b.y;
        r.y = a.z * b.x - a.x * b.z;
        r.z = a.x * b.y - a.y * b.x;
        r.w = 1.f;
    #endif
    return r;
}

SFR_FUNC sfrvec sfr_vec_norm(sfrvec v) {
    const f32 l = sfr_vec_length(v);
    return l > SFR_EPSILON ? sfr_vec_mul(v, 1.f / l) : (sfrvec){0};
}

SFR_FUNC sfrvec sfr_vec_normf(f32 x, f32 y, f32 z) {
    return sfr_vec_norm((sfrvec){x, y, z, 1.f});
}

SFR_FUNC sfrvec sfr_vec_face_normal(sfrvec a, sfrvec b, sfrvec c) {
    const sfrvec edge1 = sfr_vec_sub(b, a);
    const sfrvec edge2 = sfr_vec_sub(c, a);
    return sfr_vec_norm(sfr_vec_cross(edge1, edge2));
}

SFR_FUNC sfrmat sfr_mat_identity() {
    sfrmat r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = 1.f;
    r.m[2][2] = 1.f;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat sfr_mat_rot_x(f32 a) {
    sfrmat r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = sfr_cosf(a);
    r.m[1][2] = sfr_sinf(a);
    r.m[2][1] = -sfr_sinf(a);
    r.m[2][2] = sfr_cosf(a);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat sfr_mat_rot_y(f32 a) {
    sfrmat r = {0};
    r.m[0][0] = sfr_cosf(a);
    r.m[0][2] = sfr_sinf(a);
    r.m[1][1] = 1.f;
    r.m[2][0] = -sfr_sinf(a);
    r.m[2][2] = sfr_cosf(a);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat sfr_mat_rot_z(f32 a) {
    sfrmat r = {0};
    r.m[0][0] = sfr_cosf(a);
    r.m[0][1] = sfr_sinf(a);
    r.m[1][0] = -sfr_sinf(a);
    r.m[1][1] = sfr_cosf(a);
    r.m[2][2] = 1.f;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat sfr_mat_translate(f32 x, f32 y, f32 z) {
    sfrmat r = {0};
    r.m[0][0] = 1.f;
    r.m[1][1] = 1.f;
    r.m[2][2] = 1.f;
    r.m[3][0] = x;
    r.m[3][1] = y;
    r.m[3][2] = z;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat sfr_mat_scale(f32 x, f32 y, f32 z) {
    sfrmat r = {0};
    r.m[0][0] = x;
    r.m[1][1] = y;
    r.m[2][2] = z;
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat sfr_mat_proj(f32 fovDev, f32 aspect, f32 near, f32 far) {
    const f32 fov = 1.f / sfr_tanf(fovDev * 0.5f / 180.f * SFR_PI);
    sfrmat r = {0};
    r.m[0][0] = aspect * fov;
    r.m[1][1] = fov;
    r.m[2][2] = far / (far - near);
    r.m[3][2] = (-far * near) / (far - near);
    r.m[2][3] = 1.f;
    r.m[3][3] = 0.f;
    return r;
}

SFR_FUNC sfrmat sfr_mat_mul(sfrmat a, sfrmat b) {
    sfrmat r;
    #ifdef SFR_USE_SIMD
        const __m128 b0 = b.rows[0].v;
        const __m128 b1 = b.rows[1].v;
        const __m128 b2 = b.rows[2].v;
        const __m128 b3 = b.rows[3].v;

        for (i32 i = 0; i < 4; i += 1) {
            const __m128 ar = a.rows[i].v;
            
            const __m128 x = _mm_shuffle_ps(ar, ar, _MM_SHUFFLE(0, 0, 0, 0));
            const __m128 y = _mm_shuffle_ps(ar, ar, _MM_SHUFFLE(1, 1, 1, 1));
            const __m128 z = _mm_shuffle_ps(ar, ar, _MM_SHUFFLE(2, 2, 2, 2));
            const __m128 w = _mm_shuffle_ps(ar, ar, _MM_SHUFFLE(3, 3, 3, 3));
            
            __m128 res = _mm_mul_ps(x, b0);
            res = _mm_add_ps(res, _mm_mul_ps(y, b1));
            res = _mm_add_ps(res, _mm_mul_ps(z, b2));
            res = _mm_add_ps(res, _mm_mul_ps(w, b3));
            
            r.rows[i].v = res;
        }
    #else
        for (i32 i = 0; i < 4; i += 1) {
            const f32 a0 = a.m[i][0], a1 = a.m[i][1], a2 = a.m[i][2], a3 = a.m[i][3];
            r.m[i][0] = a0 * b.m[0][0] + a1 * b.m[1][0] + a2 * b.m[2][0] + a3 * b.m[3][0];
            r.m[i][1] = a0 * b.m[0][1] + a1 * b.m[1][1] + a2 * b.m[2][1] + a3 * b.m[3][1];
            r.m[i][2] = a0 * b.m[0][2] + a1 * b.m[1][2] + a2 * b.m[2][2] + a3 * b.m[3][2];
            r.m[i][3] = a0 * b.m[0][3] + a1 * b.m[1][3] + a2 * b.m[2][3] + a3 * b.m[3][3];
        }
    #endif
    return r;
}

SFR_FUNC sfrvec sfr_mat_mul_vec(sfrmat m, sfrvec v) {
    sfrvec r;
    #ifdef SFR_USE_SIMD
        r.v = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(_mm_set1_ps(v.x), m.rows[0].v), _mm_mul_ps(_mm_set1_ps(v.y), m.rows[1].v)),
            _mm_add_ps(_mm_mul_ps(_mm_set1_ps(v.z), m.rows[2].v), _mm_mul_ps(_mm_set1_ps(v.w), m.rows[3].v)));
    #else
        r.x = v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + v.w * m.m[3][0];
        r.y = v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + v.w * m.m[3][1];
        r.z = v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + v.w * m.m[3][2];
        r.w = v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + v.w * m.m[3][3];
    #endif
    return r;
}

SFR_FUNC sfrmat sfr_mat_qinv(sfrmat m) {
    sfrmat r;
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

SFR_FUNC sfrmat sfr_mat_look_at(sfrvec pos, sfrvec target, sfrvec up) {
    const sfrvec forward = sfr_vec_norm(sfr_vec_sub(target, pos));
    up = sfr_vec_norm(sfr_vec_sub(up, sfr_vec_mul(forward, sfr_vec_dot(up, forward))));
    const sfrvec right = sfr_vec_cross(up, forward);

    sfrmat r;
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

// used in rasterizing to wrap texture coords
SFR_FUNC i32 sfr_wrap_coord(i32 x, i32 max) {
    return (x % max + max) % max;
}

SFR_FUNC i32 sfr_clip_tri_homogeneous(SfrTexVert out[2][3], sfrvec plane, const SfrTexVert in[3]) {
    SfrTexVert inside[3], outside[3];
    f32 insideDists[3], outsideDists[3];
    i32 insideCount = 0, outsideCount = 0;

    // precompute distances during classification
    for (i32 i = 0; i < 3; i += 1) {
        const sfrvec v = in[i].pos;
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
            .viewZ = a.viewZ + tAB * (b.viewZ - a.viewZ),
            .intensity = a.intensity + tAB * (b.intensity - a.intensity)
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
            .viewZ = a.viewZ + tAC * (c.viewZ - a.viewZ),
            .intensity = a.intensity + tAC * (c.intensity - a.intensity)
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
            .viewZ = a.viewZ + tAC * (c.viewZ - a.viewZ),
            .intensity = a.intensity + tAC * (c.intensity - a.intensity)
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
            .viewZ = b.viewZ + tBC * (c.viewZ - b.viewZ),
            .intensity = b.intensity + tBC * (c.intensity - b.intensity)
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
    
    f32 aInvZ = bin->aInvZ, auoz = bin->auoz, avoz = bin->avoz, aIntensity = bin->aIntensity;
    f32 bInvZ = bin->bInvZ, buoz = bin->buoz, bvoz = bin->bvoz, bIntensity = bin->bIntensity;
    f32 cInvZ = bin->cInvZ, cuoz = bin->cuoz, cvoz = bin->cvoz, cIntensity = bin->cIntensity;

    const SfrTexture* tex = bin->tex;
    
    // sort vertices by y coord
    if (ay > by) {
        SFR_SWAPF(ax, bx); SFR_SWAPF(ay, by); SFR_SWAPF(az, bz);
        SFR_SWAPF(aInvZ, bInvZ); SFR_SWAPF(auoz, buoz); SFR_SWAPF(avoz, bvoz);
        SFR_SWAPF(aIntensity, bIntensity);
    }
    if (ay > cy) {
        SFR_SWAPF(ax, cx); SFR_SWAPF(ay, cy); SFR_SWAPF(az, cz);
        SFR_SWAPF(aInvZ, cInvZ); SFR_SWAPF(auoz, cuoz); SFR_SWAPF(avoz, cvoz);
        SFR_SWAPF(aIntensity, cIntensity);
    }
    if (by > cy) {
        SFR_SWAPF(bx, cx); SFR_SWAPF(by, cy); SFR_SWAPF(bz, cz);
        SFR_SWAPF(bInvZ, cInvZ); SFR_SWAPF(buoz, cuoz); SFR_SWAPF(bvoz, cvoz);
        SFR_SWAPF(bIntensity, cIntensity);
    }

    const i32 texW = tex->w, texH = tex->h;
    const i32 texW1 = tex->w - 1, texH1 = tex->h - 1;

    // edge deltas for triangle
    const f32 deltaACX = cx - ax;
    const f32 deltaACZ = cz - az;
    const f32 deltaACinvZ = cInvZ - aInvZ;
    const f32 deltaACuoz = cuoz - auoz;
    const f32 deltaACvoz = cvoz - avoz;
    const f32 deltaACIntensity = cIntensity - aIntensity;
    const f32 invHeightAC = (cy != ay) ? 1.f / (cy - ay) : 0.f;

    { // lower triangle (A to B)
        const f32 deltaABX = bx - ax;
        const f32 deltaABZ = bz - az;
        const f32 deltaABinvZ = bInvZ - aInvZ;
        const f32 deltaABuoz = buoz - auoz;
        const f32 deltaABvoz = bvoz - avoz;
        const f32 deltaABIntensity = bIntensity - aIntensity;
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
            f32 sIntensity = aIntensity + deltaACIntensity * alpha;
    
            // interpolate along AB edge (end)
            f32 ex = ax + deltaABX * beta;
            f32 ez = az + deltaABZ * beta; // NDC z for depth
            f32 eInvZ = aInvZ + deltaABinvZ * beta;
            f32 eu = auoz + deltaABuoz * beta;
            f32 ev = avoz + deltaABvoz * beta;
            f32 eIntensity = aIntensity + deltaABIntensity * beta;
    
            // swap start/end if needed
            if (sx > ex) {
                SFR_SWAPF(sx, ex); SFR_SWAPF(sz, ez);
                SFR_SWAPF(sInvZ, eInvZ); SFR_SWAPF(su, eu); SFR_SWAPF(sv, ev);
                SFR_SWAPF(sIntensity, eIntensity);
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
            const f32 intensityStep = (eIntensity - sIntensity) * tStep;
    
            // initialize current values
            f32 depth = sz + (xStart - sx) * depthStep; // NDC z for depth testing
            f32 invZ = sInvZ + (xStart - sx) * invZStep;
            f32 uoz = su + (xStart - sx) * uStep;
            f32 voz = sv + (xStart - sx) * vStep;
            f32 intensity = sIntensity + (xStart - sx) * intensityStep;
    
            // rasterize scanline
            for (i32 x = xStart, i = y * sfrWidth + xStart; x < xEnd; x += 1, i += 1,
                invZ += invZStep, uoz += uStep, voz += vStep, depth += depthStep, intensity += intensityStep
            ) {
                if (depth > sfrBuffers->depth[i]) {
                    continue;
                }

                // perspective correction for texture
                const f32 zView = 1.f / invZ;

                // recover texture coords
                const f32 u = uoz * zView;
                const f32 v = voz * zView;
                
                // wrap texture coords
                const i32 tx = sfr_wrap_coord(u * texW1, texW);
                const i32 ty = sfr_wrap_coord(v * texH1, texH);
                
                const u32 texCol = tex->pixels[ty * texW + tx];
                const u8 tr = (texCol >> 16) & 0xFF;
                const u8 tg = (texCol >> 8)  & 0xFF;
                const u8 tb = (texCol >> 0)  & 0xFF;
                SFR_DIV255(fr, tr, cr);
                SFR_DIV255(fg, tg, cg);
                SFR_DIV255(fb, tb, cb);
                const u8 lr = (u8)(fr * intensity);
                const u8 lg = (u8)(fg * intensity);
                const u8 lb = (u8)(fb * intensity);

                #ifdef SFR_NO_ALPHA
                    sfrBuffers->pixel[i] = (0xFF << 24) | (lr << 16) | (lg << 8) | lb;
                    sfrBuffers->depth[i] = depth;
                #else
                    const u8 ta = (texCol >> 24) & 0xFF;
                    SFR_DIV255(fa, ta, ca);
                    if (0xFF != fa) {
                        const u8 currAlpha = sfrBuffers->accum[i].a;
                        SFR_DIV255(contribution, 255 - currAlpha, fa);

                        // accumulate premultiplied color components
                        SFR_DIV255(accumR, lr, contribution);
                        SFR_DIV255(accumG, lg, contribution);
                        SFR_DIV255(accumB, lb, contribution);
                        sfrBuffers->accum[i].r += accumR;
                        sfrBuffers->accum[i].g += accumG;
                        sfrBuffers->accum[i].b += accumB;

                        // update accumulated alpha
                        sfrBuffers->accum[i].a = currAlpha + contribution;
                        if (depth < sfrBuffers->accum[i].depth) {
                            sfrBuffers->accum[i].depth = depth;
                        }
                    } else {
                        sfrBuffers->pixel[i] = (0xFF << 24) | (lr << 16) | (lg << 8) | lb;
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
        const f32 deltaBCIntensity = cIntensity - bIntensity;
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
            f32 sIntensity = bIntensity + deltaBCIntensity * beta;
    
            // interpolate along AC edge (end)
            f32 ex = ax + deltaACX * alpha;
            f32 ez = az + deltaACZ * alpha; // NDC z for depth
            f32 eInvZ = aInvZ + deltaACinvZ * alpha;
            f32 euoz = auoz + deltaACuoz * alpha;
            f32 evoz = avoz + deltaACvoz * alpha;
            f32 eIntensity = aIntensity + deltaACIntensity * alpha;
    
            // swap start/end if needed
            if (sx > ex) {
                SFR_SWAPF(sx, ex); SFR_SWAPF(sz, ez);
                SFR_SWAPF(sInvZ, eInvZ); SFR_SWAPF(suoz, euoz); SFR_SWAPF(svoz, evoz);
                SFR_SWAPF(sIntensity, eIntensity);
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
            const f32 intensityStep = (eIntensity - sIntensity) * tStep;
    
            // initialize current values
            f32 depth = sz + (xStart - sx) * depthStep; // NDC z for depth testing
            f32 invZ = sInvZ + (xStart - sx) * invZStep;
            f32 uoz = suoz + (xStart - sx) * uStep;
            f32 voz = svoz + (xStart - sx) * vStep;
            f32 intensity = sIntensity + (xStart - sx) * intensityStep;
    
            // rasterize scanline
            for (i32 x = xStart, i = y * sfrWidth + xStart; x < xEnd; x += 1, i += 1,
                invZ += invZStep, uoz += uStep, voz += vStep, depth += depthStep, intensity += intensityStep
            ) {
                if (depth > sfrBuffers->depth[i]) {
                    continue;
                }

                // perspective correction for texture
                const f32 zView = 1.f / invZ;

                // recover texture coords
                const f32 u = uoz * zView;
                const f32 v = voz * zView;
                
                // wrap texture coords
                const i32 tx = sfr_wrap_coord(u * texW1, texW);
                const i32 ty = sfr_wrap_coord(v * texH1, texH);
                
                const u32 texCol = tex->pixels[ty * texW + tx];
                const u8 tr = (texCol >> 16) & 0xFF;
                const u8 tg = (texCol >> 8)  & 0xFF;
                const u8 tb = (texCol >> 0)  & 0xFF;
                SFR_DIV255(fr, tr, cr);
                SFR_DIV255(fg, tg, cg);
                SFR_DIV255(fb, tb, cb);
                const u8 lr = (u8)(fr * intensity);
                const u8 lg = (u8)(fg * intensity);
                const u8 lb = (u8)(fb * intensity);

                #ifdef SFR_NO_ALPHA
                    sfrBuffers->pixel[i] = (0xFF << 24) | (lr << 16) | (lg << 8) | lb;
                    sfrBuffers->depth[i] = depth;
                #else
                    const u8 ta = (texCol >> 24) & 0xFF;
                    SFR_DIV255(fa, ta, ca);
                    if (0xFF != fa) {
                        const u8 currAlpha = sfrBuffers->accum[i].a;
                        SFR_DIV255(contribution, 255 - currAlpha, fa);

                        // accumulate premultiplied color components
                        SFR_DIV255(accumR, lr, contribution);
                        SFR_DIV255(accumG, lg, contribution);
                        SFR_DIV255(accumB, lb, contribution);
                        sfrBuffers->accum[i].r += accumR;
                        sfrBuffers->accum[i].g += accumG;
                        sfrBuffers->accum[i].b += accumB;

                        // update accumulated alpha
                        sfrBuffers->accum[i].a = currAlpha + contribution;
                        if (depth < sfrBuffers->accum[i].depth) {
                            sfrBuffers->accum[i].depth = depth;
                        }
                    } else {
                        sfrBuffers->pixel[i] = (0xFF << 24) | (lr << 16) | (lg << 8) | lb;
                        sfrBuffers->depth[i] = depth;
                    }
                #endif
            }
        }
    }
}

SFR_FUNC void sfr__bin_triangle(
    f32 ax, f32 ay, f32 az, f32 aInvZ, f32 auoz, f32 avoz, f32 aIntensity,
    f32 bx, f32 by, f32 bz, f32 bInvZ, f32 buoz, f32 bvoz, f32 bIntensity,
    f32 cx, f32 cy, f32 cz, f32 cInvZ, f32 cuoz, f32 cvoz, f32 cIntensity,
    u32 col, const SfrTexture* tex
) {
    #ifdef SFR_MULTITHREADED
        // get a new triangle bin from the global pool
        const i32 binInd = sfr_atomic_add(&sfrBuffers->triangleBinAllocator, 1) - 1;
        if (binInd >= SFR_MAX_BINS_PER_FRAME) {
            return;
        }

        SfrTriangleBin* bin = &sfrBuffers->triangleBinPool[binInd];
        *bin = (SfrTriangleBin){
            ax, ay, az, aInvZ, auoz, avoz, aIntensity,
            bx, by, bz, bInvZ, buoz, bvoz, bIntensity,
            cx, cy, cz, cInvZ, cuoz, cvoz, cIntensity,
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

                if (0 == tileBinInd) {
                    const i32 workInd = sfr_atomic_add(&sfrBuffers->rasterWorkQueueCount, 1) - 1;
                    if (workInd < SFR_ARRLEN(sfrBuffers->rasterWorkQueue)) {
                        sfrBuffers->rasterWorkQueue[workInd] = tileInd;
                    }
                }
                tile->bins[tileBinInd] = bin;
            }
        }
    #else
        sfrRasterCount += 1;
        const SfrTile fullTile = {0, 0, sfrWidth, sfrHeight};
        const SfrTriangleBin bin = {
            ax, ay, az, aInvZ, auoz, avoz, aIntensity,
            bx, by, bz, bInvZ, buoz, bvoz, bIntensity,
            cx, cy, cz, cInvZ, cuoz, cvoz, cIntensity,
            col, tex
        };
        sfr_rasterize_bin(&bin, &fullTile);
    #endif
}

// core geometry pipeline for a single triangle
SFR_FUNC void sfr__process_and_bin_triangle(
    const sfrmat* model, const sfrmat* normalMat,
    f32 ax, f32 ay, f32 az, f32 au, f32 av, f32 anx, f32 any, f32 anz,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv, f32 bnx, f32 bny, f32 bnz,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv, f32 cnx, f32 cny, f32 cnz,
    u32 col, const SfrTexture* tex
) {
    // transform vertices to world space
    const sfrvec aModel = sfr_mat_mul_vec(*model, (sfrvec){ax, ay, az, 1.f});
    const sfrvec bModel = sfr_mat_mul_vec(*model, (sfrvec){bx, by, bz, 1.f});
    const sfrvec cModel = sfr_mat_mul_vec(*model, (sfrvec){cx, cy, cz, 1.f});

    // backface culling
    #ifndef SFR_NO_CULLING
        const sfrvec line0 = sfr_vec_sub(bModel, aModel);
        const sfrvec line1 = sfr_vec_sub(cModel, aModel);
        const sfrvec camRay = sfr_vec_sub(aModel, sfrCamPos);
        const sfrvec triNormal = sfr_vec_cross(line0, line1);
        if (sfr_vec_dot(triNormal, camRay) > 0.f) {
            return;
        }
    #endif

    // per vertex lighting calculation
    f32 aIntensity = 1.f, bIntensity = 1.f, cIntensity = 1.f;
    if (sfrState.lightingEnabled) {
        // transform normals
        const sfrvec nA = sfr_vec_norm(sfr_mat_mul_vec(*normalMat, (sfrvec){anx, any, anz, 0.f}));
        const sfrvec nB = sfr_vec_norm(sfr_mat_mul_vec(*normalMat, (sfrvec){bnx, bny, bnz, 0.f}));
        const sfrvec nC = sfr_vec_norm(sfr_mat_mul_vec(*normalMat, (sfrvec){cnx, cny, cnz, 0.f}));
        
        // intensity for each vertex
        aIntensity = sfr_fmaxf(sfrState.lightingAmbient, sfr_vec_dot(nA, sfrState.lightingDir));
        bIntensity = sfr_fmaxf(sfrState.lightingAmbient, sfr_vec_dot(nB, sfrState.lightingDir));
        cIntensity = sfr_fmaxf(sfrState.lightingAmbient, sfr_vec_dot(nC, sfrState.lightingDir));
    }

    // to view space
    SfrTexVert viewTri[3] = {
        {sfr_mat_mul_vec(sfrMatView, aModel), au, av, 0, aIntensity},
        {sfr_mat_mul_vec(sfrMatView, bModel), bu, bv, 0, bIntensity},
        {sfr_mat_mul_vec(sfrMatView, cModel), cu, cv, 0, cIntensity}
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
        clipTris[0][i].intensity = viewTri[i].intensity;
    }

    // check if clipping is needed at all
    const f32 guardBand = 1.2f; // 20 percent    
    u8 needsClipping = 0;
    for (i32 i = 0; i < 3; i += 1) {
        const sfrvec p = clipTris[0][i].pos;
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
    const sfrvec frustumPlanes[6] = {
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
    const SfrTexture* texToUse = tex ? tex : &sfrState.baseTex;
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
            const sfrvec ndc = sfr_vec_div(tri[j].pos, tri[j].pos.w);
            screen[j].pos.x =  (ndc.x + 1.f) * sfrState.halfWidth;
            screen[j].pos.y = (-ndc.y + 1.f) * sfrState.halfHeight;
            screen[j].pos.z = ndc.z;
            screen[j].u = tri[j].u;
            screen[j].v = tri[j].v;
            screen[j].viewZ = tri[j].viewZ;
            screen[j].intensity = tri[j].intensity;
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
            screen[0].pos.x, screen[0].pos.y, screen[0].pos.z, aInvZ, auoz, avoz, screen[0].intensity,
            screen[1].pos.x, screen[1].pos.y, screen[1].pos.z, bInvZ, buoz, bvoz, screen[1].intensity,
            screen[2].pos.x, screen[2].pos.y, screen[2].pos.z, cInvZ, cuoz, cvoz, screen[2].intensity,
            col, texToUse
        );
    }
}

// helper for updating normal mat used for shading
SFR_FUNC void sfr_update_normal_mat(void) {
    const f32* m = &sfrMatModel.m[0][0];
    
    const f32 a = m[0], b = m[4], c = m[8];
    const f32 d = m[1], e = m[5], f = m[9];
    const f32 g = m[2], h = m[6], i = m[10];

    const f32 det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    const f32 invDet = (0 != det) ? 1.f / det : 0.f;

    sfrState.matNormal.rows[0] = (sfrvec){ (e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet, 0.f };
    sfrState.matNormal.rows[1] = (sfrvec){ (f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet, 0.f };
    sfrState.matNormal.rows[2] = (sfrvec){ (d * h - e * g) * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet, 0.f };
    sfrState.matNormal.rows[3] = (sfrvec){ 0.f, 0.f, 0.f, 1.f };

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
        sfr_atomic_set(&sfrBuffers->rasterWorkQueueCount, 0);
        sfr_atomic_set(&sfrBuffers->rasterWorkQueueHead, 0);
        
        sfr_atomic_set(&sfrBuffers->meshJobAllocator, 0);
        sfr_atomic_set(&sfrBuffers->geometryWorkQueueCount, 0);
        sfr_atomic_set(&sfrBuffers->geometryWorkQueueHead, 0);

        sfr_semaphore_init(&sfrBuffers->geometryStartSem, 0);
        sfr_semaphore_init(&sfrBuffers->geometryDoneSem, 0);
        sfr_semaphore_init(&sfrBuffers->rasterStartSem, 0);
        sfr_semaphore_init(&sfrBuffers->rasterDoneSem, 0);

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
    sfrState.baseTex = (SfrTexture){ .w = 1, .h = 1, .pixels = sfrState.baseTexPixels };
}

#ifdef SFR_MULTITHREADED

SFR_FUNC void sfr_shutdown(void) {
    sfr_flush_and_wait();

    sfrState.shutdown = 1;
    // post to both semaphores to ensure threads wake up from either wait state
    sfr_semaphore_post(&sfrBuffers->geometryStartSem, SFR_THREAD_COUNT);
    sfr_semaphore_post(&sfrBuffers->rasterStartSem, SFR_THREAD_COUNT);

    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        #ifdef _WIN32
            WaitForSingleObject(sfrBuffers->threads[i].handle, INFINITE);
            CloseHandle(sfrBuffers->threads[i].handle);
        #else
            pthread_join(sfrBuffers->threads[i].handle, NULL);
        #endif
    }

    sfr_semaphore_destroy(&sfrBuffers->geometryStartSem);
    sfr_semaphore_destroy(&sfrBuffers->geometryDoneSem);
    sfr_semaphore_destroy(&sfrBuffers->rasterStartSem);
    sfr_semaphore_destroy(&sfrBuffers->rasterDoneSem);
}

// dispatches jobs to workers and waits for them to complete
SFR_FUNC void sfr_flush_and_wait(void) {
    // always dispatch geometry phase to keep workers in sync
    sfr_semaphore_post(&sfrBuffers->geometryStartSem, SFR_THREAD_COUNT);
    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        sfr_semaphore_wait(&sfrBuffers->geometryDoneSem);
    }

    // reset geometry queue
    sfr_atomic_set(&sfrBuffers->meshJobAllocator, 0);
    sfr_atomic_set(&sfrBuffers->geometryWorkQueueCount, 0);
    sfr_atomic_set(&sfrBuffers->geometryWorkQueueHead, 0);

    // always dispatch rasterization phase
    sfr_semaphore_post(&sfrBuffers->rasterStartSem, SFR_THREAD_COUNT);
    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        sfr_semaphore_wait(&sfrBuffers->rasterDoneSem);
    }

    // reset rasterization data
    sfr_atomic_set(&sfrBuffers->triangleBinAllocator, 0);
    sfr_atomic_set(&sfrBuffers->rasterWorkQueueCount, 0);
    sfr_atomic_set(&sfrBuffers->rasterWorkQueueHead, 0);
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
    sfrState.clipPlanes[0][0] = (sfrvec){0.f, 0.5f, 0.f, 1.f};
    sfrState.clipPlanes[0][1] = (sfrvec){0.f, 1.f,  0.f, 1.f};
    
    // bottom
    sfrState.clipPlanes[1][0] = (sfrvec){0.f, height, 0.f, 1.f};
    sfrState.clipPlanes[1][1] = (sfrvec){0.f, -1.f,   0.f, 1.f};

    // left
    sfrState.clipPlanes[2][0] = (sfrvec){0.5f, 0.f, 0.f, 1.f};
    sfrState.clipPlanes[2][1] = (sfrvec){1.f,  0.f, 0.f, 1.f};
    
    // right
    sfrState.clipPlanes[3][0] = (sfrvec){width, 0.f, 0.f, 1.f};
    sfrState.clipPlanes[3][1] = (sfrvec){-1.f,  0.f, 0.f, 1.f};

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
    const sfrmat rot = sfr_mat_rot_x(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_rotate_y(f32 theta) {
    const sfrmat rot = sfr_mat_rot_y(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_rotate_z(f32 theta) {
    const sfrmat rot = sfr_mat_rot_z(theta);
    sfrMatModel = sfr_mat_mul(sfrMatModel, rot);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_translate(f32 x, f32 y, f32 z) {
    const sfrmat trans = sfr_mat_translate(x, y, z);
    sfrMatModel = sfr_mat_mul(sfrMatModel, trans);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_scale(f32 x, f32 y, f32 z) {
    const sfrmat scale = sfr_mat_scale(x, y, z);
    sfrMatModel = sfr_mat_mul(sfrMatModel, scale);
    sfrState.normalMatDirty = 1;
}

SFR_FUNC void sfr_look_at(f32 x, f32 y, f32 z) {
    const sfrvec up = {0.f, 1.f, 0.f, 1.f};
    const sfrmat view = sfr_mat_look_at(sfrCamPos, (sfrvec){x, y, z, 1.f}, up);
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
    f32 ax, f32 ay, f32 az, f32 anx, f32 any, f32 anz,
    f32 bx, f32 by, f32 bz, f32 bnx, f32 bny, f32 bnz,
    f32 cx, f32 cy, f32 cz, f32 cnx, f32 cny, f32 cnz,
    u32 col
) {
    sfr_triangle_tex(
        ax, ay, az, 0.f, 0.f, anx, any, anz,
        bx, by, bz, 0.f, 0.f, bnx, bny, bnz,
        cx, cy, cz, 0.f, 0.f, cnx, cny, cnz,
        col, &sfrState.baseTex);
}

SFR_FUNC void sfr_triangle_tex(
    f32 ax, f32 ay, f32 az, f32 au, f32 av, f32 anx, f32 any, f32 anz,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv, f32 bnx, f32 bny, f32 bnz,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv, f32 cnx, f32 cny, f32 cnz,
    u32 col, const SfrTexture* tex
) {
    if (sfrState.normalMatDirty) {
        sfr_update_normal_mat();
    }

    sfr__process_and_bin_triangle(
        &sfrMatModel, &sfrState.matNormal,
        ax, ay, az, au, av, anx, any, anz,
        bx, by, bz, bu, bv, bnx, bny, bnz,
        cx, cy, cz, cu, cv, cnx, cny, cnz,
        col, tex
    );
}

SFR_FUNC void sfr_billboard(u32 col, const SfrTexture* tex) {
    const sfrmat savedModel = sfrMatModel;
    sfrMatModel = sfr_mat_identity();

    const sfrvec center = { savedModel.m[3][0], savedModel.m[3][1], savedModel.m[3][2], 1.f };

    const f32 sx = 0.5f * sfr_sqrtf(
        savedModel.m[0][0] * savedModel.m[0][0] + 
        savedModel.m[0][1] * savedModel.m[0][1] + 
        savedModel.m[0][2] * savedModel.m[0][2]);
    const f32 sy = 0.5f * sfr_sqrtf(
        savedModel.m[1][0] * savedModel.m[1][0] + 
        savedModel.m[1][1] * savedModel.m[1][1] + 
        savedModel.m[1][2] * savedModel.m[1][2]);

    const sfrvec right = { sx * sfrMatView.m[0][0], sx * sfrMatView.m[1][0], sx * sfrMatView.m[2][0], 0.f };
    const sfrvec up    = { sy * sfrMatView.m[0][1], sy * sfrMatView.m[1][1], sy * sfrMatView.m[2][1], 0.f };

    const sfrvec a = sfr_vec_add(center, sfr_vec_add(sfr_vec_mul(right, -1.f), sfr_vec_mul(up, -1.f)));
    const sfrvec b = sfr_vec_add(center, sfr_vec_add(sfr_vec_mul(right,  1.f), sfr_vec_mul(up, -1.f)));
    const sfrvec c = sfr_vec_add(center, sfr_vec_add(sfr_vec_mul(right,  1.f), sfr_vec_mul(up,  1.f)));
    const sfrvec d = sfr_vec_add(center, sfr_vec_add(sfr_vec_mul(right, -1.f), sfr_vec_mul(up,  1.f)));

    const sfrvec normal = sfr_vec_norm(sfr_vec_sub(sfrCamPos, center));

    sfr_triangle_tex(
        a.x, a.y, a.z, 0.f, 1.f, normal.x, normal.y, normal.z,
        c.x, c.y, c.z, 1.f, 0.f, normal.x, normal.y, normal.z,
        b.x, b.y, b.z, 1.f, 1.f, normal.x, normal.y, normal.z,
        col, tex);
    sfr_triangle_tex(
        a.x, a.y, a.z, 0.f, 1.f, normal.x, normal.y, normal.z,
        d.x, d.y, d.z, 0.f, 0.f, normal.x, normal.y, normal.z,
        c.x, c.y, c.z, 1.f, 0.f, normal.x, normal.y, normal.z,
        col, tex);

    sfrMatModel = savedModel;
}

SFR_FUNC void sfr_cube(u32 col, const SfrTexture* tex) {
    // front face
    sfr_triangle_tex(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1, col, tex);
    sfr_triangle_tex(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,0,-1, col, tex);
    // right face
    sfr_triangle_tex(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,1.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0, col, tex);
    sfr_triangle_tex(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0,
         0.5f,-0.5f, 0.5f, 0.f,0.f, 1,0,0, col, tex);
    // back face
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
         0.5f, 0.5f, 0.5f, 1.f,1.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1, col, tex);
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1,
        -0.5f,-0.5f, 0.5f, 0.f,0.f, 0,0,1, col, tex);
    // left face
    sfr_triangle_tex(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0, col, tex);
    sfr_triangle_tex(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0,
        -0.5f,-0.5f,-0.5f, 0.f,0.f, -1,0,0, col, tex);
    // top face
    sfr_triangle_tex(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0, col, tex);
    sfr_triangle_tex(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0,
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,1,0, col, tex);
    // bottom face
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0, col, tex);
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,-1,0, col, tex);
}

SFR_FUNC void sfr_cube_ex(u32 col[12]) {
    // front face
    sfr_triangle_tex(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1, col[0], &sfrState.baseTex);
    sfr_triangle_tex(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,0,-1, col[1], &sfrState.baseTex);
    // right face
    sfr_triangle_tex(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,1.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0, col[2], &sfrState.baseTex);
    sfr_triangle_tex(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0,
         0.5f,-0.5f, 0.5f, 0.f,0.f, 1,0,0, col[3], &sfrState.baseTex);
    // back face
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
         0.5f, 0.5f, 0.5f, 1.f,1.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1, col[4], &sfrState.baseTex);
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1,
        -0.5f,-0.5f, 0.5f, 0.f,0.f, 0,0,1, col[5], &sfrState.baseTex);
    // left face
    sfr_triangle_tex(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0, col[6], &sfrState.baseTex);
    sfr_triangle_tex(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0,
        -0.5f,-0.5f,-0.5f, 0.f,0.f, -1,0,0, col[7], &sfrState.baseTex);
    // top face
    sfr_triangle_tex(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0, col[8], &sfrState.baseTex);
    sfr_triangle_tex(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0,
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,1,0, col[9], &sfrState.baseTex);
    // bottom face
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0, col[10], &sfrState.baseTex);
    sfr_triangle_tex(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,-1,0, col[11], &sfrState.baseTex);
}

SFR_FUNC void sfr_mesh(const SfrMesh* mesh, u32 col, const SfrTexture* tex) {
    const i32 triangleCount = mesh->vertCount / 9;
    if (0 == triangleCount) {
        return;
    }

    if (sfrState.normalMatDirty) {
        sfr_update_normal_mat();
    }

    #ifdef SFR_MULTITHREADED
        // divide the mesh into jobs
        for (i32 i = 0; i < triangleCount; i += SFR_GEOMETRY_JOB_SIZE) {
            const i32 jobInd = sfr_atomic_add(&sfrBuffers->meshJobAllocator, 1) - 1;
            if (jobInd >= SFR_MAX_GEOMETRY_JOBS) {
                // not enough job slots,
                // process remaining triangles serially so they aren't just dropped
                sfr_flush_and_wait();
                const i32 remaining = triangleCount - i;
                for (int j = 0; j < remaining; j += 1) {
                    const i32 triInd = (i + j) * 9;
                    const i32 uvInd = (i + j) * 6;
                     sfr__process_and_bin_triangle(
                        &sfrMatModel, &sfrState.matNormal,
                        mesh->tris[triInd + 0], mesh->tris[triInd + 1], mesh->tris[triInd + 2],
                        mesh->uvs ? mesh->uvs[uvInd + 0] : 0.f, mesh->uvs ? mesh->uvs[uvInd + 1] : 0.f,
                        mesh->normals[triInd + 0], mesh->normals[triInd + 1], mesh->normals[triInd + 2],

                        mesh->tris[triInd + 3], mesh->tris[triInd + 4], mesh->tris[triInd + 5],
                        mesh->uvs ? mesh->uvs[uvInd + 2] : 0.f, mesh->uvs ? mesh->uvs[uvInd + 3] : 0.f,
                        mesh->normals[triInd + 3], mesh->normals[triInd + 4], mesh->normals[triInd + 5],
                        
                        mesh->tris[triInd + 6], mesh->tris[triInd + 7], mesh->tris[triInd + 8],
                        mesh->uvs ? mesh->uvs[uvInd + 4] : 0.f, mesh->uvs ? mesh->uvs[uvInd + 5] : 0.f,
                        mesh->normals[triInd + 6], mesh->normals[triInd + 7], mesh->normals[triInd + 8],
                        col, tex
                    );
                }
                return;
            }

            // create and populate the job
            SfrMeshChunkJob* job = &sfrBuffers->meshJobPool[jobInd];
            job->mesh = mesh;
            job->matModel = sfrMatModel;
            job->matNormal = sfrState.matNormal;
            job->col = col;
            job->tex = tex;
            job->startTriangle = i;
            job->triangleCount = SFR_MIN(SFR_GEOMETRY_JOB_SIZE, triangleCount - i);

            // add job to the queue
            const i32 queueInd = sfr_atomic_add(&sfrBuffers->geometryWorkQueueCount, 1) - 1;
            if (queueInd < SFR_MAX_GEOMETRY_JOBS) {
                sfrBuffers->geometryWorkQueue[queueInd] = jobInd;
            }
        }
    #else
        for (i32 i = 0; i < mesh->vertCount; i += 9) {
            const i32 uvInd = (i / 9) * 6;
            sfr__process_and_bin_triangle(
                &sfrMatModel, &sfrState.matNormal,
                mesh->tris[i + 0], mesh->tris[i + 1], mesh->tris[i + 2],
                mesh->uvs ? mesh->uvs[uvInd + 0] : 0.f, mesh->uvs ? mesh->uvs[uvInd + 1] : 0.f,
                mesh->normals[i + 0], mesh->normals[i + 1], mesh->normals[i + 2],

                mesh->tris[i + 3], mesh->tris[i + 4], mesh->tris[i + 5],
                mesh->uvs ? mesh->uvs[uvInd + 2] : 0.f, mesh->uvs ? mesh->uvs[uvInd + 3] : 0.f,
                mesh->normals[i + 3], mesh->normals[i + 4], mesh->normals[i + 5],
                
                mesh->tris[i + 6], mesh->tris[i + 7], mesh->tris[i + 8],
                mesh->uvs ? mesh->uvs[uvInd + 4] : 0.f, mesh->uvs ? mesh->uvs[uvInd + 5] : 0.f,
                mesh->normals[i + 6], mesh->normals[i + 7], mesh->normals[i + 8],
                col, tex
            );
        }
    #endif
}

SFR_FUNC void sfr_string(const SfrFont* font, const char* s, i32 sLength, u32 col) {
    SFR_ERR_RET(, "sfr_string: TODO not implemented, 'sfr_glyph' is implemented\n");
}

SFR_FUNC void sfr_glyph(const SfrFont* font, u16 id, u32 col) {
    if (id >= SFR_FONT_GLYPH_MAX) {
        SFR_ERR_RET(, "sfr_glyph: invalid id (%d >= %d)\n", id, SFR_FONT_GLYPH_MAX);
    }

    const f32* t = font->verts[id];
    for (i32 i = 0; i < SFR_FONT_VERT_MAX; i += 6) {
        if (SFR_FONT_VERT_EMPTY == font->verts[id][i]) {
            break;
        }

        sfr_triangle(
            t[i + 0], t[i + 1], 0.f, 0,0,1,
            t[i + 2], t[i + 3], 0.f, 0,0,1,
            t[i + 4], t[i + 5], 0.f, 0,0,1,
            col);
    }
}

SFR_FUNC i32 sfr_world_to_screen(f32 x, f32 y, f32 z, i32* screenX, i32* screenY) {
    sfrvec p = {x, y, z, 1.f};
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
    sfrCamPos = (sfrvec){x, y, z, 1.f};
    sfrvec up = {0.f, 1.f, 0.f, 1.f};
    sfrvec target = {0.f, 0.f, 1.f, 1.f};

    const sfrmat rotX = sfr_mat_rot_x(pitch);
    const sfrmat rotY = sfr_mat_rot_y(yaw);
    const sfrmat rotZ = sfr_mat_rot_z(roll);

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

SFR_FUNC void sfr_set_lighting(i32 on, sfrvec dir, f32 ambientIntensity) {
    sfrState.lightingEnabled = on;
    sfrState.lightingDir = sfr_vec_norm(dir);
    sfrState.lightingAmbient = ambientIntensity;
}

#ifndef SFR_NO_STD

SFR_FUNC SfrMesh* sfr_load_mesh(const char* filename) {
    SfrMesh* mesh = (SfrMesh*)malloc(sizeof(SfrMesh));
    if (!mesh) {
        SFR_ERR_RET(NULL, "sfr_load_mesh: failed to allocate SfrMesh struct\n");
    }
    *mesh = (SfrMesh){.tris = NULL, .uvs = NULL, .normals = NULL, .vertCount = 0};

    FILE* objFile = fopen(filename, "rb");
    if (!objFile) {
        free(mesh);
        SFR_ERR_RET(NULL, "sfr_load_mesh: failed to open file '%s'\n", filename);
    }

    // read all raw data into temporary dynamic arrays
    // TODO make dynamic array macros
    sfrvec* tempVerts = NULL; i32 tempVertsCap = 0; i32 vertCount = 0;
    sfrvec* tempUVs = NULL; i32 tempUVsCap = 0; i32 uvCount = 0;
    sfrvec* tempNormals = NULL; i32 tempNormalsCap = 0; i32 normalCount = 0;
    
    // store face data as indices
    typedef struct { i32 v[4], vt[4], vn[4], count; } FaceIndices;
    FaceIndices* tempFaces = NULL; i32 tempFacesCap = 0; i32 faceCount = 0;

    // TODO handle failing and free
    char line[256];
    while (fgets(line, sizeof(line), objFile)) {
        if ('v' == line[0]) {
            if (' ' == line[1]) { // vertex position
                if (vertCount >= tempVertsCap) {
                    tempVertsCap = (0 == tempVertsCap) ? 128 : tempVertsCap * 2;
                    tempVerts = (sfrvec*)realloc(tempVerts, tempVertsCap * sizeof(sfrvec));
                    if (!tempVerts) {
                        SFR_ERR_EXIT("sfr_load_mesh: realloc failed for tempVerts\n");
                    }
                }
                sscanf(line, "v %f %f %f", &tempVerts[vertCount].x, &tempVerts[vertCount].y, &tempVerts[vertCount].z);
                vertCount += 1;
            } else if ('t' == line[1]) { // texture coordinate
                if (uvCount >= tempUVsCap) {
                    tempUVsCap = (0 == tempUVsCap) ? 128 : tempUVsCap * 2;
                    tempUVs = (sfrvec*)realloc(tempUVs, tempUVsCap * sizeof(sfrvec));
                    if (!tempUVs) {
                        SFR_ERR_EXIT("sfr_load_mesh: realloc failed for tempUVs\n");
                    }
                }
                sscanf(line, "vt %f %f", &tempUVs[uvCount].x, &tempUVs[uvCount].y);
                uvCount += 1;
            } else if ('n' == line[1]) { // vertex normal
                if (normalCount >= tempNormalsCap) {
                    tempNormalsCap = (0 == tempNormalsCap) ? 128 : tempNormalsCap * 2;
                    tempNormals = (sfrvec*)realloc(tempNormals, tempNormalsCap * sizeof(sfrvec));
                    if (!tempNormals) {
                        SFR_ERR_EXIT("sfr_load_mesh: realloc failed for tempNormals\n");
                    }
                }
                sscanf(line, "vn %f %f %f", &tempNormals[normalCount].x, &tempNormals[normalCount].y, &tempNormals[normalCount].z);
                normalCount += 1;
            }
        } else if ('f' == line[0]) { // face
            if (faceCount >= tempFacesCap) {
                tempFacesCap = (0 == tempFacesCap) ? 128 : tempFacesCap * 2;
                tempFaces = (FaceIndices*)realloc(tempFaces, tempFacesCap * sizeof(FaceIndices));
                if (!tempFaces) {
                    SFR_ERR_EXIT("sfr_load_mesh: realloc failed for tempFaces\n");
                }
            }
            FaceIndices* f = &tempFaces[faceCount];
            f->count = 0;
            char* token = sfr_strtok(line + 1, " \t\n");
            while (token && f->count < 4) {
                f->v[f->count] = f->vt[f->count] = f->vn[f->count] = 0;
                if (sscanf(token, "%d/%d/%d", &f->v[f->count], &f->vt[f->count], &f->vn[f->count]) != 3) {
                    if (sscanf(token, "%d//%d", &f->v[f->count], &f->vn[f->count]) != 2) {
                        if (sscanf(token, "%d/%d", &f->v[f->count], &f->vt[f->count]) != 2) {
                            sscanf(token, "%d", &f->v[f->count]);
                        }
                    }
                }
                f->v[f->count] -= 1;
                f->vt[f->count] -= 1;
                f->vn[f->count] -= 1;
                token = sfr_strtok(NULL, " \t\n");
                f->count += 1;
            }
            faceCount += 1;
        }
    }
    fclose(objFile);

    // process faces and build final mesh data
    i32 finalVertCount = 0;
    for (i32 i = 0; i < faceCount; i += 1) {
        finalVertCount += (3 == tempFaces[i].count) ? 3 : 6;
    }

    mesh->vertCount = finalVertCount * 3;
    mesh->tris = (f32*)malloc(mesh->vertCount * sizeof(f32));
    mesh->uvs = (f32*)malloc(finalVertCount * 2 * sizeof(f32));
    mesh->normals = (f32*)malloc(mesh->vertCount * sizeof(f32));

    if (!mesh->tris || !mesh->uvs || !mesh->normals) {
        SFR_ERR_EXIT("sfr_load_mesh: failed to allocate final mesh buffers\n");
    }

    sfrvec* computedNormals = NULL;
    if (0 == normalCount && vertCount > 0) { // if no normals in file
        computedNormals = (sfrvec*)calloc(vertCount, sizeof(sfrvec));
        if (!computedNormals) {
            SFR_ERR_EXIT("sfr_load_mesh: calloc failed for computedNormals\n");
        }
        for (i32 i = 0; i < faceCount; i += 1) {
            const FaceIndices* f = &tempFaces[i];
            const sfrvec v0 = tempVerts[f->v[0]];
            const sfrvec v1 = tempVerts[f->v[1]];
            const sfrvec v2 = tempVerts[f->v[2]];
            const sfrvec faceNormal = sfr_vec_face_normal(v0, v1, v2);
            for (i32 j = 0; j < f->count; j += 1) {
                computedNormals[f->v[j]] = sfr_vec_add(computedNormals[f->v[j]], faceNormal);
            }
        }
        for (i32 i = 0; i < vertCount; i += 1) {
            computedNormals[i] = sfr_vec_norm(computedNormals[i]);
        }
    }
    
    i32 triInd = 0;
    for (i32 i = 0; i < faceCount; i += 1) {
        const FaceIndices* f = &tempFaces[i];
        const i32 indices[6] = {0, 1, 2, 0, 2, 3};
        const i32 n = (f->count < 4) ? 1 : 2;
        for (i32 t = 0; t < n; t += 1) {
            for (i32 v = 0; v < 3; v += 1) {
                const i32 ind = indices[t * 3 + v];
                const i32 vInd = f->v[ind];
                const i32 vtInd = f->vt[ind];
                const i32 vnInd = f->vn[ind];
                
                // position
                mesh->tris[triInd * 9 + v * 3 + 0] = tempVerts[vInd].x;
                mesh->tris[triInd * 9 + v * 3 + 1] = tempVerts[vInd].y;
                mesh->tris[triInd * 9 + v * 3 + 2] = tempVerts[vInd].z;
                
                // uvs
                if (vtInd >= 0 && vtInd < uvCount) {
                    mesh->uvs[triInd * 6 + v * 2 + 0] = tempUVs[vtInd].x;
                    mesh->uvs[triInd * 6 + v * 2 + 1] = 1.f - tempUVs[vtInd].y;
                } else {
                    mesh->uvs[triInd * 6 + v * 2 + 0] = 0;
                    mesh->uvs[triInd * 6 + v * 2 + 1] = 0;
                }

                // normals
                if (vnInd >= 0 && vnInd < normalCount) {
                    mesh->normals[triInd * 9 + v * 3 + 0] = tempNormals[vnInd].x;
                    mesh->normals[triInd * 9 + v * 3 + 1] = tempNormals[vnInd].y;
                    mesh->normals[triInd * 9 + v * 3 + 2] = tempNormals[vnInd].z;
                } else if (computedNormals) {
                    mesh->normals[triInd * 9 + v * 3 + 0] = computedNormals[vInd].x;
                    mesh->normals[triInd * 9 + v * 3 + 1] = computedNormals[vInd].y;
                    mesh->normals[triInd * 9 + v * 3 + 2] = computedNormals[vInd].z;
                } else { // fallback
                    mesh->normals[triInd * 9 + v * 3 + 0] = 0;
                    mesh->normals[triInd * 9 + v * 3 + 1] = 1;
                    mesh->normals[triInd * 9 + v * 3 + 2] = 0;
                }
            }
            triInd += 1;
        }
    }

    // free temporary memory
    free(tempVerts);
    free(tempUVs);
    free(tempNormals);
    free(tempFaces);
    free(computedNormals);

    return mesh;
}

SFR_FUNC void sfr_release_mesh(SfrMesh** mesh) {
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

    if ((*mesh)->normals) {
        free((*mesh)->normals);
        (*mesh)->normals = NULL;
    }

    free(*mesh);
    *mesh = NULL;
}

SFR_FUNC SfrTexture* sfr_load_texture(const char* filename) {
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
    SfrTexture* tex = (SfrTexture*)malloc(sizeof(SfrTexture));
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

SFR_FUNC void sfr_release_texture(SfrTexture** tex) {
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

SFR_FUNC SfrFont* sfr_load_font(const char* filename) {
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
    SfrFont* font = (SfrFont*)malloc(sizeof(SfrFont));
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

SFR_FUNC void sfr_release_font(SfrFont** font) {
    if (!font || !(*font)) {
        return;
    }

    free(*font);
    *font = NULL;
}

#endif // !SFR_NO_STD

SFR_FUNC SfrParticleSystem sfr_particles_create(SfrParticle* buffer, i32 count, const SfrTexture* tex) {
    return (SfrParticleSystem){
        .particles = buffer,
        .active = 0, .total = count,
        .tex = tex
    };
}

SFR_FUNC void sfr_particles_update(SfrParticleSystem* sys, f32 frameTime) {
    for (i32 i = 0; i < sys->active;) {
        SfrParticle* p = &sys->particles[i];
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

SFR_FUNC void sfr_particles_draw(const SfrParticleSystem* sys) {
    for (i32 i = 0; i < sys->active; i += 1) {
        SfrParticle* p = &sys->particles[i];

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

SFR_FUNC void sfr_particles_emit(SfrParticleSystem* sys,
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
    SfrParticle* p = &sys->particles[sys->active++];
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
        // vv geometry phase vv
        sfr_semaphore_wait(&sfrBuffers->geometryStartSem);
        if (sfrState.shutdown) {
            break;
        }
        
        while (1) {
            // get the next geometry job from the queue
            const i32 head = sfr_atomic_add(&sfrBuffers->geometryWorkQueueHead, 1) - 1;
            if (head >= sfr_atomic_get(&sfrBuffers->geometryWorkQueueCount)) {
                break; // no more geometry work
            }

            const i32 jobInd = sfrBuffers->geometryWorkQueue[head];
            const SfrMeshChunkJob* job = &sfrBuffers->meshJobPool[jobInd];

            // process all triangles in this job
            for (i32 i = 0; i < job->triangleCount; i += 1) {
                const i32 triInd = (job->startTriangle + i) * 9;
                const i32 uvInd = (job->startTriangle + i) * 6;
                sfr__process_and_bin_triangle(
                    &job->matModel, &job->matNormal,
                    job->mesh->tris[triInd + 0], job->mesh->tris[triInd + 1], job->mesh->tris[triInd + 2],
                    job->mesh->uvs ? job->mesh->uvs[uvInd + 0] : 0.f,
                    job->mesh->uvs ? job->mesh->uvs[uvInd + 1] : 0.f,
                    job->mesh->normals[triInd + 0], job->mesh->normals[triInd + 1], job->mesh->normals[triInd + 2],

                    job->mesh->tris[triInd + 3], job->mesh->tris[triInd + 4], job->mesh->tris[triInd + 5],
                    job->mesh->uvs ? job->mesh->uvs[uvInd + 2] : 0.f,
                    job->mesh->uvs ? job->mesh->uvs[uvInd + 3] : 0.f,
                    job->mesh->normals[triInd + 3], job->mesh->normals[triInd + 4], job->mesh->normals[triInd + 5],
                    
                    job->mesh->tris[triInd + 6], job->mesh->tris[triInd + 7], job->mesh->tris[triInd + 8],
                    job->mesh->uvs ? job->mesh->uvs[uvInd + 4] : 0.f,
                    job->mesh->uvs ? job->mesh->uvs[uvInd + 5] : 0.f,
                    job->mesh->normals[triInd + 6], job->mesh->normals[triInd + 7], job->mesh->normals[triInd + 8],
                    job->col, job->tex
                );
            }
        }
        sfr_semaphore_post(&sfrBuffers->geometryDoneSem, 1);

        // vv rasterization phase vv
        sfr_semaphore_wait(&sfrBuffers->rasterStartSem);
        if (sfrState.shutdown) {
            break;
        }

        while (1) {
            // get the next tile index from the work queue
            const i32 head = sfr_atomic_add(&sfrBuffers->rasterWorkQueueHead, 1) - 1;
            if (head >= sfr_atomic_get(&sfrBuffers->rasterWorkQueueCount)) {
                break; // no more raster work
            }

            // get the tile and rasterize its contents
            SfrTile* tile = &sfrBuffers->tiles[sfrBuffers->rasterWorkQueue[head]];
            i32 binCount = sfr_atomic_get(&tile->binCount);
            binCount = SFR_MIN(binCount, SFR_MAX_BINS_PER_TILE);
            for (i32 i = 0; i < binCount; i += 1) {
                sfr_rasterize_bin(tile->bins[i], tile);
            }

            sfr_atomic_set(&tile->binCount, 0);
        }
        sfr_semaphore_post(&sfrBuffers->rasterDoneSem, 1);
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
