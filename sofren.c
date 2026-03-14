#ifndef SFR_H
#define SFR_H

#ifdef __cplusplus
extern "C" {
#endif

//================================================
//: PUBLIC API
//================================================

#ifndef SFR_MAX_WIDTH
    #define SFR_MAX_WIDTH 1280
#endif
#ifndef SFR_MAX_HEIGHT
    #define SFR_MAX_HEIGHT 720
#endif

//: threading config
#ifdef SFR_THREAD_COUNT
    #if SFR_THREAD_COUNT < 1 || SFR_THREAD_COUNT > 32
        #error "SFR ERROR: SFR_THREAD_COUNT must be between 1 and 32 (32 arbitrary)"
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
    #define SFR_TILE_DIM (SFR_TILE_WIDTH * SFR_TILE_HEIGHT)
    #ifndef SFR_GEOMETRY_JOB_SIZE
        // number of triangles per geometry job
        #define SFR_GEOMETRY_JOB_SIZE 64
    #endif
    #ifndef SFR_BIN_PAGE_SIZE
        // number of triangles per page
        #define SFR_BIN_PAGE_SIZE 4096
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
        #include <errno.h>
        typedef pthread_t SfrThread;
        typedef sem_t SfrSemaphore;
        typedef pthread_mutex_t SfrMutex;
        typedef volatile int SfrAtomic32;
        typedef volatile long long SfrAtomic64;
    #endif

    #ifndef SFR_THREAD_LOCAL
        #if defined(_MSC_VER)
            #define SFR_THREAD_LOCAL __declspec(thread)
        #elif defined(__GNUC__) || defined(__clang__)
            #define SFR_THREAD_LOCAL __thread
        #else
            #define SFR_THREAD_LOCAL _Thread_local
        #endif
    #endif
#else
    #define SfrAtomic32 i32
    #define SfrAtomic64 i64
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

typedef union sfrvec sfrvec;
typedef union sfrmat sfrmat;

typedef struct sfrmesh SfrMesh;
typedef struct sfrtex  SfrTexture;
typedef struct sfrfont SfrFont;

typedef struct sfrScene SfrScene;
typedef struct sfrSceneObject SfrSceneObject;

typedef struct sfrRayHit SfrRayHit;

typedef struct sfrParticle  SfrParticle;
typedef struct sfrParticles SfrParticleSystem;

typedef struct sfrlight SfrLight;

#ifdef SFR_USE_CGLTF
    typedef struct sfrModel SfrModel;
#endif

//: extern variables
extern i32 sfrWidth, sfrHeight;

extern u32* sfrPixelBuf;
extern f32* sfrDepthBuf;
extern SfrLight sfrLight;

// variables below can be managed by you, however
// there is probably a function that will do what you want

extern SfrAtomic32 sfrRasterCount; // how many triangles have been rasterized since the last call to clear

extern sfrmat sfrMatModel, sfrMatView, sfrMatProj;
extern sfrvec sfrCamPos, sfrCamUp, sfrCamTarget;
extern f32 sfrCamFov;
extern f32 sfrNearDist, sfrFarDist;

#if defined(SFR_FUNC) && defined(SFR_USE_INLINE) && !defined(SFR_NO_WARNINGS)
    #warning "SFR WARNING: SFR_FUNC and SFR_USE_INLINE both being defined is contradictory, using SFR_FUNC"
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
SFR_FUNC sfrvec sfr_vec_lerp(sfrvec a, sfrvec b, f32 t);
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
SFR_FUNC void sfr_mat_decompose(sfrmat m, sfrvec* pos, sfrvec* rot, sfrvec* scale);
SFR_FUNC sfrmat sfr_mat_from_quat(sfrvec q);
SFR_FUNC sfrvec sfr_quat_mul(sfrvec a, sfrvec b);
SFR_FUNC sfrvec sfr_quat_slerp(sfrvec a, sfrvec b, f32 t);

//: core functions
SFR_FUNC void sfr_init(i32 w, i32 h, f32 fovDeg,
    void* (*mallocFunc)(u64), void (*freeFunc)(void*), void* (*reallocFunc)(void*, u64));
SFR_FUNC void sfr_release(void);

// finish rendering triangles, must be called for any thread count
SFR_FUNC void sfr_flush_and_wait(void);

SFR_FUNC void sfr_resize(i32 width, i32 height);  // resize internals to new dimensions

SFR_FUNC void sfr_reset(void);                    // reset model matrix to identity
SFR_FUNC void sfr_rotate_x(f32 theta);            // rotate model matrix about x by theta radians
SFR_FUNC void sfr_rotate_y(f32 theta);            // rotate model matrix about y by theta radians
SFR_FUNC void sfr_rotate_z(f32 theta);            // rotate model matrix about z by theta radians
SFR_FUNC void sfr_translate(f32 x, f32 y, f32 z); // translate model matrix by x y z
SFR_FUNC void sfr_scale(f32 x, f32 y, f32 z);     // scale model matrix by x y z
SFR_FUNC void sfr_look_at(f32 x, f32 y, f32 z);   // set view matrix to look at x y z

// clear all buffers to default states
SFR_FUNC void sfr_clear(u32 clearCol);
SFR_FUNC void sfr_clear_depth(void);

// texture order: front, right, back, left, top, bottom
SFR_FUNC void sfr_skybox(const SfrTexture* faces[6]);

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
    u32 col, const SfrTexture* tex);

// other drawing functions, if tex is null sfrState.baseTex (white 1x1 texture) will be used
SFR_FUNC void sfr_point(f32 worldX, f32 worldY, f32 worldZ, i32 radius, u32 col);
SFR_FUNC void sfr_billboard(u32 col, const SfrTexture* tex);
SFR_FUNC void sfr_cube(u32 col, const SfrTexture* tex);
SFR_FUNC void sfr_cube_ex(u32 col[12]);
SFR_FUNC void sfr_mesh(const SfrMesh* mesh, u32 col, const SfrTexture* tex);
SFR_FUNC void sfr_string(const SfrFont* font, const char* s, i32 sLength, u32 col); // not yet implemented
SFR_FUNC void sfr_glyph(const SfrFont* font, u16 id, u32 col); // draw a single character

// static scene functions
SFR_FUNC SfrScene* sfr_scene_create(SfrSceneObject* objects, i32 count);
SFR_FUNC void sfr_scene_release(SfrScene** scene, u8 freeObjects);
SFR_FUNC void sfr_scene_draw(const SfrScene* scene);
SFR_FUNC SfrRayHit sfr_scene_raycast(const SfrScene* scene, f32 ox, f32 oy, f32 oz, f32 dx, f32 dy, f32 dz);

// project the world position specified to screen coordinates
SFR_FUNC u8 sfr_world_to_screen(f32 x, f32 y, f32 z, i32* screenX, i32* screenY);

// update the camera with the new position and view
SFR_FUNC void sfr_set_camera(f32 x, f32 y, f32 z, f32 yaw, f32 pitch, f32 roll);
SFR_FUNC void sfr_set_fov(f32 fovDeg); // update projection matrix with new fov
SFR_FUNC void sfr_set_lighting(u8 enabled);

#ifdef SFR_USE_CGLTF
    SFR_FUNC void sfr_model_animate(SfrModel* model, i32 animInd, f32 time);
    SFR_FUNC SfrModel* sfr_load_gltf(const char* filename, i32 uvChannel);
    SFR_FUNC void sfr_model_draw(const SfrModel* model, sfrmat transform, const SfrTexture* overrideTex);
    SFR_FUNC SfrScene* sfr_scene_from_model(const SfrModel* model);
    SFR_FUNC void sfr_release_model(SfrModel** model);

#endif

// things requiring stdio
#ifndef SFR_NO_STD
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
SFR_FUNC i32 sfr_rand_int(i32 min, i32 max); // random int in range [min, max)
SFR_FUNC f32 sfr_rand_flt(f32 min, f32 max); // random f32 in range [min, max)

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
//: PUBLIC MACROS
//================================================

#define SFR_PI (3.14159265358979323846)
#define SFR_EPSILON (1e-10)

#define SFR_ARRLEN(_arr)  (sizeof(_arr) / sizeof((_arr)[0]))

#define SFR_MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))
#define SFR_MAX(_a, _b) ((_a) > (_b) ? (_a) : (_b))
#define SFR_CLAMP(_x, _min, _max) ((_x) < (_min) ? (_min) : ((_x) > (_max) ? (_max) : (_x)))


//================================================
//: TYPES
//================================================

#ifdef _MSC_VER
    #define SFR_ALIGNED(n) __declspec(align(n))
#else
    #define SFR_ALIGNED(n) __attribute__((aligned(n)))
#endif

#ifndef SFR_NO_SIMD
    #include <immintrin.h>
    typedef union SFR_ALIGNED(16) sfrvec {
        __m128 v;
        struct { f32 x, y, z, w; };
    } sfrvec;

    typedef struct sfrvec8 {
        __m256 x, y, z, w;
    } sfrvec8;
#else
    typedef union sfrvec { struct { f32 x, y, z, w; }; } sfrvec;
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

    u32* mipPixels[14]; // levels 1-14, up to 16k x 16k textures
    i32 mipW[14];
    i32 mipH[14];
    i32 mipLevels; // total levels including base, i.e. 1 => no extra levels
} SfrTexture;

typedef struct sfrfont {
    // xy pairs [x0, y0, x1, y1, x2, ...]
    f32 verts[SFR_FONT_GLYPH_MAX][SFR_FONT_VERT_MAX];
} SfrFont;

struct sfrBounds {
    f32 minX, minY, minZ;
    f32 maxX, maxY, maxZ;
};

typedef struct sfrScene {
    SfrSceneObject* objects;
    i32 count;
} SfrScene;

struct sfrBvhNode {
    f32 minX, minY, minZ;
    f32 maxX, maxY, maxZ;

    // if count == 0 this is an internal node and 'leftFirst' is the index of the left child
    // the right child is always at 'leftFirst + 1'
    // if count > 0 this is a leaf node and 'leftFirst' is the index of the first triangle
    i32 leftFirst;
    i32 count;
};

typedef struct sfrSceneObject {
    // all provided when creating the list of objects
    SfrMesh* mesh;
    sfrvec pos, rot, scale;
    SfrTexture* tex;
    u32 col;

    // calculated and set in sfr_scene_create
    sfrmat _model, _invModel, _normal;
    
    struct sfrBvhNode* _bvhNodes;
    i32 _bvhRoot;
    i32 _bvhNodeCount;
} SfrSceneObject;

typedef struct sfrRayHit {
    u8 hit;              // 1 if hit, 0 if not
    f32 distance;        // distance along the ray to the hit

    sfrvec pos;          // world space hit position
    sfrvec normal;       // geometric normal of the triangle hit

    i32 objectInd;       // index of the object in the scene->objects array
    i32 triangleInd;     // index of the triangle within that object's mesh
    SfrSceneObject* obj; // pointer to the object hit
} SfrRayHit;

#ifdef SFR_USE_CGLTF

enum sfrAnimPathType {
    SFR_ANIM_PATH_TRANSLATION,
    SFR_ANIM_PATH_ROTATION,
    SFR_ANIM_PATH_SCALE
};

struct sfrAnimSampler {
    f32* inputs;  // times
    f32* outputs; // values
    i32 count;
};

struct sfrAnimChannel {
    i32 transformNodeInd; // into model->transforms
    i32 samplerInd;
    enum sfrAnimPathType path;
};

struct sfrAnimation {
    char* name;
    f32 duration;

    struct sfrAnimSampler* samplers;
    i32 samplerCount;

    struct sfrAnimChannel* channels;
    i32 channelCount;
};

struct sfrTransformNode {
    // local transform components
    sfrvec localPos;
    sfrvec localRot; // quaternion
    sfrvec localScale;

    i32 parentInd;

    // cached matrices
    sfrmat localMatrix;
    sfrmat worldMatrix;
};

struct sfrModelNode {
    SfrMesh* mesh;
    SfrTexture* tex;
    i32 transformInd; // link to the scene graph hierarchy
};

typedef struct sfrModel {
    struct sfrModelNode* nodes; // renderable parts
    i32 nodeCount;

    struct sfrTransformNode* transforms; // hierarchy logic (1:1 with gltf nodes)
    i32 transformCount;

    struct sfrAnimation* animations;
    i32 animCount;

    // resource tracking for cleanup
    SfrMesh** _allMeshes;
    i32 _meshCount;
    SfrTexture** _allTextures;
    i32 _texCount;
} SfrModel;

#endif // SFR_USE_CGLTF

typedef struct sfrParticle {
    f32 px, py, pz; // position
    f32 vx, vy, vz; // velocity
    f32 ax, ay, az; // acceleration
    f32 startSize, endSize;
    u32 startCol, endCol;
    f32 lifetime, age;
} SfrParticle;

typedef struct sfrParticles {
    SfrParticle* particles;
    i32 total, active;
    const SfrTexture* tex;
} SfrParticleSystem;

typedef struct sfrlight {
    f32 dirX, dirY, dirZ;
    f32 ambient, intensity;
    f32 r, g, b; // [0.0, 1.0]
} SfrLight;

struct sfrTriangleBin {
    i32 binId;
    f32 invDet;
    f32 dzdx, dzdy, zBase;
    f32 dudx, dudy, uBase;
    f32 dvdx, dvdy, vBase;
    f32 didx, didy, iBase;
    f32 A0, B0, C0;
    f32 A1, B1, C1;
    f32 A2, B2, C2;
    i32 minX, maxX;
    i32 minY, maxY;
    u32 col;
    const SfrTexture* tex;
};

struct sfrTile {
    i32 minX, minY, maxX, maxY;
    f32 maxZ; // furthest visible depth in the tile

    #ifdef SFR_MULTITHREADED
        struct sfrTriangleBin** bins[SFR_THREAD_COUNT + 1];
        i32 binsCapacity[SFR_THREAD_COUNT + 1];
        i32 binCount[SFR_THREAD_COUNT + 1];
        SfrAtomic32 hasWork;

        SFR_ALIGNED(32) f32 localDepth[SFR_TILE_DIM];
        SFR_ALIGNED(32) i32 localId[SFR_TILE_DIM];
    #endif
};

#ifdef SFR_MULTITHREADED
    struct sfrThreadData {
        SfrThread handle;
        i32 threadInd;
    };

    struct sfrMeshChunkJob {
        const f32* tris;
        const f32* uvs;
        const f32* normals;
        sfrmat matNormal;
        sfrmat matMVP;
        u32 col;
        const SfrTexture* tex;
        i32 startTriangle;
        i32 triangleCount;
    };

    struct sfrThreadBuf {
        // tiling system data
        struct sfrTile tiles[
            ((SFR_MAX_HEIGHT + SFR_TILE_WIDTH - 1) / SFR_TILE_WIDTH) *
            ((SFR_MAX_WIDTH + SFR_TILE_WIDTH - 1) / SFR_TILE_WIDTH)];
        i32 tileCols, tileRows, tileCount;

        struct sfrTriangleBin** binPages; // dynamic array of pointers to fixed size pages
        i32 binPagesCapacity; // how many pages pointers have been allocated for
        SfrAtomic32 triangleBinAllocator; // total bins allocated across all pages
        SfrMutex binPoolMutex; // protects the binPages array resizing

        // rasterizer work dispatch data
        i32 rasterWorkQueue[(SFR_MAX_WIDTH / SFR_TILE_WIDTH + 1) * (SFR_MAX_HEIGHT / SFR_TILE_HEIGHT + 1)];
        SfrAtomic32 rasterWorkQueueCount;
        SfrAtomic32 rasterWorkQueueHead;

        // dynamic pool for mesh jobs
        struct sfrMeshChunkJob* meshJobPool;
        i32 meshJobPoolCapacity;
        SfrAtomic32 meshJobAllocator;

        // dynamic queue for geometry work
        i32* geometryWorkQueue;
        i32 geometryWorkQueueCapacity;
        SfrAtomic32 geometryWorkQueueCount;
        SfrAtomic32 geometryWorkQueueHead;

        SfrMutex geometryMutex; // protects resizing of job pool and work queue

        // thread management data
        struct sfrThreadData threads[SFR_THREAD_COUNT];
        SfrSemaphore geometryStartSem;
        SfrSemaphore geometryDoneSem;
        SfrSemaphore rasterStartSem;
        SfrSemaphore rasterDoneSem;
    };
#endif

struct sfrState {
    u8 lightingEnabled;
    i32 activeLightCount;

    sfrmat matNormal;
    u8 normalMatDirty;

    u32 randState;

    f32 halfWidth, halfHeight;

    sfrvec clipPlanes[4][2];

    u32 baseTexPixels[1];
    SfrTexture baseTex;

#ifdef SFR_MULTITHREADED
    u8 shutdown;
#else
    struct sfrTriangleBin* globalBins;
    i32 globalBinsCount, globalBinsCap;

    i32* idBuf;
#endif
};

// helper to track vertex attributes during clipping of textured triangles
struct sfrTexVert {
    sfrvec pos;      // position in view space
    f32 u, v;        // texture coords
    sfrvec normal;   // world space normal for lighting
    f32 viewZ;       // z in view space for perspective correction
    f32 intensity;   // intensity for goraud shading
};


//================================================
//: IMPLEMENTATION
//================================================

#ifdef SFR_IMPL

i32 sfrWidth, sfrHeight;

u32* sfrPixelBuf;
f32* sfrDepthBuf;
#ifdef SFR_MULTITHREADED
    static struct sfrThreadBuf* sfrThreadBuf;
#endif
SfrLight sfrLight;

SfrAtomic32 sfrRasterCount;

sfrmat sfrMatModel, sfrMatView, sfrMatProj;
sfrvec sfrCamPos, sfrCamUp, sfrCamTarget;
f32 sfrCamFov;
f32 sfrNearDist = 0.1f, sfrFarDist = 100.f;

static struct sfrState sfrState = {0};
static void* (*sfrMalloc)(u64);
static void  (*sfrFree)(void*);
static void* (*sfrRealloc)(void*, u64);

#ifdef SFR_MULTITHREADED
    // for bin batching to avoid atomic operation each time
    static SFR_THREAD_LOCAL i32 sfrTlsBinStart = 0;
    static SFR_THREAD_LOCAL i32 sfrTlsBinEnd = 0;

    // track thread inds including main thread
    static SFR_THREAD_LOCAL i32 sfrTlsThreadInd = SFR_THREAD_COUNT;
#endif


//================================================
//: OPTIONAL LIBS SETUP
//================================================

#ifndef SFR_CGLTF_PATH
    #define SFR_CGLTF_PATH "optional/cgltf.h"
#endif
#ifndef SFR_STB_IMAGE_PATH
    #define SFR_STB_IMAGE_PATH "optional/stb_image.h"
#endif

#ifdef SFR_USE_STB_IMAGE
    #define STB_IMAGE_IMPLEMENTATION
    #define STBI_MALLOC(sz)    sfrMalloc(sz)
    #define STBI_FREE(p)       sfrFree(p)
    #define STBI_REALLOC(p,sz) sfrRealloc(p,sz)
    #define STBI_ASSERT(x) 
    #include SFR_STB_IMAGE_PATH

    // STB RGBA to sofren ARGB
    static void sfr__swizzle_stb(u32* pixels, i32 count) {
        for (i32 i = 0; i < count; i += 1) {
            const u32 c = pixels[i];
            const u32 r = (c >> 0)  & 0xFF;
            const u32 g = (c >> 8)  & 0xFF;
            const u32 b = (c >> 16) & 0xFF;
            const u32 a = (c >> 24) & 0xFF;
            pixels[i] = (a << 24) | (r << 16) | (g << 8) | b;
        }
    }
#endif

#ifdef SFR_USE_CGLTF
    #ifdef SFR_NO_MATH
        #error "SFR ERROR: To use cgltf, math.h must be included (but SFR_NO_MATH is defined)"
    #endif
    #ifdef SFR_NO_STD
        #error "SFR ERROR: To use cgltf, standard lib must be included (but SFR_NO_STD is defined)"
    #endif

    #define CGLTF_IMPLEMENTATION
    #include SFR_CGLTF_PATH

    #ifndef SFR_USE_STB_IMAGE
        #warning "SFR WARNING: SFR_USE_CGLTF defined without SFR_USE_STB_IMAGE. Model textures won't be loaded."
    #endif

    static void* sfr__cgltf_alloc(void* user, cgltf_size size) {
        (void)user;
        return sfrMalloc((u64)size);
    }
    static void sfr__cgltf_free(void* user, void* ptr) {
        (void)user;
        sfrFree(ptr);
    }
#endif

#ifndef SFR_NO_STRING
    #include <string.h>
    #define sfr_memset memset
    #define sfr_memcpy memcpy
#else
    SFR_FUNC void* sfr_memset(void* dest, char c, i32 count) {
        char* p = (char*)dest;
        while (count--) {
            *p++ = c;
        }
        return dest;
    }
    SFR_FUNC void* sfr_memcpy(void* dest, const void* const src, u64 n) {
        u8* d = (u8*)dest;
        const u8* s = (const u8*)src;

        // copy until dest is aligned to word size
        while (n && ((u64)d & (sizeof(u64) - 1))) {
            *d++ = *s++;
            n--;
        }

        // copy word sized chunks
        u64* dw = (u64*)d;
        const u64* sw = (const u64*)s;
        while (n >= sizeof(u64)) {
            *dw++ = *sw++;
            n -= sizeof(u64);
        }

        // copy remaining bytes
        d = (u8*)dw;
        s = (const u8*)sw;
        while (n--) {
            *d++ = *s++;
        }

        return dest;
    }
#endif


//================================================
//: MISC HELPER MACROS
//================================================

#define SFR__SWAP(type, _a, _b) { const type _swapTemp = (_a); (_a) = (_b); (_b) = _swapTemp; }
#define SFR__LERPF(_a, _b, _t) ((_a) + (_t) * ((_b) - (_a)))

#define SFR__DIV255(_r, _a, _b) \
    const u8 _r = (_a * _b * 0x8081) >> 23;

#define SFR__MAT_IDENTITY (sfrmat){ .m = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} }

#define SFR__TRIS_PER_BOUNDS 32

#ifndef SFR_NO_STD
    #include <stdio.h>
    #include <stdlib.h>

    #define SFR__ERR_EXIT(...) { \
        fprintf(stderr, "SFR error (%s) at line %d:\n\t", __FILE__, __LINE__); \
        fprintf(stderr, __VA_ARGS__); \
        exit(1); \
    }
    #define SFR__ERR_RET(_r, ...) { \
        fprintf(stderr, "SFR error (%s) at line %d:\n\t", __FILE__, __LINE__); \
        fprintf(stderr, __VA_ARGS__); \
        return _r; \
    }
#else
    #ifndef SFR_NO_WARNINGS
        #warning "SFR WARNING: If there is an internal error it will not be reported (SFR_NO_STD defined)"
    #endif
    #define SFR__ERR_EXIT(...) { sfrPixelBuf[999999999] = 1; } // crash the program to exit, hopefully there arent 1 billion pixels
    #define SFR__ERR_RET(_r, ...) return _r
#endif


//================================================
//: MATH
//================================================

SFR_FUNC f32 sfr_fmaxf(f32 a, f32 b) { return a > b ? a : b; }
SFR_FUNC f32 sfr_fminf(f32 a, f32 b) { return a < b ? a : b; }

#ifndef SFR_NO_MATH
    #include <math.h>
    #define sfr_floorf floorf
    #define sfr_ceilf ceilf
    #define sfr_fabsf fabsf
    #define sfr_sqrtf sqrtf
    #define sfr_cosf cosf
    #define sfr_sinf sinf
    #define sfr_tanf tanf
    #define sfr_powf powf
    #define sfr_fmodf fmodf
#else
    SFR_FUNC f32 sfr_floorf(f32 x) {
        const i32 ix = (i32)x;
        return ix - 1 * (x < ix);
        // return (x < ix) ? ix - 1 : ix;
    }

    SFR_FUNC f32 sfr_ceilf(f32 x) {
        // works fine for normal inputs
        return (i32)(x + 0.999f * (x >= 0.f));
        // return (x < 0.f) ? (i32)x : (i32)(x + 0.999f);
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

    SFR_FUNC f32 sfr_powf(f32 base, f32 exp) {
        if (base < 0 && sfr_floorf(exp) != exp) {
            return 0.f;
        }

        if (0 == exp) return 1.f;
        if (1 == exp) return base;

        f32 res = 1.f;
        if (exp < 0) {
            base = 1.f / base;
            exp = -exp;
        }

        // this sucks and is very inaccurate
        i32 iexp = (i32)exp;
        while (iexp > 0) {
            if (1 == iexp % 2) {
                res *= base;
            }
            base *= base;
            iexp /= 2;
        }
        return res;
    }

    SFR_FUNC f32 sfr_fmodf(f32 x, f32 y) {
        return x - (i32)(x / y) * y;
    }
#endif

SFR_FUNC sfrvec sfr_vec_add(sfrvec a, sfrvec b) {
    sfrvec r;
    #ifndef SFR_NO_SIMD
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
    #ifndef SFR_NO_SIMD
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
    #ifndef SFR_NO_SIMD
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
    #ifndef SFR_NO_SIMD
        const __m128 t = _mm_set1_ps(b);
        r.v = _mm_div_ps(a.v, t);
        r.w = 1.f;
    #else
        const f32 ib = 1.f / b;
        r.x = a.x * ib;
        r.y = a.y * ib;
        r.z = a.z * ib;
        r.w = 1.f;
    #endif
    return r;
}

SFR_FUNC f32 sfr_vec_dot(sfrvec a, sfrvec b) {
    #ifndef SFR_NO_SIMD
        return _mm_cvtss_f32(_mm_dp_ps(a.v, b.v, 0x71));
    #else
        return a.x * b.x + a.y * b.y + a.z * b.z;
    #endif
}

SFR_FUNC f32 sfr_vec_length(sfrvec v) {
    #ifndef SFR_NO_SIMD
        return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(v.v, v.v, 0x71)));
    #else
        return sfr_sqrtf(sfr_vec_dot(v, v));
    #endif
}

SFR_FUNC f32 sfr_vec_length2(sfrvec v) {
    return sfr_vec_dot(v, v);
}

SFR_FUNC sfrvec sfr_vec_cross(sfrvec a, sfrvec b) {
    sfrvec r;
    // #ifndef SFR_NO_SIMD
    //     // y z x w, z x y w
    //     r.v = _mm_sub_ps(
    //         _mm_mul_ps(_mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b.v, b.v, _MM_SHUFFLE(3, 1, 0, 2))),
    //         _mm_mul_ps(_mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b.v, b.v, _MM_SHUFFLE(3, 0, 2, 1))));
    //     r.w = 1.f;
    // #else
        r.x = a.y * b.z - a.z * b.y;
        r.y = a.z * b.x - a.x * b.z;
        r.z = a.x * b.y - a.y * b.x;
        r.w = 1.f;
    // #endif
    return r;
}

SFR_FUNC sfrvec sfr_vec_norm(sfrvec v) {
    #ifndef SFR_NO_SIMD
        const __m128 dp = _mm_dp_ps(v.v, v.v, 0x7F);
        return _mm_cvtss_f32(dp) > SFR_EPSILON ?
            (sfrvec){ .v = _mm_mul_ps(v.v, _mm_rsqrt_ps(dp)) } :
            (sfrvec){0};
    #else
        const f32 l = sfr_vec_length(v);
        return l > SFR_EPSILON ? sfr_vec_mul(v, 1.f / l) : (sfrvec){0};
    #endif
}

SFR_FUNC sfrvec sfr_vec_normf(f32 x, f32 y, f32 z) {
    return sfr_vec_norm((sfrvec){ .x = x, .y = y, .z = z, .w = 1.f });
}

SFR_FUNC sfrvec sfr_vec_face_normal(sfrvec a, sfrvec b, sfrvec c) {
    const sfrvec edge1 = sfr_vec_sub(b, a);
    const sfrvec edge2 = sfr_vec_sub(c, a);
    return sfr_vec_norm(sfr_vec_cross(edge1, edge2));
}

SFR_FUNC sfrvec sfr_vec_lerp(sfrvec a, sfrvec b, f32 t) {
    return sfr_vec_add(a, sfr_vec_mul(sfr_vec_sub(b, a), t));
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
    sfrmat r = SFR__MAT_IDENTITY;
    const f32 c = sfr_cosf(a);
    const f32 s = sfr_sinf(a);
    r.m[1][1] = c;
    r.m[1][2] = s;
    r.m[2][1] = -s;
    r.m[2][2] = c;
    return r;
}

SFR_FUNC sfrmat sfr_mat_rot_y(f32 a) {
    sfrmat r = SFR__MAT_IDENTITY;
    const f32 c = sfr_cosf(a);
    const f32 s = sfr_sinf(a);
    r.m[0][0] = c;
    r.m[0][2] = s;
    r.m[2][0] = -s;
    r.m[2][2] = c;
    return r;
}

SFR_FUNC sfrmat sfr_mat_rot_z(f32 a) {
    sfrmat r = SFR__MAT_IDENTITY;
    const f32 c = sfr_cosf(a);
    const f32 s = sfr_sinf(a);
    r.m[0][0] = c;
    r.m[0][1] = s;
    r.m[1][0] = -s;
    r.m[1][1] = c;
    return r;
}

SFR_FUNC sfrmat sfr_mat_translate(f32 x, f32 y, f32 z) {
    sfrmat r = SFR__MAT_IDENTITY;
    r.m[3][0] = x;
    r.m[3][1] = y;
    r.m[3][2] = z;
    return r;
}

SFR_FUNC sfrmat sfr_mat_scale(f32 x, f32 y, f32 z) {
    sfrmat r = SFR__MAT_IDENTITY;
    r.m[0][0] = x;
    r.m[1][1] = y;
    r.m[2][2] = z;
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
    #ifndef SFR_NO_SIMD
        for (int i = 0; i < 4; i += 1) {
            const __m128 ar = a.rows[i].v;
            const __m128 x = _mm_shuffle_ps(ar, ar, _MM_SHUFFLE(0, 0, 0, 0));
            const __m128 y = _mm_shuffle_ps(ar, ar, _MM_SHUFFLE(1, 1, 1, 1));
            const __m128 z = _mm_shuffle_ps(ar, ar, _MM_SHUFFLE(2, 2, 2, 2));
            const __m128 w = _mm_shuffle_ps(ar, ar, _MM_SHUFFLE(3, 3, 3, 3));

            const __m128 r0 = _mm_mul_ps(x, b.rows[0].v);
            const __m128 r1 = _mm_mul_ps(y, b.rows[1].v);
            const __m128 r2 = _mm_mul_ps(z, b.rows[2].v);
            const __m128 r3 = _mm_mul_ps(w, b.rows[3].v);

            const __m128 sum01 = _mm_add_ps(r0, r1);
            const __m128 sum23 = _mm_add_ps(r2, r3);
            r.rows[i].v = _mm_add_ps(sum01, sum23);
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
    #ifndef SFR_NO_SIMD
        const __m128 vx = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(0, 0, 0, 0));
        const __m128 vy = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(1, 1, 1, 1));
        const __m128 vz = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(2, 2, 2, 2));
        const __m128 vw = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(3, 3, 3, 3));

        const __m128 mul0 = _mm_mul_ps(vx, m.rows[0].v);
        const __m128 mul1 = _mm_mul_ps(vy, m.rows[1].v);
        const __m128 mul2 = _mm_mul_ps(vz, m.rows[2].v);
        const __m128 mul3 = _mm_mul_ps(vw, m.rows[3].v);

        r.v = _mm_add_ps(_mm_add_ps(mul0, mul1),  _mm_add_ps(mul2, mul3));
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

SFR_FUNC void sfr_mat_decompose(sfrmat m, sfrvec* pos, sfrvec* rot, sfrvec* scale) {
    // extract translation (row 3)
    if (pos) {
        pos->x = m.m[3][0];
        pos->y = m.m[3][1];
        pos->z = m.m[3][2];
        pos->w = 1.f;
    }

    // extract scale (length of basis vectors)
    // row 0 is x axis, 1 is y, 2 is z
    f32 sx = sfr_sqrtf(m.m[0][0] * m.m[0][0] + m.m[0][1] * m.m[0][1] + m.m[0][2] * m.m[0][2]);
    f32 sy = sfr_sqrtf(m.m[1][0] * m.m[1][0] + m.m[1][1] * m.m[1][1] + m.m[1][2] * m.m[1][2]);
    f32 sz = sfr_sqrtf(m.m[2][0] * m.m[2][0] + m.m[2][1] * m.m[2][1] + m.m[2][2] * m.m[2][2]);

    if (scale) {
        scale->x = sx;
        scale->y = sy;
        scale->z = sz;
        scale->w = 1.f;
    }

    // extract rotation (euler xyz)
    // remove scale from the rotation matrix components
    if (rot) {
        if (sx < SFR_EPSILON) sx = 1.f;
        if (sy < SFR_EPSILON) sy = 1.f;
        if (sz < SFR_EPSILON) sz = 1.f;

        const f32 r00 = m.m[0][0] / sx;
        const f32 r01 = m.m[0][1] / sx;
        const f32 r02 = m.m[0][2] / sx;
        const f32 r11 = m.m[1][1] / sy;
        const f32 r12 = m.m[1][2] / sy;
        const f32 r21 = m.m[2][1] / sz;
        const f32 r22 = m.m[2][2] / sz;

        // decompose (rx * ry * rz)
        // m[0][2] = -sin(y)
        rot->y = -asinf(SFR_CLAMP(r02, -1.f, 1.f));

        if (sfr_cosf(rot->y) > SFR_EPSILON) {
            // standard case
            rot->x = atan2f(r12, r22);
            rot->z = atan2f(r01, r00);
        } else {
            // gimbal lock case (y is +- 90 degrees)
            rot->x = atan2f(-r21, r11);
            rot->z = 0.f;
        }
        rot->w = 0.f;
    }
}

SFR_FUNC sfrmat sfr_mat_from_quat(sfrvec q) {
    sfrmat m = SFR__MAT_IDENTITY;
    const f32 x = q.x, y = q.y, z = q.z, w = q.w;
    const f32 x2 = x + x, y2 = y + y, z2 = z + z;
    const f32 xx = x * x2, xy = x * y2, xz = x * z2;
    const f32 yy = y * y2, yz = y * z2, zz = z * z2;
    const f32 wx = w * x2, wy = w * y2, wz = w * z2;

    m.m[0][0] = 1.f - (yy + zz);
    m.m[0][1] = xy + wz;
    m.m[0][2] = xz - wy;
    m.m[0][3] = 0.f;

    m.m[1][0] = xy - wz;
    m.m[1][1] = 1.f - (xx + zz);
    m.m[1][2] = yz + wx;
    m.m[1][3] = 0.f;

    m.m[2][0] = xz + wy;
    m.m[2][1] = yz - wx;
    m.m[2][2] = 1.f - (xx + yy);
    m.m[2][3] = 0.f;

    m.m[3][3] = 1.f;
    return m;
}

SFR_FUNC sfrvec sfr_quat_mul(sfrvec a, sfrvec b) {
    sfrvec r;
    r.x =  a.x * b.w + a.y * b.z - a.z * b.y + a.w * b.x;
    r.y = -a.x * b.z + a.y * b.w + a.z * b.x + a.w * b.y;
    r.z =  a.x * b.y - a.y * b.x + a.z * b.w + a.w * b.z;
    r.w = -a.x * b.x - a.y * b.y - a.z * b.z + a.w * b.w;
    return r;
}

SFR_FUNC sfrvec sfr_quat_slerp(sfrvec a, sfrvec b, f32 t) {
    f32 dot = sfr_vec_dot(a, b);
    if (dot < 0.f) {
        b = sfr_vec_mul(b, -1.f);
        dot = -dot;
    }

    if (dot > 0.9995f) {
        // lerp for close angles
        return sfr_vec_norm(sfr_vec_lerp(a, b, t));
    }

    const f32 theta0 = acosf(dot);
    const f32 theta = theta0 * t;
    const f32 sinTheta = sinf(theta);
    const f32 sinTheta0 = 1.f / sinf(theta0);

    const f32 s0 = cosf(theta) - dot * sinTheta * sinTheta0;
    const f32 s1 = sinTheta * sinTheta0;

    return sfr_vec_add(sfr_vec_mul(a, s0), sfr_vec_mul(b, s1));
}


//================================================
//: PRIVATE FUNCTIONS
//================================================

static void sfr__mesh_swap_tri(SfrMesh* mesh, i32 a, i32 b) {
    if (a == b) {
        return;
    }

    // swap positions
    for (i32 i = 0; i < 9; i += 1) {
        SFR__SWAP(f32, mesh->tris[a * 9 + i], mesh->tris[b * 9 + i]);
    }

    // swap uvs
    if (mesh->uvs) {
        for (i32 i = 0; i < 6; i += 1) {
            SFR__SWAP(f32, mesh->uvs[a * 6 + i], mesh->uvs[b * 6 + i]);
        }
    }

    // swap normals
    if (mesh->normals) {
        for (i32 i = 0; i < 9; i += 1) {
            SFR__SWAP(f32, mesh->normals[a * 9 + i], mesh->normals[b * 9 + i]);
        }
    }
}

// recursively build bvh
static void sfr__bvh_update_bounds(i32 nodeInd, struct sfrBvhNode* const nodes, const SfrMesh* mesh, i32 count) {
    struct sfrBvhNode* const node = &nodes[nodeInd];
    node->minX = node->minY = node->minZ = 1e30f;
    node->maxX = node->maxY = node->maxZ = -1e30f;

    for (i32 i = 0; i < count; i += 1) {
        // current triangle ind in the mesh arrays
        const i32 triInd = node->leftFirst + i; // because they're swapped they're contiguous

        for (i32 v = 0; v < 3; v += 1) {
            const f32 vx = mesh->tris[triInd * 9 + v * 3 + 0];
            const f32 vy = mesh->tris[triInd * 9 + v * 3 + 1];
            const f32 vz = mesh->tris[triInd * 9 + v * 3 + 2];

            node->minX = SFR_MIN(node->minX, vx);
            node->minY = SFR_MIN(node->minY, vy);
            node->minZ = SFR_MIN(node->minZ, vz);
            node->maxX = SFR_MAX(node->maxX, vx);
            node->maxY = SFR_MAX(node->maxY, vy);
            node->maxZ = SFR_MAX(node->maxZ, vz);
        }
    }
}

static void sfr__bvh_subdivide(struct sfrBvhNode* const nodes, i32 nodeInd, i32* nodePtr, SfrMesh* mesh) {
    struct sfrBvhNode* const node = &nodes[nodeInd];

    if (node->count <= 4) {
        return;
    }

    const f32 extentX = node->maxX - node->minX;
    const f32 extentY = node->maxY - node->minY;
    const f32 extentZ = node->maxZ - node->minZ;

    const i32 axis = (extentZ > extentY && extentZ > extentX) ? 2 : ((extentY > extentX) ? 1 : 0);
    // i32 axis = 0;
    // if (extentY > extentX) axis = 1;
    // if (extentZ > extentY && extentZ > extentX) axis = 2;

    const f32 splitPos =
        (0 == axis) ? (node->minX + extentX * 0.5f) :
        (1 == axis) ? (node->minY + extentY * 0.5f) :
                      (node->minZ + extentZ * 0.5f);

    i32 i = node->leftFirst;
    i32 j = i + node->count - 1;

    while (i <= j) {
        const f32 c =
            (0 == axis) ? (mesh->tris[i * 9 + 0] + mesh->tris[i * 9 + 3] + mesh->tris[i * 9 + 6]) / 3.f :
            (1 == axis) ? (mesh->tris[i * 9 + 1] + mesh->tris[i * 9 + 4] + mesh->tris[i * 9 + 7]) / 3.f :
                          (mesh->tris[i * 9 + 2] + mesh->tris[i * 9 + 5] + mesh->tris[i * 9 + 8]) / 3.f;

        if (c < splitPos) {
            i += 1;
        } else {
            sfr__mesh_swap_tri(mesh, i, j);
            j -= 1;
        }
    }

    const i32 leftCount = i - node->leftFirst;
    if (0 == leftCount || leftCount == node->count) {
        return;
    }

    // save originals
    const i32 originalLeftFirst = node->leftFirst;
    const i32 originalCount = node->count;

    // allocate children
    const i32 leftChildInd = *nodePtr;
    (*nodePtr) += 2;

    // update current node to internal
    node->leftFirst = leftChildInd;
    node->count = 0;

    // init left and right
    nodes[leftChildInd].leftFirst = originalLeftFirst;
    nodes[leftChildInd].count = leftCount;
    nodes[leftChildInd + 1].leftFirst = i; // i is the start of the right partition
    nodes[leftChildInd + 1].count = originalCount - leftCount;

    // recurse
    sfr__bvh_update_bounds(leftChildInd, nodes, mesh, leftCount);
    sfr__bvh_subdivide(nodes, leftChildInd, nodePtr, mesh);
    sfr__bvh_update_bounds(leftChildInd + 1, nodes, mesh, nodes[leftChildInd + 1].count);
    sfr__bvh_subdivide(nodes, leftChildInd + 1, nodePtr, mesh);
}

// for mipmapping
static i32 sfr__fast_log2(f32 x) {
    u32 bits;
    sfr_memcpy(&bits, &x, 4);
    return (i32)((bits >> 23) & 0xFF) - 127;
}

static struct sfrTexVert sfr__lerp_vert(struct sfrTexVert a, struct sfrTexVert b, f32 t) {
    return (struct sfrTexVert){
        .pos = sfr_vec_lerp(a.pos, b.pos, t),
        .u = SFR__LERPF(a.u, b.u, t),
        .v = SFR__LERPF(a.v, b.v, t),
        .viewZ = SFR__LERPF(a.viewZ, b.viewZ, t),
        .intensity = SFR__LERPF(a.intensity, b.intensity, t),
        .normal = sfr_vec_lerp(a.normal, b.normal, t)
    };
}

static i32 sfr__clip_tri_homogeneous(struct sfrTexVert out[restrict 2][3], sfrvec plane, const struct sfrTexVert in[3]) {
    f32 dists[3];
    i32 clipMask = 0;

    // calculate distances and build a bitmask of which verts are inside
    for (i32 i = 0; i < 3; i += 1) {
        const sfrvec v = in[i].pos;
        dists[i] = plane.x * v.x + plane.y * v.y + plane.z * v.z + plane.w * v.w;
        clipMask |= (dists[i] >= 0.f) << i;
    }

    // switch based on the mask topology
    switch (clipMask) {
        case 0: { // 000, all outside
            return 0;
        }

        case 7: { // 111, all inside
            sfr_memcpy(out[0], in, sizeof(struct sfrTexVert) * 3);
            return 1;
        }

        // vv 1 in, 2 out (triangle shrinks) vv
        case 1: // 001, only v0 inside
        case 2: // 010, only v1 inside
        case 4: // 100, only v2 inside
        {
            // which index is the one inside (the tip)
            const i32 inInd = clipMask >> 1;

            // the next two vertices in winding order
            const i32 bInd = (2 == inInd) ? 0 : inInd + 1;
            const i32 cInd = (0 == inInd) ? 2 : inInd - 1;

            const struct sfrTexVert vA = in[inInd];
            const struct sfrTexVert vB = in[bInd];
            const struct sfrTexVert vC = in[cInd];

            // interpolate new verts on the edges leaving vA
            const f32 dA = dists[inInd];
            const f32 tAB = dA / (dA - dists[bInd]);
            const f32 tAC = dA / (dA - dists[cInd]);

            const struct sfrTexVert newAB = sfr__lerp_vert(vA, vB, tAB);
            const struct sfrTexVert newAC = sfr__lerp_vert(vA, vC, tAC);

            out[0][0] = vA;
            out[0][1] = newAB;
            out[0][2] = newAC;

            return 1;
        }

        // vv 2 in, 1 out (quad split into 2 tris) vv
        case 3: // 011, v2 is out
        case 5: // 101, v1 is out
        case 6: // 110, v0 is out
        {
            // which index is the one outside
            const i32 outInd = (~clipMask & 7) >> 1;

            const i32 aInd = (outInd == 0) ? 2 : outInd - 1; // one before
            const i32 cInd = (outInd == 2) ? 0 : outInd + 1; // one after

            const struct sfrTexVert vA = in[aInd];
            const struct sfrTexVert vB = in[outInd];
            const struct sfrTexVert vC = in[cInd];

            const f32 dA = dists[aInd];
            const f32 dB = dists[outInd];
            const f32 dC = dists[cInd];
            const f32 tAB = dA / (dA - dB);
            const f32 tCB = dC / (dC - dB);

            const struct sfrTexVert newAB = sfr__lerp_vert(vA, vB, tAB);
            const struct sfrTexVert newCB = sfr__lerp_vert(vC, vB, tCB);

            // decomposition of (vA, newAB, newCB, vC)
            // vA, newAB, vC
            // newAB, newCB, vC

            out[0][0] = vA;
            out[0][1] = newAB;
            out[0][2] = vC;
            out[1][0] = newAB;
            out[1][1] = newCB;
            out[1][2] = vC;
            return 2;
        }
    }

    return 0;
}

static void sfr__rasterize_bin(const struct sfrTriangleBin* bin, struct sfrTile* tile) {
    const f32 dzdx = bin->dzdx, dzdy = bin->dzdy, zBase = bin->zBase;

    const f32 A0 = bin->A0, B0 = bin->B0, C0 = bin->C0;
    const f32 A1 = bin->A1, B1 = bin->B1, C1 = bin->C1;
    const f32 A2 = bin->A2, B2 = bin->B2, C2 = bin->C2;

    { // hi-z
        const f32 tMinX = (f32)tile->minX, tMinY = (f32)tile->minY;
        const f32 tMaxX = (f32)tile->maxX, tMaxY = (f32)tile->maxY;

        // closest z check
        const f32 xmInvZ = (dzdx > 0.f) ? tMaxX : tMinX;
        const f32 ymInvZ = (dzdy > 0.f) ? tMaxY : tMinY;
        const f32 maxInvZ = zBase + xmInvZ * dzdx + ymInvZ * dzdy;

        if (maxInvZ > SFR_EPSILON) {
            const f32 minDepth = 1.f / maxInvZ;
            if (minDepth > tile->maxZ) {
                return;
            }

            // update tile->maxZ
            const f32 xmE0 = (A0 > 0.f) ? tMinX : tMaxX, ymE0 = (B0 > 0.f) ? tMinY : tMaxY;
            const f32 minE0 = C0 + A0 * xmE0 + B0 * ymE0;
            const f32 xmE1 = (A1 > 0.f) ? tMinX : tMaxX, ymE1 = (B1 > 0.f) ? tMinY : tMaxY;
            const f32 minE1 = C1 + A1 * xmE1 + B1 * ymE1;
            const f32 xmE2 = (A2 > 0.f) ? tMinX : tMaxX, ymE2 = (B2 > 0.f) ? tMinY : tMaxY;
            const f32 minE2 = C2 + A2 * xmE2 + B2 * ymE2;

            if (minE0 >= 0.f && minE1 >= 0.f && minE2 >= 0.f) {
                const f32 x = (dzdx < 0.f) ? tMaxX : tMinX, y = (dzdy < 0.f) ? tMaxY : tMinY;
                const f32 minInvZInTile = zBase + x * dzdx + y * dzdy;
                if (minInvZInTile > SFR_EPSILON) {
                    const f32 maxDepthInTile = 1.f / minInvZInTile;
                    if (maxDepthInTile < tile->maxZ) {
                        tile->maxZ = maxDepthInTile;
                    }
                }
            }
        }
    }

    const i32 minX = (i32)sfr_fmaxf(tile->minX, bin->minX);
    const i32 maxX = (i32)sfr_fminf(tile->maxX, bin->maxX);
    const i32 minY = (i32)sfr_fmaxf(tile->minY, bin->minY);
    const i32 maxY = (i32)sfr_fminf(tile->maxY, bin->maxY);

    #ifdef SFR_MULTITHREADED
        f32* const targetDepthBuf = tile->localDepth;
        i32* const targetIdBuf = tile->localId;
    #else
        f32* const targetDepthBuf = sfrDepthBuf;
        i32* const targetIdBuf = sfrState.idBuf;
    #endif

#ifndef SFR_NO_SIMD
    if (maxX - minX <= 4 && maxY - minY <= 4) {
        goto SCALAR; // SIMD will do too much wasted work so use scalar for tiny triangles
    }

    // align minX to 8 for SIMD
    const i32 alignedMinX = minX & ~7;

    // vv setup vectors vv
    const __m256 vA0 = _mm256_set1_ps(A0), vB0 = _mm256_set1_ps(B0), vC0 = _mm256_set1_ps(C0);
    const __m256 vA1 = _mm256_set1_ps(A1), vB1 = _mm256_set1_ps(B1), vC1 = _mm256_set1_ps(C1);
    const __m256 vA2 = _mm256_set1_ps(A2), vB2 = _mm256_set1_ps(B2), vC2 = _mm256_set1_ps(C2);

    const __m256 vZBase = _mm256_set1_ps(zBase), vdzdx = _mm256_set1_ps(dzdx), vdzdy = _mm256_set1_ps(dzdy);

    const __m256 vXOffsets = _mm256_setr_ps(0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f);
    const __m256i vIndex = _mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7);

    const __m256i vBinId = _mm256_set1_epi32(bin->binId);

    for (i32 yBase = minY; yBase < maxY; yBase += 8) {
        const f32 fy = (f32)yBase;
        const __m256 vy = _mm256_set1_ps(fy);

        // trivial rejection test
        for (i32 xBase = alignedMinX; xBase < maxX; xBase += 8) {
            // a plane E(x, y) is linear, E < 0 at all 4 corners => its < 0 everywhere inside

            const f32 fx = (f32)xBase;
            const f32 e0tl = C0 + A0 * fx + B0 * fy;
            const f32 e0tr = e0tl + A0 * 8.f;
            const f32 e0bl = e0tl + B0 * 8.f;
            const f32 e0br = e0tl + A0 * 8.f + B0 * 8.f;
            if (sfr_fmaxf(sfr_fmaxf(e0tl, e0tr), sfr_fmaxf(e0bl, e0br)) < 0.f) {
                // if the max value at any corner is < 0, the whole block is outside edge 0
                continue;
            }

            const f32 e1tl = C1 + A1 * fx + B1 * fy;
            const f32 e1tr = e1tl + A1 * 8.f;
            const f32 e1bl = e1tl + B1 * 8.f;
            const f32 e1br = e1tl + A1 * 8.f + B1 * 8.f;
            if (sfr_fmaxf(sfr_fmaxf(e1tl, e1tr), sfr_fmaxf(e1bl, e1br)) < 0.f) {
                continue;
            }

            const f32 e2tl = C2 + A2 * fx + B2 * fy;
            const f32 e2tr = e2tl + A2 * 8.f;
            const f32 e2bl = e2tl + B2 * 8.f;
            const f32 e2br = e2tl + A2 * 8.f + B2 * 8.f;
            if (sfr_fmaxf(sfr_fmaxf(e2tl, e2tr), sfr_fmaxf(e2bl, e2br)) < 0.f) {
                continue;
            }

            const __m256 vXBlock = _mm256_add_ps(_mm256_set1_ps(fx), vXOffsets);

            // start values at (xBase, yBase)
            __m256 vE0 = _mm256_add_ps(vC0, _mm256_add_ps(_mm256_mul_ps(vA0, vXBlock), _mm256_mul_ps(vB0, vy)));
            __m256 vE1 = _mm256_add_ps(vC1, _mm256_add_ps(_mm256_mul_ps(vA1, vXBlock), _mm256_mul_ps(vB1, vy)));
            __m256 vE2 = _mm256_add_ps(vC2, _mm256_add_ps(_mm256_mul_ps(vA2, vXBlock), _mm256_mul_ps(vB2, vy)));
            __m256 vz  = _mm256_add_ps(vZBase, _mm256_add_ps(_mm256_mul_ps(vdzdx, vXBlock), _mm256_mul_ps(vdzdy, vy)));

            // constants to step y by 1 for the inner 8 lines
            const __m256 vB0Step = vB0, vB1Step = vB1, vB2Step = vB2;
            const __m256 vdZdyStep = vdzdy;

            // limit y to tile bounds
            const i32 yLimit = SFR_MIN(yBase + 8, maxY);

            for (i32 y = yBase; y < yLimit; y += 1) {
                // determine Coverage
                __m256 mask = _mm256_and_ps(
                    _mm256_cmp_ps(vE0, _mm256_setzero_ps(), _CMP_GE_OQ),
                    _mm256_and_ps(
                        _mm256_cmp_ps(vE1, _mm256_setzero_ps(), _CMP_GE_OQ),
                        _mm256_cmp_ps(vE2, _mm256_setzero_ps(), _CMP_GE_OQ)
                    )
                );

                // alignment check
                if (xBase < minX || xBase + 8 > maxX) {
                    const __m256i vGlobalX = _mm256_add_epi32(_mm256_set1_epi32(xBase), vIndex);
                    const __m256 vBoundMask = _mm256_and_ps(
                        _mm256_cmp_ps(_mm256_cvtepi32_ps(vGlobalX), _mm256_set1_ps((f32)minX), _CMP_GE_OQ),
                        _mm256_cmp_ps(_mm256_cvtepi32_ps(vGlobalX), _mm256_set1_ps((f32)maxX), _CMP_LT_OQ)
                    );
                    mask = _mm256_and_ps(mask, vBoundMask);
                }

                i32 maskInt = _mm256_movemask_ps(mask);
                if (maskInt) {
                    #ifdef SFR_MULTITHREADED
                        const i32 localY = y - tile->minY;
                        const i32 localX = xBase - tile->minX;
                        const i32 pixelInd = localY * SFR_TILE_WIDTH + localX;
                        // doesn't work with mipmaps (TODO fix alignment ?)
                        // const __m256 vOldDepth = _mm256_load_ps(&targetDepthBuf[pixelInd]);
                    #else
                        const i32 pixelInd = y * sfrWidth + xBase;
                    #endif
                    const __m256 vOldDepth = _mm256_loadu_ps(&targetDepthBuf[pixelInd]);

                    __m256 vRecipZ = _mm256_rcp_ps(vz);
                    vRecipZ = _mm256_mul_ps(vRecipZ, _mm256_sub_ps(_mm256_set1_ps(2.f), _mm256_mul_ps(vz, vRecipZ)));

                    const __m256 depthMask = _mm256_cmp_ps(vRecipZ, vOldDepth, _CMP_LT_OQ);
                    mask = _mm256_and_ps(mask, depthMask);
                    maskInt = _mm256_movemask_ps(mask);

                    if (0xFF == maskInt) {
                        // 100% coverage so skip the load/blend and store
                        _mm256_storeu_ps(&targetDepthBuf[pixelInd], vRecipZ);
                        _mm256_storeu_si256((__m256i*)&targetIdBuf[pixelInd], vBinId);
                    } else if (maskInt) {
                        const __m256 vNewDepth = _mm256_blendv_ps(vOldDepth, vRecipZ, mask);
                        _mm256_storeu_ps(&targetDepthBuf[pixelInd], vNewDepth);

                        // load the old IDs, blend, and store
                        // cast to _ps to use the fast fp blend unit
                        const __m256i vOldId = _mm256_loadu_si256((__m256i*)&targetIdBuf[pixelInd]);
                        const __m256 vNewIdF = _mm256_blendv_ps(_mm256_castsi256_ps(vOldId),
                                                          _mm256_castsi256_ps(vBinId),
                                                          mask);
                        _mm256_storeu_si256((__m256i*)&targetIdBuf[pixelInd], _mm256_castps_si256(vNewIdF));
                    }
                }

                // step y by 1 for the next row in this block
                vE0 = _mm256_add_ps(vE0, vB0Step);
                vE1 = _mm256_add_ps(vE1, vB1Step);
                vE2 = _mm256_add_ps(vE2, vB2Step);
                vz = _mm256_add_ps(vz, vdZdyStep);
            }
        }
    }

    return;
#endif

    SCALAR:;
    for (i32 y = minY; y < maxY; y += 1) {
        f32 E0 = C0 + A0 * minX + B0 * y;
        f32 E1 = C1 + A1 * minX + B1 * y;
        f32 E2 = C2 + A2 * minX + B2 * y;
        f32 z = zBase + dzdx * minX + dzdy * y;

        #ifdef SFR_MULTITHREADED
            const i32 localY = y - tile->minY;
            i32 pixelInd = localY * SFR_TILE_WIDTH + (minX - tile->minX);
        #else
            i32 pixelInd = y * sfrWidth + minX;
        #endif

        for (i32 x = minX; x < maxX; x += 1, E0 += A0, E1 += A1, E2 += A2, z += dzdx, pixelInd += 1) {
            if (E0 < 0.f || E1 < 0.f || E2 < 0.f) {
                continue;
            }

            const f32 invZ = 1.f / z;
            if (invZ >= targetDepthBuf[pixelInd]) {
                continue;
            }

            targetDepthBuf[pixelInd] = invZ;
            targetIdBuf[pixelInd] = bin->binId;
        }
    }
}

static void sfr__bin_triangle(
    f32 ax, f32 ay, f32 aInvZ, f32 auoz, f32 avoz,
    f32 bx, f32 by, f32 bInvZ, f32 buoz, f32 bvoz,
    f32 cx, f32 cy, f32 cInvZ, f32 cuoz, f32 cvoz,
    f32 aIntensity, f32 bIntensity, f32 cIntensity,
    u32 col, const SfrTexture* tex
) {
    const f32 det = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
    if (0.f == det) {
        return;
    }
    const f32 invDet = 1.f / det;

    const f32 ai = aIntensity * aInvZ;
    const f32 bi = bIntensity * bInvZ;
    const f32 ci = cIntensity * cInvZ;

    const f32 dzdx = ((bInvZ - aInvZ) * (cy - ay) - (cInvZ - aInvZ) * (by - ay)) * invDet;
    const f32 dzdy = ((cInvZ - aInvZ) * (bx - ax) - (bInvZ - aInvZ) * (cx - ax)) * invDet;
    const f32 zBase = aInvZ - (ax * dzdx + ay * dzdy);
    const f32 dudx = ((buoz - auoz) * (cy - ay) - (cuoz - auoz) * (by - ay)) * invDet;
    const f32 dudy = ((cuoz - auoz) * (bx - ax) - (buoz - auoz) * (cx - ax)) * invDet;
    const f32 uBase = auoz - (ax * dudx + ay * dudy);
    const f32 dvdx = ((bvoz - avoz) * (cy - ay) - (cvoz - avoz) * (by - ay)) * invDet;
    const f32 dvdy = ((cvoz - avoz) * (bx - ax) - (bvoz - avoz) * (cx - ax)) * invDet;
    const f32 vBase = avoz - (ax * dvdx + ay * dvdy);
    const f32 didx = ((bi - ai) * (cy - ay) - (ci - ai) * (by - ay)) * invDet;
    const f32 didy = ((ci - ai) * (bx - ax) - (bi - ai) * (cx - ax)) * invDet;
    const f32 iBase = ai - (ax * didx + ay * didy);

    const f32 A0 = ay - by, B0 = bx - ax, C0 = ax * by - ay * bx;
    const f32 A1 = by - cy, B1 = cx - bx, C1 = bx * cy - by * cx;
    const f32 A2 = cy - ay, B2 = ax - cx, C2 = cx * ay - cy * ax;

    const f32 minX = sfr_fmaxf(0.f,             sfr_floorf(sfr_fminf(ax, SFR_MIN(bx, cx))));
    const f32 maxX = sfr_fminf(sfrWidth - 1.f,  sfr_ceilf (sfr_fmaxf(ax, SFR_MAX(bx, cx))));
    const f32 minY = sfr_fmaxf(0.f,             sfr_floorf(sfr_fminf(ay, SFR_MIN(by, cy))));
    const f32 maxY = sfr_fminf(sfrHeight - 1.f, sfr_ceilf (sfr_fmaxf(ay, SFR_MAX(by, cy))));

#ifdef SFR_MULTITHREADED
    // auto reserve slot
    if (sfrTlsBinStart == sfrTlsBinEnd) {
        sfrTlsBinStart = sfr_atomic_add(&sfrThreadBuf->triangleBinAllocator, 1024);
        sfrTlsBinEnd = sfrTlsBinStart + 1024;
    }
    const i32 globalInd = sfrTlsBinStart++;

    // calculate page and offset
    const i32 pageInd = globalInd / SFR_BIN_PAGE_SIZE;
    const i32 pageOffset = globalInd % SFR_BIN_PAGE_SIZE;

    // ensure page exists (rare lock)
    if (pageInd >= sfrThreadBuf->binPagesCapacity || !sfrThreadBuf->binPages[pageInd]) {
        sfr_mutex_lock(&sfrThreadBuf->binPoolMutex);

        // resize page pointer array
        if (pageInd >= sfrThreadBuf->binPagesCapacity) {
            i32 newCap = sfrThreadBuf->binPagesCapacity * 2;
            while (pageInd >= newCap) newCap *= 2;

            struct sfrTriangleBin** newPageArr = (struct sfrTriangleBin**)sfrRealloc(sfrThreadBuf->binPages, sizeof(struct sfrTriangleBin*) * newCap);
            if (newPageArr) {
                sfr_memset(newPageArr + sfrThreadBuf->binPagesCapacity, 0,
                    (newCap - sfrThreadBuf->binPagesCapacity) * sizeof(struct sfrTriangleBin*));
                sfrThreadBuf->binPages = newPageArr;
                sfrThreadBuf->binPagesCapacity = newCap;
            }
        }

        // allocate specific page if it's missing
        if (!sfrThreadBuf->binPages[pageInd]) {
            sfrThreadBuf->binPages[pageInd] = (struct sfrTriangleBin*)sfrMalloc(sizeof(struct sfrTriangleBin) * SFR_BIN_PAGE_SIZE);
        }

        sfr_mutex_unlock(&sfrThreadBuf->binPoolMutex);
    }

    if (!sfrThreadBuf->binPages[pageInd]) {
        return;
    }

    struct sfrTriangleBin* bin = &sfrThreadBuf->binPages[pageInd][pageOffset];

    *bin = (struct sfrTriangleBin){
        .binId = globalInd,

        .invDet = invDet,

        .dzdx = dzdx, .dzdy = dzdy, .zBase = zBase,
        .dudx = dudx, .dudy = dudy, .uBase = uBase,
        .dvdx = dvdx, .dvdy = dvdy, .vBase = vBase,
        .didx = didx, .didy = didy, .iBase = iBase,

        .A0 = A0, .B0 = B0, .C0 = C0,
        .A1 = A1, .B1 = B1, .C1 = C1,
        .A2 = A2, .B2 = B2, .C2 = C2,

        .minX = minX, .maxX = maxX,
        .minY = minY, .maxY = maxY,

        .col = col, .tex = tex
    };
    sfr_atomic_add(&sfrRasterCount, 1);

    const i32 xStart = (i32)minX / SFR_TILE_WIDTH;
    const i32 xEnd   = (i32)maxX / SFR_TILE_WIDTH;
    const i32 yStart = (i32)minY / SFR_TILE_HEIGHT;
    const i32 yEnd   = (i32)maxY / SFR_TILE_HEIGHT;

    // dynamic tile binning
    const i32 tInd = sfrTlsThreadInd; // current thread's index
    for (i32 ty = yStart; ty <= yEnd; ty += 1) {
        for (i32 tx = xStart; tx <= xEnd; tx += 1) {
            const i32 tileInd = ty * sfrThreadBuf->tileCols + tx;
            struct sfrTile* tile = &sfrThreadBuf->tiles[tileInd];

            // thread local resize
            if (tile->binCount[tInd] >= tile->binsCapacity[tInd]) {
                const i32 newCap = tile->binsCapacity[tInd] * 2;
                struct sfrTriangleBin** const newBins = (struct sfrTriangleBin**)sfrRealloc(
                    tile->bins[tInd], sizeof(struct sfrTriangleBin*) * newCap);
                if (newBins) {
                    tile->bins[tInd] = newBins;
                    tile->binsCapacity[tInd] = newCap;
                }
            }

            // add bin to thread local array
            if (tile->bins[tInd]) {
                tile->bins[tInd][tile->binCount[tInd]++] = bin;
            }
        }
    }

#else // !SFR_MULTITHREADED

    sfrRasterCount += 1;
    struct sfrTile fullTile = {
        .minX = 0, .minY = 0, .maxX = sfrWidth, .maxY = sfrHeight,
        .maxZ = sfrFarDist
    };

    if (sfrState.globalBinsCount >= sfrState.globalBinsCap) {
        sfrState.globalBinsCap = (i32)(sfrState.globalBinsCap * 1.5f);
        sfrState.globalBins = (struct sfrTriangleBin*)sfrRealloc(
            sfrState.globalBins, sizeof(struct sfrTriangleBin) * (u64)sfrState.globalBinsCap);
    }

    struct sfrTriangleBin* bin = &sfrState.globalBins[sfrState.globalBinsCount];
    *bin = (struct sfrTriangleBin){
        .binId = sfrState.globalBinsCount,

        .invDet = invDet,

        .dzdx = dzdx, .dzdy = dzdy, .zBase = zBase,
        .dudx = dudx, .dudy = dudy, .uBase = uBase,
        .dvdx = dvdx, .dvdy = dvdy, .vBase = vBase,
        .didx = didx, .didy = didy, .iBase = iBase,

        .A0 = A0, .B0 = B0, .C0 = C0,
        .A1 = A1, .B1 = B1, .C1 = C1,
        .A2 = A2, .B2 = B2, .C2 = C2,

        .minX = minX, .maxX = maxX,
        .minY = minY, .maxY = maxY,

        .col = col, .tex = tex
    };

    sfrState.globalBinsCount++;

    sfr__rasterize_bin(bin, &fullTile);
#endif // SFR_MULTITHREADED
}

// core geometry pipeline for a single triangle
static void sfr__process_and_bin_triangle(
    const sfrmat* matMVP, const sfrmat* matNormal,
    f32 ax, f32 ay, f32 az, f32 au, f32 av, f32 anx, f32 any, f32 anz,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv, f32 bnx, f32 bny, f32 bnz,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv, f32 cnx, f32 cny, f32 cnz,
    u32 col, const SfrTexture* tex
) {
    // start in clip space
    const sfrvec aClip = sfr_mat_mul_vec(*matMVP, (sfrvec){ax, ay, az, 1.f});
    const sfrvec bClip = sfr_mat_mul_vec(*matMVP, (sfrvec){bx, by, bz, 1.f});
    const sfrvec cClip = sfr_mat_mul_vec(*matMVP, (sfrvec){cx, cy, cz, 1.f});

    const SfrTexture* texToUse = tex ? tex : &sfrState.baseTex;

    // if all vertices are safely in front of the near plane skip clipping
    if (aClip.w > SFR_EPSILON && bClip.w > SFR_EPSILON && cClip.w > SFR_EPSILON) {
        // perspective divide
        const f32 aInvW = 1.f / aClip.w;
        const f32 bInvW = 1.f / bClip.w;
        const f32 cInvW = 1.f / cClip.w;

        // map directly to screen pixels
        const f32 sax = (aClip.x * aInvW + 1.f) * sfrState.halfWidth;
        const f32 say = (-aClip.y * aInvW + 1.f) * sfrState.halfHeight;
        const f32 sbx = (bClip.x * bInvW + 1.f) * sfrState.halfWidth;
        const f32 sby = (-bClip.y * bInvW + 1.f) * sfrState.halfHeight;
        const f32 scx = (cClip.x * cInvW + 1.f) * sfrState.halfWidth;
        const f32 scy = (-cClip.y * cInvW + 1.f) * sfrState.halfHeight;

        // screen space culling
        const f32 area = (sbx - sax) * (scy - say) - (sby - say) * (scx - sax);

        #ifndef SFR_NO_CULLING
            // backface culling
            if (area <= 0.f) {
                return;
            }
        #endif

        // small triangle culling
        #ifndef SFR_NO_SMALL_CULLING
            if (area < 0.1f) { // 0.1 - 0.4 seems to be good, larger => fades quicker
                return;
            }
        #endif

        // fast frustum xy guard band culling
        const f32 gb = 1.1f * sfrState.halfWidth;
        if ((sax < -gb && sbx < -gb && scx < -gb) ||
            (sax > sfrWidth + gb && sbx > sfrWidth + gb && scx > sfrWidth + gb) ||
            (say < -gb && sby < -gb && scy < -gb) ||
            (say > sfrHeight + gb && sby > sfrHeight + gb && scy > sfrHeight + gb)
        ) {
            return;
        }

        // lighting
        f32 aIntensity = 1.f, bIntensity = 1.f, cIntensity = 1.f;
        if (sfrState.lightingEnabled) {
            const sfrvec na = sfr_mat_mul_vec(*matNormal, (sfrvec){anx, any, anz, 0.f});
            const sfrvec nb = sfr_mat_mul_vec(*matNormal, (sfrvec){bnx, bny, bnz, 0.f});
            const sfrvec nc = sfr_mat_mul_vec(*matNormal, (sfrvec){cnx, cny, cnz, 0.f});
            const sfrvec lightDir = {sfrLight.dirX, sfrLight.dirY, sfrLight.dirZ, 0.f};
            aIntensity = sfrLight.ambient + sfr_fmaxf(0.f, sfr_vec_dot(na, lightDir)) * sfrLight.intensity;
            bIntensity = sfrLight.ambient + sfr_fmaxf(0.f, sfr_vec_dot(nb, lightDir)) * sfrLight.intensity;
            cIntensity = sfrLight.ambient + sfr_fmaxf(0.f, sfr_vec_dot(nc, lightDir)) * sfrLight.intensity;
        }

        sfr__bin_triangle(
            sax, say, aInvW, au * aInvW, av * aInvW,
            sbx, sby, bInvW, bu * bInvW, bv * bInvW,
            scx, scy, cInvW, cu * cInvW, cv * cInvW,
            aIntensity, bIntensity, cIntensity, col, texToUse
        );
        return;
    }

    // slow path, intersects near plane and needs clipping
    const sfrvec na = sfr_mat_mul_vec(*matNormal, (sfrvec){anx, any, anz, 0.f});
    const sfrvec nb = sfr_mat_mul_vec(*matNormal, (sfrvec){bnx, bny, bnz, 0.f});
    const sfrvec nc = sfr_mat_mul_vec(*matNormal, (sfrvec){cnx, cny, cnz, 0.f});

    f32 aIntensity = 1.f, bIntensity = 1.f, cIntensity = 1.f;
    if (sfrState.lightingEnabled) {
        const sfrvec lightDir = {sfrLight.dirX, sfrLight.dirY, sfrLight.dirZ, 0.f};
        aIntensity = sfrLight.ambient + sfr_fmaxf(0.f, sfr_vec_dot(na, lightDir)) * sfrLight.intensity;
        bIntensity = sfrLight.ambient + sfr_fmaxf(0.f, sfr_vec_dot(nb, lightDir)) * sfrLight.intensity;
        cIntensity = sfrLight.ambient + sfr_fmaxf(0.f, sfr_vec_dot(nc, lightDir)) * sfrLight.intensity;
    }

    struct sfrTexVert clipTris[16][3];
    struct sfrTexVert (*input)[3] = clipTris;
    i32 inputCount = 1;

    clipTris[0][0].pos = aClip;
    clipTris[0][1].pos = bClip;
    clipTris[0][2].pos = cClip;

    clipTris[0][0].u = au;
    clipTris[0][0].v = av;
    clipTris[0][0].normal = na;
    clipTris[0][0].intensity = aIntensity;
    clipTris[0][0].viewZ = aClip.w;

    clipTris[0][1].u = bu;
    clipTris[0][1].v = bv;
    clipTris[0][1].normal = nb;
    clipTris[0][1].intensity = bIntensity;
    clipTris[0][1].viewZ = bClip.w;

    clipTris[0][2].u = cu;
    clipTris[0][2].v = cv;
    clipTris[0][2].normal = nc;
    clipTris[0][2].intensity = cIntensity;
    clipTris[0][2].viewZ = cClip.w;

    // frustum planes in homogeneous clip space
    const sfrvec frustumPlanes[6] = {
        {0.f, 0.f, 1.f, 1.f},  // near
        {0.f, 0.f, -1.f, 1.f}, // far
        {1.f, 0.f, 0.f, 1.f},  // left
        {-1.f, 0.f, 0.f, 1.f}, // right
        {0.f, 1.f, 0.f, 1.f},  // bottom
        {0.f, -1.f, 0.f, 1.f}  // top
    };

    struct sfrTexVert buffer[SFR_ARRLEN(clipTris)][3];
    struct sfrTexVert (*output)[3] = buffer;

    // process each clipping plane
    for (i32 p = 0; p < 6; p += 1) {
        i32 outputCount = 0;
        for (i32 i = 0; i < inputCount; i += 1) {
            struct sfrTexVert clipped[2][3];
            const i32 count = sfr__clip_tri_homogeneous(clipped, frustumPlanes[p], input[i]);
            for (i32 j = 0; j < count; j += 1) {
                output[outputCount][0] = clipped[j][0];
                output[outputCount][1] = clipped[j][1];
                output[outputCount][2] = clipped[j][2];
                outputCount += 1;
            }
        }
        struct sfrTexVert (*temp)[3] = input;
        input = output;
        output = temp;
        inputCount = outputCount;
    }

    for (i32 i = 0; i < inputCount; i += 1) {
        struct sfrTexVert* tri = input[i];
        struct sfrTexVert screen[3];

        u8 skip = 0;
        for (i32 j = 0; j < 3; j += 1) {
            if (tri[j].pos.w <= SFR_EPSILON) {
                skip = 1; break;
            }
            const f32 iw = 1.f / tri[j].pos.w;
            const f32 ndcx = tri[j].pos.x * iw;
            const f32 ndcy = tri[j].pos.y * iw;
            screen[j].pos.x =  (ndcx + 1.f) * sfrState.halfWidth;
            screen[j].pos.y = (-ndcy + 1.f) * sfrState.halfHeight;
            screen[j].u = tri[j].u; screen[j].v = tri[j].v;
            screen[j].viewZ = tri[j].viewZ; screen[j].intensity = tri[j].intensity;
        }

        if (skip) {
            continue;
        }

        const f32 aInvZ = 1.f / screen[0].viewZ, bInvZ = 1.f / screen[1].viewZ, cInvZ = 1.f / screen[2].viewZ;
        sfr__bin_triangle(
            screen[0].pos.x, screen[0].pos.y, aInvZ, screen[0].u * aInvZ, screen[0].v * aInvZ,
            screen[1].pos.x, screen[1].pos.y, bInvZ, screen[1].u * bInvZ, screen[1].v * bInvZ,
            screen[2].pos.x, screen[2].pos.y, cInvZ, screen[2].u * cInvZ, screen[2].v * cInvZ,
            screen[0].intensity, screen[1].intensity, screen[2].intensity, col, texToUse
        );
    }
}

#ifndef SFR_NO_SIMD

static sfrvec8 sfr__mat_mul_vec8(const sfrmat* m, const sfrvec8* v) {
    sfrvec8 r;

    // x, col 0
    r.x = _mm256_mul_ps(  _mm256_set1_ps(m->m[0][0]), v->x);
    r.x = _mm256_fmadd_ps(_mm256_set1_ps(m->m[1][0]), v->y, r.x);
    r.x = _mm256_fmadd_ps(_mm256_set1_ps(m->m[2][0]), v->z, r.x);
    r.x = _mm256_fmadd_ps(_mm256_set1_ps(m->m[3][0]), v->w, r.x);

    // y, col 1
    r.y = _mm256_mul_ps(  _mm256_set1_ps(m->m[0][1]), v->x);
    r.y = _mm256_fmadd_ps(_mm256_set1_ps(m->m[1][1]), v->y, r.y);
    r.y = _mm256_fmadd_ps(_mm256_set1_ps(m->m[2][1]), v->z, r.y);
    r.y = _mm256_fmadd_ps(_mm256_set1_ps(m->m[3][1]), v->w, r.y);

    // z, col 2
    r.z = _mm256_mul_ps(  _mm256_set1_ps(m->m[0][2]), v->x);
    r.z = _mm256_fmadd_ps(_mm256_set1_ps(m->m[1][2]), v->y, r.z);
    r.z = _mm256_fmadd_ps(_mm256_set1_ps(m->m[2][2]), v->z, r.z);
    r.z = _mm256_fmadd_ps(_mm256_set1_ps(m->m[3][2]), v->w, r.z);

    // w, col 3
    r.w = _mm256_mul_ps(  _mm256_set1_ps(m->m[0][3]), v->x);
    r.w = _mm256_fmadd_ps(_mm256_set1_ps(m->m[1][3]), v->y, r.w);
    r.w = _mm256_fmadd_ps(_mm256_set1_ps(m->m[2][3]), v->z, r.w);
    r.w = _mm256_fmadd_ps(_mm256_set1_ps(m->m[3][3]), v->w, r.w);

    return r;
}

static __m256 sfr__vec_dot8(sfrvec8 a, sfrvec8 b) {
    const __m256 xx = _mm256_mul_ps(a.x, b.x);
    const __m256 yy = _mm256_fmadd_ps(a.y, b.y, xx);
    return _mm256_fmadd_ps(a.z, b.z, yy); 
}

static void sfr__process_and_bin_triangles8(
    const sfrmat* matMVP, const sfrmat* matNormal,
    const f32* tris, const f32* uvs, const f32* normals,
    u32 col, const SfrTexture* tex
) {
    const __m256 one = _mm256_set1_ps(1.0f);
    const __m256 zero = _mm256_setzero_ps();

    const __m256i stride_a = _mm256_set_epi32(63, 54, 45, 36, 27, 18, 9, 0);
    const __m256i stride_b = _mm256_set_epi32(66, 57, 48, 39, 30, 21, 12, 3);
    const __m256i stride_c = _mm256_set_epi32(69, 60, 51, 42, 33, 24, 15, 6);
    const __m256i stride_uv_a = _mm256_set_epi32(42, 36, 30, 24, 18, 12, 6, 0);
    const __m256i stride_uv_b = _mm256_set_epi32(44, 38, 32, 26, 20, 14, 8, 2);
    const __m256i stride_uv_c = _mm256_set_epi32(46, 40, 34, 28, 22, 16, 10, 4);

    // gather positions
    sfrvec8 a, b, c;
    a.x = _mm256_i32gather_ps(tris + 0, stride_a, 4);
    a.y = _mm256_i32gather_ps(tris + 1, stride_a, 4);
    a.z = _mm256_i32gather_ps(tris + 2, stride_a, 4);
    a.w = one;
    b.x = _mm256_i32gather_ps(tris + 0, stride_b, 4);
    b.y = _mm256_i32gather_ps(tris + 1, stride_b, 4);
    b.z = _mm256_i32gather_ps(tris + 2, stride_b, 4);
    b.w = one;
    c.x = _mm256_i32gather_ps(tris + 0, stride_c, 4);
    c.y = _mm256_i32gather_ps(tris + 1, stride_c, 4);
    c.z = _mm256_i32gather_ps(tris + 2, stride_c, 4);
    c.w = one;

    // transform
    sfrvec8 aClip = sfr__mat_mul_vec8(matMVP, &a);
    sfrvec8 bClip = sfr__mat_mul_vec8(matMVP, &b);
    sfrvec8 cClip = sfr__mat_mul_vec8(matMVP, &c);

    // z clip check
    const __m256 eps = _mm256_set1_ps(SFR_EPSILON);
    const __m256 zSafeA = _mm256_cmp_ps(aClip.w, eps, _CMP_GT_OQ);
    const __m256 zSafeB = _mm256_cmp_ps(bClip.w, eps, _CMP_GT_OQ);
    const __m256 zSafeC = _mm256_cmp_ps(cClip.w, eps, _CMP_GT_OQ);
    const __m256 zSafeAll = _mm256_and_ps(zSafeA, _mm256_and_ps(zSafeB, zSafeC));
    
    const i32 zMask = _mm256_movemask_ps(zSafeAll);

    // vv scalar fallback for clipped triangles vv
    const i32 slowMask = (~zMask) & 0xFF;
    if (0 != slowMask) {
        for (i32 i = 0; i < 8; i += 1) {
            if (!(slowMask & (1 << i))) {
                continue;
            }

            const i32 tInd = i * 9;
            const i32 uInd = i * 6;
            sfr__process_and_bin_triangle(
                matMVP, matNormal,
                tris[tInd+0], tris[tInd+1], tris[tInd+2],
                uvs ? uvs[uInd+0] : 0.f, uvs ? uvs[uInd+1] : 0.f,
                normals[tInd+0], normals[tInd+1], normals[tInd+2],
                
                tris[tInd+3], tris[tInd+4], tris[tInd+5],
                uvs ? uvs[uInd+2] : 0.f, uvs ? uvs[uInd+3] : 0.f,
                normals[tInd+3], normals[tInd+4], normals[tInd+5],
                
                tris[tInd+6], tris[tInd+7], tris[tInd+8],
                uvs ? uvs[uInd+4] : 0.f, uvs ? uvs[uInd+5] : 0.f,
                normals[tInd+6], normals[tInd+7], normals[tInd+8],
                col, tex
            );
        }
    }

    if (zMask == 0) {
        return; // all 8 needed clipping
    }

    // vv fast path vv
    const __m256 aInvW = _mm256_div_ps(one, aClip.w);
    const __m256 bInvW = _mm256_div_ps(one, bClip.w);
    const __m256 cInvW = _mm256_div_ps(one, cClip.w);

    const __m256 halfW = _mm256_set1_ps(sfrState.halfWidth);
    const __m256 halfH = _mm256_set1_ps(sfrState.halfHeight);

    //  aClip.x * aInvW + 1.f
    // -aClip.y * aInvW + 1.f
    const __m256 sax = _mm256_mul_ps(_mm256_fmadd_ps(aClip.x, aInvW, one), halfW);
    const __m256 say = _mm256_mul_ps(_mm256_fnmadd_ps(aClip.y, aInvW, one), halfH); 
    const __m256 sbx = _mm256_mul_ps(_mm256_fmadd_ps(bClip.x, bInvW, one), halfW);
    const __m256 sby = _mm256_mul_ps(_mm256_fnmadd_ps(bClip.y, bInvW, one), halfH);
    const __m256 scx = _mm256_mul_ps(_mm256_fmadd_ps(cClip.x, cInvW, one), halfW);
    const __m256 scy = _mm256_mul_ps(_mm256_fnmadd_ps(cClip.y, cInvW, one), halfH);

    // area and screen culling
    const __m256 abx = _mm256_sub_ps(sbx, sax);
    const __m256 acy = _mm256_sub_ps(scy, say);
    const __m256 aby = _mm256_sub_ps(sby, say);
    const __m256 acx = _mm256_sub_ps(scx, sax);
    const __m256 area = _mm256_fmsub_ps(abx, acy, _mm256_mul_ps(aby, acx));

    // mask starts with lanes that survived near plane
    __m256 cullMask = zSafeAll;

    #ifndef SFR_NO_CULLING
        cullMask = _mm256_and_ps(cullMask, _mm256_cmp_ps(area, zero, _CMP_GT_OQ));
    #endif

    #ifndef SFR_NO_SMALL_CULLING
        cullMask = _mm256_and_ps(cullMask, _mm256_cmp_ps(area, _mm256_set1_ps(0.1f), _CMP_GE_OQ));
    #endif

    // guard band culling
    const __m256 gb = _mm256_set1_ps(1.1f * sfrState.halfWidth);
    const __m256 negGb = _mm256_sub_ps(zero, gb);
    const __m256 widthGb = _mm256_set1_ps(sfrWidth + 1.1f * sfrState.halfWidth);
    const __m256 heightGb = _mm256_set1_ps(sfrHeight + 1.1f * sfrState.halfWidth);

    const __m256 left = _mm256_and_ps(_mm256_cmp_ps(sax, negGb, _CMP_LT_OQ),
        _mm256_and_ps(_mm256_cmp_ps(sbx, negGb, _CMP_LT_OQ), _mm256_cmp_ps(scx, negGb, _CMP_LT_OQ)));
    const __m256 right = _mm256_and_ps(_mm256_cmp_ps(sax, widthGb, _CMP_GT_OQ),
        _mm256_and_ps(_mm256_cmp_ps(sbx, widthGb, _CMP_GT_OQ), _mm256_cmp_ps(scx, widthGb, _CMP_GT_OQ)));
    const __m256 top = _mm256_and_ps(_mm256_cmp_ps(say, negGb, _CMP_LT_OQ),
        _mm256_and_ps(_mm256_cmp_ps(sby, negGb, _CMP_LT_OQ), _mm256_cmp_ps(scy, negGb, _CMP_LT_OQ)));
    const __m256 bottom = _mm256_and_ps(_mm256_cmp_ps(say, heightGb, _CMP_GT_OQ),
        _mm256_and_ps(_mm256_cmp_ps(sby, heightGb, _CMP_GT_OQ), _mm256_cmp_ps(scy, heightGb, _CMP_GT_OQ)));

    const __m256 cullGb = _mm256_or_ps(left, _mm256_or_ps(right, _mm256_or_ps(top, bottom)));
    cullMask = _mm256_andnot_ps(cullGb, cullMask);

    const i32 activeLanes = _mm256_movemask_ps(cullMask);
    if (0 == activeLanes) {
        return;
    }

    // lighting
    __m256 aInt = one, bInt = one, cInt = one;
    if (sfrState.lightingEnabled) {
        sfrvec8 an, bn, cn;
        an.x = _mm256_i32gather_ps(normals + 0, stride_a, 4); an.y = _mm256_i32gather_ps(normals + 1, stride_a, 4); an.z = _mm256_i32gather_ps(normals + 2, stride_a, 4); an.w = zero;
        bn.x = _mm256_i32gather_ps(normals + 0, stride_b, 4); bn.y = _mm256_i32gather_ps(normals + 1, stride_b, 4); bn.z = _mm256_i32gather_ps(normals + 2, stride_b, 4); bn.w = zero;
        cn.x = _mm256_i32gather_ps(normals + 0, stride_c, 4); cn.y = _mm256_i32gather_ps(normals + 1, stride_c, 4); cn.z = _mm256_i32gather_ps(normals + 2, stride_c, 4); cn.w = zero;

        const sfrvec8 na = sfr__mat_mul_vec8(matNormal, &an);
        const sfrvec8 nb = sfr__mat_mul_vec8(matNormal, &bn);
        const sfrvec8 nc = sfr__mat_mul_vec8(matNormal, &cn);

        const sfrvec8 lightDir = { _mm256_set1_ps(sfrLight.dirX), _mm256_set1_ps(sfrLight.dirY), _mm256_set1_ps(sfrLight.dirZ), zero };
        const __m256 ambient = _mm256_set1_ps(sfrLight.ambient);
        const __m256 intensity = _mm256_set1_ps(sfrLight.intensity);

        aInt = _mm256_fmadd_ps(_mm256_max_ps(zero, sfr__vec_dot8(na, lightDir)), intensity, ambient);
        bInt = _mm256_fmadd_ps(_mm256_max_ps(zero, sfr__vec_dot8(nb, lightDir)), intensity, ambient);
        cInt = _mm256_fmadd_ps(_mm256_max_ps(zero, sfr__vec_dot8(nc, lightDir)), intensity, ambient);
    }

    // gather uvs
    __m256 auX = zero, auY = zero, buX = zero, buY = zero, cuX = zero, cuY = zero;
    if (uvs) {
        auX = _mm256_i32gather_ps(uvs + 0, stride_uv_a, 4); auY = _mm256_i32gather_ps(uvs + 1, stride_uv_a, 4);
        buX = _mm256_i32gather_ps(uvs + 0, stride_uv_b, 4); buY = _mm256_i32gather_ps(uvs + 1, stride_uv_b, 4);
        cuX = _mm256_i32gather_ps(uvs + 0, stride_uv_c, 4); cuY = _mm256_i32gather_ps(uvs + 1, stride_uv_c, 4);
    }

    // store to buffers and bin
    // TODO next thing that should be fixed later when meshes are loaded in SoA format
    f32 SFR_ALIGNED(32) saxBuf[8], sayBuf[8], aInvWBuf[8], auBuf[8], avBuf[8], aIntBuf[8];
    f32 SFR_ALIGNED(32) sbxBuf[8], sbyBuf[8], bInvWBuf[8], buBuf[8], bvBuf[8], bIntBuf[8];
    f32 SFR_ALIGNED(32) scxBuf[8], scyBuf[8], cInvWBuf[8], cuBuf[8], cvBuf[8], cIntBuf[8];

    _mm256_store_ps(saxBuf, sax); _mm256_store_ps(sayBuf, say); _mm256_store_ps(aInvWBuf, aInvW);
    _mm256_store_ps(auBuf, _mm256_mul_ps(auX, aInvW)); _mm256_store_ps(avBuf, _mm256_mul_ps(auY, aInvW)); _mm256_store_ps(aIntBuf, aInt);
    
    _mm256_store_ps(sbxBuf, sbx); _mm256_store_ps(sbyBuf, sby); _mm256_store_ps(bInvWBuf, bInvW);
    _mm256_store_ps(buBuf, _mm256_mul_ps(buX, bInvW)); _mm256_store_ps(bvBuf, _mm256_mul_ps(buY, bInvW)); _mm256_store_ps(bIntBuf, bInt);
    
    _mm256_store_ps(scxBuf, scx); _mm256_store_ps(scyBuf, scy); _mm256_store_ps(cInvWBuf, cInvW);
    _mm256_store_ps(cuBuf, _mm256_mul_ps(cuX, cInvW)); _mm256_store_ps(cvBuf, _mm256_mul_ps(cuY, cInvW)); _mm256_store_ps(cIntBuf, cInt);

    const SfrTexture* texToUse = tex ? tex : &sfrState.baseTex;

    for (i32 i = 0; i < 8; i += 1) {
        if (activeLanes & (1 << i)) {
            sfr__bin_triangle(
                saxBuf[i], sayBuf[i], aInvWBuf[i], auBuf[i], avBuf[i],
                sbxBuf[i], sbyBuf[i], bInvWBuf[i], buBuf[i], bvBuf[i],
                scxBuf[i], scyBuf[i], cInvWBuf[i], cuBuf[i], cvBuf[i],
                aIntBuf[i], bIntBuf[i], cIntBuf[i], col, texToUse
            );
        }
    }
}

static void sfr__resolve_chunk(u32 firstId, i32 globalX, i32 globalY, i32 globalInd, __m256i writeMask) {
    #ifdef SFR_MULTITHREADED
        const struct sfrTriangleBin* bin = &sfrThreadBuf->binPages[firstId / SFR_BIN_PAGE_SIZE][firstId % SFR_BIN_PAGE_SIZE];
    #else
        const struct sfrTriangleBin* bin = &sfrState.globalBins[firstId];
    #endif

    const SfrTexture* tex = bin->tex;
    const u8 isPot = (0 == (tex->w & (tex->w - 1))) && (0 == (tex->h & (tex->h - 1)));

    const f32 cx = globalX + 3.5f;
    const f32 cy = globalY + 0.5f;
    const f32 cz0 = bin->zBase + cx * bin->dzdx + cy * bin->dzdy;
    const f32 cu0 = bin->uBase + cx * bin->dudx + cy * bin->dudy;
    const f32 cv0 = bin->vBase + cx * bin->dvdx + cy * bin->dvdy;
    const f32 invZ0 = 1.f / cz0;

    const f32 czX = cz0 + bin->dzdx, cuX = cu0 + bin->dudx, cvX = cv0 + bin->dvdx;
    const f32 invZX = 1.f / czX;

    const f32 czY = cz0 + bin->dzdy, cuY = cu0 + bin->dudy, cvY = cv0 + bin->dvdy;
    const f32 invZY = 1.f / czY;

    const f32 U0 = cu0 * invZ0 * (f32)tex->w, V0 = cv0 * invZ0 * (f32)tex->h;
    const f32 UX = cuX * invZX * (f32)tex->w, VX = cvX * invZX * (f32)tex->h;
    const f32 UY = cuY * invZY * (f32)tex->w, VY = cvY * invZY * (f32)tex->h;

    const f32 rhoX = sfr_fmaxf(sfr_fabsf(UX - U0), sfr_fabsf(VX - V0));
    const f32 rhoY = sfr_fmaxf(sfr_fabsf(UY - U0), sfr_fabsf(VY - V0));
    const f32 rhoMax = sfr_fmaxf(rhoX, rhoY);

    i32 lod = 0;
    if (rhoMax > 1.f) {
        lod = sfr__fast_log2(rhoMax);
    }
    lod = SFR_CLAMP(lod, 0, tex->mipLevels - 1);

    const u32* const currentMipPixels = (0 == lod) ? tex->pixels : tex->mipPixels[lod - 1];
    const i32 currentTexW = (0 == lod) ? tex->w : tex->mipW[lod - 1];
    const i32 currentTexH = (0 == lod) ? tex->h : tex->mipH[lod - 1];

    const f32 rBase = ((f32)((bin->col >> 16) & 0xFF) / 255.f) * sfrLight.r;
    const f32 gBase = ((f32)((bin->col >> 8)  & 0xFF) / 255.f) * sfrLight.g;
    const f32 bBase = ((f32)((bin->col >> 0)  & 0xFF) / 255.f) * sfrLight.b;

    const __m256 vRBase = _mm256_set1_ps(rBase);
    const __m256 vGBase = _mm256_set1_ps(gBase);
    const __m256 vBBase = _mm256_set1_ps(bBase);

    const __m256 vX = _mm256_add_ps(_mm256_set1_ps((f32)globalX), _mm256_setr_ps(0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f));
    const __m256 vY = _mm256_set1_ps((f32)globalY);

    const __m256 vu = _mm256_add_ps(_mm256_set1_ps(bin->uBase),
        _mm256_add_ps(_mm256_mul_ps(_mm256_set1_ps(bin->dudx), vX), _mm256_mul_ps(_mm256_set1_ps(bin->dudy), vY)));
    const __m256 vv = _mm256_add_ps(_mm256_set1_ps(bin->vBase),
        _mm256_add_ps(_mm256_mul_ps(_mm256_set1_ps(bin->dvdx), vX), _mm256_mul_ps(_mm256_set1_ps(bin->dvdy), vY)));
    const __m256 vi = _mm256_add_ps(_mm256_set1_ps(bin->iBase),
        _mm256_add_ps(_mm256_mul_ps(_mm256_set1_ps(bin->didx), vX), _mm256_mul_ps(_mm256_set1_ps(bin->didy), vY)));
    const __m256 vz = _mm256_add_ps(_mm256_set1_ps(bin->zBase),
        _mm256_add_ps(_mm256_mul_ps(_mm256_set1_ps(bin->dzdx), vX), _mm256_mul_ps(_mm256_set1_ps(bin->dzdy), vY)));

    const __m256 vRecipZ = _mm256_div_ps(_mm256_set1_ps(1.f), vz);

    __m256 vCorrectU = _mm256_mul_ps(vu, vRecipZ);
    __m256 vCorrectV = _mm256_mul_ps(vv, vRecipZ);
    const __m256 vCorrectI = _mm256_mul_ps(vi, vRecipZ);

    __m256i vTexX, vTexY;
    const __m256 vFTexW = _mm256_cvtepi32_ps(_mm256_set1_epi32(currentTexW));
    const __m256 vFTexH = _mm256_cvtepi32_ps(_mm256_set1_epi32(currentTexH));

    if (isPot) {
        vCorrectU = _mm256_mul_ps(vCorrectU, vFTexW);
        vCorrectV = _mm256_mul_ps(vCorrectV, vFTexH);

        vTexX = _mm256_cvttps_epi32(vCorrectU);
        vTexY = _mm256_cvttps_epi32(vCorrectV);

        vTexX = _mm256_and_si256(vTexX, _mm256_set1_epi32(currentTexW - 1));
        vTexY = _mm256_and_si256(vTexY, _mm256_set1_epi32(currentTexH - 1));
    } else {
        vCorrectU = _mm256_sub_ps(vCorrectU, _mm256_floor_ps(vCorrectU));
        vCorrectV = _mm256_sub_ps(vCorrectV, _mm256_floor_ps(vCorrectV));
        vCorrectU = _mm256_mul_ps(vCorrectU, vFTexW);
        vCorrectV = _mm256_mul_ps(vCorrectV, vFTexH);

        vTexX = _mm256_cvttps_epi32(vCorrectU);
        vTexY = _mm256_cvttps_epi32(vCorrectV);

        vTexX = _mm256_min_epi32(vTexX, _mm256_set1_epi32(currentTexW - 1));
        vTexY = _mm256_min_epi32(vTexY, _mm256_set1_epi32(currentTexH - 1));
    }

    const __m256i vTexInds = _mm256_add_epi32(_mm256_mullo_epi32(vTexY, _mm256_set1_epi32(currentTexW)), vTexX);

    const __m256i vTexPixels = _mm256_i32gather_epi32((const i32*)currentMipPixels, vTexInds, 4);

    const __m256i v0xFF = _mm256_set1_epi32(0xFF);
    const __m256 vtr = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(vTexPixels, 16), v0xFF));
    const __m256 vtg = _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(vTexPixels, 8),  v0xFF));
    const __m256 vtb = _mm256_cvtepi32_ps(_mm256_and_si256(vTexPixels, v0xFF));

    const __m256 v255 = _mm256_set1_ps(255.f);
    const __m256 vfr = _mm256_min_ps(_mm256_mul_ps(vtr, _mm256_mul_ps(vCorrectI, vRBase)), v255);
    const __m256 vfg = _mm256_min_ps(_mm256_mul_ps(vtg, _mm256_mul_ps(vCorrectI, vGBase)), v255);
    const __m256 vfb = _mm256_min_ps(_mm256_mul_ps(vtb, _mm256_mul_ps(vCorrectI, vBBase)), v255);

    const __m256i vrr = _mm256_cvtps_epi32(vfr);
    const __m256i vrg = _mm256_cvtps_epi32(vfg);
    const __m256i vrb = _mm256_cvtps_epi32(vfb);

    const __m256i vFinalPixel = _mm256_or_si256(
        _mm256_set1_epi32(0xFF000000),
        _mm256_or_si256(_mm256_slli_epi32(vrr, 16), _mm256_or_si256(_mm256_slli_epi32(vrg, 8), vrb)));

    __m256i vOldPix = _mm256_loadu_si256((__m256i*)&sfrPixelBuf[globalInd]);
    __m256 vNewPixF = _mm256_blendv_ps(_mm256_castsi256_ps(vOldPix),
                                       _mm256_castsi256_ps(vFinalPixel),
                                       _mm256_castsi256_ps(writeMask));
    _mm256_storeu_si256((__m256i*)&sfrPixelBuf[globalInd], _mm256_castps_si256(vNewPixF));}
#endif

static void sfr__resolve_pixel(u32 id, i32 globalX, i32 globalY, i32 globalInd) {
    #ifdef SFR_MULTITHREADED
        const struct sfrTriangleBin* bin = &sfrThreadBuf->binPages[id / SFR_BIN_PAGE_SIZE][id % SFR_BIN_PAGE_SIZE];
    #else
        const struct sfrTriangleBin* bin = &sfrState.globalBins[id];
    #endif

    const SfrTexture* const tex = bin->tex;
    const i32 texW = tex->w, texH = tex->h;
    const u8 isPot = (0 == (texW & (texW - 1))) && (0 == (texH & (texH - 1)));

    const f32 cx = (f32)globalX + 0.5f;
    const f32 cy = (f32)globalY + 0.5f;
    const f32 cz0 = bin->zBase + cx * bin->dzdx + cy * bin->dzdy;
    const f32 cu0 = bin->uBase + cx * bin->dudx + cy * bin->dudy;
    const f32 cv0 = bin->vBase + cx * bin->dvdx + cy * bin->dvdy;
    const f32 invZ0 = 1.f / cz0;

    const f32 czX = cz0 + bin->dzdx, cuX = cu0 + bin->dudx, cvX = cv0 + bin->dvdx;
    const f32 invZX = 1.f / czX;

    const f32 czY = cz0 + bin->dzdy, cuY = cu0 + bin->dudy, cvY = cv0 + bin->dvdy;
    const f32 invZY = 1.f / czY;

    const f32 U0 = cu0 * invZ0 * (f32)tex->w, V0 = cv0 * invZ0 * (f32)tex->h;
    const f32 UX = cuX * invZX * (f32)tex->w, VX = cvX * invZX * (f32)tex->h;
    const f32 UY = cuY * invZY * (f32)tex->w, VY = cvY * invZY * (f32)tex->h;

    const f32 rhoX = sfr_fmaxf(sfr_fabsf(UX - U0), sfr_fabsf(VX - V0));
    const f32 rhoY = sfr_fmaxf(sfr_fabsf(UY - U0), sfr_fabsf(VY - V0));
    const f32 rhoMax = sfr_fmaxf(rhoX, rhoY);

    i32 lod = 0;
    if (rhoMax > 1.f) {
        lod = sfr__fast_log2(rhoMax);
    }
    lod = SFR_CLAMP(lod, 0, tex->mipLevels - 1);

    const u32* const currentMipPixels = (0 == lod) ? tex->pixels : tex->mipPixels[lod - 1];
    const i32 currentTexW = (0 == lod) ? tex->w : tex->mipW[lod - 1];
    const i32 currentTexH = (0 == lod) ? tex->h : tex->mipH[lod - 1];

    const f32 gx = (f32)globalX;
    const f32 gy = (f32)globalY;

    const f32 z = bin->zBase + bin->dzdx * gx + bin->dzdy * gy;
    const f32 invZ = 1.f / z;
    f32 cu = (bin->uBase + bin->dudx * gx + bin->dudy * gy) * invZ;
    f32 cv = (bin->vBase + bin->dvdx * gx + bin->dvdy * gy) * invZ;
    const f32 ci = (bin->iBase + bin->didx * gx + bin->didy * gy) * invZ;

    i32 tx, ty;
    if (isPot) {
        cu *= (f32)currentTexW;
        cv *= (f32)currentTexH;
        tx = (i32)cu & (currentTexW - 1);
        ty = (i32)cv & (currentTexH - 1);
    } else {
        cu -= sfr_floorf(cu);
        cv -= sfr_floorf(cv);
        tx = (i32)(cu * currentTexW);
        ty = (i32)(cv * currentTexH);
        if (tx >= currentTexW) tx = currentTexW - 1;
        if (ty >= currentTexH) ty = currentTexH - 1;
    }

    const u32 texInd = (u32)(ty * currentTexW + tx);
    const u32 texCol = currentMipPixels[texInd];

    const f32 rBase = ((f32)((bin->col >> 16) & 0xFF) / 255.f) * sfrLight.r;
    const f32 gBase = ((f32)((bin->col >> 8)  & 0xFF) / 255.f) * sfrLight.g;
    const f32 bBase = ((f32)((bin->col >> 0)  & 0xFF) / 255.f) * sfrLight.b;

    const u8 tr = (texCol >> 16) & 0xFF, tg = (texCol >> 8) & 0xFF, tb = (texCol >> 0) & 0xFF;
    const u8 fr = (u8)sfr_fminf((tr * ci * rBase), 255.f);
    const u8 fg = (u8)sfr_fminf((tg * ci * gBase), 255.f);
    const u8 fb = (u8)sfr_fminf((tb * ci * bBase), 255.f);

    sfrPixelBuf[globalInd] = (0xFF << 24) | (fr << 16) | (fg << 8) | fb;
}

#ifdef SFR_MULTITHREADED

// pulls the current global screen state into the tile's
static void sfr__load_tile(struct sfrTile* tile) {
    const i32 w = tile->maxX - tile->minX;
    const i32 h = tile->maxY - tile->minY;

    for (i32 y = 0; y < h; y += 1) {
        const i32 globalY = tile->minY + y;
        const i32 globalInd = globalY * sfrWidth + tile->minX;

        // the stride in the local buffer is always SFR_TILE_WIDTH even if w is smaller
        const i32 localInd = y * SFR_TILE_WIDTH;

        sfr_memcpy(&tile->localDepth[localInd], &sfrDepthBuf[globalInd], sizeof(f32) * w);
        sfr_memset(&tile->localId[localInd], -1, sizeof(i32) * w);
    }
}

#endif // SFR_MULTITHREADED

static void sfr__resolve_tile(const struct sfrTile* tile) {
    const i32 w = tile->maxX - tile->minX;
    const i32 h = tile->maxY - tile->minY;

    #ifdef SFR_MULTITHREADED
        const i32* const targetIdBuf = tile->localId;
    #else
        const i32* const targetIdBuf = sfrState.idBuf;
    #endif

    for (i32 y = 0; y < h; y += 1) {
        const i32 globalY = tile->minY + y;
        const i32 globalIndBase = globalY * sfrWidth + tile->minX;

        #ifdef SFR_MULTITHREADED
            const i32 bufIndBase = y * SFR_TILE_WIDTH;
        #else
            const i32 bufIndBase = globalIndBase;
        #endif

        i32 x = 0;

        #ifndef SFR_NO_SIMD
            for (; x <= w - 8; x += 8) {
                const i32 bufInd = bufIndBase + x;
                const i32 globalInd = globalIndBase + x;
                const i32 globalX = tile->minX + x;

                const __m256i vIds = _mm256_loadu_si256((__m256i*)&targetIdBuf[bufInd]);

                i32 processedMask = 0;

                // loop until all 8 pixels in this chunk have been resolved
                while (0xFF != processedMask) {
                    // find the id of the first pixel not yet processed
                    const i32 remainingBits = (~processedMask) & 0xFF;
                    i32 firstUnprocessedInd = 0;
                    while (0 == (remainingBits & (1 << firstUnprocessedInd))) {
                        firstUnprocessedInd += 1;
                    }
                    const u32 currentId = targetIdBuf[bufInd + firstUnprocessedInd];

                    // find all pixels in this block that share this exact same id
                    const __m256i vCmpMask = _mm256_cmpeq_epi32(vIds, _mm256_set1_epi32(currentId));
                    const i32 currentLaneMask = _mm256_movemask_ps(_mm256_castsi256_ps(vCmpMask));

                    if (-1 != currentId) {
                        sfr__resolve_chunk(currentId, globalX, globalY, globalInd, vCmpMask);
                    }

                    // mark these lanes as processed
                    processedMask |= currentLaneMask;
                }
            }
        #endif

        // tail loop / SFR_NO_SIMD
        for (; x < w; x += 1) {
            const i32 bufInd = bufIndBase + x;
            const i32 globalInd = globalIndBase + x;
            const u32 id = targetIdBuf[bufInd];
            if (-1 != id) {
                sfr__resolve_pixel(id, tile->minX + x, globalY, globalInd);
            }
        }
    }
}

// inverse transpose of model
static sfrmat sfr__calc_normal_mat(sfrmat model) {
    const f32* m = &model.m[0][0];

    const f32 a = m[0], b = m[4], c = m[8];
    const f32 d = m[1], e = m[5], f = m[9];
    const f32 g = m[2], h = m[6], i = m[10];

    const f32 det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    const f32 invDet = (0 != det) ? 1.f / det : 0.f;

    sfrmat res = (sfrmat){
        .rows = {
            [0] = (sfrvec){ .x = (e * i - f * h) * invDet, .y = (c * h - b * i) * invDet, .z = (b * f - c * e) * invDet, .w = 0.f },
            [1] = (sfrvec){ .x = (f * g - d * i) * invDet, .y = (a * i - c * g) * invDet, .z = (c * d - a * f) * invDet, .w = 0.f },
            [2] = (sfrvec){ .x = (d * h - e * g) * invDet, .y = (b * g - a * h) * invDet, .z = (a * e - b * d) * invDet, .w = 0.f },
            [3] = (sfrvec){ .x = 0.f,                      .y = 0.f,                      .z = 0.f,                      .w = 1.f }
        }
    };

    // pre normalize
    res.rows[0] = sfr_vec_norm(res.rows[0]);
    res.rows[1] = sfr_vec_norm(res.rows[1]);
    res.rows[2] = sfr_vec_norm(res.rows[2]);
    return res;
}

// helper for updating normal mat used for shading
static void sfr__update_normal_mat(void) {
    sfrState.matNormal = sfr__calc_normal_mat(sfrMatModel);
    sfrState.normalMatDirty = 0;
}

static void sfr__triangle_tex_norm(
    f32 ax, f32 ay, f32 az, f32 au, f32 av, f32 anx, f32 any, f32 anz,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv, f32 bnx, f32 bny, f32 bnz,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv, f32 cnx, f32 cny, f32 cnz,
    u32 col, const SfrTexture* tex
) {
    if (sfrState.normalMatDirty) {
        sfr__update_normal_mat();
    }

    const sfrmat matMVP = sfr_mat_mul(sfr_mat_mul(sfrMatModel, sfrMatView), sfrMatProj);

    sfr__process_and_bin_triangle(
        &matMVP, &sfrState.matNormal,
        ax, ay, az, au, av, anx, any, anz,
        bx, by, bz, bu, bv, bnx, bny, bnz,
        cx, cy, cz, cu, cv, cnx, cny, cnz,
        col, tex
    );
}

// slab method
static u8 sfr__intersect_bounds(sfrvec rayOrigin, sfrvec rayDirInv, struct sfrBounds box, f32 dist) {
    const f32 tx1 = (box.minX - rayOrigin.x) * rayDirInv.x;
    const f32 tx2 = (box.maxX - rayOrigin.x) * rayDirInv.x;
    const f32 ty1 = (box.minY - rayOrigin.y) * rayDirInv.y;
    const f32 ty2 = (box.maxY - rayOrigin.y) * rayDirInv.y;
    const f32 tz1 = (box.minZ - rayOrigin.z) * rayDirInv.z;
    const f32 tz2 = (box.maxZ - rayOrigin.z) * rayDirInv.z;

    const f32 tmin = sfr_fmaxf(sfr_fmaxf(sfr_fminf(tx1, tx2), sfr_fminf(ty1, ty2)), sfr_fminf(tz1, tz2));
    const f32 tmax = sfr_fminf(sfr_fminf(sfr_fmaxf(tx1, tx2), sfr_fmaxf(ty1, ty2)), sfr_fmaxf(tz1, tz2));

    return tmax >= tmin && tmin < dist && tmax > 0;
}

// updates t, u, v if closer hit found
static u8 sfr__intersect_triangle(
    sfrvec rayOrigin, sfrvec rayDir,
    sfrvec v0, sfrvec v1, sfrvec v2,
    f32* t, f32* u, f32* v
) {
    const sfrvec v0v1 = sfr_vec_sub(v1, v0);
    const sfrvec v0v2 = sfr_vec_sub(v2, v0);
    const sfrvec pvec = sfr_vec_cross(rayDir, v0v2);
    const f32 det = sfr_vec_dot(v0v1, pvec);

    if (det > -SFR_EPSILON && det < SFR_EPSILON) {
        return 0;
    }

    const f32 invDet = 1.f / det;
    const sfrvec tvec = sfr_vec_sub(rayOrigin, v0);
    const f32 valU = sfr_vec_dot(tvec, pvec) * invDet;

    if (valU < 0.f || valU > 1.f) {
        return 0;
    }

    const sfrvec qvec = sfr_vec_cross(tvec, v0v1);
    const f32 valV = sfr_vec_dot(rayDir, qvec) * invDet;

    if (valV < 0.f || valU + valV > 1.f) {
        return 0;
    }

    const f32 valT = sfr_vec_dot(v0v2, qvec) * invDet;

    // if this hit is closer than the current best
    if (valT > SFR_EPSILON && valT < *t) {
        *t = valT;
        *u = valU;
        *v = valV;
        return 1;
    }

    return 0;
}

static struct sfrBounds sfr__calc_mesh_bounds(const SfrMesh* mesh) {
    struct sfrBounds b = {0};

    for (i32 i = 0; i < mesh->vertCount; i += 9) {
        const f32 ax = mesh->tris[i + 0];
        const f32 ay = mesh->tris[i + 1];
        const f32 az = mesh->tris[i + 2];
        const f32 bx = mesh->tris[i + 3];
        const f32 by = mesh->tris[i + 4];
        const f32 bz = mesh->tris[i + 5];
        const f32 cx = mesh->tris[i + 6];
        const f32 cy = mesh->tris[i + 7];
        const f32 cz = mesh->tris[i + 8];

        b.minX = sfr_fminf(b.minX, sfr_fminf(ax, sfr_fminf(bx, cx)));
        b.minY = sfr_fminf(b.minY, sfr_fminf(ay, sfr_fminf(by, cy)));
        b.minZ = sfr_fminf(b.minZ, sfr_fminf(az, sfr_fminf(bz, cz)));

        b.maxX = sfr_fmaxf(b.maxX, sfr_fmaxf(ax, sfr_fmaxf(bx, cx)));
        b.maxY = sfr_fmaxf(b.maxY, sfr_fmaxf(ay, sfr_fmaxf(by, cy)));
        b.maxZ = sfr_fmaxf(b.maxZ, sfr_fmaxf(az, sfr_fmaxf(bz, cz)));
    }

    return b;
}

static u32 sfr__lerp_col(u32 c1, u32 c2, f32 t) {
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

static void sfr__generate_mipmaps(SfrTexture* tex) {
    tex->mipLevels = 1;
    i32 cw = tex->w;
    i32 ch = tex->h;
    u32* src = tex->pixels;

    // loop until 1x1
    while (cw > 1 || ch > 1) {
        const i32 nw = cw > 1 ? cw / 2 : 1;
        const i32 nh = ch > 1 ? ch / 2 : 1;

        const i32 mipInd = tex->mipLevels - 1;
        u32* dest = (u32*)sfrMalloc(nw * nh * sizeof(u32));

        tex->mipPixels[mipInd] = dest;
        tex->mipW[mipInd] = nw;
        tex->mipH[mipInd] = nh;

        for (i32 y = 0; y < nh; y += 1) {
            for (i32 x = 0; x < nw; x += 1) {
                const i32 sx = x * 2;
                const i32 sy = y * 2;

                const u32 c00 = src[sy * cw + sx];
                const u32 c01 = (sx + 1 < cw) ? src[sy * cw + (sx + 1)] : c00;
                const u32 c10 = (sy + 1 < ch) ? src[(sy + 1) * cw + sx] : c00;
                const u32 c11 = ((sx + 1 < cw) && (sy + 1 < ch)) ? src[(sy + 1) * cw + (sx + 1)] : c00;

                const u32 r = (((c00 >> 16) & 0xFF) + ((c01 >> 16) & 0xFF) + ((c10 >> 16) & 0xFF) + ((c11 >> 16) & 0xFF)) / 4;
                const u32 g = (((c00 >> 8) & 0xFF)  + ((c01 >> 8) & 0xFF)  + ((c10 >> 8) & 0xFF)  + ((c11 >> 8) & 0xFF)) / 4;
                const u32 b = (((c00 >> 0) & 0xFF)  + ((c01 >> 0) & 0xFF)  + ((c10 >> 0) & 0xFF)  + ((c11 >> 0) & 0xFF)) / 4;
                const u32 a = (((c00 >> 24) & 0xFF) + ((c01 >> 24) & 0xFF) + ((c10 >> 24) & 0xFF) + ((c11 >> 24) & 0xFF)) / 4;

                dest[y * nw + x] = (a << 24) | (r << 16) | (g << 8) | b;
            }
        }

        src = dest;
        cw = nw;
        ch = nh;
        tex->mipLevels += 1;
    }
}


//================================================
//: PUBLIC API FUNCTION DEFINITIONS
//================================================

SFR_FUNC void sfr_init(i32 w, i32 h, f32 fovDeg, void* (*mallocFunc)(u64), void (*freeFunc)(void*), void* (*reallocFunc)(void*, u64)) {
    if (!mallocFunc || !freeFunc || !reallocFunc) {
        SFR__ERR_EXIT("sfr_init: malloc, free, and realloc must be provided\n");
    }
    sfrMalloc = mallocFunc, sfrFree = freeFunc, sfrRealloc = reallocFunc;

    if (w > SFR_MAX_WIDTH) {
        SFR__ERR_EXIT("sfr_init: width > SFR_MAX_WIDTH (%d > %d)\n", w, SFR_MAX_WIDTH);
    }
    if (h > SFR_MAX_HEIGHT) {
        SFR__ERR_EXIT("sfr_init: height > SFR_MAX_HEIGHT (%d > %d)\n", w, SFR_MAX_HEIGHT);
    }

    sfrPixelBuf = (u32*)sfrMalloc(sizeof(u32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
    if (!sfrPixelBuf) {
        SFR__ERR_EXIT("sfr_init: failed to allocate sfrPixelBuf (%ld bytes)\n",
            sizeof(u32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
    }

    sfrDepthBuf = (f32*)sfrMalloc(sizeof(f32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
    if (!sfrPixelBuf) {
        sfrFree(sfrPixelBuf);
        SFR__ERR_EXIT("sfr_init: failed to allocate sfrPixelBuf (%ld bytes)\n",
            sizeof(u32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
    }

    #ifdef SFR_MULTITHREADED
        sfrThreadBuf = (struct sfrThreadBuf*)sfrMalloc(sizeof(struct sfrThreadBuf));
        if (!sfrThreadBuf) {
            sfrFree(sfrPixelBuf);
            sfrFree(sfrDepthBuf);
            SFR__ERR_EXIT("sfr_init: failed to allocate sfrThreadBuf (%ld bytes)\n", sizeof(struct sfrThreadBuf));
        }

        sfr_mutex_init(&sfrThreadBuf->binPoolMutex);
        sfr_mutex_init(&sfrThreadBuf->geometryMutex);

        sfrThreadBuf->binPagesCapacity = 64;
        sfrThreadBuf->binPages = (struct sfrTriangleBin**)sfrMalloc(sizeof(struct sfrTriangleBin*) * sfrThreadBuf->binPagesCapacity);
        sfr_memset(sfrThreadBuf->binPages, 0, sizeof(struct sfrTriangleBin*) * sfrThreadBuf->binPagesCapacity);

        // pre allocate first page
        sfrThreadBuf->binPages[0] = (struct sfrTriangleBin*)sfrMalloc(sizeof(struct sfrTriangleBin) * SFR_BIN_PAGE_SIZE);
        sfrThreadBuf->meshJobPoolCapacity = 1024 * 8;
        sfrThreadBuf->meshJobPool = (struct sfrMeshChunkJob*)sfrMalloc(sizeof(struct sfrMeshChunkJob) * sfrThreadBuf->meshJobPoolCapacity);

        sfrThreadBuf->geometryWorkQueueCapacity = 1024 * 8;
        sfrThreadBuf->geometryWorkQueue = (i32*)sfrMalloc(sizeof(i32) * sfrThreadBuf->geometryWorkQueueCapacity);

        if (!sfrThreadBuf->binPages || !sfrThreadBuf->meshJobPool || !sfrThreadBuf->geometryWorkQueue) {
            SFR__ERR_EXIT("sfr_init: failed to allocate dynamic buffers\n");
        }

        sfrState.shutdown = 0;
        sfr_atomic_set(&sfrThreadBuf->triangleBinAllocator, 0);
        sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueCount, 0);
        sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueHead, 0);

        sfr_atomic_set(&sfrThreadBuf->meshJobAllocator, 0);
        sfr_atomic_set(&sfrThreadBuf->geometryWorkQueueCount, 0);
        sfr_atomic_set(&sfrThreadBuf->geometryWorkQueueHead, 0);

        sfr_semaphore_init(&sfrThreadBuf->geometryStartSem, 0);
        sfr_semaphore_init(&sfrThreadBuf->geometryDoneSem, 0);
        sfr_semaphore_init(&sfrThreadBuf->rasterStartSem, 0);
        sfr_semaphore_init(&sfrThreadBuf->rasterDoneSem, 0);

        for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
            sfrThreadBuf->threads[i].threadInd = i;
            #ifdef _WIN32
                const u64 handle = _beginthreadex(NULL, 0, sfr__worker_thread_func, &sfrThreadBuf->threads[i], 0, NULL);
                sfrThreadBuf->threads[i].handle = (SfrThread)handle;
            #else
                pthread_create(&sfrThreadBuf->threads[i].handle, NULL, sfr__worker_thread_func, &sfrThreadBuf->threads[i]);
            #endif
        }
    #else
        sfrState.idBuf = (i32*)sfrMalloc(sizeof(i32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
        if (!sfrState.idBuf) {
            SFR__ERR_EXIT("sfr_init: failed to allocate sfrState.idBuf (%ld bytes)\n",
                sizeof(i32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
        }

        sfrState.globalBinsCap = 1024 * 32;
        sfrState.globalBinsCount = 0;
        sfrState.globalBins = (struct sfrTriangleBin*)sfrMalloc(sizeof(struct sfrTriangleBin) * sfrState.globalBinsCap);
        if (!sfrState.globalBins) {
            SFR__ERR_EXIT("sfr_init: failed to allcoate sfrState.globalBins (%ld bytes)\n",
                sizeof(struct sfrTriangleBin) * sfrState.globalBinsCap);
        }
    #endif

    sfr_resize(w, h); // call resize to setup tiling system
    sfr_clear(0xFF000000);
    sfr_reset();
    sfr_set_fov(fovDeg);
    sfr_set_camera(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

    sfrState.baseTexPixels[0] = 0xFFFFFFFF;
    sfrState.baseTex = (SfrTexture){ .w = 1, .h = 1, .pixels = sfrState.baseTexPixels, .mipLevels = 1 };

    if (0 == sfrLight.r && 0 == sfrLight.g && 0 == sfrLight.b) {
        sfrLight.r = sfrLight.g = sfrLight.b = 1.f;
    }
}

SFR_FUNC void sfr__shutdown(void); // I dont wanna move sfr__shutdown to here
SFR_FUNC void sfr_release(void) {
    if (sfrPixelBuf) {
        sfrFree(sfrPixelBuf);
        sfrPixelBuf = (void*)0;
    }
    if (sfrDepthBuf) {
        sfrFree(sfrDepthBuf);
        sfrDepthBuf = (void*)0;
    }
    #ifdef SFR_MULTITHREADED
        sfr__shutdown();
        if (sfrThreadBuf) {
            sfrFree(sfrThreadBuf);
            sfrThreadBuf = (void*)0;
        }
    #endif
}

#ifdef SFR_MULTITHREADED

SFR_FUNC void sfr__shutdown(void) {
    sfr_flush_and_wait();

    sfrState.shutdown = 1;
    // post to both semaphores to ensure threads wake up from either wait state
    sfr_semaphore_post(&sfrThreadBuf->geometryStartSem, SFR_THREAD_COUNT);
    sfr_semaphore_post(&sfrThreadBuf->rasterStartSem, SFR_THREAD_COUNT);

    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        #ifdef _WIN32
            WaitForSingleObject(sfrThreadBuf->threads[i].handle, INFINITE);
            CloseHandle(sfrThreadBuf->threads[i].handle);
        #else
            pthread_join(sfrThreadBuf->threads[i].handle, NULL);
        #endif
    }

    if (sfrThreadBuf->meshJobPool) sfrFree(sfrThreadBuf->meshJobPool);
    if (sfrThreadBuf->geometryWorkQueue) sfrFree(sfrThreadBuf->geometryWorkQueue);

    sfr_mutex_destroy(&sfrThreadBuf->binPoolMutex);
    sfr_mutex_destroy(&sfrThreadBuf->geometryMutex);

    if (sfrThreadBuf->binPages) {
        for (i32 i = 0; i < sfrThreadBuf->binPagesCapacity; i += 1) {
            if (sfrThreadBuf->binPages[i]) {
                sfrFree(sfrThreadBuf->binPages[i]);
            }
        }
        sfrFree(sfrThreadBuf->binPages);
    }

    sfr_semaphore_destroy(&sfrThreadBuf->geometryStartSem);
    sfr_semaphore_destroy(&sfrThreadBuf->geometryDoneSem);
    sfr_semaphore_destroy(&sfrThreadBuf->rasterStartSem);
    sfr_semaphore_destroy(&sfrThreadBuf->rasterDoneSem);
}

#endif // SFR_MULTITHREADED

// dispatches jobs to workers and waits for them to complete
SFR_FUNC void sfr_flush_and_wait(void) {
#ifdef SFR_MULTITHREADED
    // always dispatch geometry phase to keep workers in sync
    sfr_semaphore_post(&sfrThreadBuf->geometryStartSem, SFR_THREAD_COUNT);
    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        sfr_semaphore_wait(&sfrThreadBuf->geometryDoneSem);
    }

    // reset geometry queue
    sfr_atomic_set(&sfrThreadBuf->meshJobAllocator, 0);
    sfr_atomic_set(&sfrThreadBuf->geometryWorkQueueCount, 0);
    sfr_atomic_set(&sfrThreadBuf->geometryWorkQueueHead, 0);

    // build raster work queue without atomic cas
    i32 workCount = 0;
    for (i32 i = 0; i < sfrThreadBuf->tileCount; i += 1) {
        struct sfrTile* tile = &sfrThreadBuf->tiles[i];

        // check if any thread put triangles into this tile
        u8 hasWork = 0;
        for (i32 t = 0; t <= SFR_THREAD_COUNT; t += 1) {
            if (tile->binCount[t] > 0) {
                hasWork = 1;
                break;
            }
        }

        if (hasWork) {
            sfrThreadBuf->rasterWorkQueue[workCount++] = i;
        }
    }
    sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueCount, workCount);
    sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueHead, 0);

    // always dispatch rasterization phase
    sfr_semaphore_post(&sfrThreadBuf->rasterStartSem, SFR_THREAD_COUNT);
    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        sfr_semaphore_wait(&sfrThreadBuf->rasterDoneSem);
    }

    // reset rasterization data
    sfr_atomic_set(&sfrThreadBuf->triangleBinAllocator, 0);
    sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueHead, 0);

    // reset so it doesn't get stomped
    sfrTlsBinStart = 0;
    sfrTlsBinEnd = 0;
#else
    if (0 == sfrState.globalBinsCount) {
        return;
    }

    const struct sfrTile fullTile = {
        .minX = 0, .minY = 0, .maxX = sfrWidth, .maxY = sfrHeight,
    };

    sfr__resolve_tile(&fullTile);
#endif
}

SFR_FUNC void sfr_resize(i32 width, i32 height) {
    if (width > SFR_MAX_WIDTH || height > SFR_MAX_HEIGHT) {
        SFR__ERR_RET(, "sfr_resize: dimensions out of range (%d, %d) > (%d, %d)\n",
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
        sfrThreadBuf->tileCols = (width + SFR_TILE_WIDTH - 1) / SFR_TILE_WIDTH;
        sfrThreadBuf->tileRows = (height + SFR_TILE_HEIGHT - 1) / SFR_TILE_HEIGHT;
        sfrThreadBuf->tileCount = sfrThreadBuf->tileCols * sfrThreadBuf->tileRows;

        for (i32 y = 0, i = 0; y < sfrThreadBuf->tileRows; y += 1) {
            for (i32 x = 0; x < sfrThreadBuf->tileCols; x += 1, i += 1) {
                struct sfrTile* tile = &sfrThreadBuf->tiles[i];
                tile->minX = x * SFR_TILE_WIDTH;
                tile->minY = y * SFR_TILE_HEIGHT;
                tile->maxX = SFR_MIN((x + 1) * SFR_TILE_WIDTH, width);
                tile->maxY = SFR_MIN((y + 1) * SFR_TILE_HEIGHT, height);
                tile->maxZ = sfrFarDist;

                for (i32 t = 0; t <= SFR_THREAD_COUNT; t += 1) {
                    tile->binsCapacity[t] = 512;
                    tile->bins[t] = (struct sfrTriangleBin**)sfrMalloc(
                        sizeof(struct sfrTriangleBin) * tile->binsCapacity[t]);
                    tile->binCount[t] = 0;
                }

                sfr_atomic_set(&tile->hasWork, 0);
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

    const i32 count = sfrWidth * sfrHeight;

    #if !defined(SFR_NO_SIMD) && !defined(SFR_NO_STRING)
        const __m256i vCol = _mm256_set1_epi32(clearCol);

        u32 depthBits;
        memcpy(&depthBits, &sfrFarDist, sizeof(u32));
        const __m256i vDepth = _mm256_set1_epi32(depthBits);

        #ifndef SFR_MULTITHREADED
            const __m256i vId = _mm256_set1_epi32(-1);
        #endif

        i32 i = 0;
        for (; i <= count - 8; i += 8) {
            _mm256_storeu_si256((__m256i*)&sfrPixelBuf[i], vCol);
            _mm256_storeu_si256((__m256i*)&sfrDepthBuf[i], vDepth);
            #ifndef SFR_MULTITHREADED
                _mm256_storeu_si256((__m256i*)&sfrState.idBuf[i], vId);
            #endif
        }

        for (; i < count; i += 1) {
            sfrPixelBuf[i] = clearCol;
            sfrDepthBuf[i] = sfrFarDist;
            #ifndef SFR_MULTITHREADED
                sfrState.idBuf[i] = -1;
            #endif
        }
    #else
        for (i32 i = count - 1; i >= 0; i -= 1) {
            sfrPixelBuf[i] = clearCol;
            sfrDepthBuf[i] = sfrFarDist;
            #ifndef SFR_MULTITHREADED
                sfrState.idBuf[i] = -1;
            #endif
        }
    #endif

    #ifdef SFR_MULTITHREADED
        for (i32 i = 0; i < sfrThreadBuf->tileCount; i += 1) {
            for (i32 t = 0; t <= SFR_THREAD_COUNT; t += 1) {
                sfrThreadBuf->tiles[i].binCount[t] = 0;
            }
            sfr_atomic_set(&sfrThreadBuf->tiles[i].hasWork, 0);
            sfrThreadBuf->tiles[i].maxZ = sfrFarDist;
        }

        sfr_atomic_set(&sfrRasterCount, 0);
    #else
        sfrRasterCount = 0;
        sfrState.globalBinsCount = 0;
    #endif
}

SFR_FUNC void sfr_clear_depth(void) {
    sfr_flush_and_wait();

    const i32 count = sfrWidth * sfrHeight;

    #if !defined(SFR_NO_SIMD) && !defined(SFR_NO_STRING)
        u32 depthBits;
        memcpy(&depthBits, &sfrFarDist, sizeof(u32));
        const __m256i vDepth = _mm256_set1_epi32(depthBits);

        i32 i = 0;
        for (; i <= count - 8; i += 8) {
            _mm256_storeu_si256((__m256i*)&sfrDepthBuf[i], vDepth);
        }

        for (i32 j = i; j < count; j += 1) {
            sfrDepthBuf[j] = sfrFarDist;
        }
    #else
        for (i32 i = count - 1; i >= 0; i -= 1) {
            sfrDepthBuf[i] = sfrFarDist;
        }
    #endif // !SFR_NO_SIMD && !SFR_NO_STRING

    #ifdef SFR_MULTITHREADED
        for (i32 i = 0; i < sfrThreadBuf->tileCount; i += 1) {
            sfrThreadBuf->tiles[i].maxZ = sfrFarDist;
        }
    #endif
}

// front, right, back, left, top, bottom
SFR_FUNC void sfr_skybox(const SfrTexture* faces[6]) {
    if (!faces) {
        return;
    }

    const sfrmat savedModel = sfrMatModel;
    const sfrmat savedNormal = sfrState.matNormal;
    const u8 savedLighting = sfrState.lightingEnabled;
    const u8 savedDirty = sfrState.normalMatDirty;

    sfrState.lightingEnabled = 0;

    sfrMatModel = SFR__MAT_IDENTITY;
    const f32 scale = sfrFarDist * 0.5f;
    sfr_scale(scale, scale, scale);
    sfr_translate(sfrCamPos.x, sfrCamPos.y, sfrCamPos.z);

    // front face
    sfr__triangle_tex_norm(
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,0,1,
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,0,1,
        -0.5f,-0.5f,-0.5f, 1.f,1.f, 0,0,1, 0xFFFFFFFF, faces[0]);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 0.f,1.f, 0,0,1,
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,0,1,
        -0.5f,-0.5f,-0.5f, 1.f,1.f, 0,0,1, 0xFFFFFFFF, faces[0]);
    // right face
    sfr__triangle_tex_norm(
         0.5f, 0.5f, 0.5f, 0.f,0.f, -1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,0.f, -1,0,0,
         0.5f,-0.5f,-0.5f, 1.f,1.f, -1,0,0, 0xFFFFFFFF, faces[1]);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 0.f,1.f, -1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,0.f, -1,0,0,
         0.5f,-0.5f,-0.5f, 1.f,1.f, -1,0,0, 0xFFFFFFFF, faces[1]);
    // back face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f, 0.5f, 0.f,0.f, 0,0,-1,
         0.5f, 0.5f, 0.5f, 1.f,0.f, 0,0,-1,
         0.5f,-0.5f, 0.5f, 1.f,1.f, 0,0,-1, 0xFFFFFFFF, faces[2]);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 0.f,1.f, 0,0,-1,
        -0.5f, 0.5f, 0.5f, 0.f,0.f, 0,0,-1,
         0.5f,-0.5f, 0.5f, 1.f,1.f, 0,0,-1, 0xFFFFFFFF, faces[2]);
    // left face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 0.f,0.f, 1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,0.f, 1,0,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 1,0,0, 0xFFFFFFFF, faces[3]);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,0.f, 1,0,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 1,0,0, 0xFFFFFFFF, faces[3]);
    // top face
    sfr__triangle_tex_norm(
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,-1,0,
        -0.5f, 0.5f, 0.5f, 0.f,0.f, 0,-1,0,
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,-1,0, 0xFFFFFFFF, faces[4]);
    sfr__triangle_tex_norm(
        0.5f, 0.5f,-0.5f, 1.f,1.f, 0,-1,0,
        0.5f, 0.5f, 0.5f, 0.f,1.f, 0,-1,0,
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,-1,0, 0xFFFFFFFF, faces[4]);
    // bottom face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,1,0, 0xFFFFFFFF, faces[5]);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,1,0,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,1,0, 0xFFFFFFFF, faces[5]);

    sfrMatModel = savedModel;
    sfrState.matNormal = savedNormal;
    sfrState.lightingEnabled = savedLighting;
    sfrState.normalMatDirty = savedDirty;
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
    u32 col, const SfrTexture* tex
) {
    const sfrvec n = sfr_vec_face_normal(
        (sfrvec){ .x = ax, .y = ay, .z = az },
        (sfrvec){ .x = bx, .y = by, .z = bz },
        (sfrvec){ .x = cx, .y = cy, .z = cz }
    );

    sfr__triangle_tex_norm(
        ax, ay, az, au, av, n.x, n.y, n.z,
        bx, by, bz, bu, bv, n.x, n.y, n.z,
        cx, cy, cz, cu, cv, n.x, n.y, n.z,
        col, tex
    );
}

SFR_FUNC void sfr_point(f32 worldX, f32 worldY, f32 worldZ, i32 radius, u32 col) {
    sfr_flush_and_wait();

    i32 sx, sy;
    if (!sfr_world_to_screen(worldX, worldY, worldZ, &sx, &sy)) {
        return;
    }

    for (i32 y = -radius; y <= radius; y += 1) {
        for (i32 x = -radius; x <= radius; x += 1) {
            const i32 i = (y + sy) * sfrWidth + (x + sx);
            if (i >= 0 && i < sfrWidth * sfrHeight) {
                sfrPixelBuf[i] = col;
            }
        }
    }
}

SFR_FUNC void sfr_billboard(u32 col, const SfrTexture* tex) {
    const sfrmat savedModel = sfrMatModel;
    const sfrmat savedNormal = sfrState.matNormal;
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

    sfr__triangle_tex_norm(
        a.x, a.y, a.z, 0.f, 1.f, normal.x, normal.y, normal.z,
        c.x, c.y, c.z, 1.f, 0.f, normal.x, normal.y, normal.z,
        b.x, b.y, b.z, 1.f, 1.f, normal.x, normal.y, normal.z,
        col, tex);
    sfr__triangle_tex_norm(
        a.x, a.y, a.z, 0.f, 1.f, normal.x, normal.y, normal.z,
        d.x, d.y, d.z, 0.f, 0.f, normal.x, normal.y, normal.z,
        c.x, c.y, c.z, 1.f, 0.f, normal.x, normal.y, normal.z,
        col, tex);

    sfrMatModel = savedModel;
    sfrState.matNormal = savedNormal;
}

SFR_FUNC void sfr_cube(u32 col, const SfrTexture* tex) {
    // front face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1, col, tex);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,0,-1, col, tex);
    // right face
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,1.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0, col, tex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0,
         0.5f,-0.5f, 0.5f, 0.f,0.f, 1,0,0, col, tex);
    // back face
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
         0.5f, 0.5f, 0.5f, 1.f,1.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1, col, tex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1,
        -0.5f,-0.5f, 0.5f, 0.f,0.f, 0,0,1, col, tex);
    // left face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0, col, tex);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0,
        -0.5f,-0.5f,-0.5f, 0.f,0.f, -1,0,0, col, tex);
    // top face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0, col, tex);
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0,
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,1,0, col, tex);
    // bottom face
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0, col, tex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,-1,0, col, tex);
}

SFR_FUNC void sfr_cube_ex(u32 col[12]) {
    // front face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1, col[0], &sfrState.baseTex);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,0,-1, col[1], &sfrState.baseTex);
    // right face
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,1.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0, col[2], &sfrState.baseTex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0,
         0.5f,-0.5f, 0.5f, 0.f,0.f, 1,0,0, col[3], &sfrState.baseTex);
    // back face
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
         0.5f, 0.5f, 0.5f, 1.f,1.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1, col[4], &sfrState.baseTex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1,
        -0.5f,-0.5f, 0.5f, 0.f,0.f, 0,0,1, col[5], &sfrState.baseTex);
    // left face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0, col[6], &sfrState.baseTex);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0,
        -0.5f,-0.5f,-0.5f, 0.f,0.f, -1,0,0, col[7], &sfrState.baseTex);
    // top face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0, col[8], &sfrState.baseTex);
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0,
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,1,0, col[9], &sfrState.baseTex);
    // bottom face
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0, col[10], &sfrState.baseTex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,-1,0, col[11], &sfrState.baseTex);
}

SFR_FUNC void sfr_cube_inv(u32 col, const SfrTexture* tex) {
    // front face
    sfr__triangle_tex_norm(
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,1,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, 0,0,1,
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,1, col, tex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,0,1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,1,
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,1, col, tex);
    // right face
    sfr__triangle_tex_norm(
         0.5f, 0.5f, 0.5f, 0.f,1.f, -1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,1.f, -1,0,0,
         0.5f,-0.5f,-0.5f, 1.f,0.f, -1,0,0, col, tex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 0.f,0.f, -1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, -1,0,0,
         0.5f,-0.5f,-0.5f, 1.f,0.f, -1,0,0, col, tex);
    // back face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,-1,
         0.5f, 0.5f, 0.5f, 1.f,1.f, 0,0,-1,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,-1, col, tex);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 0.f,0.f, 0,0,-1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,-1,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,-1, col, tex);
    // left face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 0.f,1.f, 1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 1,0,0,
        -0.5f,-0.5f, 0.5f, 1.f,0.f, 1,0,0, col, tex);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 0.f,0.f, 1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, 1,0,0,
        -0.5f,-0.5f, 0.5f, 1.f,0.f, 1,0,0, col, tex);
    // top face
    sfr__triangle_tex_norm(
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,-1,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 0,-1,0,
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,-1,0, col, tex);
    sfr__triangle_tex_norm(
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,-1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,-1,0,
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,-1,0, col, tex);
    // bottom face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,1,0, col, tex);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,1,0,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,1,0, col, tex);
}

SFR_FUNC void sfr_mesh(const SfrMesh* mesh, u32 col, const SfrTexture* tex) {
    const i32 triangleCount = mesh->vertCount / 9;
    if (0 == triangleCount) {
        return;
    }

    if (sfrState.normalMatDirty) {
        sfr__update_normal_mat();
    }

    const sfrmat matMVP = sfr_mat_mul(sfr_mat_mul(sfrMatModel, sfrMatView), sfrMatProj);

    #ifdef SFR_MULTITHREADED
        // divide the mesh into jobs
        for (i32 i = 0; i < triangleCount; i += SFR_GEOMETRY_JOB_SIZE) {
            const i32 jobInd = sfr_atomic_add(&sfrThreadBuf->meshJobAllocator, 1) - 1;
            if (jobInd >= sfrThreadBuf->meshJobPoolCapacity) {
                sfr_mutex_lock(&sfrThreadBuf->geometryMutex);
                if (jobInd >= sfrThreadBuf->meshJobPoolCapacity) {
                    i32 newCap = sfrThreadBuf->meshJobPoolCapacity * 2;
                    while (jobInd >= newCap) {
                        newCap *= 2;
                    }

                    struct sfrMeshChunkJob* const newPool = (struct sfrMeshChunkJob*)sfrRealloc(sfrThreadBuf->meshJobPool, sizeof(struct sfrMeshChunkJob) * newCap);
                    if (newPool) {
                        sfrThreadBuf->meshJobPool = newPool;
                        sfrThreadBuf->meshJobPoolCapacity = newCap;
                    }
                }
                sfr_mutex_unlock(&sfrThreadBuf->geometryMutex);
            }

            // create and populate the job
            struct sfrMeshChunkJob* job = &sfrThreadBuf->meshJobPool[jobInd];
            job->tris = &mesh->tris[i * 9];
            job->uvs = mesh->uvs ? &mesh->uvs[i * 6] : NULL;
            job->normals = &mesh->normals[i * 9];
            job->matNormal = sfrState.matNormal;
            job->matMVP = matMVP;
            job->col = col;
            job->tex = tex;
            job->startTriangle = i;
            job->triangleCount = SFR_MIN(SFR_GEOMETRY_JOB_SIZE, triangleCount - i);

            // add job to the queue
            const i32 queueInd = sfr_atomic_add(&sfrThreadBuf->geometryWorkQueueCount, 1) - 1;
            if (queueInd >= sfrThreadBuf->geometryWorkQueueCapacity) {
                sfr_mutex_lock(&sfrThreadBuf->geometryMutex);
                if (queueInd >= sfrThreadBuf->geometryWorkQueueCapacity) {
                    i32 newCap = sfrThreadBuf->geometryWorkQueueCapacity * 2;
                    while (queueInd >= newCap) {
                        newCap *= 2;
                    }

                    i32* const newQ = (i32*)sfrRealloc(sfrThreadBuf->geometryWorkQueue, sizeof(i32) * newCap);
                    if (newQ) {
                        sfrThreadBuf->geometryWorkQueue = newQ;
                        sfrThreadBuf->geometryWorkQueueCapacity = newCap;
                    }
                }
                sfr_mutex_unlock(&sfrThreadBuf->geometryMutex);
            }

            sfrThreadBuf->geometryWorkQueue[queueInd] = jobInd;
        }
    #else
        for (i32 i = 0; i < mesh->vertCount; i += 9) {
            const i32 uvInd = (i / 9) * 6;
            sfr__process_and_bin_triangle(
                &matMVP, &sfrState.matNormal,
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
    (void)font; (void)s; (void)sLength; (void)col;
    SFR__ERR_RET(, "sfr_string: TODO not implemented, 'sfr_glyph' is implemented\n");
}

SFR_FUNC void sfr_glyph(const SfrFont* font, u16 id, u32 col) {
    if (id >= SFR_FONT_GLYPH_MAX) {
        SFR__ERR_RET(, "sfr_glyph: invalid id (%d >= %d)\n", id, SFR_FONT_GLYPH_MAX);
    }

    const f32* t = font->verts[id];
    for (i32 i = 0; i < SFR_FONT_VERT_MAX; i += 6) {
        if (SFR_FONT_VERT_EMPTY == font->verts[id][i]) {
            break;
        }

        sfr_triangle(
            t[i + 0], t[i + 1], 0.f,
            t[i + 2], t[i + 3], 0.f,
            t[i + 4], t[i + 5], 0.f,
            col);
    }
}

SFR_FUNC SfrScene* sfr_scene_create(SfrSceneObject* objects, i32 count) {
    SfrScene* scene = (SfrScene*)sfrMalloc(sizeof(SfrScene));
    if (!objects || count <= 0) {
        SFR__ERR_RET(scene, "sfr_scene_create: !objects (%p) || count (%d) <= 0\n", objects, count);
    }

    scene->objects = objects;
    scene->count = count;

    for (i32 i = 0; i < count; i += 1) {
        SfrSceneObject* const obj = &scene->objects[i];

        const sfrvec scale = obj->scale;
        const sfrvec rot = obj->rot;
        const sfrvec pos = obj->pos;

        // init model matrix, scale, rotate (zyx), translate
        sfrmat m = sfr_mat_scale(scale.x, scale.y, scale.z);
        m = sfr_mat_mul(m, sfr_mat_rot_x(rot.x));
        m = sfr_mat_mul(m, sfr_mat_rot_y(rot.y));
        m = sfr_mat_mul(m, sfr_mat_rot_z(rot.z));
        m = sfr_mat_mul(m, sfr_mat_translate(pos.x, pos.y, pos.z));
        obj->_model = m;

        // manually construct the inverse (scale * rot * trans)
        sfrmat invM = sfr_mat_translate(-pos.x, -pos.y, -pos.z);
        invM = sfr_mat_mul(invM, sfr_mat_rot_z(-rot.z));
        invM = sfr_mat_mul(invM, sfr_mat_rot_y(-rot.y));
        invM = sfr_mat_mul(invM, sfr_mat_rot_x(-rot.x));
        invM = sfr_mat_mul(invM, sfr_mat_scale(1.f / scale.x, 1.f / scale.y, 1.f / scale.z));
        obj->_invModel = invM;

        obj->_normal = sfr__calc_normal_mat(m);

        const SfrMesh* const mesh = obj->mesh;
        const i32 triCount = mesh->vertCount / 9;
        if (triCount > 0) {
            // a binary tree has at most 2 * n - 1
            obj->_bvhNodes = (struct sfrBvhNode*)sfrMalloc(sizeof(struct sfrBvhNode) * (2 * triCount - 1));
            obj->_bvhRoot = 0;

            // setup root
            obj->_bvhNodes[0].leftFirst = 0;
            obj->_bvhNodes[0].count = triCount;

            i32 nodePtr = 1; // next free is 1
            sfr__bvh_update_bounds(0, obj->_bvhNodes, mesh, triCount);
            sfr__bvh_subdivide(obj->_bvhNodes, 0, &nodePtr, obj->mesh);
            obj->_bvhNodeCount = nodePtr;
        } else {
            obj->_bvhNodes = NULL;
            obj->_bvhNodeCount = 0;
        }
    }

    return scene;
}

SFR_FUNC void sfr_scene_release(SfrScene** scene, u8 freeObjects) {
    if (!scene || !(*scene)) return;

    if ((*scene)->objects) {
        for (i32 i = 0; i < (*scene)->count; i += 1) {
            if ((*scene)->objects[i]._bvhNodes) {
                sfrFree((*scene)->objects[i]._bvhNodes);
                (*scene)->objects[i]._bvhNodes = NULL;
            }
        }
        if (freeObjects) {
            sfrFree((*scene)->objects);
        }
    }
    sfrFree(*scene);
    *scene = NULL;
}

// TODO don't fully overwrite sfrMatModel (see sfr_model_draw)
SFR_FUNC void sfr_scene_draw(const SfrScene* scene) {
    const sfrmat savedModel = sfrMatModel;
    const sfrmat savedNormal = sfrState.matNormal;
    const u8 savedDirty = sfrState.normalMatDirty;

    sfrState.normalMatDirty = 0;
    for (i32 i = 0; i < scene->count; i += 1) {
        sfrMatModel = scene->objects[i]._model;
        sfrState.matNormal = scene->objects[i]._normal;

        sfr_mesh(scene->objects[i].mesh, scene->objects[i].col, scene->objects[i].tex);
    }

    sfrMatModel = savedModel;
    sfrState.matNormal = savedNormal;
    sfrState.normalMatDirty = savedDirty;
}

SFR_FUNC SfrRayHit sfr_scene_raycast(const SfrScene* scene, f32 ox, f32 oy, f32 oz, f32 dx, f32 dy, f32 dz) {
    SfrRayHit hit = {0};
    hit.hit = 0;
    hit.distance = 1e30f;
    hit.objectInd = -1;
    hit.triangleInd = -1;

    if (!scene || scene->count <= 0) {
        SFR__ERR_RET(hit, "sfr_scene_raycast: !scene (%p) || scene->count (%d) <= 0\n", scene, scene ? scene->count : 0);
    }

    const sfrvec rayOrigin = { ox, oy, oz, 1.f };
    const sfrvec rayDirRaw = { dx, dy, dz, 0.f };
    const sfrvec rayDir = sfr_vec_norm(rayDirRaw);


    for (i32 i = 0; i < scene->count; i += 1) {
        SfrSceneObject* const obj = &scene->objects[i];
        if (!obj->mesh || !obj->_bvhNodes) {
            continue;
        }

        // faster to transform the ray to the object than the box to world
        // so we must raycast in local space.

        const sfrvec localOrigin = sfr_mat_mul_vec(obj->_invModel, rayOrigin);
        const sfrvec localDirRaw = sfr_mat_mul_vec(obj->_invModel, (sfrvec){dx, dy, dz, 0.f});
        const sfrvec localDir = sfr_vec_norm(localDirRaw);

        const sfrvec localDirInv = {
            1.f / (sfr_fabsf(localDir.x) > SFR_EPSILON ? localDir.x : SFR_EPSILON),
            1.f / (sfr_fabsf(localDir.y) > SFR_EPSILON ? localDir.y : SFR_EPSILON),
            1.f / (sfr_fabsf(localDir.z) > SFR_EPSILON ? localDir.z : SFR_EPSILON),
            0.f
        };

        i32 stack[64];
        i32 stackPtr = 0;
        stack[stackPtr++] = 0; // push root

        while (stackPtr > 0) {
            const i32 nodeInd = stack[--stackPtr];
            struct sfrBvhNode* node = &obj->_bvhNodes[nodeInd];

            // vv check bounds intersection (in local space) vv
            const f32 tx1 = (node->minX - localOrigin.x) * localDirInv.x;
            const f32 tx2 = (node->maxX - localOrigin.x) * localDirInv.x;
            f32 tmin = sfr_fminf(tx1, tx2);
            f32 tmax = sfr_fmaxf(tx1, tx2);

            const f32 ty1 = (node->minY - localOrigin.y) * localDirInv.y;
            const f32 ty2 = (node->maxY - localOrigin.y) * localDirInv.y;
            tmin = sfr_fmaxf(tmin, sfr_fminf(ty1, ty2));
            tmax = sfr_fminf(tmax, sfr_fmaxf(ty1, ty2));

            const f32 tz1 = (node->minZ - localOrigin.z) * localDirInv.z;
            const f32 tz2 = (node->maxZ - localOrigin.z) * localDirInv.z;
            tmin = sfr_fmaxf(tmin, sfr_fminf(tz1, tz2));
            tmax = sfr_fminf(tmax, sfr_fmaxf(tz1, tz2));

            // if missed box or box is further than current closest hit
            if (tmax < tmin || tmax < 0 || tmin > hit.distance) {
                if (tmax < tmin || tmax < 0) {
                    continue;
                }
            }

            // leaf or internal
            if (node->count > 0) { // leaf
                for (i32 t = 0; t < node->count; t += 1) {
                    const i32 baseInd = (node->leftFirst + t) * 9;

                    sfrvec v0 = { obj->mesh->tris[baseInd + 0], obj->mesh->tris[baseInd + 1], obj->mesh->tris[baseInd + 2], 1.f };
                    sfrvec v1 = { obj->mesh->tris[baseInd + 3], obj->mesh->tris[baseInd + 4], obj->mesh->tris[baseInd + 5], 1.f };
                    sfrvec v2 = { obj->mesh->tris[baseInd + 6], obj->mesh->tris[baseInd + 7], obj->mesh->tris[baseInd + 8], 1.f };

                    // transform to world space for the actual hit check
                    v0 = sfr_mat_mul_vec(obj->_model, v0);
                    v1 = sfr_mat_mul_vec(obj->_model, v1);
                    v2 = sfr_mat_mul_vec(obj->_model, v2);

                    f32 u, v;
                    if (sfr__intersect_triangle(rayOrigin, rayDir, v0, v1, v2, &hit.distance, &u, &v)) {
                        hit.hit = 1;
                        hit.objectInd = i;
                        hit.triangleInd = node->leftFirst + t;
                        hit.obj = obj;
                        hit.pos = sfr_vec_add(rayOrigin, sfr_vec_mul(rayDir, hit.distance));
                        hit.normal = sfr_vec_face_normal(v0, v1, v2);
                    }
                }
            } else { // internal
                stack[stackPtr++] = node->leftFirst;
                stack[stackPtr++] = node->leftFirst + 1;
            }
        }
    }

    return hit;
}

SFR_FUNC u8 sfr_world_to_screen(f32 x, f32 y, f32 z, i32* screenX, i32* screenY) {
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
    sfrCamUp = (sfrvec){0.f, 1.f, 0.f, 1.f};
    sfrCamTarget = (sfrvec){0.f, 0.f, 1.f, 1.f};

    const sfrmat rotX = sfr_mat_rot_x(pitch);
    const sfrmat rotY = sfr_mat_rot_y(yaw);
    const sfrmat rotZ = sfr_mat_rot_z(roll);

    sfrCamUp = sfr_mat_mul_vec(rotZ, sfrCamUp);
    sfrCamTarget = sfr_mat_mul_vec(rotX, sfrCamTarget);
    sfrCamTarget = sfr_mat_mul_vec(rotY, sfrCamTarget);
    sfrCamTarget = sfr_vec_add(sfrCamPos, sfrCamTarget);

    sfrMatView = sfr_mat_qinv(sfr_mat_look_at(sfrCamPos, sfrCamTarget, sfrCamUp));
}

SFR_FUNC void sfr_set_fov(f32 fovDeg) {
    const f32 aspect = (f32)sfrHeight / sfrWidth;
    sfrMatProj = sfr_mat_proj(fovDeg, aspect, sfrNearDist, sfrFarDist);
    sfrCamFov = fovDeg;
}

SFR_FUNC void sfr_set_lighting(u8 enabled) {
    sfrState.lightingEnabled = enabled;
}

#ifdef SFR_USE_CGLTF

SFR_FUNC void sfr_model_animate(SfrModel* model, i32 animInd, f32 time) {
    if (!model) {
        SFR__ERR_RET(, "sfr_model_animate: model is NULL\n");
    }
    if (animInd < 0 || animInd >= model->animCount) {
        SFR__ERR_RET(, "sfr_model_animate: animInd (%d) out of bounds [%d, %d)\n", animInd, 0, model->animCount);
    }

    const struct sfrAnimation* const anim = &model->animations[animInd];

    // loop
    if (anim->duration > SFR_EPSILON) {
        time = sfr_fmodf(time, anim->duration);
    }

    // vv update local transforms from channels vv
    for (i32 i = 0; i < anim->channelCount; i += 1) {
        const struct sfrAnimChannel* const channel = &anim->channels[i];
        const struct sfrAnimSampler* const sampler = &anim->samplers[channel->samplerInd];
        struct sfrTransformNode* const node = &model->transforms[channel->transformNodeInd];

        // find frame
        i32 f0 = 0, f1 = 0;
        f32 t = 0.f;

        if (time <= sampler->inputs[0]) {
            f0 = f1 = 0;
        } else if (time >= sampler->inputs[sampler->count - 1]) {
            f0 = f1 = sampler->count - 1;
        } else {
            for (i32 k = 0; k < sampler->count - 1; k += 1) {
                if (time >= sampler->inputs[k] && time < sampler->inputs[k+1]) {
                    f0 = k;
                    f1 = k + 1;
                    t = (time - sampler->inputs[k]) / (sampler->inputs[k+1] - sampler->inputs[k]);
                    break;
                }
            }
        }

        const sfrvec* const values = (const sfrvec* const)sampler->outputs;
        const sfrvec v0 = values[f0];
        const sfrvec v1 = values[f1];

        switch (channel->path) {
            case SFR_ANIM_PATH_ROTATION: {
                node->localRot = sfr_quat_slerp(v0, v1, t);
            } break;
            case SFR_ANIM_PATH_TRANSLATION: {
                node->localPos = sfr_vec_lerp(v0, v1, t);
            } break;
            case SFR_ANIM_PATH_SCALE: {
                node->localScale = sfr_vec_lerp(v0, v1, t);
            } break;
        }
    }

    // vv recompute hierarchy vv
    for (i32 i = 0; i < model->transformCount; i += 1) {
        struct sfrTransformNode* node = &model->transforms[i];

        // reconstruct local matrix
        const sfrmat scale = sfr_mat_scale(node->localScale.x, node->localScale.y, node->localScale.z);
        const sfrmat rot   = sfr_mat_from_quat(node->localRot);
        const sfrmat trans = sfr_mat_translate(node->localPos.x, node->localPos.y, node->localPos.z);

        // multiply order S * R * T
        node->localMatrix = sfr_mat_mul(scale, sfr_mat_mul(rot, trans));

        if (node->parentInd >= 0) {
            node->worldMatrix = sfr_mat_mul(node->localMatrix, model->transforms[node->parentInd].worldMatrix);
        } else {
            node->worldMatrix = node->localMatrix;
        }
    }
}

SFR_FUNC SfrModel* sfr_load_gltf(const char* filename, i32 uvChannel) {
    // vv read file vv
    FILE* file = fopen(filename, "rb");
    if (!file) {
        SFR__ERR_RET(NULL, "sfr_load_gltf: failed to open '%s'\n", filename);
    }

    fseek(file, 0, SEEK_END);
    const i64 fileSize = ftell(file);
    rewind(file);

    u8* const fileContent = (u8*)sfrMalloc(fileSize);
    if (fread(fileContent, 1, fileSize, file) != (u64)fileSize) {
        fclose(file);
        sfrFree(fileContent);
        SFR__ERR_RET(NULL, "sfr_load_gltf: failed to read '%s'\n", filename);
    }
    fclose(file);

    // vv parse gltf vv
    cgltf_options options = {0};
    options.memory.alloc_func = sfr__cgltf_alloc;
    options.memory.free_func  = sfr__cgltf_free;

    cgltf_data* data = NULL;
    const cgltf_result parseRes = cgltf_parse(&options, fileContent, fileSize, &data);
    if (cgltf_result_success != parseRes) {
        sfrFree(fileContent);
        SFR__ERR_RET(NULL, "sfr_load_gltf: parse error %d\n", parseRes);
    }

    // vv load buffers vv
    const cgltf_result loadBufRes = cgltf_load_buffers(&options, data, filename);
    if (cgltf_result_success != loadBufRes) {
        cgltf_free(data);
        sfrFree(fileContent);
        SFR__ERR_RET(NULL, "sfr_load_gltf: buffer load error %d\n", loadBufRes);
    }

    SfrModel* const model = (SfrModel*)sfrMalloc(sizeof(SfrModel));
    sfr_memset(model, 0, sizeof(SfrModel));

    // vv load textures if possible vv
    #ifdef SFR_USE_STB_IMAGE
        if (data->textures_count <= 0) {
            goto SKIP;
        }

        model->_allTextures = (SfrTexture**)sfrMalloc(sizeof(SfrTexture*) * data->textures_count);
        model->_texCount = (i32)data->textures_count;
        sfr_memset(model->_allTextures, 0, sizeof(SfrTexture*) * model->_texCount);

        for (i32 i = 0; i < model->_texCount; i += 1) {
            const cgltf_image* const img = data->textures[i].image;
            if (!img) {
                continue;
            }

            u8* rawData = NULL;
            i32 w, h, c;

            if (img->buffer_view) {
                const u8* const buf = (u8*)img->buffer_view->buffer->data + img->buffer_view->offset;
                rawData = stbi_load_from_memory(buf, (i32)img->buffer_view->size, &w, &h, &c, 4);
            } else if (img->uri) {
                if (0 == strncmp(img->uri, "data:", 5)) {
                    void* buf = NULL;
                    if (cgltf_result_success == cgltf_load_buffer_base64(&options, 0, img->uri, &buf)) {
                        rawData = stbi_load_from_memory((u8*)buf, 0, &w, &h, &c, 4);
                        sfrFree(buf);
                    }
                } else {
                    rawData = stbi_load(img->uri, &w, &h, &c, 4);
                }
            }

            if (rawData) {
                SfrTexture* const tex = (SfrTexture*)sfrMalloc(sizeof(SfrTexture));
                tex->w = w;
                tex->h = h;
                tex->pixels = (u32*)sfrMalloc(w * h * 4);
                if (!tex->pixels) {
                    // TODO memory leak
                    SFR__ERR_RET(NULL, "sfr_load_gltf: failed to allocate pixel buffer for texture %d (%ld bytes)\n",
                        i, (u64)(w * h * 4));
                }

                sfr_memcpy(tex->pixels, rawData, w * h * 4);

                stbi_image_free(rawData);

                sfr__swizzle_stb(tex->pixels, w * h);
                sfr__generate_mipmaps(tex);
                model->_allTextures[i] = tex;
            }
        }

        SKIP:;
    #endif

    // vv build transform hierarchy (1:1 with gltf nodes) vv
    model->transforms = (struct sfrTransformNode*)sfrMalloc(sizeof(struct sfrTransformNode) * data->nodes_count);
    model->transformCount = (i32)data->nodes_count;

    for (i32 i = 0; i < model->transformCount; i += 1) {
        const cgltf_node* const cnode = &data->nodes[i];
        struct sfrTransformNode* const tnode = &model->transforms[i];

        tnode->parentInd = -1;
        if (cnode->parent) {
            tnode->parentInd = (i32)(cnode->parent - data->nodes);
        }

        if (cnode->has_matrix) {
            sfrmat m;
            const f32* cm = cnode->matrix;

            // gltf is col major sofren is row major so transpose
            m.m[0][0] = cm[0];
            m.m[0][1] = cm[4];
            m.m[0][2] = cm[8];
            m.m[0][3] = cm[12];
            m.m[1][0] = cm[1];
            m.m[1][1] = cm[5];
            m.m[1][2] = cm[9];
            m.m[1][3] = cm[13];
            m.m[2][0] = cm[2];
            m.m[2][1] = cm[6];
            m.m[2][2] = cm[10];
            m.m[2][3] = cm[14];
            m.m[3][0] = cm[3];
            m.m[3][1] = cm[7];
            m.m[3][2] = cm[11];
            m.m[3][3] = cm[15];

            // convert right handed y up matrix to sofren's left handed flipped z space
            const sfrmat flipZ = sfr_mat_scale(1.f, 1.f, -1.f);
            tnode->localMatrix = sfr_mat_mul(flipZ, sfr_mat_mul(m, flipZ));

            // extract approximate TRS to satisfy struct
            tnode->localPos = (sfrvec){tnode->localMatrix.m[3][0], tnode->localMatrix.m[3][1], tnode->localMatrix.m[3][2], 1.f};
            tnode->localRot = (sfrvec){0.f, 0.f, 0.f, 1.f};
            tnode->localScale = (sfrvec){
                sfr_sqrtf(tnode->localMatrix.m[0][0] * tnode->localMatrix.m[0][0] +
                          tnode->localMatrix.m[0][1] * tnode->localMatrix.m[0][1] +
                          tnode->localMatrix.m[0][2] * tnode->localMatrix.m[0][2]),
                sfr_sqrtf(tnode->localMatrix.m[1][0] * tnode->localMatrix.m[1][0] +
                          tnode->localMatrix.m[1][1] * tnode->localMatrix.m[1][1] +
                          tnode->localMatrix.m[1][2] * tnode->localMatrix.m[1][2]),
                sfr_sqrtf(tnode->localMatrix.m[2][0] * tnode->localMatrix.m[2][0] +
                          tnode->localMatrix.m[2][1] * tnode->localMatrix.m[2][1] +
                          tnode->localMatrix.m[2][2] * tnode->localMatrix.m[2][2]),
                1.f
            };
        } else {
            // init transforms from TRS
            if (cnode->has_translation) {
                tnode->localPos = (sfrvec){cnode->translation[0], cnode->translation[1], -cnode->translation[2], 1.f};
            } else {
                tnode->localPos = (sfrvec){0.f, 0.f, 0.f, 1.f};
            }
            if (cnode->has_rotation) {
                tnode->localRot = (sfrvec){-cnode->rotation[0], -cnode->rotation[1], cnode->rotation[2], cnode->rotation[3]};
            } else {
                tnode->localRot = (sfrvec){0.f, 0.f, 0.f, 1.f};
            }
            if (cnode->has_scale) {
                tnode->localScale = (sfrvec){cnode->scale[0], cnode->scale[1], cnode->scale[2], 1.f};
            } else {
                tnode->localScale = (sfrvec){1.f, 1.f, 1.f, 1.f};
            }

            const sfrmat scale = sfr_mat_scale(tnode->localScale.x, tnode->localScale.y, tnode->localScale.z);
            const sfrmat rot   = sfr_mat_from_quat(tnode->localRot);
            const sfrmat trans = sfr_mat_translate(tnode->localPos.x, tnode->localPos.y, tnode->localPos.z);

            tnode->localMatrix = sfr_mat_mul(scale, sfr_mat_mul(rot, trans));
        }
    }

    { // vv safely resolve world matrices vv
        u8* computed = (u8*)sfrMalloc(model->transformCount);
        sfr_memset(computed, 0, model->transformCount);

        // gltf doesn't guarantee parents appear before children in the array
        for (i32 computedCount = 0; computedCount < model->transformCount;) {
            for (i32 i = 0; i < model->transformCount; i += 1) {
                if (computed[i]) {
                    continue;
                }

                struct sfrTransformNode* const tnode = &model->transforms[i];

                if (tnode->parentInd < 0) {
                    // root node
                    tnode->worldMatrix = tnode->localMatrix;
                    computed[i] = 1;
                    computedCount += 1;
                } else if (computed[tnode->parentInd]) {
                    // parent is fully calculated, safe to compute child
                    const sfrmat local = tnode->localMatrix;
                    const sfrmat parentWorld = model->transforms[tnode->parentInd].worldMatrix;
                    tnode->worldMatrix = sfr_mat_mul(local, parentWorld);

                    computed[i] = 1;
                    computedCount += 1;
                }
            }
        }
        sfrFree(computed);
    }

    // vv count primitives with each becoming an struct sfrModelNode vv
    i32 totalPrimitives = 0;
    for (cgltf_size i = 0; i < data->nodes_count; i += 1) {
        if (data->nodes[i].mesh) {
            totalPrimitives += (i32)data->nodes[i].mesh->primitives_count;
        }
    }

    model->nodes = (struct sfrModelNode*)sfrMalloc(sizeof(struct sfrModelNode) * totalPrimitives);
    model->nodeCount = totalPrimitives;
    // allocate pointers for all meshes even though they map 1:1 with nodes now
    model->_allMeshes = (SfrMesh**)sfrMalloc(sizeof(SfrMesh*) * totalPrimitives);
    model->_meshCount = totalPrimitives;

    i32 currentNodeInd = 0;

    // vv process nodes (flattening hierarchy + splitting prims) vv
    for (cgltf_size i = 0; i < data->nodes_count; i += 1) {
        const cgltf_node* const node = &data->nodes[i];
        if (!node->mesh) {
            continue;
        }

        for (cgltf_size p = 0; p < node->mesh->primitives_count; p += 1) {
            const cgltf_primitive* const prim = &node->mesh->primitives[p];
            if (cgltf_primitive_type_triangles != prim->type) {
                continue; // TODO handle all / more types
            }

            // extract mesh data for this primitive
            cgltf_accessor *posA = NULL, *normA = NULL, *uvA = NULL;
            for (cgltf_size a = 0; a < prim->attributes_count; a += 1) {
                switch (prim->attributes[a].type) {
                    case cgltf_attribute_type_position: {
                        posA = prim->attributes[a].data;
                    } break;
                    case cgltf_attribute_type_normal: {
                        normA = prim->attributes[a].data;
                    } break;
                    case cgltf_attribute_type_texcoord: {
                        if (uvChannel == prim->attributes[a].index) {
                            uvA = prim->attributes[a].data;
                        }
                    } break;
                    default: break;
                }
            }

            if (!posA) {
                continue; // positions required
            }

            const i32 count = prim->indices ? (i32)prim->indices->count : (i32)posA->count;

            SfrMesh* const sm = (SfrMesh*)sfrMalloc(sizeof(SfrMesh));
            sm->vertCount = count * 3;
            sm->tris    = (f32*)sfrMalloc(sizeof(f32) * sm->vertCount);
            sm->normals = (f32*)sfrMalloc(sizeof(f32) * sm->vertCount);
            sm->uvs     = (f32*)sfrMalloc(sizeof(f32) * count * 2);
            if (!sm->tris || !sm->normals || !sm->uvs) {
                // TODO memory leak
                SFR__ERR_RET(NULL, "sfr_load_gltf: failed to allocate mesh buffers (%ld, %ld, %ld bytes)\n",
                    (u64)(sizeof(f32) * sm->vertCount), (u64)(sizeof(f32) * sm->vertCount), (u64)(sizeof(f32) * count * 2));
            }

            for (i32 k = 0; k < count; k += 1) {
                // flip winding order and negate z and xy rotation
                const i32 kr = k - (k % 3) + (i32[3]){0, 2, 1}[k % 3];
                const i32 ind = prim->indices ? (i32)cgltf_accessor_read_index(prim->indices, kr) : kr;

                f32 buf[3];
                cgltf_accessor_read_float(posA, ind, buf, 3);
                sm->tris[k * 3 + 0] = buf[0];
                sm->tris[k * 3 + 1] = buf[1];
                sm->tris[k * 3 + 2] = -buf[2];

                if (normA) {
                    cgltf_accessor_read_float(normA, ind, buf, 3);
                } else {
                    buf[0] = 0;
                    buf[1] = 1;
                    buf[2] = 0;
                }
                sm->normals[k * 3 + 0] = buf[0];
                sm->normals[k * 3 + 1] = buf[1];
                sm->normals[k * 3 + 2] = -buf[2];

                if (uvA) {
                    cgltf_accessor_read_float(uvA, ind, buf, 2);
                } else {
                    buf[0] = 0;
                    buf[1] = 0;
                }
                sm->uvs[k * 2 + 0] = buf[0];
                sm->uvs[k * 2 + 1] = buf[1];
            }

            // vv assign to model node vv
            model->nodes[currentNodeInd].mesh = sm;
            model->nodes[currentNodeInd].tex = NULL;
            model->nodes[currentNodeInd].transformInd = i;

            // vv assign texture for this specific primitive vv
            #ifdef SFR_USE_STB_IMAGE
                if (prim->material &&
                    prim->material->has_pbr_metallic_roughness &&
                    prim->material->pbr_metallic_roughness.base_color_texture.texture &&
                    model->_allTextures
                ) {
                    const cgltf_texture* const texPtr = prim->material->pbr_metallic_roughness.base_color_texture.texture;
                    const ptrdiff_t texInd = texPtr - data->textures;
                    if (texInd >= 0 && texInd < model->_texCount) {
                        model->nodes[currentNodeInd].tex = model->_allTextures[texInd];
                    }
                }
            #endif

            model->_allMeshes[currentNodeInd] = sm;
            currentNodeInd += 1;
        }
    }

    model->nodeCount = currentNodeInd;
    model->_meshCount = currentNodeInd;

    // vv load animations vv
    model->animCount = (i32)data->animations_count;
    if (model->animCount > 0) {
        model->animations = (struct sfrAnimation*)sfrMalloc(sizeof(struct sfrAnimation) * model->animCount);

        for (i32 i = 0; i < model->animCount; i += 1) {
            const cgltf_animation* const ca = &data->animations[i];
            struct sfrAnimation* const sa = &model->animations[i];

            sa->duration = 0.f;
            sa->samplerCount = (i32)ca->samplers_count;
            sa->channelCount = (i32)ca->channels_count;
            sa->samplers = (struct sfrAnimSampler*)sfrMalloc(sizeof(struct sfrAnimSampler) * sa->samplerCount);
            sa->channels = (struct sfrAnimChannel*)sfrMalloc(sizeof(struct sfrAnimChannel) * sa->channelCount);

            for (i32 j = 0; j < sa->samplerCount; j += 1) {
                const cgltf_animation_sampler* const cs = &ca->samplers[j];
                struct sfrAnimSampler* const ss = &sa->samplers[j];

                ss->count = (i32)cs->input->count;
                ss->inputs = (f32*)sfrMalloc(sizeof(f32) * ss->count);
                ss->outputs = (f32*)sfrMalloc(sizeof(f32) * ss->count * 4);

                for (i32 k = 0; k < ss->count; k += 1) {
                    cgltf_accessor_read_float(cs->input, k, &ss->inputs[k], 1);
                    cgltf_accessor_read_float(cs->output, k, &ss->outputs[k * 4], 4);
                }

                if (ss->count > 0 && ss->inputs[ss->count - 1] > sa->duration) {
                    sa->duration = ss->inputs[ss->count - 1];
                }
            }

            for (i32 j = 0; j < sa->channelCount; j += 1) {
                const cgltf_animation_channel* const cc = &ca->channels[j];
                struct sfrAnimChannel* const sc = &sa->channels[j];

                sc->transformNodeInd = (i32)(cc->target_node - data->nodes);
                sc->samplerInd = (i32)(cc->sampler - ca->samplers);

                switch (cc->target_path) {
                    case cgltf_animation_path_type_translation: {
                        sc->path = SFR_ANIM_PATH_TRANSLATION;
                    } break;
                    case cgltf_animation_path_type_rotation: {
                        sc->path = SFR_ANIM_PATH_ROTATION;
                    } break;
                    case cgltf_animation_path_type_scale: {
                        sc->path = SFR_ANIM_PATH_SCALE;
                    } break;
                    default: break;
                }
            }

            { // vv fix animations vv
                u8* patchedSamplers = (u8*)sfrMalloc(sa->samplerCount);
                sfr_memset(patchedSamplers, 0, sa->samplerCount);

                for (i32 j = 0; j < sa->channelCount; j += 1) {
                    struct sfrAnimChannel* const sc = &sa->channels[j];
                    if (patchedSamplers[sc->samplerInd]) continue;
                    patchedSamplers[sc->samplerInd] = 1;

                    struct sfrAnimSampler* const ss = &sa->samplers[sc->samplerInd];

                    if (SFR_ANIM_PATH_TRANSLATION == sc->path) {
                        for (i32 k = 0; k < ss->count; k += 1) {
                            ss->outputs[k * 4 + 2] = -ss->outputs[k * 4 + 2]; // negate z translation
                        }
                    } else if (SFR_ANIM_PATH_ROTATION == sc->path) {
                        for (i32 k = 0; k < ss->count; k += 1) {
                            ss->outputs[k * 4 + 0] = -ss->outputs[k * 4 + 0]; // negate x rotation
                            ss->outputs[k * 4 + 1] = -ss->outputs[k * 4 + 1]; // negate y rotation
                        }
                    }
                }

                sfrFree(patchedSamplers);
            }
        }
    } else {
        model->animations = NULL;
    }

    cgltf_free(data);
    sfrFree(fileContent);

    return model;
}

SFR_FUNC void sfr_model_draw(const SfrModel* model, sfrmat transform, const SfrTexture* overrideTex) {
    if (!model) {
        SFR__ERR_RET(, "sfr_model_draw: model is NULL\n");
    }

    const sfrmat savedModel = sfrMatModel;
    const sfrmat savedNormal = sfrState.matNormal;
    const u8 savedDirty = sfrState.normalMatDirty;

    for (i32 i = 0; i < model->nodeCount; i += 1) {
        const sfrmat localWorld = model->transforms[model->nodes[i].transformInd].worldMatrix;

        const sfrmat m = sfr_mat_mul(localWorld, transform);
        sfrMatModel = sfr_mat_mul(m, savedModel);
        sfrState.normalMatDirty = 1;

        const SfrTexture* tex = overrideTex ? overrideTex : (model->nodes[i].tex ? model->nodes[i].tex : &sfrState.baseTex);
        sfr_mesh(model->nodes[i].mesh, 0xFFFFFFFF, tex);
    }

    sfrMatModel = savedModel;
    sfrState.matNormal = savedNormal;
    sfrState.normalMatDirty = savedDirty;
}

// lossy because of sfr_mat_decompose so might not work perfectly
SFR_FUNC SfrScene* sfr_scene_from_model(const SfrModel* model) {
    if (!model || model->nodeCount <= 0) {
        SFR__ERR_RET(NULL, "sfr_scene_from_model: !model (%p) || model->nodeCount <= 0 (%d)\n", model, model ? model->nodeCount : 0);
    }

    SfrSceneObject* objs = (SfrSceneObject*)sfrMalloc(sizeof(SfrSceneObject) * model->nodeCount);
    if (!objs) {
        SFR__ERR_RET(NULL, "sfr_scene_from_model: failed to allocate objs\n");
    }

    for (i32 i = 0; i < model->nodeCount; i += 1) {
        SfrSceneObject* const obj = &objs[i];

        obj->mesh = model->nodes[i].mesh;
        obj->tex  = model->nodes[i].tex ? model->nodes[i].tex : &sfrState.baseTex;
        obj->col  = 0xFFFFFFFF;

        sfr_mat_decompose(model->transforms[model->nodes[i].transformInd].worldMatrix, &obj->pos, &obj->rot, &obj->scale);
    }

    return sfr_scene_create(objs, model->nodeCount);
}

SFR_FUNC void sfr_release_model(SfrModel** model) {
    if (!model || !(*model)) return;

    SfrModel* m = *model;

    // meshes
    for (i32 i = 0; i < m->_meshCount; i += 1) {
        sfr_release_mesh(&m->_allMeshes[i]);
    }
    sfrFree(m->_allMeshes);

    // textures
    if (m->_allTextures) {
        for (i32 i = 0; i < m->_texCount; i += 1) {
            if (m->_allTextures[i]) sfr_release_texture(&m->_allTextures[i]);
        }
        sfrFree(m->_allTextures);
    }

    // animation data
    if (m->animations) {
        for (i32 i = 0; i < m->animCount; i += 1){
            sfrFree(m->animations[i].samplers->inputs);
            sfrFree(m->animations[i].samplers->outputs);
            sfrFree(m->animations[i].samplers);
            sfrFree(m->animations[i].channels);
        }
        sfrFree(m->animations);
    }

    // hierarchy
    if (m->transforms) {
        sfrFree(m->transforms);
    }
    if (m->nodes) {
        sfrFree(m->nodes);
    }

    sfrFree(m);
    *model = NULL;
}

#endif // SFR_USE_CGLTF

#ifndef SFR_NO_STD

SFR_FUNC SfrMesh* sfr_load_mesh(const char* filename) {
    SfrMesh* mesh = (SfrMesh*)sfrMalloc(sizeof(SfrMesh));
    if (!mesh) {
        SFR__ERR_RET(NULL, "sfr_load_mesh: failed to allocate SfrMesh struct\n");
    }
    *mesh = (SfrMesh){.tris = NULL, .uvs = NULL, .normals = NULL, .vertCount = 0};

    FILE* objFile = fopen(filename, "rb");
    if (!objFile) {
        sfrFree(mesh);
        SFR__ERR_RET(NULL, "sfr_load_mesh: failed to open file '%s'\n", filename);
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
                        SFR__ERR_EXIT("sfr_load_mesh: realloc failed for tempVerts\n");
                    }
                }
                sscanf(line, "v %f %f %f", &tempVerts[vertCount].x, &tempVerts[vertCount].y, &tempVerts[vertCount].z);
                vertCount += 1;
            } else if ('t' == line[1]) { // texture coordinate
                if (uvCount >= tempUVsCap) {
                    tempUVsCap = (0 == tempUVsCap) ? 128 : tempUVsCap * 2;
                    tempUVs = (sfrvec*)realloc(tempUVs, tempUVsCap * sizeof(sfrvec));
                    if (!tempUVs) {
                        SFR__ERR_EXIT("sfr_load_mesh: realloc failed for tempUVs\n");
                    }
                }
                sscanf(line, "vt %f %f", &tempUVs[uvCount].x, &tempUVs[uvCount].y);
                uvCount += 1;
            } else if ('n' == line[1]) { // vertex normal
                if (normalCount >= tempNormalsCap) {
                    tempNormalsCap = (0 == tempNormalsCap) ? 128 : tempNormalsCap * 2;
                    tempNormals = (sfrvec*)realloc(tempNormals, tempNormalsCap * sizeof(sfrvec));
                    if (!tempNormals) {
                        SFR__ERR_EXIT("sfr_load_mesh: realloc failed for tempNormals\n");
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
                    SFR__ERR_EXIT("sfr_load_mesh: realloc failed for tempFaces\n");
                }
            }
            FaceIndices* f = &tempFaces[faceCount];
            f->count = 0;
            char* token = strtok(line + 1, " \t\n");
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
                token = strtok(NULL, " \t\n");
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
    mesh->tris = (f32*)sfrMalloc(mesh->vertCount * sizeof(f32));
    mesh->uvs = (f32*)sfrMalloc(finalVertCount * 2 * sizeof(f32));
    mesh->normals = (f32*)sfrMalloc(mesh->vertCount * sizeof(f32));

    if (!mesh->tris || !mesh->uvs || !mesh->normals) {
        SFR__ERR_EXIT("sfr_load_mesh: failed to allocate final mesh buffers\n");
    }

    sfrvec* computedNormals = NULL;
    if (0 == normalCount && vertCount > 0) { // if no normals in file
        computedNormals = (sfrvec*)calloc(vertCount, sizeof(sfrvec));
        if (!computedNormals) {
            SFR__ERR_EXIT("sfr_load_mesh: calloc failed for computedNormals\n");
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
    sfrFree(tempVerts);
    sfrFree(tempUVs);
    sfrFree(tempNormals);
    sfrFree(tempFaces);
    sfrFree(computedNormals);

    return mesh;
}

SFR_FUNC void sfr_release_mesh(SfrMesh** mesh) {
    if (!mesh || !(*mesh)) {
        return;
    }

    if ((*mesh)->tris) {
        sfrFree((*mesh)->tris);
        (*mesh)->tris = NULL;
    }

    if ((*mesh)->uvs) {
        sfrFree((*mesh)->uvs);
        (*mesh)->uvs = NULL;
    }

    if ((*mesh)->normals) {
        sfrFree((*mesh)->normals);
        (*mesh)->normals = NULL;
    }

    sfrFree(*mesh);
    *mesh = NULL;
}

SFR_FUNC SfrTexture* sfr_load_texture(const char* filename) {
#ifdef SFR_USE_STB_IMAGE
    i32 w, h, channels;
    u8* data = stbi_load(filename, &w, &h, &channels, 4); // force 4 channels
    if (!data) {
        SFR__ERR_RET(NULL, "sfr_load_texture: stbi failed to load '%s'\n", filename);
    }

    SfrTexture* tex = (SfrTexture*)sfrMalloc(sizeof(SfrTexture));
    tex->w = w;
    tex->h = h;
    tex->pixels = (u32*)sfrMalloc(sizeof(u32) * w * h);

    sfr_memcpy(tex->pixels, data, w * h * 4);
    stbi_image_free(data);

    sfr__swizzle_stb(tex->pixels, w * h);
    sfr__generate_mipmaps(tex);
    return tex;
#else
    FILE* file = fopen(filename, "rb");
    if (!file) {
        SFR__ERR_RET(NULL, "sfr_load_texture: failed to open file '%s'\n", filename);
    }

    // read bmp headers
    u8 header[54];
    if (fread(header, 1, 54, file) != 54) {
        fclose(file);
        SFR__ERR_RET(NULL, "sfr_load_texture: not a valid BMP file ('%s')\n", filename);
    }

    if (header[0] != 'B' || header[1] != 'M') {
        fclose(file);
        SFR__ERR_RET(NULL, "sfr_load_texture: not a BMP file ('%s')\n", filename);
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
        SFR__ERR_RET(NULL, "sfr_load_texture: unsupported BMP compression\n");
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
            SFR__ERR_RET(NULL, "sfr_load_texture: failed to read color palette\n");
        }
    } else if (3 == comp && 4 != fread(masks, 4, 4, file)) { // BI_BITFIELDS
        fclose(file);
        SFR__ERR_RET(NULL, "sfr_load_texture: failed to read bit masks\n");
    }

    // stride and data size
    const i32 stride = ((width * bpp + 31) / 32) * 4;
    const i32 dataSize = stride * height;

    // read pixel data
    u8* pixelData = (u8*)sfrMalloc(dataSize);
    if (!pixelData) {
        fclose(file);
        SFR__ERR_RET(NULL, "sfr_load_texture: failed to allocate pixel data\n");
    }

    fseek(file, dataOffset, SEEK_SET);
    if (fread(pixelData, 1, dataSize, file) != dataSize) {
        sfrFree(pixelData);
        fclose(file);
        SFR__ERR_RET(NULL, "sfr_load_texture: failed to read pixel data\n");
    }
    fclose(file);

    // create texture
    SfrTexture* tex = (SfrTexture*)sfrMalloc(sizeof(SfrTexture));
    if (!tex) {
        sfrFree(pixelData);
        SFR__ERR_RET(NULL, "sfr_load_texture: failed to allocate texture struct\n");
    }

    tex->w = width;
    tex->h = height;
    tex->pixels = (u32*)sfrMalloc(width * height * sizeof(u32));
    if (!tex->pixels) {
        sfrFree(pixelData);
        sfrFree(tex);
        SFR__ERR_RET(NULL, "sfr_load_texture: failed to allocate texture pixels\n");
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
                    sfrFree(pixelData);
                    sfrFree(tex->pixels);
                    sfrFree(tex);
                    SFR__ERR_RET(NULL, "sfr_load_texture: unsupported bit depth: %d\n", bpp);
                } break;
            }

            tex->pixels[i] = col;
        }
    }

    sfrFree(pixelData);

    sfr__generate_mipmaps(tex);

    return tex;
#endif
}

SFR_FUNC void sfr_release_texture(SfrTexture** tex) {
    if (!tex || !(*tex)) {
        return;
    }

    if ((*tex)->pixels) {
        sfrFree((*tex)->pixels);
        (*tex)->pixels = NULL;
    }

    for (i32 i = 0; i < (*tex)->mipLevels - 1; i += 1) {
        if ((*tex)->mipPixels[i]) {
            sfrFree((*tex)->mipPixels[i]);
        }
    }

    sfrFree(*tex);
    *tex = NULL;
}

SFR_FUNC SfrFont* sfr_load_font(const char* filename) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        SFR__ERR_RET(NULL, "sfr_load_font: failed to open file '%s'\n", filename);
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
        SFR__ERR_RET(NULL, "sfr_load_font: not a valid .srft file ('%s')\n", filename);
    }

    // allocate space for font
    SfrFont* font = (SfrFont*)sfrMalloc(sizeof(SfrFont));
    if (!font) {
        fclose(file);
        SFR__ERR_RET(NULL, "sfr_load_font: failed to allocate struct\n");
    }

    // load font
    for (i32 i = 0; i < SFR_FONT_GLYPH_MAX; i += 1) {
        for (i32 j = 0; j < SFR_FONT_VERT_MAX; j += 1) {
            font->verts[i][j] = SFR_FONT_VERT_EMPTY;
        }

        u8 vertCount;
        if (1 != fread(&vertCount, 1, 1, file)) {
            fclose(file);
            sfrFree(font);
            SFR__ERR_RET(NULL, "sfr_load_font: error reading from file\n");
        }
        if (vertCount >= SFR_FONT_VERT_MAX) {
            fclose(file);
            sfrFree(font);
            SFR__ERR_RET(NULL,
                "sfr_load_font: vert count out of bounds for glyph '%c' (%d) (%d >= %d)\n",
                i, i, vertCount, SFR_FONT_VERT_MAX);
        }

        if (vertCount && vertCount != fread(font->verts[i], 4, vertCount, file)) {
            fclose(file);
            sfrFree(font);
            SFR__ERR_RET(NULL, "sfr_load_font: error reading vertices from file\n");
        }
    }

    return font;
}

SFR_FUNC void sfr_release_font(SfrFont** font) {
    if (!font || !(*font)) {
        return;
    }

    sfrFree(*font);
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
        const f32 size = SFR__LERPF(p->startSize, p->endSize, t);
        const u32 col = sfr__lerp_col(p->startCol, p->endCol, t);

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
        SFR__ERR_RET(min, "sfr_rand_int: min >= max (%d >= %d)", min, max);
    }
    return (i32)(sfr_rand_next() % (max - min)) + min;
}

SFR_FUNC f32 sfr_rand_flt(f32 min, f32 max) {
    if (min >= max) {
        SFR__ERR_RET(min, "sfr_rand_flt: min >= max (%f >= %f)", min, max);
    }
    return (f32)(min + sfr_rand_next() / (f64)0xFFFFFFFF * ((f64)max - (f64)min));
}

#ifdef SFR_MULTITHREADED

#ifdef _WIN32
static unsigned __stdcall sfr__worker_thread_func(void* arg) {
#else
static void* sfr__worker_thread_func(void* arg) {
#endif
    (void)arg;
    const struct sfrThreadData* self = (struct sfrThreadData*)arg;
    sfrTlsThreadInd = self->threadInd;

    while (1) {
        // vv geometry phase vv
        sfr_semaphore_wait(&sfrThreadBuf->geometryStartSem);
        if (sfrState.shutdown) {
            break;
        }

        sfrTlsBinStart = 0;
        sfrTlsBinEnd = 0;

        while (1) {
            // get the next geometry job from the queue
            const i32 head = sfr_atomic_add(&sfrThreadBuf->geometryWorkQueueHead, 1) - 1;
            if (head >= sfr_atomic_get(&sfrThreadBuf->geometryWorkQueueCount)) {
                break; // no more geometry work
            }

            const i32 jobInd = sfrThreadBuf->geometryWorkQueue[head];
            const struct sfrMeshChunkJob* job = &sfrThreadBuf->meshJobPool[jobInd];

            i32 i = 0;

            #ifndef SFR_NO_SIMD
                for (; i <= job->triangleCount - 8; i += 8) {
                    const i32 triInd = i * 9;
                    const i32 uvInd = i * 6;
                    sfr__process_and_bin_triangles8(
                        &job->matMVP, &job->matNormal,
                        &job->tris[triInd],
                        job->uvs ? &job->uvs[uvInd] : NULL,
                        &job->normals[triInd],
                        job->col, job->tex
                    );
                }
            #endif

            // process all triangles in this job
            for (; i < job->triangleCount; i += 1) {
                const i32 triInd = i * 9;
                const i32 uvInd = i * 6;
                sfr__process_and_bin_triangle(
                    &job->matMVP, &job->matNormal,
                    job->tris[triInd + 0], job->tris[triInd + 1], job->tris[triInd + 2],
                    job->uvs ? job->uvs[uvInd + 0] : 0.f,
                    job->uvs ? job->uvs[uvInd + 1] : 0.f,
                    job->normals[triInd + 0], job->normals[triInd + 1], job->normals[triInd + 2],

                    job->tris[triInd + 3], job->tris[triInd + 4], job->tris[triInd + 5],
                    job->uvs ? job->uvs[uvInd + 2] : 0.f,
                    job->uvs ? job->uvs[uvInd + 3] : 0.f,
                    job->normals[triInd + 3], job->normals[triInd + 4], job->normals[triInd + 5],

                    job->tris[triInd + 6], job->tris[triInd + 7], job->tris[triInd + 8],
                    job->uvs ? job->uvs[uvInd + 4] : 0.f,
                    job->uvs ? job->uvs[uvInd + 5] : 0.f,
                    job->normals[triInd + 6], job->normals[triInd + 7], job->normals[triInd + 8],
                    job->col, job->tex
                );
            }
        }
        sfr_semaphore_post(&sfrThreadBuf->geometryDoneSem, 1);

        // vv rasterization phase vv
        sfr_semaphore_wait(&sfrThreadBuf->rasterStartSem);
        if (sfrState.shutdown) {
            break;
        }

        while (1) {
            // get the next tile index from the work queue
            const i32 head = sfr_atomic_add(&sfrThreadBuf->rasterWorkQueueHead, 1) - 1;
            if (head >= sfr_atomic_get(&sfrThreadBuf->rasterWorkQueueCount)) {
                break; // no more raster work
            }

            // get the tile and rasterize its contents
            struct sfrTile* tile = &sfrThreadBuf->tiles[sfrThreadBuf->rasterWorkQueue[head]];

            sfr__load_tile(tile);

            // loop through all thread bins for this tile
            for (i32 t = 0; t <= SFR_THREAD_COUNT; t += 1) {
                const i32 count = tile->binCount[t];
                for (i32 i = 0; i < count; i += 1) {
                    sfr__rasterize_bin(tile->bins[t][i], tile);
                }

                tile->binCount[t] = 0;
            }

            sfr__resolve_tile(tile);

            sfr_atomic_set(&tile->hasWork, 0);
        }
        sfr_semaphore_post(&sfrThreadBuf->rasterDoneSem, 1);
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
