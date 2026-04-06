#define SFR_IMPL
#define SFR_THREAD_COUNT 8
#define SFR_USE_CGLTF
#define SFR_USE_STB_IMAGE

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

typedef struct sfrmesh     SfrMesh;
typedef struct sfrtex      SfrTexture;
typedef struct sfrMaterial SfrMaterial;
typedef struct sfrfont     SfrFont;

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
SFR_FUNC void sfr_skybox(const SfrMaterial* faces[6]);

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
    u32 col, const SfrMaterial* mat);

// other drawing functions, if mat is null sfrState.baseMat (white 1x1 texture) will be used
SFR_FUNC void sfr_point(f32 worldX, f32 worldY, f32 worldZ, i32 radius, u32 col);
SFR_FUNC void sfr_billboard(u32 col, const SfrMaterial* mat);
SFR_FUNC void sfr_cube(u32 col, const SfrMaterial* mat);
SFR_FUNC void sfr_cube_ex(u32 col[12]);
SFR_FUNC void sfr_mesh(const SfrMesh* mesh, u32 col, const SfrMaterial* mat);
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
SFR_FUNC void sfr_set_rendermode(i32 mode); // takes SFR_RENDERMODE_[mode] enum

// helpers for adding and removing lights, only enabled when lighting is enabled
SFR_FUNC SfrLight* sfr_light_add_point(f32 posX, f32 posY, f32 posZ, f32 ambient, f32 intensity, f32 attenuation, f32 r, f32 g, f32 b);
SFR_FUNC SfrLight* sfr_light_add_directional(f32 dirX, f32 dirY, f32 dirZ, f32 ambient, f32 intensity, f32 r, f32 g, f32 b);
SFR_FUNC void sfr_light_remove(SfrLight* light);

#ifdef SFR_USE_CGLTF
    SFR_FUNC void sfr_model_animate(SfrModel* model, i32 animInd, f32 time);
    SFR_FUNC SfrModel* sfr_load_gltf(const char* filename, i32 uvChannel);
    SFR_FUNC void sfr_model_draw(const SfrModel* model, sfrmat transform, const SfrMaterial* overrideMat);
    SFR_FUNC SfrScene* sfr_scene_from_model(const SfrModel* model);
    SFR_FUNC void sfr_release_model(SfrModel** model);
#endif

// things requiring stdio
#ifndef SFR_NO_STD
    SFR_FUNC SfrMesh* sfr_load_mesh(const char* filename); // load an obj file into a struct that sofren can use
    SFR_FUNC void sfr_release_mesh(SfrMesh** mesh);        // release loaded mesh's memory

    // wrappers of sfr_load_texture and sfr_release_texture
    SFR_FUNC SfrMaterial* sfr_load_material(const char* albedoPath, const char* normalPath, const char* metRoughPath);
    SFR_FUNC void sfr_release_material(SfrMaterial** mat);

    SFR_FUNC SfrTexture* sfr_load_texture(const char* filename); // load a BMP texture
    SFR_FUNC void sfr_release_texture(SfrTexture** texture);     // release loaded texture's memory

    SFR_FUNC SfrFont* sfr_load_font(const char* filename); // load a .srft (sofren font type) font, see 'sfr-fontmaker'
    SFR_FUNC void sfr_release_font(SfrFont** font);        // release loaded font's memory
#endif

// all particle related functions
SFR_FUNC SfrParticleSystem sfr_particles_create(SfrParticle* buffer, i32 count, const SfrMaterial* mat);
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
#else
    typedef union sfrvec { struct { f32 x, y, z, w; }; } sfrvec;
#endif

typedef union sfrmat {
    struct { f32 m[4][4]; };
    sfrvec rows[4];
} sfrmat;

typedef struct sfrmesh {
    f32* tris;     // vertex positions (3 floats per vert)
    f32* uvs;      // uv coordinates (2 floats per vert)
    f32* normals;  // vertex normals (3 floats per vert)
    f32* tangents; // vertex tangents (4 floats per vert (x, y, z, sign))
    i32 vertCount; // total number of floats in tris array
} SfrMesh;

typedef struct sfrtex {
    u32* pixels;
    i32 w, h;

    u32* mipPixels[14]; // levels 1-14, up to 16k x 16k textures
    i32 mipW[14];
    i32 mipH[14];
    i32 mipLevels; // total levels including base, i.e. 1 => no extra levels
} SfrTexture;

typedef struct sfrMaterial {
    // texture maps
    SfrTexture* albedoTex;
    SfrTexture* metallicRoughnessTex; // g = roughness, b = metallic

    // fallbacks
    u32 baseColor;
    f32 metallicFactor;
    f32 roughnessFactor;
} SfrMaterial;

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
    SfrMaterial* mat;
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
    SfrMaterial* mat;
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
    SfrMaterial** _allMaterials;
    i32 _matCount;
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
    const SfrMaterial* mat;
} SfrParticleSystem;

typedef struct sfrlight {
    enum sfrLightType {
        SFR_LIGHT_DIRECTIONAL,
        SFR_LIGHT_POINT,
    } type;
    f32 x, y, z;     // position in point lights, direction in directional lights
    f32 r, g, b;     // color [0.0, 1.0]
    f32 ambient;     // ambient contribution
    f32 intensity;   // diffuse/specular multiplier
    f32 attenuation; // radius/falloff for point lights
} SfrLight;

// pre divided data
struct sfrVertexData {
    f32 invZ;
    f32 u, v;
    f32 nx, ny, nz;
    f32 tx, ty, tz, tw;
    f32 wx, wy, wz;
};

// helper to track vertex attributes during clipping
struct sfrTexVert {
    sfrvec pos;      // position in view space
    f32 u, v;        // texture coords
    sfrvec normal;   // world space normal for lighting
    sfrvec tangent;  // transformed tangent and handedness sign in w
    sfrvec worldPos; // world position
    f32 viewZ;       // z in view space for perspective correction
};

struct sfrTriangleBin {
    i32 binId;

    // needed for barycentric weights
    f32 invDet;

    // depth buffer equations (rasterizer)
    f32 dzdx, dzdy, zBase;

    // edge equations (rasterizer, barycentric weights)
    f32 A0, B0, C0;
    f32 A1, B1, C1;
    f32 A2, B2, C2;

    // bounding box (rasterizer)
    i32 minX, maxX;
    i32 minY, maxY;

    // the 3 verts and their attributes (resolver)
    struct sfrVertexData v0;
    struct sfrVertexData v1;
    struct sfrVertexData v2;

    u32 col;
    const SfrMaterial* mat;
};

struct sfrTile {
    i32 minX, minY, maxX, maxY;
    f32 minInvZ; // furthest visible depth in the tile (stored natively as 1/Z)

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
        const f32* tangents;
        sfrmat matNormal;
        sfrmat matMVP;
        sfrmat matModel;
        u32 col;
        const SfrMaterial* mat;
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

    sfrmat matNormal;
    u8 normalMatDirty;

    u32 randState;

    f32 halfWidth, halfHeight;

    sfrvec clipPlanes[4][2];

    SfrMaterial baseMat;

    SfrLight* lights;
    i32 lightCount, lightCap;

    enum {
        SFR_RENDERMODE_SHADED,
        SFR_RENDERMODE_NORMALS,
        SFR_RENDERMODE_FRAGPOS,
        SFR_RENDERMODE_COUNT
    } renderMode;

#ifdef SFR_MULTITHREADED
    u8 shutdown;
#else
    struct sfrTriangleBin* globalBins;
    i32 globalBinsCount, globalBinsCap;

    i32* idBuf;
#endif
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

// only used during sfr__resolve_pixel not sfr__resolve_chunk, so
// sfr__resolve_chunk currently approximates at ~2.0 instead of 2.2
static u8 sfrGammaLUT[256];


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
        fprintf(stderr, "fatal sofren (%s) error at line %d in function '%s':\n\t", __FILE__, __LINE__, __FUNCTION__); \
        fprintf(stderr, __VA_ARGS__); \
        exit(1); \
    }
    #define SFR__ERR_RET(_r, ...) { \
        fprintf(stderr, "sofren (%s) error at line %d in function '%s':\n\t", __FILE__, __LINE__, __FUNCTION__); \
        fprintf(stderr, __VA_ARGS__); \
        return _r; \
    }

    #define SFR__MALLOC(r, s) \
        (r) = sfrMalloc(s); \
        if (!(r)) { SFR__ERR_EXIT("malloc failed (%ld bytes)\n", (i64)(s)); }

#else
    #ifndef SFR_NO_WARNINGS
        #warning "SFR WARNING: If there is an internal error it will not be reported (SFR_NO_STD defined)"
    #endif
    #define SFR__ERR_EXIT(...) { sfrPixelBuf[999999999] = 1; } // crash the program to exit, hopefully there arent 1 billion pixels
    #define SFR__ERR_RET(_r, ...) return _r

    #define SFR__MALLOC(r, s) r = sfrMalloc(s)
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
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    r.w = 1.f;
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

    if (mesh->tangents) {
        for (i32 i = 0; i < 12; i += 1) {
            SFR__SWAP(f32, mesh->tangents[a * 12 + i], mesh->tangents[b * 12 + i]);
        }
    }
}

static void sfr__compute_mesh_tangents(SfrMesh* mesh) {
    if (!mesh->uvs || !mesh->normals) {
        return;
    }

    const i32 vertexCount = mesh->vertCount / 3;
    if (!mesh->tangents) {
        SFR__MALLOC(mesh->tangents, sizeof(f32) * vertexCount * 4);
        sfr_memset(mesh->tangents, 0, sizeof(f32) * vertexCount * 4);
    }

    // arrays to accumulate tangents and bitangents before orthonormalization
    sfrvec *tan1, *tan2;
    SFR__MALLOC(tan1, sizeof(sfrvec) * vertexCount);
    SFR__MALLOC(tan2, sizeof(sfrvec) * vertexCount);
    sfr_memset(tan1, 0, sizeof(sfrvec) * vertexCount);
    sfr_memset(tan2, 0, sizeof(sfrvec) * vertexCount);

    for (i32 i = 0; i < vertexCount; i += 3) {
        // get positions
        const sfrvec p0 = {mesh->tris[(i + 0) * 3 + 0], mesh->tris[(i + 0) * 3 + 1], mesh->tris[(i + 0) * 3 + 2], 0.f};
        const sfrvec p1 = {mesh->tris[(i + 1) * 3 + 0], mesh->tris[(i + 1) * 3 + 1], mesh->tris[(i + 1) * 3 + 2], 0.f};
        const sfrvec p2 = {mesh->tris[(i + 2) * 3 + 0], mesh->tris[(i + 2) * 3 + 1], mesh->tris[(i + 2) * 3 + 2], 0.f};

        // get uvs
        const f32 u0 = mesh->uvs[(i + 0) * 2 + 0], v0 = mesh->uvs[(i + 0) * 2 + 1];
        const f32 u1 = mesh->uvs[(i + 1) * 2 + 0], v1 = mesh->uvs[(i + 1) * 2 + 1];
        const f32 u2 = mesh->uvs[(i + 2) * 2 + 0], v2 = mesh->uvs[(i + 2) * 2 + 1];

        const sfrvec edge1 = sfr_vec_sub(p1, p0);
        const sfrvec edge2 = sfr_vec_sub(p2, p0);

        const f32 deltaU1 = u1 - u0;
        const f32 deltaV1 = v1 - v0;
        const f32 deltaU2 = u2 - u0;
        const f32 deltaV2 = v2 - v0;

        const f32 f = 1.f / (deltaU1 * deltaV2 - deltaU2 * deltaV1);

        const sfrvec t = {
            f * (deltaV2 * edge1.x - deltaV1 * edge2.x),
            f * (deltaV2 * edge1.y - deltaV1 * edge2.y),
            f * (deltaV2 * edge1.z - deltaV1 * edge2.z),
            0.f
        };

        const sfrvec b = {
            f * (-deltaU2 * edge1.x + deltaU1 * edge2.x),
            f * (-deltaU2 * edge1.y + deltaU1 * edge2.y),
            f * (-deltaU2 * edge1.z + deltaU1 * edge2.z),
            0.f
        };

        tan1[i + 0] = sfr_vec_add(tan1[i + 0], t);
        tan1[i + 1] = sfr_vec_add(tan1[i + 1], t);
        tan1[i + 2] = sfr_vec_add(tan1[i + 2], t);

        tan2[i + 0] = sfr_vec_add(tan2[i + 0], b);
        tan2[i + 1] = sfr_vec_add(tan2[i + 1], b);
        tan2[i + 2] = sfr_vec_add(tan2[i + 2], b);
    }

    // gram-schmidt orthogonalize
    for (i32 i = 0; i < vertexCount; i += 1) {
        const sfrvec n = { mesh->normals[i * 3 + 0], mesh->normals[i * 3 + 1], mesh->normals[i * 3 + 2], 0.f };
        const sfrvec t = tan1[i];

        // T' = normalize(T - N * dot(N, T))
        const f32 dotNT = sfr_vec_dot(n, t);
        const sfrvec tOrtho = sfr_vec_norm(sfr_vec_sub(t, sfr_vec_mul(n, dotNT)));

        // calculate handedness
        const sfrvec crossNT = sfr_vec_cross(n, tOrtho);
        const f32 w = (sfr_vec_dot(crossNT, tan2[i]) < 0.f) ? -1.f : 1.f;

        mesh->tangents[i * 4 + 0] = tOrtho.x;
        mesh->tangents[i * 4 + 1] = tOrtho.y;
        mesh->tangents[i * 4 + 2] = tOrtho.z;
        mesh->tangents[i * 4 + 3] = w;
    }

    sfrFree(tan1);
    sfrFree(tan2);
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
    union { f32 f; u32 u; } temp = { .f = x };
    return (i32)((temp.u >> 23) & 0xFF) - 127;
}

static struct sfrTexVert sfr__lerp_vert(struct sfrTexVert a, struct sfrTexVert b, f32 t) {
    return (struct sfrTexVert){
        .pos = sfr_vec_lerp(a.pos, b.pos, t),
        .u = SFR__LERPF(a.u, b.u, t),
        .v = SFR__LERPF(a.v, b.v, t),
        .viewZ = SFR__LERPF(a.viewZ, b.viewZ, t),
        .normal = sfr_vec_lerp(a.normal, b.normal, t),
        .tangent = sfr_vec_lerp(a.tangent, b.tangent, t),
        .worldPos = sfr_vec_lerp(a.worldPos, b.worldPos, t)
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
        const f32 xmInvZ = ((dzdx > 0.f) ? tMaxX : tMinX) - bin->minX;
        const f32 ymInvZ = ((dzdy > 0.f) ? tMaxY : tMinY) - bin->minY;
        const f32 maxInvZ = zBase + xmInvZ * dzdx + ymInvZ * dzdy;

        // maxInvZ is the closest possible point of this triangle block,
        // if the closest point is still further away (smaller 1/Z) than the furthest pixel
        // recorded in this tile (minInvZ) then it's completely occluded
        if (maxInvZ < tile->minInvZ) {
            return;
        }

        // update tile->minInvZ
        const f32 xmE0 = ((A0 > 0.f) ? tMinX : tMaxX) - bin->minX;
        const f32 ymE0 = ((B0 > 0.f) ? tMinY : tMaxY) - bin->minY;
        const f32 minE0 = C0 + A0 * xmE0 + B0 * ymE0;

        const f32 xmE1 = ((A1 > 0.f) ? tMinX : tMaxX) - bin->minX;
        const f32 ymE1 = ((B1 > 0.f) ? tMinY : tMaxY) - bin->minY;
        const f32 minE1 = C1 + A1 * xmE1 + B1 * ymE1;

        const f32 xmE2 = ((A2 > 0.f) ? tMinX : tMaxX) - bin->minX;
        const f32 ymE2 = ((B2 > 0.f) ? tMinY : tMaxY) - bin->minY;
        const f32 minE2 = C2 + A2 * xmE2 + B2 * ymE2;

        if (minE0 >= 0.f && minE1 >= 0.f && minE2 >= 0.f) {
            const f32 x = ((dzdx < 0.f) ? tMaxX : tMinX) - bin->minX;
            const f32 y = ((dzdy < 0.f) ? tMaxY : tMinY) - bin->minY;
            const f32 minInvZInTile = zBase + x * dzdx + y * dzdy;
            if (minInvZInTile > tile->minInvZ) {
                tile->minInvZ = minInvZInTile;
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

    // vv precompute max offsets for trivial rejection vv
    const f32 e0MaxOffset = sfr_fmaxf(0.f, A0 * 7.f) + sfr_fmaxf(0.f, B0 * 7.f);
    const f32 e1MaxOffset = sfr_fmaxf(0.f, A1 * 7.f) + sfr_fmaxf(0.f, B1 * 7.f);
    const f32 e2MaxOffset = sfr_fmaxf(0.f, A2 * 7.f) + sfr_fmaxf(0.f, B2 * 7.f);

    // vv setup vectors vv
    const __m256 vA0 = _mm256_set1_ps(A0), vB0 = _mm256_set1_ps(B0), vC0 = _mm256_set1_ps(C0);
    const __m256 vA1 = _mm256_set1_ps(A1), vB1 = _mm256_set1_ps(B1), vC1 = _mm256_set1_ps(C1);
    const __m256 vA2 = _mm256_set1_ps(A2), vB2 = _mm256_set1_ps(B2), vC2 = _mm256_set1_ps(C2);

    const __m256 vZBase = _mm256_set1_ps(zBase), vdzdx = _mm256_set1_ps(dzdx), vdzdy = _mm256_set1_ps(dzdy);

    const __m256 vXOffsets = _mm256_setr_ps(0.5f, 1.5f, 2.5f, 3.5f, 4.5f, 5.5f, 6.5f, 7.5f);
    const __m256i vIndex = _mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7);

    const __m256i vBinId = _mm256_set1_epi32(bin->binId);

    for (i32 yBase = minY; yBase < maxY; yBase += 8) {
        const f32 fy = (f32)(yBase - bin->minY) + 0.5f;
        const __m256 vy = _mm256_set1_ps(fy);

        // trivial rejection test
        for (i32 xBase = alignedMinX; xBase < maxX; xBase += 8) {
            const f32 fx = (f32)(xBase - bin->minX);

            // trivial rejection
            if (C0 + A0 * (fx + 0.5f) + B0 * fy + e0MaxOffset < 0.f) continue;
            if (C1 + A1 * (fx + 0.5f) + B1 * fy + e1MaxOffset < 0.f) continue;
            if (C2 + A2 * (fx + 0.5f) + B2 * fy + e2MaxOffset < 0.f) continue;

            const __m256 vXBlock = _mm256_add_ps(_mm256_set1_ps(fx), vXOffsets);

            // start values at (xBase, yBase)
            __m256 vE0 = _mm256_add_ps(vC0, _mm256_add_ps(_mm256_mul_ps(vA0, vXBlock), _mm256_mul_ps(vB0, vy)));
            __m256 vE1 = _mm256_add_ps(vC1, _mm256_add_ps(_mm256_mul_ps(vA1, vXBlock), _mm256_mul_ps(vB1, vy)));
            __m256 vE2 = _mm256_add_ps(vC2, _mm256_add_ps(_mm256_mul_ps(vA2, vXBlock), _mm256_mul_ps(vB2, vy)));

            // vz is completely native 1/Z now
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
                    #else
                        const i32 pixelInd = y * sfrWidth + xBase;
                    #endif
                    const __m256 vOldDepth = _mm256_loadu_ps(&targetDepthBuf[pixelInd]);

                    // closer objects have larger 1/Z values
                    const __m256 depthMask = _mm256_cmp_ps(vz, vOldDepth, _CMP_GT_OQ);
                    mask = _mm256_and_ps(mask, depthMask);
                    maskInt = _mm256_movemask_ps(mask);

                    if (0xFF == maskInt) {
                        // 100% coverage so skip the load/blend and store
                        _mm256_storeu_ps(&targetDepthBuf[pixelInd], vz);
                        _mm256_storeu_si256((__m256i*)&targetIdBuf[pixelInd], vBinId);
                    } else if (maskInt) {
                        const __m256 vNewDepth = _mm256_blendv_ps(vOldDepth, vz, mask);
                        _mm256_storeu_ps(&targetDepthBuf[pixelInd], vNewDepth);
                        _mm256_maskstore_epi32((int*)&targetIdBuf[pixelInd], _mm256_castps_si256(mask), vBinId);
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
        const f32 ly = (f32)(y - bin->minY) + 0.5f;
        const f32 lx = (f32)(minX - bin->minX) + 0.5f;
        f32 E0 = C0 + A0 * lx + B0 * ly;
        f32 E1 = C1 + A1 * lx + B1 * ly;
        f32 E2 = C2 + A2 * lx + B2 * ly;
        f32 z = zBase + dzdx * lx + dzdy * ly;

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

            // closer objects have larger values so if z < what's stored it's behind
            if (z <= targetDepthBuf[pixelInd]) {
                continue;
            }

            targetDepthBuf[pixelInd] = z;
            targetIdBuf[pixelInd] = bin->binId;
        }
    }
}

static void sfr__bin_triangle(
    f32 ax, f32 ay, f32 bx, f32 by, f32 cx, f32 cy,
    const struct sfrVertexData* v0,
    const struct sfrVertexData* v1,
    const struct sfrVertexData* v2,
    u32 col, const SfrMaterial* mat
) {
    const f32 det = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
    if (0.f == det) {
        return;
    }
    const f32 invDet = 1.f / det;

    const f32 minX = sfr_fmaxf(0.f,             sfr_floorf(sfr_fminf(ax, SFR_MIN(bx, cx))));
    const f32 maxX = sfr_fminf(sfrWidth - 1.f,  sfr_ceilf (sfr_fmaxf(ax, SFR_MAX(bx, cx))));
    const f32 minY = sfr_fmaxf(0.f,             sfr_floorf(sfr_fminf(ay, SFR_MIN(by, cy))));
    const f32 maxY = sfr_fminf(sfrHeight - 1.f, sfr_ceilf (sfr_fmaxf(ay, SFR_MAX(by, cy))));

    const f32 fMinX = (f32)minX, fMinY = (f32)minY;
    const f32 lx0 = ax - fMinX, ly0 = ay - fMinY;
    const f32 lx1 = bx - fMinX, ly1 = by - fMinY;
    const f32 lx2 = cx - fMinX, ly2 = cy - fMinY;

    // local edge equations
    const f32 A0 = ly0 - ly1, B0 = lx1 - lx0, C0 = lx0 * ly1 - ly0 * lx1;
    const f32 A1 = ly1 - ly2, B1 = lx2 - lx1, C1 = lx1 * ly2 - ly1 * lx2;
    const f32 A2 = ly2 - ly0, B2 = lx0 - lx2, C2 = lx2 * ly0 - ly2 * lx0;

    // localize zBase
    const f32 dzdx = ((v1->invZ - v0->invZ) * (cy - ay) - (v2->invZ - v0->invZ) * (by - ay)) * invDet;
    const f32 dzdy = ((v2->invZ - v0->invZ) * (bx - ax) - (v1->invZ - v0->invZ) * (cx - ax)) * invDet;
    const f32 zBase = v0->invZ - (lx0 * dzdx + ly0 * dzdy);

#ifdef SFR_MULTITHREADED
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

        // allocate specific page if its missing
        if (!sfrThreadBuf->binPages[pageInd]) {
            SFR__MALLOC(sfrThreadBuf->binPages[pageInd], sizeof(struct sfrTriangleBin) * SFR_BIN_PAGE_SIZE);
        }

        sfr_mutex_unlock(&sfrThreadBuf->binPoolMutex);
    }

    if (!sfrThreadBuf->binPages[pageInd]) {
        return;
    }

    struct sfrTriangleBin* bin = &sfrThreadBuf->binPages[pageInd][pageOffset];
#else
    if (sfrState.globalBinsCount >= sfrState.globalBinsCap) {
        sfrState.globalBinsCap = (i32)(sfrState.globalBinsCap * 1.5f);
        sfrState.globalBins = (struct sfrTriangleBin*)sfrRealloc(sfrState.globalBins,
            sizeof(struct sfrTriangleBin) * (u64)sfrState.globalBinsCap);
    }
    struct sfrTriangleBin* bin = &sfrState.globalBins[sfrState.globalBinsCount];
    sfrState.globalBinsCount += 1;
#endif

    *bin = (struct sfrTriangleBin){
#ifdef SFR_MULTITHREADED
        .binId = globalInd,
#else
        .binId = sfrState.globalBinsCount - 1,
#endif
        .invDet = invDet,
        .dzdx = dzdx, .dzdy = dzdy, .zBase = zBase,
        .A0 = A0, .B0 = B0, .C0 = C0,
        .A1 = A1, .B1 = B1, .C1 = C1,
        .A2 = A2, .B2 = B2, .C2 = C2,
        .minX = minX, .maxX = maxX,
        .minY = minY, .maxY = maxY,

        .v0 = *v0, .v1 = *v1, .v2 = *v2,

        .col = (mat ? (mat->albedoTex ? col : mat->baseColor) : col),
        .mat = mat
    };

#ifdef SFR_MULTITHREADED
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

            if (tile->bins[tInd]) {
                tile->bins[tInd][tile->binCount[tInd]++] = bin;
            }
        }
    }
#else
    sfrRasterCount += 1;
    struct sfrTile fullTile = {
        .minX = 0, .minY = 0,
        .maxX = sfrWidth, .maxY = sfrHeight,
        .minInvZ = 0.f
    };
    sfr__rasterize_bin(bin, &fullTile);
#endif
}

static void sfr__process_and_bin_triangle(
    const sfrmat* matMVP, const sfrmat* matModel, const sfrmat* matNormal,
    const f32* posA, const f32* uvA, const f32* normA, const f32* tanA,
    const f32* posB, const f32* uvB, const f32* normB, const f32* tanB,
    const f32* posC, const f32* uvC, const f32* normC, const f32* tanC,
    u32 col, const SfrMaterial* mat
) {
    const sfrvec ap = {posA[0], posA[1], posA[2], 1.f};
    const sfrvec bp = {posB[0], posB[1], posB[2], 1.f};
    const sfrvec cp = {posC[0], posC[1], posC[2], 1.f};

    const sfrvec aClip = sfr_mat_mul_vec(*matMVP, ap);
    const sfrvec bClip = sfr_mat_mul_vec(*matMVP, bp);
    const sfrvec cClip = sfr_mat_mul_vec(*matMVP, cp);

    // calculate world positions
    const sfrvec aWorld = sfr_mat_mul_vec(*matModel, ap);
    const sfrvec bWorld = sfr_mat_mul_vec(*matModel, bp);
    const sfrvec cWorld = sfr_mat_mul_vec(*matModel, cp);

    // transform normals
    const sfrvec na = sfr_mat_mul_vec(*matNormal, (sfrvec){normA[0], normA[1], normA[2], 0.f});
    const sfrvec nb = sfr_mat_mul_vec(*matNormal, (sfrvec){normB[0], normB[1], normB[2], 0.f});
    const sfrvec nc = sfr_mat_mul_vec(*matNormal, (sfrvec){normC[0], normC[1], normC[2], 0.f});

    // transform tangents and preserve w
    sfrvec ta = {0}, tb = {0}, tc = {0};
    if (tanA) { ta = sfr_mat_mul_vec(*matNormal, (sfrvec){tanA[0], tanA[1], tanA[2], 0.f}); ta.w = tanA[3]; }
    if (tanB) { tb = sfr_mat_mul_vec(*matNormal, (sfrvec){tanB[0], tanB[1], tanB[2], 0.f}); tb.w = tanB[3]; }
    if (tanC) { tc = sfr_mat_mul_vec(*matNormal, (sfrvec){tanC[0], tanC[1], tanC[2], 0.f}); tc.w = tanC[3]; }

    const f32 uA = uvA ? uvA[0] : 0.f, vA = uvA ? uvA[1] : 0.f;
    const f32 uB = uvB ? uvB[0] : 0.f, vB = uvB ? uvB[1] : 0.f;
    const f32 uC = uvC ? uvC[0] : 0.f, vC = uvC ? uvC[1] : 0.f;

    const SfrMaterial* matToUse = mat ? mat : &sfrState.baseMat;

    if (aClip.w > SFR_EPSILON && bClip.w > SFR_EPSILON && cClip.w > SFR_EPSILON) {
        const f32 aInvW = 1.f / aClip.w, bInvW = 1.f / bClip.w, cInvW = 1.f / cClip.w;

        const f32 sax =  (aClip.x * aInvW + 1.f) * sfrState.halfWidth;
        const f32 say = (-aClip.y * aInvW + 1.f) * sfrState.halfHeight;
        const f32 sbx =  (bClip.x * bInvW + 1.f) * sfrState.halfWidth;
        const f32 sby = (-bClip.y * bInvW + 1.f) * sfrState.halfHeight;
        const f32 scx =  (cClip.x * cInvW + 1.f) * sfrState.halfWidth;
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

        const struct sfrVertexData v0 = {
            aInvW, uA * aInvW, vA * aInvW,
            na.x * aInvW, na.y * aInvW, na.z * aInvW,
            ta.x * aInvW, ta.y * aInvW, ta.z * aInvW, ta.w * aInvW,
            aWorld.x * aInvW, aWorld.y * aInvW, aWorld.z * aInvW
        };
        const struct sfrVertexData v1 = {
            bInvW, uB * bInvW, vB * bInvW,
            nb.x * bInvW, nb.y * bInvW, nb.z * bInvW,
            tb.x * bInvW, tb.y * bInvW, tb.z * bInvW, tb.w * bInvW,
            bWorld.x * bInvW, bWorld.y * bInvW, bWorld.z * bInvW
        };
        const struct sfrVertexData v2 = {
            cInvW, uC * cInvW, vC * cInvW,
            nc.x * cInvW, nc.y * cInvW, nc.z * cInvW,
            tc.x * cInvW, tc.y * cInvW, tc.z * cInvW, tc.w * cInvW,
            cWorld.x * cInvW, cWorld.y * cInvW, cWorld.z * cInvW
        };

        sfr__bin_triangle(sax, say, sbx, sby, scx, scy, &v0, &v1, &v2, col, matToUse);
        return;
    }

    // slow path, intersects near plane and needs clipping
    struct sfrTexVert clipTris[16][3];
    struct sfrTexVert (*input)[3] = clipTris;
    i32 inputCount = 1;

    clipTris[0][0].u = uA;
    clipTris[0][0].v = vA;
    clipTris[0][0].normal = na;
    clipTris[0][0].tangent = ta;
    clipTris[0][0].worldPos = aWorld;
    clipTris[0][0].viewZ = aClip.w;
    clipTris[0][0].pos = aClip;

    clipTris[0][1].u = uB;
    clipTris[0][1].v = vB;
    clipTris[0][1].normal = nb;
    clipTris[0][1].tangent = tb;
    clipTris[0][1].worldPos = bWorld;
    clipTris[0][1].viewZ = bClip.w;
    clipTris[0][1].pos = bClip;

    clipTris[0][2].u = uC;
    clipTris[0][2].v = vC;
    clipTris[0][2].normal = nc;
    clipTris[0][2].tangent = tc;
    clipTris[0][2].worldPos = cWorld;
    clipTris[0][2].viewZ = cClip.w;
    clipTris[0][2].pos = cClip;

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
            screen[j].viewZ = tri[j].viewZ;
            screen[j].normal = tri[j].normal;
            screen[j].tangent = tri[j].tangent;
            screen[j].worldPos = tri[j].worldPos;
        }

        if (skip) {
            continue;
        }

        const f32 aInvZ = 1.f / screen[0].viewZ, bInvZ = 1.f / screen[1].viewZ, cInvZ = 1.f / screen[2].viewZ;

        const struct sfrVertexData v0 = {
            .invZ = aInvZ, .u = screen[0].u * aInvZ, .v = screen[0].v * aInvZ,
            .nx = screen[0].normal.x * aInvZ, .ny = screen[0].normal.y * aInvZ, .nz = screen[0].normal.z * aInvZ,
            .tx = screen[0].tangent.x * aInvZ, .ty = screen[0].tangent.y * aInvZ, .tz = screen[0].tangent.z * aInvZ, .tw = screen[0].tangent.w * aInvZ,
            .wx = screen[0].worldPos.x * aInvZ, .wy = screen[0].worldPos.y * aInvZ, .wz = screen[0].worldPos.z * aInvZ
        };
        const struct sfrVertexData v1 = {
            .invZ = bInvZ, .u = screen[1].u * bInvZ, .v = screen[1].v * bInvZ,
            .nx = screen[1].normal.x * bInvZ, .ny = screen[1].normal.y * bInvZ, .nz = screen[1].normal.z * bInvZ,
            .tx = screen[1].tangent.x * bInvZ, .ty = screen[1].tangent.y * bInvZ, .tz = screen[1].tangent.z * bInvZ, .tw = screen[1].tangent.w * bInvZ,
            .wx = screen[1].worldPos.x * bInvZ, .wy = screen[1].worldPos.y * bInvZ, .wz = screen[1].worldPos.z * bInvZ
        };
        const struct sfrVertexData v2 = {
            .invZ = cInvZ, .u = screen[2].u * cInvZ, .v = screen[2].v * cInvZ,
            .nx = screen[2].normal.x * cInvZ, .ny = screen[2].normal.y * cInvZ, .nz = screen[2].normal.z * cInvZ,
            .tx = screen[2].tangent.x * cInvZ, .ty = screen[2].tangent.y * cInvZ, .tz = screen[2].tangent.z * cInvZ, .tw = screen[2].tangent.w * cInvZ,
            .wx = screen[2].worldPos.x * cInvZ, .wy = screen[2].worldPos.y * cInvZ, .wz = screen[2].worldPos.z * cInvZ
        };

        sfr__bin_triangle(
            screen[0].pos.x, screen[0].pos.y,
            screen[1].pos.x, screen[1].pos.y,
            screen[2].pos.x, screen[2].pos.y,
            &v0, &v1, &v2, col, matToUse
        );
    }
}

#ifndef SFR_NO_SIMD

static __m256 sfr__mm256_log2_ps(__m256 x) {
    const __m256i i = _mm256_castps_si256(x);
    const __m256 e = _mm256_cvtepi32_ps(_mm256_sub_epi32(i, _mm256_set1_epi32(0x3F800000)));
    return _mm256_mul_ps(e, _mm256_set1_ps(1.1920928955078125e-7f));
}

static __m256 sfr__mm256_exp2_ps(__m256 x) {
    // clamp x to avoid underflowing the IEEE-754 exponent into negative numbers/NaNs
    x = _mm256_max_ps(x, _mm256_set1_ps(-126.f));

    const __m256i i = _mm256_cvtps_epi32(_mm256_mul_ps(x, _mm256_set1_ps(8388608.f)));
    return _mm256_castsi256_ps(_mm256_add_epi32(i, _mm256_set1_epi32(0x3F800000)));
}

// fast pow(x, y)
static __m256 sfr__mm256_pow_ps(__m256 x, __m256 y) {
    // clamp x to avoid taking the log of zero or negative numbers (NaN/inf)
    const __m256 cx = _mm256_max_ps(x, _mm256_set1_ps(0.000001f));
    return sfr__mm256_exp2_ps(_mm256_mul_ps(y, sfr__mm256_log2_ps(cx)));
}

static void sfr__resolve_chunk(u32 firstId, i32 globalX, i32 globalY, i32 globalInd, __m256i writeMask) {
#ifdef SFR_MULTITHREADED
    const struct sfrTriangleBin* bin = &sfrThreadBuf->binPages[firstId / SFR_BIN_PAGE_SIZE][firstId % SFR_BIN_PAGE_SIZE];
#else
    const struct sfrTriangleBin* bin = &sfrState.globalBins[firstId];
#endif

    const SfrMaterial* mat = bin->mat;

    // vv scalar chunk LOD vv
    const f32 ccx = (f32)(globalX - bin->minX) + 3.5f;
    const f32 ccy = (f32)(globalY - bin->minY) + 0.5f;

    const f32 cw0 = (bin->C1 + bin->A1 * ccx + bin->B1 * ccy) * bin->invDet;
    const f32 cw1 = (bin->C2 + bin->A2 * ccx + bin->B2 * ccy) * bin->invDet;
    const f32 cw2 = (bin->C0 + bin->A0 * ccx + bin->B0 * ccy) * bin->invDet;

    const f32 cw0x = cw0 + bin->A1 * bin->invDet, cw1x = cw1 + bin->A2 * bin->invDet, cw2x = cw2 + bin->A0 * bin->invDet;
    const f32 cw0y = cw0 + bin->B1 * bin->invDet, cw1y = cw1 + bin->B2 * bin->invDet, cw2y = cw2 + bin->B0 * bin->invDet;

    const f32 cinvZ = cw0 * bin->v0.invZ + cw1 * bin->v1.invZ + cw2 * bin->v2.invZ;
    const f32 cz = 1.f / cinvZ;
    const f32 ccu = (cw0 * bin->v0.u + cw1 * bin->v1.u + cw2 * bin->v2.u) * cz;
    const f32 ccv = (cw0 * bin->v0.v + cw1 * bin->v1.v + cw2 * bin->v2.v) * cz;

    const f32 cinvZX = cw0x * bin->v0.invZ + cw1x * bin->v1.invZ + cw2x * bin->v2.invZ;
    const f32 cuX = (cw0x * bin->v0.u + cw1x * bin->v1.u + cw2x * bin->v2.u) / cinvZX;
    const f32 cvX = (cw0x * bin->v0.v + cw1x * bin->v1.v + cw2x * bin->v2.v) / cinvZX;

    const f32 cinvZY = cw0y * bin->v0.invZ + cw1y * bin->v1.invZ + cw2y * bin->v2.invZ;
    const f32 cuY = (cw0y * bin->v0.u + cw1y * bin->v1.u + cw2y * bin->v2.u) / cinvZY;
    const f32 cvY = (cw0y * bin->v0.v + cw1y * bin->v1.v + cw2y * bin->v2.v) / cinvZY;

    const f32 dudx = cuX - ccu, dvdx = cvX - ccv;
    const f32 dudy = cuY - ccu, dvdy = cvY - ccv;

    i32 aLod = 0;

    if (mat->albedoTex) {
        const f32 ux = dudx * (f32)mat->albedoTex->w, vx = dvdx * (f32)mat->albedoTex->h;
        const f32 uy = dudy * (f32)mat->albedoTex->w, vy = dvdy * (f32)mat->albedoTex->h;
        const f32 rhoMax = sfr_fmaxf(sfr_fmaxf(sfr_fabsf(ux), sfr_fabsf(vx)), sfr_fmaxf(sfr_fabsf(uy), sfr_fabsf(vy)));
        if (rhoMax > 1.f) {
            aLod = sfr__fast_log2(rhoMax);
        }
        aLod = SFR_CLAMP(aLod, 0, mat->albedoTex->mipLevels - 1);
    }

    // vv barycentrics and depth vv
    const __m256 vx = _mm256_add_ps(_mm256_set1_ps((f32)(globalX - bin->minX) + 0.5f), _mm256_setr_ps(0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f));
    const __m256 vy = _mm256_set1_ps((f32)(globalY - bin->minY) + 0.5f);
    const __m256 vInvDet = _mm256_set1_ps(bin->invDet);

    // constants
    const __m256 A0 = _mm256_set1_ps(bin->A0);
    const __m256 B0 = _mm256_set1_ps(bin->B0);
    const __m256 C0 = _mm256_set1_ps(bin->C0);

    const __m256 A1 = _mm256_set1_ps(bin->A1);
    const __m256 B1 = _mm256_set1_ps(bin->B1);
    const __m256 C1 = _mm256_set1_ps(bin->C1);

    const __m256 A2 = _mm256_set1_ps(bin->A2);
    const __m256 B2 = _mm256_set1_ps(bin->B2);
    const __m256 C2 = _mm256_set1_ps(bin->C2);

    // w0 = (A1*vx + B1*vy + C1) * invDet
    const __m256 vw0 = _mm256_mul_ps(
        _mm256_fmadd_ps(A1, vx, _mm256_fmadd_ps(B1, vy, C1)),
        vInvDet);

    // w1 = (A2*vx + B2*vy + C2) * invDet
    const __m256 vw1 = _mm256_mul_ps(
        _mm256_fmadd_ps(A2, vx, _mm256_fmadd_ps(B2, vy, C2)),
        vInvDet);

    // w2 = (A0*vx + B0*vy + C0) * invDet
    const __m256 vw2 = _mm256_mul_ps(
        _mm256_fmadd_ps(A0, vx, _mm256_fmadd_ps(B0, vy, C0)),
        vInvDet);

    // invZ = w0*z0 + w1*z1 + w2*z2
    const __m256 vZ = _mm256_rcp_ps(_mm256_fmadd_ps(
        vw0, _mm256_set1_ps(bin->v0.invZ),
        _mm256_fmadd_ps(
            vw1, _mm256_set1_ps(bin->v1.invZ),
            _mm256_mul_ps(vw2, _mm256_set1_ps(bin->v2.invZ)))));

    // ((w0*a0 + w1*a1 + w2*a2) * vZ)
    #define INTERP(attr) \
        _mm256_mul_ps( \
            _mm256_fmadd_ps(vw0, _mm256_set1_ps(bin->v0.attr), \
                _mm256_fmadd_ps(vw1, _mm256_set1_ps(bin->v1.attr), \
                    _mm256_mul_ps(vw2, _mm256_set1_ps(bin->v2.attr)) \
                ) \
            ), \
            vZ \
        )

    // attribute interpolation 1
    const __m256 vu = INTERP(u);
    const __m256 vv = INTERP(v);

    // vv base material properties vv
    __m256 vAlbedoR = _mm256_set1_ps((f32)((bin->col >> 16) & 0xFF) / 255.f);
    __m256 vAlbedoG = _mm256_set1_ps((f32)((bin->col >> 8)  & 0xFF) / 255.f);
    __m256 vAlbedoB = _mm256_set1_ps((f32)((bin->col >> 0)  & 0xFF) / 255.f);

    __m256 vMetallic = _mm256_set1_ps(mat->metallicFactor);
    __m256 vRoughness = _mm256_max_ps(_mm256_set1_ps(0.001f), _mm256_set1_ps(mat->roughnessFactor));

    // vv albedo & metallic / roughness mapping vv
    if (mat->albedoTex) {
        const SfrTexture* tex = mat->albedoTex;
        const u32* const mipPixels = (0 == aLod) ? tex->pixels : tex->mipPixels[aLod - 1];
        const i32 currW = (0 == aLod) ? tex->w : tex->mipW[aLod - 1];
        const i32 currH = (0 == aLod) ? tex->h : tex->mipH[aLod - 1];
        const u8 isPot = (0 == (tex->w & (tex->w - 1))) && (0 == (tex->h & (tex->h - 1)));

        __m256i vatx, vaty;
        __m256 vau = _mm256_mul_ps(vu, _mm256_set1_ps((f32)currW));
        __m256 vav = _mm256_mul_ps(vv, _mm256_set1_ps((f32)currH));

        if (isPot) {
            vatx = _mm256_and_si256(_mm256_cvttps_epi32(vau), _mm256_set1_epi32(currW - 1));
            vaty = _mm256_and_si256(_mm256_cvttps_epi32(vav), _mm256_set1_epi32(currH - 1));
        } else {
            vau = _mm256_mul_ps(_mm256_sub_ps(vu, _mm256_floor_ps(vu)), _mm256_set1_ps((f32)currW));
            vav = _mm256_mul_ps(_mm256_sub_ps(vv, _mm256_floor_ps(vv)), _mm256_set1_ps((f32)currH));
            vatx = _mm256_min_epi32(_mm256_cvttps_epi32(vau), _mm256_set1_epi32(currW - 1));
            vaty = _mm256_min_epi32(_mm256_cvttps_epi32(vav), _mm256_set1_epi32(currH - 1));
        }

        __m256i vTexInds = _mm256_add_epi32(_mm256_mullo_epi32(vaty, _mm256_set1_epi32(currW)), vatx);
        // only fetch active pixels
        __m256i vCol = _mm256_mask_i32gather_epi32(_mm256_setzero_si256(), (const int*)mipPixels, vTexInds, writeMask, 4);

        const __m256 vInv255 = _mm256_set1_ps(1.f / 255.f);
        const __m256i v0xFF  = _mm256_set1_epi32(0xFF);
        vAlbedoR = _mm256_mul_ps(vAlbedoR, _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(vCol, 16), v0xFF)), vInv255));
        vAlbedoG = _mm256_mul_ps(vAlbedoG, _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(vCol, 8), v0xFF)), vInv255));
        vAlbedoB = _mm256_mul_ps(vAlbedoB, _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(vCol, v0xFF)), vInv255));

        if (mat->metallicRoughnessTex) {
            const SfrTexture* mrTex = mat->metallicRoughnessTex;
            const u32* const mrMipPixels = (0 == aLod) ? mrTex->pixels : mrTex->mipPixels[aLod - 1];
            const __m256i vMrCol = _mm256_mask_i32gather_epi32(_mm256_setzero_si256(), (const int*)mrMipPixels, vTexInds, writeMask, 4);

            __m256 vTexRoughness = _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(_mm256_srli_epi32(vMrCol, 8), v0xFF)), vInv255);
            vRoughness = _mm256_max_ps(_mm256_set1_ps(0.001f), vTexRoughness);

            vMetallic = _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_and_si256(vMrCol, v0xFF)), vInv255);
        }
    }

    // vv diffuse, F0 derivations vv
    const __m256 vOne = _mm256_set1_ps(1.f);
    const __m256 vInvMetallic = _mm256_sub_ps(vOne, vMetallic);

    const __m256 vDiffR = _mm256_mul_ps(vAlbedoR, vInvMetallic);
    const __m256 vDiffG = _mm256_mul_ps(vAlbedoG, vInvMetallic);
    const __m256 vDiffB = _mm256_mul_ps(vAlbedoB, vInvMetallic);

    const __m256 vF0Base = _mm256_mul_ps(_mm256_set1_ps(0.04f), vInvMetallic);
    const __m256 vF0R = _mm256_fmadd_ps(vAlbedoR, vMetallic, vF0Base);
    const __m256 vF0G = _mm256_fmadd_ps(vAlbedoG, vMetallic, vF0Base);
    const __m256 vF0B = _mm256_fmadd_ps(vAlbedoB, vMetallic, vF0Base);

    // vv convert PBR roughness to blinn-phong shininess vv
    const __m256 vAlpha = _mm256_mul_ps(vRoughness, vRoughness);
    const __m256 vAlphaSq = _mm256_mul_ps(vAlpha, vAlpha);
    __m256 vShininess = _mm256_sub_ps(_mm256_div_ps(_mm256_set1_ps(2.f), vAlphaSq), _mm256_set1_ps(2.f));
    vShininess = _mm256_max_ps(_mm256_set1_ps(1.f), _mm256_min_ps(vShininess, _mm256_set1_ps(4096.f)));

    const __m256 vEnergyCons = _mm256_mul_ps(_mm256_add_ps(_mm256_set1_ps(8.f), vShininess), _mm256_set1_ps(1.f / (8.f * SFR_PI)));

    // vv lighting vv
    __m256 vFinalR = _mm256_setzero_ps();
    __m256 vFinalG = _mm256_setzero_ps();
    __m256 vFinalB = _mm256_setzero_ps();

    // attribute interpolation 2
    const __m256 vFragX = INTERP(wx);
    const __m256 vFragY = INTERP(wy);
    const __m256 vFragZ = INTERP(wz);
    __m256 vnx = INTERP(nx);
    __m256 vny = INTERP(ny);
    __m256 vnz = INTERP(nz);

    // vv fast vector normalization vv
    __m256 vnDot = _mm256_fmadd_ps(vnx, vnx, _mm256_fmadd_ps(vny, vny, _mm256_mul_ps(vnz, vnz)));
    __m256 vnRsqrt = _mm256_rsqrt_ps(vnDot);
    vnx = _mm256_mul_ps(vnx, vnRsqrt);
    vny = _mm256_mul_ps(vny, vnRsqrt);
    vnz = _mm256_mul_ps(vnz, vnRsqrt);

    if (sfrState.lightingEnabled && sfrState.lightCount > 0) {
        __m256 vViewX = _mm256_sub_ps(_mm256_set1_ps(sfrCamPos.x), vFragX);
        __m256 vViewY = _mm256_sub_ps(_mm256_set1_ps(sfrCamPos.y), vFragY);
        __m256 vViewZ = _mm256_sub_ps(_mm256_set1_ps(sfrCamPos.z), vFragZ);
        __m256 vViewDot = _mm256_fmadd_ps(vViewX, vViewX, _mm256_fmadd_ps(vViewY, vViewY, _mm256_mul_ps(vViewZ, vViewZ)));
        const __m256 vViewRsqrt = _mm256_rsqrt_ps(vViewDot);
        vViewX = _mm256_mul_ps(vViewX, vViewRsqrt);
        vViewY = _mm256_mul_ps(vViewY, vViewRsqrt);
        vViewZ = _mm256_mul_ps(vViewZ, vViewRsqrt);

        for (i32 i = 0; i < sfrState.lightCount; i += 1) {
            const SfrLight* const light = &sfrState.lights[i];

            __m256 vLDirX, vLDirY, vLDirZ;
            __m256 vAtten = vOne;

            if (SFR_LIGHT_DIRECTIONAL == light->type) {
                vLDirX = _mm256_set1_ps(-light->x);
                vLDirY = _mm256_set1_ps(-light->y);
                vLDirZ = _mm256_set1_ps(-light->z);
            } else {
                const __m256 vDiffX = _mm256_sub_ps(_mm256_set1_ps(light->x), vFragX);
                const __m256 vDiffY = _mm256_sub_ps(_mm256_set1_ps(light->y), vFragY);
                const __m256 vDiffZ = _mm256_sub_ps(_mm256_set1_ps(light->z), vFragZ);

                const __m256 vDistSq = _mm256_add_ps(_mm256_mul_ps(vDiffX, vDiffX), _mm256_add_ps(_mm256_mul_ps(vDiffY, vDiffY), _mm256_mul_ps(vDiffZ, vDiffZ)));
                const __m256 vDistRsqrt = _mm256_rsqrt_ps(vDistSq);
                const __m256 vDist = _mm256_mul_ps(vDistSq, vDistRsqrt);

                vLDirX = _mm256_mul_ps(vDiffX, vDistRsqrt);
                vLDirY = _mm256_mul_ps(vDiffY, vDistRsqrt);
                vLDirZ = _mm256_mul_ps(vDiffZ, vDistRsqrt);

                const __m256 vAttenCore = _mm256_max_ps(_mm256_setzero_ps(),
                    _mm256_sub_ps(vOne, _mm256_mul_ps(vDist, _mm256_set1_ps(1.f / light->attenuation))));
                vAtten = _mm256_mul_ps(vAttenCore, vAttenCore);
            }

            const __m256 vnDotL = _mm256_add_ps(_mm256_mul_ps(vnx, vLDirX), _mm256_add_ps(_mm256_mul_ps(vny, vLDirY), _mm256_mul_ps(vnz, vLDirZ)));
            const __m256 vDiff = _mm256_max_ps(_mm256_setzero_ps(), vnDotL);

            __m256 vhx = _mm256_add_ps(vLDirX, vViewX);
            __m256 vhy = _mm256_add_ps(vLDirY, vViewY);
            __m256 vhz = _mm256_add_ps(vLDirZ, vViewZ);
            const __m256 vhDot = _mm256_add_ps(_mm256_mul_ps(vhx, vhx), _mm256_add_ps(_mm256_mul_ps(vhy, vhy), _mm256_mul_ps(vhz, vhz)));
            const __m256 vhRsqrt = _mm256_rsqrt_ps(vhDot);
            vhx = _mm256_mul_ps(vhx, vhRsqrt);
            vhy = _mm256_mul_ps(vhy, vhRsqrt);
            vhz = _mm256_mul_ps(vhz, vhRsqrt);

            const __m256 vnDotH = _mm256_max_ps(_mm256_setzero_ps(),
                _mm256_add_ps(_mm256_mul_ps(vnx, vhx), _mm256_add_ps(_mm256_mul_ps(vny, vhy), _mm256_mul_ps(vnz, vhz))));

            __m256 vSpec = sfr__mm256_pow_ps(vnDotH, vShininess);

            // apply energy conservation and mask zero diffuse
            vSpec = _mm256_mul_ps(vSpec, vEnergyCons);
            vSpec = _mm256_blendv_ps(_mm256_setzero_ps(), vSpec, _mm256_cmp_ps(vDiff, _mm256_setzero_ps(), _CMP_GT_OQ));

            const __m256 vIntensity = _mm256_mul_ps(_mm256_set1_ps(light->intensity), vAtten);
            const __m256 vDiffuseIntensity = _mm256_mul_ps(vDiff, vIntensity);
            const __m256 vSpecularIntensity = _mm256_mul_ps(vSpec, vDiffuseIntensity);

            const __m256 vLightR = _mm256_set1_ps(light->r);
            const __m256 vLightG = _mm256_set1_ps(light->g);
            const __m256 vLightB = _mm256_set1_ps(light->b);
            const __m256 vAmbient = _mm256_set1_ps(light->ambient);

            const __m256 vDiffTerm = _mm256_add_ps(vAmbient, vDiffuseIntensity);
            const __m256 vSpecTerm = _mm256_add_ps(vAmbient, vSpecularIntensity);

            const __m256 vLightDiffR = _mm256_mul_ps(vLightR, vDiffR);
            const __m256 vLightSpecR = _mm256_mul_ps(vLightR, vF0R);
            vFinalR = _mm256_fmadd_ps(vDiffTerm, vLightDiffR, vFinalR);
            vFinalR = _mm256_fmadd_ps(vSpecTerm, vLightSpecR, vFinalR);

            const __m256 vLightDiffG = _mm256_mul_ps(vLightG, vDiffG);
            const __m256 vLightSpecG = _mm256_mul_ps(vLightG, vF0G);
            vFinalG = _mm256_fmadd_ps(vDiffTerm, vLightDiffG, vFinalG);
            vFinalG = _mm256_fmadd_ps(vSpecTerm, vLightSpecG, vFinalG);

            const __m256 vLightDiffB = _mm256_mul_ps(vLightB, vDiffB);
            const __m256 vLightSpecB = _mm256_mul_ps(vLightB, vF0B);
            vFinalB = _mm256_fmadd_ps(vDiffTerm, vLightDiffB, vFinalB);
            vFinalB = _mm256_fmadd_ps(vSpecTerm, vLightSpecB, vFinalB);
        }
    } else {
        vFinalR = vAlbedoR;
        vFinalG = vAlbedoG;
        vFinalB = vAlbedoB;
    }

    // vv render modes vv
    __m256i vFinalPixel;
    if (SFR_RENDERMODE_SHADED == sfrState.renderMode) {
        vFinalR = _mm256_sqrt_ps(vFinalR);
        vFinalG = _mm256_sqrt_ps(vFinalG);
        vFinalB = _mm256_sqrt_ps(vFinalB);
    } else if (SFR_RENDERMODE_NORMALS == sfrState.renderMode) {
        const __m256 vHalf = _mm256_set1_ps(0.5f);
        vFinalR = _mm256_max_ps(_mm256_setzero_ps(), _mm256_min_ps(_mm256_add_ps(_mm256_mul_ps(vnx, vHalf), vHalf), vOne));
        vFinalG = _mm256_max_ps(_mm256_setzero_ps(), _mm256_min_ps(_mm256_add_ps(_mm256_mul_ps(vny, vHalf), vHalf), vOne));
        vFinalB = _mm256_max_ps(_mm256_setzero_ps(), _mm256_min_ps(_mm256_add_ps(_mm256_mul_ps(vnz, vHalf), vHalf), vOne));
    } else if (SFR_RENDERMODE_FRAGPOS == sfrState.renderMode) {
        vFinalR = _mm256_sub_ps(vFragX, _mm256_floor_ps(vFragX));
        vFinalG = _mm256_sub_ps(vFragY, _mm256_floor_ps(vFragY));
        vFinalB = _mm256_sub_ps(vFragZ, _mm256_floor_ps(vFragZ));
    }

    const __m256 v255 = _mm256_set1_ps(255.f);
    const __m256 vfr = _mm256_min_ps(_mm256_mul_ps(vFinalR, v255), v255);
    const __m256 vfg = _mm256_min_ps(_mm256_mul_ps(vFinalG, v255), v255);
    const __m256 vfb = _mm256_min_ps(_mm256_mul_ps(vFinalB, v255), v255);

    vFinalPixel = _mm256_or_si256(_mm256_set1_epi32(0xFF000000),
        _mm256_or_si256(_mm256_slli_epi32(_mm256_cvttps_epi32(vfr), 16),
        _mm256_or_si256(_mm256_slli_epi32(_mm256_cvttps_epi32(vfg), 8), _mm256_cvttps_epi32(vfb))));

    _mm256_maskstore_epi32((int*)&sfrPixelBuf[globalInd], writeMask, vFinalPixel);

    #undef INTERP
}

#endif // !SFR_NO_SIMD

static void sfr__resolve_pixel(u32 id, i32 globalX, i32 globalY, i32 globalInd) {
#ifdef SFR_MULTITHREADED
    const struct sfrTriangleBin* bin = &sfrThreadBuf->binPages[id / SFR_BIN_PAGE_SIZE][id % SFR_BIN_PAGE_SIZE];
#else
    const struct sfrTriangleBin* bin = &sfrState.globalBins[id];
#endif

    const SfrMaterial* const mat = bin->mat;

    // pixel centers
    const f32 cx = (f32)(globalX - bin->minX) + 0.5f;
    const f32 cy = (f32)(globalY - bin->minY) + 0.5f;

    // base barycentric weights
    const f32 w0 = (bin->C1 + bin->A1 * cx + bin->B1 * cy) * bin->invDet;
    const f32 w1 = (bin->C2 + bin->A2 * cx + bin->B2 * cy) * bin->invDet;
    const f32 w2 = (bin->C0 + bin->A0 * cx + bin->B0 * cy) * bin->invDet;

    // offset barycentric weights for screen space derivatives
    const f32 w0x = w0 + bin->A1 * bin->invDet, w1x = w1 + bin->A2 * bin->invDet, w2x = w2 + bin->A0 * bin->invDet;
    const f32 w0y = w0 + bin->B1 * bin->invDet, w1y = w1 + bin->B2 * bin->invDet, w2y = w2 + bin->B0 * bin->invDet;

    // perspective correction (base)
    const f32 invZ = w0 * bin->v0.invZ + w1 * bin->v1.invZ + w2 * bin->v2.invZ;
    const f32 z = 1.f / invZ;
    const f32 cu = (w0 * bin->v0.u + w1 * bin->v1.u + w2 * bin->v2.u) * z;
    const f32 cv = (w0 * bin->v0.v + w1 * bin->v1.v + w2 * bin->v2.v) * z;

    // perspective correction (X + 1)
    const f32 invZX = w0x * bin->v0.invZ + w1x * bin->v1.invZ + w2x * bin->v2.invZ;
    const f32 zX = 1.f / invZX;
    const f32 cuX = (w0x * bin->v0.u + w1x * bin->v1.u + w2x * bin->v2.u) * zX;
    const f32 cvX = (w0x * bin->v0.v + w1x * bin->v1.v + w2x * bin->v2.v) * zX;

    // perspective correction (Y + 1)
    const f32 invZY = w0y * bin->v0.invZ + w1y * bin->v1.invZ + w2y * bin->v2.invZ;
    const f32 zY = 1.f / invZY;
    const f32 cuY = (w0y * bin->v0.u + w1y * bin->v1.u + w2y * bin->v2.u) * zY;
    const f32 cvY = (w0y * bin->v0.v + w1y * bin->v1.v + w2y * bin->v2.v) * zY;

    // normalized uv gradients
    const f32 dudx = cuX - cu, dvdx = cvX - cv;
    const f32 dudy = cuY - cu, dvdy = cvY - cv;

    // interpolate normal
    sfrvec normal = sfr_vec_normf(
        (w0 * bin->v0.nx + w1 * bin->v1.nx + w2 * bin->v2.nx) * z,
        (w0 * bin->v0.ny + w1 * bin->v1.ny + w2 * bin->v2.ny) * z,
        (w0 * bin->v0.nz + w1 * bin->v1.nz + w2 * bin->v2.nz) * z
    );

    // interpolate world pos
    const sfrvec fragPos = {
        (w0 * bin->v0.wx + w1 * bin->v1.wx + w2 * bin->v2.wx) * z,
        (w0 * bin->v0.wy + w1 * bin->v1.wy + w2 * bin->v2.wy) * z,
        (w0 * bin->v0.wz + w1 * bin->v1.wz + w2 * bin->v2.wz) * z,
        1.f
    };

    // vertex / material base color
    f32 albedoR = (f32)((bin->col >> 16) & 0xFF) / 255.f;
    f32 albedoG = (f32)((bin->col >> 8)  & 0xFF) / 255.f;
    f32 albedoB = (f32)((bin->col >> 0)  & 0xFF) / 255.f;

    f32 metallic = mat->metallicFactor;
    // clamped to avoid division by zero later
    f32 roughness = sfr_fmaxf(0.001f, mat->roughnessFactor);

    // albedo mapping
    if (mat->albedoTex) {
        const SfrTexture* tex = mat->albedoTex;
        const i32 texW = tex->w, texH = tex->h;
        const u8 isPot = (0 == (texW & (texW - 1))) && (0 == (texH & (texH - 1)));

        // LOD
        const f32 aux = dudx * (f32)texW, avx = dvdx * (f32)texH;
        const f32 auy = dudy * (f32)texW, avy = dvdy * (f32)texH;
        const f32 rhoMax = sfr_fmaxf(sfr_fmaxf(sfr_fabsf(aux), sfr_fabsf(avx)), sfr_fmaxf(sfr_fabsf(auy), sfr_fabsf(avy)));

        i32 lod = 0;
        if (rhoMax > 1.f) {
            lod = sfr__fast_log2(rhoMax);
        }
        lod = SFR_CLAMP(lod, 0, tex->mipLevels - 1);

        const u32* const aMipPixels = (0 == lod) ? tex->pixels : tex->mipPixels[lod - 1];
        const i32 currW = (0 == lod) ? tex->w : tex->mipW[lod - 1];
        const i32 currH = (0 == lod) ? tex->h : tex->mipH[lod - 1];

        // sample albedo map
        f32 acu = cu, acv = cv;
        i32 atx, aty;
        if (isPot) {
            acu *= (f32)currW;
            acv *= (f32)currH;
            atx = (i32)acu & (currW - 1);
            aty = (i32)acv & (currH - 1);
        } else {
            acu -= sfr_floorf(acu);
            acv -= sfr_floorf(acv);
            atx = (i32)(acu * currW);
            aty = (i32)(acv * currH);
            if (atx >= currW) atx = currW - 1;
            if (aty >= currH) aty = currH - 1;
        }

        const u32 col = aMipPixels[aty * currW + atx];
        const f32 i255 = 1.f / 255.f;
        albedoR *= (f32)((col >> 16) & 0xFF) * i255;
        albedoG *= (f32)((col >> 8)  & 0xFF) * i255;
        albedoB *= (f32)((col >> 0)  & 0xFF) * i255;

        // assume metallicRoughnessTex and albedoTex have the same dimensions
        if (mat->metallicRoughnessTex) {
            const SfrTexture* mrTex = mat->metallicRoughnessTex;
            const u32* const mrMipPixels = (0 == lod) ? mrTex->pixels : mrTex->mipPixels[lod - 1];

            // g = roughness, b = metallic
            const u32 col = mrMipPixels[aty * currW + atx];
            roughness = sfr_fmaxf(0.001f, (f32)((col >> 8)  & 0xFF) * i255);
            metallic = (f32)((col >> 0)  & 0xFF) * i255;
        }
    }

    // metals have no diffuse light, dielectrics keep their albedo
    const f32 diffR = albedoR * (1.f - metallic);
    const f32 diffG = albedoG * (1.f - metallic);
    const f32 diffB = albedoB * (1.f - metallic);

    // F0 is the base specular reflectance
    // dielectrics use 4%, metals use their base albedo
    const f32 f0R = 0.04f * (1.f - metallic) + albedoR * metallic;
    const f32 f0G = 0.04f * (1.f - metallic) + albedoG * metallic;
    const f32 f0B = 0.04f * (1.f - metallic) + albedoB * metallic;

    // convert PBR roughness to blinn-phong shininess
    const f32 alpha = roughness * roughness;
    f32 shininess = (2.f / (alpha * alpha)) - 2.f;
    shininess = sfr_fmaxf(1.f, sfr_fminf(shininess, 4096.f));

    const f32 energyConservation = (8.f + shininess) * (1.f / (8.f * SFR_PI));

    // blinn-phong
    f32 finalR = 0.f, finalG = 0.f, finalB = 0.f;
    if (sfrState.lightingEnabled && sfrState.lightCount > 0) {
        const sfrvec viewDir = sfr_vec_norm(sfr_vec_sub(sfrCamPos, fragPos));

        for (i32 i = 0; i < sfrState.lightCount; i += 1) {
            const SfrLight* const light = &sfrState.lights[i];

            sfrvec lightDir;
            f32 attenuation = 1.f;

            if (SFR_LIGHT_DIRECTIONAL == light->type) {
                lightDir = sfr_vec_normf(-light->x, -light->y, -light->z);
            } else if (SFR_LIGHT_POINT == light->type) {
                const sfrvec diffPos = { light->x - fragPos.x, light->y - fragPos.y, light->z - fragPos.z, 0.f };
                const f32 dist = sfr_vec_length(diffPos);
                lightDir = sfr_vec_mul(diffPos, 1.f / dist);

                // quadratic falloff
                attenuation = sfr_fmaxf(0.f, 1.f - (dist / light->attenuation));
                attenuation *= attenuation;
            }

            // diffuse term
            const f32 diff = sfr_fmaxf(0.f, sfr_vec_dot(normal, lightDir));

            // specular term
            f32 spec = 0.f;
            if (diff > 0.f) {
                sfrvec halfwayDir = sfr_vec_norm(sfr_vec_add(lightDir, viewDir));
                spec = sfr_powf(sfr_fmaxf(0.f, sfr_vec_dot(normal, halfwayDir)), shininess);
                // so rough materials don't blow out
                spec *= energyConservation;
            }

            const f32 diffuseIntensity = diff * light->intensity * attenuation;
            const f32 specularIntensity = spec * diffuseIntensity;

            finalR += (light->ambient + diffuseIntensity)  * light->r * diffR +
                      (light->ambient + specularIntensity) * light->r * f0R;

            finalG += (light->ambient + diffuseIntensity)  * light->g * diffG +
                      (light->ambient + specularIntensity) * light->g * f0G;

            finalB += (light->ambient + diffuseIntensity)  * light->b * diffB +
                      (light->ambient + specularIntensity) * light->b * f0B;
        }
    } else {
        // unlit
        finalR = albedoR;
        finalG = albedoG;
        finalB = albedoB;
    }

    u8 fr, fg, fb;
    if (SFR_RENDERMODE_SHADED == sfrState.renderMode) {
        finalR = sfr_fmaxf(0.f, sfr_fminf(finalR, 1.f));
        finalG = sfr_fmaxf(0.f, sfr_fminf(finalG, 1.f));
        finalB = sfr_fmaxf(0.f, sfr_fminf(finalB, 1.f));
        fr = sfrGammaLUT[(i32)(finalR * 255.f)];
        fg = sfrGammaLUT[(i32)(finalG * 255.f)];
        fb = sfrGammaLUT[(i32)(finalB * 255.f)];
    } else if (SFR_RENDERMODE_NORMALS == sfrState.renderMode) {
        fr = (u8)(sfr_fmaxf(0.f, sfr_fminf(normal.x * 0.5f + 0.5f, 1.f)) * 255.f);
        fg = (u8)(sfr_fmaxf(0.f, sfr_fminf(normal.y * 0.5f + 0.5f, 1.f)) * 255.f);
        fb = (u8)(sfr_fmaxf(0.f, sfr_fminf(normal.z * 0.5f + 0.5f, 1.f)) * 255.f);
    } else if (SFR_RENDERMODE_FRAGPOS == sfrState.renderMode) {
        const f32 fx = fragPos.x - sfr_floorf(fragPos.x);
        const f32 fy = fragPos.y - sfr_floorf(fragPos.y);
        const f32 fz = fragPos.z - sfr_floorf(fragPos.z);
        fr = (u8)(fx * 255.f);
        fg = (u8)(fy * 255.f);
        fb = (u8)(fz * 255.f);
    }

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
    u32 col, const SfrMaterial* mat
) {
    if (sfrState.normalMatDirty) {
        sfr__update_normal_mat();
    }

    const sfrmat matMVP = sfr_mat_mul(sfr_mat_mul(sfrMatModel, sfrMatView), sfrMatProj);

    sfr__process_and_bin_triangle(
        &matMVP, &sfrMatModel, &sfrState.matNormal,
        (f32[3]){ax, ay, az}, (f32[2]){au, av}, (f32[3]){anx, any, anz}, (void*)0,
        (f32[3]){bx, by, bz}, (f32[2]){bu, bv}, (f32[3]){bnx, bny, bnz}, (void*)0,
        (f32[3]){cx, cy, cz}, (f32[2]){cu, cv}, (f32[3]){cnx, cny, cnz}, (void*)0,
        col, mat
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

        u32* dest;
        SFR__MALLOC(dest, sizeof(u32) * nw * nh);

        const i32 mipInd = tex->mipLevels - 1;
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
                const u32 g = (((c00 >> 8)  & 0xFF) + ((c01 >> 8)  & 0xFF) + ((c10 >> 8)  & 0xFF) + ((c11 >> 8)  & 0xFF)) / 4;
                const u32 b = (((c00 >> 0)  & 0xFF) + ((c01 >> 0)  & 0xFF) + ((c10 >> 0)  & 0xFF) + ((c11 >> 0)  & 0xFF)) / 4;
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
        SFR__ERR_EXIT("malloc, free, and realloc must be provided\n");
    }
    sfrMalloc = mallocFunc, sfrFree = freeFunc, sfrRealloc = reallocFunc;

    if (w > SFR_MAX_WIDTH) {
        SFR__ERR_EXIT("width > SFR_MAX_WIDTH (%d > %d)\n", w, SFR_MAX_WIDTH);
    }
    if (h > SFR_MAX_HEIGHT) {
        SFR__ERR_EXIT("height > SFR_MAX_HEIGHT (%d > %d)\n", w, SFR_MAX_HEIGHT);
    }

    SFR__MALLOC(sfrPixelBuf, sizeof(u32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
    SFR__MALLOC(sfrDepthBuf, sizeof(u32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);

    #ifdef SFR_MULTITHREADED
        SFR__MALLOC(sfrThreadBuf, sizeof(struct sfrThreadBuf));

        sfr_mutex_init(&sfrThreadBuf->binPoolMutex);
        sfr_mutex_init(&sfrThreadBuf->geometryMutex);

        sfrThreadBuf->binPagesCapacity = 64;
        SFR__MALLOC(sfrThreadBuf->binPages, sizeof(struct sfrTriangleBin*) * sfrThreadBuf->binPagesCapacity);
        sfr_memset(sfrThreadBuf->binPages, 0, sizeof(struct sfrTriangleBin*) * sfrThreadBuf->binPagesCapacity);

        // pre allocate first page
        SFR__MALLOC(sfrThreadBuf->binPages[0], sizeof(struct sfrTriangleBin) * SFR_BIN_PAGE_SIZE);
        sfrThreadBuf->meshJobPoolCapacity = 1024 * 8;
        SFR__MALLOC(sfrThreadBuf->meshJobPool, sizeof(struct sfrMeshChunkJob) * sfrThreadBuf->meshJobPoolCapacity);

        sfrThreadBuf->geometryWorkQueueCapacity = 1024 * 8;
        SFR__MALLOC(sfrThreadBuf->geometryWorkQueue, sizeof(i32) * sfrThreadBuf->geometryWorkQueueCapacity);

        if (!sfrThreadBuf->binPages || !sfrThreadBuf->meshJobPool || !sfrThreadBuf->geometryWorkQueue) {
            SFR__ERR_EXIT("failed to allocate dynamic buffers\n");
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
        SFR__MALLOC(sfrState.idBuf, sizeof(i32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
        if (!sfrState.idBuf) {
            SFR__ERR_EXIT("failed to allocate sfrState.idBuf (%ld bytes)\n",
                sizeof(i32) * SFR_MAX_WIDTH * SFR_MAX_HEIGHT);
        }

        sfrState.globalBinsCap = 1024 * 32;
        sfrState.globalBinsCount = 0;
        SFR__MALLOC(sfrState.globalBins, sizeof(struct sfrTriangleBin) * sfrState.globalBinsCap);
        if (!sfrState.globalBins) {
            SFR__ERR_EXIT("failed to allcoate sfrState.globalBins (%ld bytes)\n",
                sizeof(struct sfrTriangleBin) * sfrState.globalBinsCap);
        }
    #endif

    sfr_resize(w, h); // call resize to setup tiling system
    sfr_clear(0xFF000000);
    sfr_reset();
    sfr_set_fov(fovDeg);
    sfr_set_camera(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

    static u32 baseTexPixels[1] = { 0xFFFFFFFF };
    static SfrTexture baseTex = { .w = 1, .h = 1, .pixels = baseTexPixels, .mipLevels = 1 };
    sfrState.baseMat = (SfrMaterial){0};
    sfrState.baseMat.albedoTex = &baseTex;
    sfrState.baseMat.baseColor = 0xFFFFFFFF;
    sfrState.baseMat.roughnessFactor = 0.8f;
    sfrState.baseMat.metallicFactor = 0.3f;

    sfrState.renderMode = SFR_RENDERMODE_SHADED;

    for (i32 i = 0; i < 256; i += 1) {
        const f32 linear = (f32)i / 255.f;
        // 2.2 gamma correction
        const f32 g = sfr_powf(linear, 1.f / 2.2f);
        sfrGammaLUT[i] = (u8)(g * 255.f);
    }

    sfrState.lightCount = 0;
    sfrState.lightCap = 4;
    SFR__MALLOC(sfrState.lights, sizeof(SfrLight) * sfrState.lightCap);
    sfr_memset(sfrState.lights, 0, sizeof(SfrLight) * sfrState.lightCap);
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
        .minInvZ = 0.f
    };

    sfr__resolve_tile(&fullTile);
#endif
}

SFR_FUNC void sfr_resize(i32 width, i32 height) {
    if (width > SFR_MAX_WIDTH || height > SFR_MAX_HEIGHT) {
        SFR__ERR_RET(, "dimensions out of range (%d, %d) > (%d, %d)\n",
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
                tile->minInvZ = 0.f;

                for (i32 t = 0; t <= SFR_THREAD_COUNT; t += 1) {
                    tile->binsCapacity[t] = 512;
                    if (tile->bins[t]) {
                        sfrFree(tile->bins[t]);
                    }
                    SFR__MALLOC(tile->bins[t],
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
        const __m256i vDepth = _mm256_set1_epi32(0);

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
            sfrDepthBuf[i] = 0.f;
            #ifndef SFR_MULTITHREADED
                sfrState.idBuf[i] = -1;
            #endif
        }
    #else
        for (i32 i = count - 1; i >= 0; i -= 1) {
            sfrPixelBuf[i] = clearCol;
            sfrDepthBuf[i] = 0.f;
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
            sfrThreadBuf->tiles[i].minInvZ = 0.f;
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
        const __m256i vDepth = _mm256_set1_epi32(0);

        i32 i = 0;
        for (; i <= count - 8; i += 8) {
            _mm256_storeu_si256((__m256i*)&sfrDepthBuf[i], vDepth);
        }

        for (i32 j = i; j < count; j += 1) {
            sfrDepthBuf[j] = 0.f;
        }
    #else
        for (i32 i = count - 1; i >= 0; i -= 1) {
            sfrDepthBuf[i] = 0.f;
        }
    #endif // !SFR_NO_SIMD && !SFR_NO_STRING

    #ifdef SFR_MULTITHREADED
        for (i32 i = 0; i < sfrThreadBuf->tileCount; i += 1) {
            sfrThreadBuf->tiles[i].minInvZ = 0.f;
        }
    #endif
}

// front, right, back, left, top, bottom
SFR_FUNC void sfr_skybox(const SfrMaterial* faces[6]) {
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
        col, &sfrState.baseMat);
}

SFR_FUNC void sfr_triangle_tex(
    f32 ax, f32 ay, f32 az, f32 au, f32 av,
    f32 bx, f32 by, f32 bz, f32 bu, f32 bv,
    f32 cx, f32 cy, f32 cz, f32 cu, f32 cv,
    u32 col, const SfrMaterial* tex
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

SFR_FUNC void sfr_billboard(u32 col, const SfrMaterial* mat) {
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
        col, mat);
    sfr__triangle_tex_norm(
        a.x, a.y, a.z, 0.f, 1.f, normal.x, normal.y, normal.z,
        d.x, d.y, d.z, 0.f, 0.f, normal.x, normal.y, normal.z,
        c.x, c.y, c.z, 1.f, 0.f, normal.x, normal.y, normal.z,
        col, mat);

    sfrMatModel = savedModel;
    sfrState.matNormal = savedNormal;
}

SFR_FUNC void sfr_cube(u32 col, const SfrMaterial* mat) {
    // front face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1, col, mat);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,0,-1, col, mat);
    // right face
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,1.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0, col, mat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0,
         0.5f,-0.5f, 0.5f, 0.f,0.f, 1,0,0, col, mat);
    // back face
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
         0.5f, 0.5f, 0.5f, 1.f,1.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1, col, mat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1,
        -0.5f,-0.5f, 0.5f, 0.f,0.f, 0,0,1, col, mat);
    // left face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0, col, mat);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0,
        -0.5f,-0.5f,-0.5f, 0.f,0.f, -1,0,0, col, mat);
    // top face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0, col, mat);
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0,
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,1,0, col, mat);
    // bottom face
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0, col, mat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,-1,0, col, mat);
}

SFR_FUNC void sfr_cube_ex(u32 col[12]) {
    // front face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1, col[0], &sfrState.baseMat);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,-1,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,0,-1, col[1], &sfrState.baseMat);
    // right face
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,1.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0, col[2], &sfrState.baseMat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 1.f,0.f, 1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 1,0,0,
         0.5f,-0.5f, 0.5f, 0.f,0.f, 1,0,0, col[3], &sfrState.baseMat);
    // back face
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
         0.5f, 0.5f, 0.5f, 1.f,1.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1, col[4], &sfrState.baseMat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,1,
        -0.5f,-0.5f, 0.5f, 0.f,0.f, 0,0,1, col[5], &sfrState.baseMat);
    // left face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0, col[6], &sfrState.baseMat);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 1.f,0.f, -1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, -1,0,0,
        -0.5f,-0.5f,-0.5f, 0.f,0.f, -1,0,0, col[7], &sfrState.baseMat);
    // top face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0, col[8], &sfrState.baseMat);
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,1,0,
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,1,0, col[9], &sfrState.baseMat);
    // bottom face
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0, col[10], &sfrState.baseMat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,-1,0,
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,-1,0, col[11], &sfrState.baseMat);
}

SFR_FUNC void sfr_cube_inv(u32 col, const SfrMaterial* mat) {
    // front face
    sfr__triangle_tex_norm(
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,1,
        -0.5f, 0.5f,-0.5f, 1.f,1.f, 0,0,1,
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,1, col, mat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,0,1,
         0.5f, 0.5f,-0.5f, 0.f,1.f, 0,0,1,
        -0.5f,-0.5f,-0.5f, 1.f,0.f, 0,0,1, col, mat);
    // right face
    sfr__triangle_tex_norm(
         0.5f, 0.5f, 0.5f, 0.f,1.f, -1,0,0,
         0.5f, 0.5f,-0.5f, 1.f,1.f, -1,0,0,
         0.5f,-0.5f,-0.5f, 1.f,0.f, -1,0,0, col, mat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f, 0.5f, 0.f,0.f, -1,0,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, -1,0,0,
         0.5f,-0.5f,-0.5f, 1.f,0.f, -1,0,0, col, mat);
    // back face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,-1,
         0.5f, 0.5f, 0.5f, 1.f,1.f, 0,0,-1,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,-1, col, mat);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f, 0.5f, 0.f,0.f, 0,0,-1,
        -0.5f, 0.5f, 0.5f, 0.f,1.f, 0,0,-1,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,0,-1, col, mat);
    // left face
    sfr__triangle_tex_norm(
        -0.5f, 0.5f,-0.5f, 0.f,1.f, 1,0,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 1,0,0,
        -0.5f,-0.5f, 0.5f, 1.f,0.f, 1,0,0, col, mat);
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 0.f,0.f, 1,0,0,
        -0.5f, 0.5f,-0.5f, 0.f,1.f, 1,0,0,
        -0.5f,-0.5f, 0.5f, 1.f,0.f, 1,0,0, col, mat);
    // top face
    sfr__triangle_tex_norm(
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,-1,0,
        -0.5f, 0.5f, 0.5f, 1.f,1.f, 0,-1,0,
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,-1,0, col, mat);
    sfr__triangle_tex_norm(
         0.5f, 0.5f,-0.5f, 0.f,0.f, 0,-1,0,
         0.5f, 0.5f, 0.5f, 0.f,1.f, 0,-1,0,
        -0.5f, 0.5f,-0.5f, 1.f,0.f, 0,-1,0, col, mat);
    // bottom face
    sfr__triangle_tex_norm(
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,1,0,
        -0.5f,-0.5f, 0.5f, 1.f,1.f, 0,1,0,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,1,0, col, mat);
    sfr__triangle_tex_norm(
         0.5f,-0.5f,-0.5f, 0.f,0.f, 0,1,0,
        -0.5f,-0.5f,-0.5f, 0.f,1.f, 0,1,0,
         0.5f,-0.5f, 0.5f, 1.f,0.f, 0,1,0, col, mat);
}

SFR_FUNC void sfr_mesh(const SfrMesh* mesh, u32 col, const SfrMaterial* mat) {
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
            job->tangents = mesh->tangents ? &mesh->tangents[i * 12] : NULL;
            job->matNormal = sfrState.matNormal;
            job->matMVP = matMVP;
            job->matModel = sfrMatModel;
            job->col = col;
            job->mat = mat;
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
            const i32 tanInd = (i / 9) * 12;
            sfr__process_and_bin_triangle(
                &matMVP, &sfrMatModel, &sfrState.matNormal,
                &mesh->tris[i + 0], mesh->uvs ? &mesh->uvs[uvInd + 0] : (void*)0,
                    &mesh->normals[i + 0], mesh->tangents ? &mesh->tangents[tanInd + 0] : (void*)0,
                &mesh->tris[i + 3], mesh->uvs ? &mesh->uvs[uvInd + 2] : (void*)0,
                    &mesh->normals[i + 3], mesh->tangents ? &mesh->tangents[tanInd + 4] : (void*)0,
                &mesh->tris[i + 6], mesh->uvs ? &mesh->uvs[uvInd + 4] : (void*)0,
                    &mesh->normals[i + 6], mesh->tangents ? &mesh->tangents[tanInd + 8] : (void*)0,
                col, mat
            );
        }
    #endif
}

SFR_FUNC void sfr_string(const SfrFont* font, const char* s, i32 sLength, u32 col) {
    (void)font; (void)s; (void)sLength; (void)col;
    SFR__ERR_RET(, "TODO not implemented, 'sfr_glyph' is implemented\n");
}

SFR_FUNC void sfr_glyph(const SfrFont* font, u16 id, u32 col) {
    if (id >= SFR_FONT_GLYPH_MAX) {
        SFR__ERR_RET(, "invalid id (%d >= %d)\n", id, SFR_FONT_GLYPH_MAX);
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
    SfrScene* scene;
    SFR__MALLOC(scene, sizeof(SfrScene));
    if (!objects || count <= 0) {
        SFR__ERR_RET(scene, "!objects (%p) || count (%d) <= 0\n", objects, count);
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
            SFR__MALLOC(obj->_bvhNodes, sizeof(struct sfrBvhNode) * (2 * triCount - 1));
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

        sfr_mesh(scene->objects[i].mesh, scene->objects[i].col, scene->objects[i].mat);
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
        SFR__ERR_RET(hit, "!scene (%p) || scene->count (%d) <= 0\n", scene, scene ? scene->count : 0);
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

SFR_FUNC void sfr_set_rendermode(i32 mode) {
    if (mode < SFR_RENDERMODE_SHADED || mode >= SFR_RENDERMODE_COUNT) {
        SFR__ERR_RET(, "invalid mode (%d), expected in the range [%d, %d)\n", mode, SFR_RENDERMODE_SHADED, SFR_RENDERMODE_COUNT);
    }
    sfrState.renderMode = mode;
}

SFR_FUNC SfrLight* sfr_light_add_point(f32 posX, f32 posY, f32 posZ, f32 ambient, f32 intensity, f32 attenuation, f32 r, f32 g, f32 b) {
    if (sfrState.lightCount >= sfrState.lightCap) {
        const i32 newCap = (i32)(sfrState.lightCap * 1.5f);
        SfrLight* newLights = (SfrLight*)sfrRealloc(
            sfrState.lights, sizeof(SfrLight) * newCap);

        if (!newLights) {
            SFR__ERR_RET((void*)0, "failed to realloc lights\n");
        }

        sfrState.lights = newLights;
        sfrState.lightCap = newCap;
    }

    SfrLight* l = &sfrState.lights[sfrState.lightCount++];

    *l = (SfrLight){
        .type = SFR_LIGHT_POINT,
        .x = posX, .y = posY, .z = posZ,
        .r = r, .g = g, .b = b,
        .ambient = ambient,
        .intensity = intensity,
        .attenuation = attenuation
    };

    return l;
}

SFR_FUNC SfrLight* sfr_light_add_directional(f32 dirX, f32 dirY, f32 dirZ, f32 ambient, f32 intensity, f32 r, f32 g, f32 b) {
    if (sfrState.lightCount >= sfrState.lightCap) {
        const i32 newCap = (i32)(sfrState.lightCap * 1.5f);
        SfrLight* newLights = (SfrLight*)sfrRealloc(
            sfrState.lights, sizeof(SfrLight) * newCap);

        if (!newLights) {
            SFR__ERR_RET((void*)0, "failed to realloc lights\n");
        }

        sfrState.lights = newLights;
        sfrState.lightCap = newCap;
    }

    SfrLight* l = &sfrState.lights[sfrState.lightCount++];

    const sfrvec d = sfr_vec_normf(dirX, dirY, dirZ);

    *l = (SfrLight){
        .type = SFR_LIGHT_DIRECTIONAL,
        .x = d.x, .y = d.y, .z = d.z,
        .r = r, .g = g, .b = b,
        .ambient = ambient,
        .intensity = intensity
    };

    return l;
}

SFR_FUNC void sfr_light_remove(SfrLight* light) {
    if (!light || 0 == sfrState.lightCount) {
        return;
    }

    SfrLight* last  = &sfrState.lights[sfrState.lightCount - 1];

    if (light != last) {
        *light = *last;
    }

    sfrState.lightCount -= 1;
}

#ifdef SFR_USE_CGLTF

SFR_FUNC void sfr_model_animate(SfrModel* model, i32 animInd, f32 time) {
    if (!model) {
        SFR__ERR_RET(, "model is NULL\n");
    }
    if (animInd < 0 || animInd >= model->animCount) {
        SFR__ERR_RET(, "animInd (%d) out of bounds [%d, %d)\n", animInd, 0, model->animCount);
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
        SFR__ERR_RET(NULL, "failed to open '%s'\n", filename);
    }

    fseek(file, 0, SEEK_END);
    const i64 fileSize = ftell(file);
    rewind(file);

    u8* _fileContent;
    SFR__MALLOC(_fileContent, fileSize);
    u8* const fileContent = _fileContent;
    if (fread(fileContent, 1, fileSize, file) != (u64)fileSize) {
        fclose(file);
        sfrFree(fileContent);
        SFR__ERR_RET(NULL, "failed to read '%s'\n", filename);
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
        SFR__ERR_RET(NULL, "parse error %d\n", parseRes);
    }

    // vv load buffers vv
    const cgltf_result loadBufRes = cgltf_load_buffers(&options, data, filename);
    if (cgltf_result_success != loadBufRes) {
        cgltf_free(data);
        sfrFree(fileContent);
        SFR__ERR_RET(NULL, "buffer load error %d\n", loadBufRes);
    }

    SfrModel* model;
    SFR__MALLOC(model, sizeof(SfrModel));
    sfr_memset(model, 0, sizeof(SfrModel));

    // vv load textures if possible vv
    #ifdef SFR_USE_STB_IMAGE
        SfrTexture** loadedTextures = NULL;

        if (data->textures_count <= 0) {
            goto SKIP1;
        }

        SFR__MALLOC(loadedTextures, sizeof(SfrTexture*) * data->textures_count);
        sfr_memset(loadedTextures, 0, sizeof(SfrTexture*) * data->textures_count);

        SFR__MALLOC(model->_allMaterials, sizeof(SfrMaterial*) * data->textures_count);
        model->_matCount = (i32)data->textures_count;
        sfr_memset(model->_allMaterials, 0, sizeof(SfrMaterial*) * model->_matCount);

        for (i32 i = 0; i < model->_matCount; i += 1) {
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
                SfrTexture* tex;
                SFR__MALLOC(tex, sizeof(SfrTexture));
                tex->w = w;
                tex->h = h;
                SFR__MALLOC(tex->pixels, sizeof(u32) * w * h);
                if (!tex->pixels) {
                    // TODO memory leak
                    SFR__ERR_RET(NULL, "failed to allocate pixel buffer for texture %d (%ld bytes)\n",
                        i, (u64)(w * h * 4));
                }

                sfr_memcpy(tex->pixels, rawData, w * h * 4);

                stbi_image_free(rawData);

                sfr__swizzle_stb(tex->pixels, w * h);
                sfr__generate_mipmaps(tex);
                loadedTextures[i] = tex;
            }
        }

        SKIP1:;

        model->_matCount = (i32)data->materials_count;
        if (model->_matCount <= 0) {
            goto SKIP2;
        }

        SFR__MALLOC(model->_allMaterials, sizeof(SfrMaterial*) * model->_matCount);
        sfr_memset(model->_allMaterials, 0, sizeof(SfrMaterial*) * model->_matCount);

        for (i32 i = 0; i < model->_matCount; i += 1) {
            const cgltf_material* const cmat = &data->materials[i];
            SfrMaterial* mat;
            SFR__MALLOC(mat, sizeof(SfrMaterial));

            *mat = (SfrMaterial){
                .albedoTex = NULL, .metallicRoughnessTex = NULL,
                .baseColor = 0xFFFFFFFF, .metallicFactor = 0.3f, .roughnessFactor = 0.8f
            };

            if (cmat->has_pbr_metallic_roughness) {
                mat->metallicFactor = cmat->pbr_metallic_roughness.metallic_factor;
                mat->roughnessFactor = cmat->pbr_metallic_roughness.roughness_factor;

                if (cmat->pbr_metallic_roughness.base_color_texture.texture && loadedTextures) {
                    const i32 texInd = cmat->pbr_metallic_roughness.base_color_texture.texture - data->textures;
                    mat->albedoTex = loadedTextures[texInd];
                }
                if (cmat->pbr_metallic_roughness.metallic_roughness_texture.texture && loadedTextures) {
                    const i32 texInd = cmat->pbr_metallic_roughness.metallic_roughness_texture.texture - data->textures;
                    mat->metallicRoughnessTex = loadedTextures[texInd];
                }
            }
            model->_allMaterials[i] = mat;
        }

        SKIP2:;

        if (loadedTextures) {
            sfrFree(loadedTextures);
        }
    #endif

    // vv build transform hierarchy (1:1 with gltf nodes) vv
    SFR__MALLOC(model->transforms, sizeof(struct sfrTransformNode) * data->nodes_count);
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
        u8* computed;
        SFR__MALLOC(computed, model->transformCount);
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

    SFR__MALLOC(model->nodes, sizeof(struct sfrModelNode) * totalPrimitives);
    model->nodeCount = totalPrimitives;
    // allocate pointers for all meshes even though they map 1:1 with nodes now
    SFR__MALLOC(model->_allMeshes, sizeof(SfrMesh*) * totalPrimitives);
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
            cgltf_accessor *posA = NULL, *normA = NULL, *uvA = NULL, *tanA = NULL;
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
                    case cgltf_attribute_type_tangent: {
                        tanA = prim->attributes[a].data;
                    } break;
                    default: break;
                }
            }

            if (!posA) {
                continue; // positions required
            }

            const i32 count = prim->indices ? (i32)prim->indices->count : (i32)posA->count;

            SfrMesh* sm;
            SFR__MALLOC(sm, sizeof(SfrMesh));
            sm->vertCount = count * 3;
            SFR__MALLOC(sm->tris, sizeof(f32) * sm->vertCount);
            SFR__MALLOC(sm->normals, sizeof(f32) * sm->vertCount);
            SFR__MALLOC(sm->uvs, sizeof(f32) * count * 2);
            SFR__MALLOC(sm->tangents, sizeof(f32) * count * 4);
            if (!sm->tris || !sm->normals || !sm->uvs || !sm->tangents) {
                // TODO memory leak
                SFR__ERR_RET(NULL, "failed to allocate mesh buffers (%ld, %ld, %ld, %ld bytes)\n",
                    (u64)(sizeof(f32) * sm->vertCount), (u64)(sizeof(f32) * sm->vertCount), (u64)(sizeof(f32) * count * 2), (u64)(sizeof(f32) * count * 4));
            }

            for (i32 k = 0; k < count; k += 1) {
                // flip winding order and negate z and xy rotation
                const i32 kr = k - (k % 3) + (i32[3]){0, 2, 1}[k % 3];
                const i32 ind = prim->indices ? (i32)cgltf_accessor_read_index(prim->indices, kr) : kr;

                f32 buf[4];
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

                if (tanA) {
                    cgltf_accessor_read_float(tanA, ind, buf, 4);
                }
            }

            if (!tanA) {
                sfr__compute_mesh_tangents(sm);
            }

            // vv assign to model node vv
            model->nodes[currentNodeInd].mesh = sm;
            model->nodes[currentNodeInd].mat = NULL;
            model->nodes[currentNodeInd].transformInd = i;

            // vv assign material for this specific primitive vv
            #ifdef SFR_USE_STB_IMAGE
                if (prim->material && model->_allMaterials) {
                    const ptrdiff_t matInd = prim->material - data->materials;
                    if (matInd >= 0 && matInd < model->_matCount) {
                        model->nodes[currentNodeInd].mat = model->_allMaterials[matInd];
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
        SFR__MALLOC(model->animations, sizeof(struct sfrAnimation) * model->animCount);

        for (i32 i = 0; i < model->animCount; i += 1) {
            const cgltf_animation* const ca = &data->animations[i];
            struct sfrAnimation* const sa = &model->animations[i];

            sa->duration = 0.f;
            sa->samplerCount = (i32)ca->samplers_count;
            sa->channelCount = (i32)ca->channels_count;
            SFR__MALLOC(sa->samplers, sizeof(struct sfrAnimSampler) * sa->samplerCount);
            SFR__MALLOC(sa->channels, sizeof(struct sfrAnimChannel) * sa->channelCount);

            for (i32 j = 0; j < sa->samplerCount; j += 1) {
                const cgltf_animation_sampler* const cs = &ca->samplers[j];
                struct sfrAnimSampler* const ss = &sa->samplers[j];

                ss->count = (i32)cs->input->count;
                SFR__MALLOC(ss->inputs, sizeof(f32) * ss->count);
                SFR__MALLOC(ss->outputs, sizeof(f32) * ss->count * 4);

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
                u8* patchedSamplers;
                SFR__MALLOC(patchedSamplers, sa->samplerCount);
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

SFR_FUNC void sfr_model_draw(const SfrModel* model, sfrmat transform, const SfrMaterial* overrideMat) {
    if (!model) {
        SFR__ERR_RET(, "model is NULL\n");
    }

    const sfrmat savedModel = sfrMatModel;
    const sfrmat savedNormal = sfrState.matNormal;
    const u8 savedDirty = sfrState.normalMatDirty;

    for (i32 i = 0; i < model->nodeCount; i += 1) {
        const sfrmat localWorld = model->transforms[model->nodes[i].transformInd].worldMatrix;

        const sfrmat m = sfr_mat_mul(localWorld, transform);
        sfrMatModel = sfr_mat_mul(m, savedModel);
        sfrState.normalMatDirty = 1;

        const SfrMaterial* mat = overrideMat ? overrideMat : (model->nodes[i].mat ? model->nodes[i].mat : &sfrState.baseMat);
        sfr_mesh(model->nodes[i].mesh, 0xFFFFFFFF, mat);
    }

    sfrMatModel = savedModel;
    sfrState.matNormal = savedNormal;
    sfrState.normalMatDirty = savedDirty;
}

// lossy because of sfr_mat_decompose so might not work perfectly
SFR_FUNC SfrScene* sfr_scene_from_model(const SfrModel* model) {
    if (!model || model->nodeCount <= 0) {
        SFR__ERR_RET(NULL, "!model (%p) || model->nodeCount <= 0 (%d)\n", model, model ? model->nodeCount : 0);
    }

    SfrSceneObject* objs;
    SFR__MALLOC(objs, sizeof(SfrSceneObject) * model->nodeCount);

    for (i32 i = 0; i < model->nodeCount; i += 1) {
        SfrSceneObject* const obj = &objs[i];

        obj->mesh = model->nodes[i].mesh;
        obj->mat  = model->nodes[i].mat ? model->nodes[i].mat : &sfrState.baseMat;
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
    if (m->_allMaterials) {
        for (i32 i = 0; i < m->_matCount; i += 1) {
            if (m->_allMaterials[i]) sfr_release_material(&m->_allMaterials[i]);
        }
        sfrFree(m->_allMaterials);
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
    SfrMesh* mesh;
    SFR__MALLOC(mesh, sizeof(SfrMesh));
    *mesh = (SfrMesh){0};

    FILE* objFile = fopen(filename, "rb");
    if (!objFile) {
        sfrFree(mesh);
        SFR__ERR_RET(NULL, "failed to open file '%s'\n", filename);
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
                        SFR__ERR_EXIT("realloc failed for tempVerts\n");
                    }
                }
                sscanf(line, "v %f %f %f", &tempVerts[vertCount].x, &tempVerts[vertCount].y, &tempVerts[vertCount].z);
                vertCount += 1;
            } else if ('t' == line[1]) { // texture coordinate
                if (uvCount >= tempUVsCap) {
                    tempUVsCap = (0 == tempUVsCap) ? 128 : tempUVsCap * 2;
                    tempUVs = (sfrvec*)realloc(tempUVs, tempUVsCap * sizeof(sfrvec));
                    if (!tempUVs) {
                        SFR__ERR_EXIT("realloc failed for tempUVs\n");
                    }
                }
                sscanf(line, "vt %f %f", &tempUVs[uvCount].x, &tempUVs[uvCount].y);
                uvCount += 1;
            } else if ('n' == line[1]) { // vertex normal
                if (normalCount >= tempNormalsCap) {
                    tempNormalsCap = (0 == tempNormalsCap) ? 128 : tempNormalsCap * 2;
                    tempNormals = (sfrvec*)realloc(tempNormals, tempNormalsCap * sizeof(sfrvec));
                    if (!tempNormals) {
                        SFR__ERR_EXIT("realloc failed for tempNormals\n");
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
                    SFR__ERR_EXIT("realloc failed for tempFaces\n");
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
    SFR__MALLOC(mesh->tris, mesh->vertCount * sizeof(f32));
    SFR__MALLOC(mesh->uvs, finalVertCount * 2 * sizeof(f32));
    SFR__MALLOC(mesh->normals, mesh->vertCount * sizeof(f32));

    if (!mesh->tris || !mesh->uvs || !mesh->normals) {
        SFR__ERR_EXIT("failed to allocate final mesh buffers\n");
    }

    sfrvec* computedNormals = NULL;
    if (0 == normalCount && vertCount > 0) { // if no normals in file
        SFR__MALLOC(computedNormals, sizeof(sfrvec) * vertCount);
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

                // const i32 chunkInd = triInd / 8;
                // const i32 lane     = triInd % 8;
                // const i32 pBase = chunkInd * 72;
                // mesh->tris[pBase + 0  + lane] = tempVerts[vInd].x;
                // mesh->tris[pBase + 8  + lane] = tempVerts[vInd].y;
                // mesh->tris[pBase + 16 + lane] = tempVerts[vInd].z;

                // position
                mesh->tris[triInd * 9 + v * 3 + 0] = tempVerts[vInd].x;
                mesh->tris[triInd * 9 + v * 3 + 1] = tempVerts[vInd].y;
                mesh->tris[triInd * 9 + v * 3 + 2] = tempVerts[vInd].z;

                // uvs
                if (vtInd >= 0 && vtInd < uvCount) {
                    // 48 floats per chunk
                    // const i32 uBase = chunkInd * 48;
                    // mesh->uvs[uBase + 0 + lane] = au;
                    // mesh->uvs[uBase + 8 + lane] = av;
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
                } else { // fallback, shouldn't ever happen
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

    // tangents aren't stored anywhere in obj files
    sfr__compute_mesh_tangents(mesh);

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

SFR_FUNC SfrMaterial* sfr_load_material(const char* albedoPath, const char* normalPath, const char* metRoughPath) {
    SfrMaterial* mat;
    SFR__MALLOC(mat, sizeof(SfrMaterial));

    SfrTexture* albedo = NULL;
    if (albedoPath) {
        albedo = sfr_load_texture(albedoPath);
    }

    SfrTexture* normal = NULL;
    if (normalPath) {
        normal = sfr_load_texture(normalPath);
    }

    SfrTexture* metRough = NULL;
    if (metRoughPath) {
        metRough = sfr_load_texture(metRoughPath);
    }

    mat->albedoTex = albedo;
    mat->metallicRoughnessTex = metRough;
    mat->baseColor = 0xFFFFFFFF;
    mat->roughnessFactor = metRough ? 0.f : 0.3f;
    mat->metallicFactor  = metRough ? 0.f : 0.2f;

    return mat;
}

SFR_FUNC void sfr_release_material(SfrMaterial** mat) {
    if (!(*mat)) {
        return;
    }

    sfr_release_texture(&(*mat)->albedoTex);
    sfr_release_texture(&(*mat)->metallicRoughnessTex);

    sfrFree(*mat);
    *mat = (void*)0;
}

SFR_FUNC SfrTexture* sfr_load_texture(const char* filename) {
#ifdef SFR_USE_STB_IMAGE
    i32 w, h, channels;
    u8* data = stbi_load(filename, &w, &h, &channels, 4); // force 4 channels
    if (!data) {
        SFR__ERR_RET(NULL, "stbi failed to load '%s'\n", filename);
    }

    SfrTexture* tex;
    SFR__MALLOC(tex, sizeof(SfrTexture));
    tex->w = w;
    tex->h = h;
    SFR__MALLOC(tex->pixels, sizeof(u32) * w * h);

    sfr_memcpy(tex->pixels, data, w * h * 4);
    stbi_image_free(data);

    sfr__swizzle_stb(tex->pixels, w * h);
    sfr__generate_mipmaps(tex);
    return tex;
#else
    FILE* file = fopen(filename, "rb");
    if (!file) {
        SFR__ERR_RET(NULL, "failed to open file '%s'\n", filename);
    }

    // read bmp headers
    u8 header[54];
    if (fread(header, 1, 54, file) != 54) {
        fclose(file);
        SFR__ERR_RET(NULL, "not a valid BMP file ('%s')\n", filename);
    }

    if (header[0] != 'B' || header[1] != 'M') {
        fclose(file);
        SFR__ERR_RET(NULL, "not a BMP file ('%s')\n", filename);
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
        SFR__ERR_RET(NULL, "unsupported BMP compression\n");
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
            SFR__ERR_RET(NULL, "failed to read color palette\n");
        }
    } else if (3 == comp && 4 != fread(masks, 4, 4, file)) { // BI_BITFIELDS
        fclose(file);
        SFR__ERR_RET(NULL, "failed to read bit masks\n");
    }

    // stride and data size
    const i32 stride = ((width * bpp + 31) / 32) * 4;
    const i32 dataSize = stride * height;

    // read pixel data
    u8* pixelData;
    SFR__MALLOC(pixelData, dataSize);
    if (!pixelData) {
        fclose(file);
        SFR__ERR_RET(NULL, "failed to allocate pixel data\n");
    }

    fseek(file, dataOffset, SEEK_SET);
    if (fread(pixelData, 1, dataSize, file) != dataSize) {
        sfrFree(pixelData);
        fclose(file);
        SFR__ERR_RET(NULL, "failed to read pixel data\n");
    }
    fclose(file);

    // create texture
    SfrTexture* tex;
    SFR__MALLOC(tex, sizeof(SfrTexture));
    if (!tex) {
        sfrFree(pixelData);
        SFR__ERR_RET(NULL, "failed to allocate texture struct\n");
    }

    tex->w = width;
    tex->h = height;
    SFR__MALLOC(tex->pixels, sizeof(u32) * width * height);
    if (!tex->pixels) {
        sfrFree(pixelData);
        sfrFree(tex);
        SFR__ERR_RET(NULL, "failed to allocate texture pixels\n");
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
                    SFR__ERR_RET(NULL, "unsupported bit depth: %d\n", bpp);
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
        SFR__ERR_RET(NULL, "failed to open file '%s'\n", filename);
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
        SFR__ERR_RET(NULL, "not a valid .srft file ('%s')\n", filename);
    }

    // allocate space for font
    SfrFont* font;
    SFR__MALLOC(font, sizeof(SfrFont));
    if (!font) {
        fclose(file);
        SFR__ERR_RET(NULL, "failed to allocate struct\n");
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
            SFR__ERR_RET(NULL, "error reading from file\n");
        }
        if (vertCount >= SFR_FONT_VERT_MAX) {
            fclose(file);
            sfrFree(font);
            SFR__ERR_RET(NULL,
                "vert count out of bounds for glyph '%c' (%d) (%d >= %d)\n",
                i, i, vertCount, SFR_FONT_VERT_MAX);
        }

        if (vertCount && vertCount != fread(font->verts[i], 4, vertCount, file)) {
            fclose(file);
            sfrFree(font);
            SFR__ERR_RET(NULL, "error reading vertices from file\n");
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

SFR_FUNC SfrParticleSystem sfr_particles_create(SfrParticle* buffer, i32 count, const SfrMaterial* mat) {
    return (SfrParticleSystem){
        .particles = buffer,
        .active = 0, .total = count,
        .mat = mat
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
        sfr_billboard(col, sys->mat);
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
        SFR__ERR_RET(min, "min >= max (%d >= %d)", min, max);
    }
    return (i32)(sfr_rand_next() % (max - min)) + min;
}

SFR_FUNC f32 sfr_rand_flt(f32 min, f32 max) {
    if (min >= max) {
        SFR__ERR_RET(min, "min >= max (%f >= %f)", min, max);
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

            // process all triangles in this job
            for (i32 i = 0; i < job->triangleCount; i += 1) {
                const i32 triInd = i * 9;
                const i32 uvInd = i * 6;
                const i32 tanInd = i * 12;
                sfr__process_and_bin_triangle(
                    &job->matMVP, &job->matModel, &job->matNormal,
                    &job->tris[triInd + 0], job->uvs ? &job->uvs[uvInd + 0] : NULL,
                        &job->normals[triInd + 0], job->tangents ? &job->tangents[tanInd + 0] : NULL,
                    &job->tris[triInd + 3], job->uvs ? &job->uvs[uvInd + 2] : NULL,
                        &job->normals[triInd + 3], job->tangents ? &job->tangents[tanInd + 4] : NULL,
                    &job->tris[triInd + 6], job->uvs ? &job->uvs[uvInd + 4] : NULL,
                        &job->normals[triInd + 6], job->tangents ? &job->tangents[tanInd + 8] : NULL,
                    job->col, job->mat
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
