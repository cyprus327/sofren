#ifndef SFR_H
#define SFR_H

#ifdef __cplusplus
extern "C" {
#endif

//================================================
//: PUBLIC API
//================================================

#ifndef NULL
    #define NULL (void*)0
#endif

//: threading config
#ifdef SFR_THREAD_COUNT
    #if SFR_THREAD_COUNT < 1 || SFR_THREAD_COUNT > 32
        #error "SFR ERROR: SFR_THREAD_COUNT must be between 1 and 32 (32 arbitrary)"
    #endif
#else
    #define SFR_THREAD_COUNT 8
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
    #define SFR_THREAD_LOCAL 
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
    #define vf32 sfrvf32_t
    #define vf32s sfrvf32s_t
    #define vi32 sfrvi32_t
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
typedef struct sfrdmesh    SfrDynamicMesh;
typedef struct sfrtex      SfrTexture;
typedef struct sfrMaterial SfrMaterial;
typedef struct sfrfont     SfrFont;

typedef struct sfrRenderTarget SfrRenderTarget;

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
SFR_FUNC sfrvec sfr_vec_perpendicular(sfrvec v);
SFR_FUNC sfrvec sfr_vec_rotate_by_quat(sfrvec v, sfrvec q);
SFR_FUNC sfrmat sfr_mat_identity();
SFR_FUNC sfrmat sfr_mat_rot_x(f32 a);
SFR_FUNC sfrmat sfr_mat_rot_y(f32 a);
SFR_FUNC sfrmat sfr_mat_rot_z(f32 a);
SFR_FUNC sfrmat sfr_mat_translate(f32 x, f32 y, f32 z);
SFR_FUNC sfrmat sfr_mat_scale(f32 x, f32 y, f32 z);
SFR_FUNC sfrmat sfr_mat_proj(f32 fovDeg, f32 aspect, f32 near, f32 far);
SFR_FUNC sfrmat sfr_mat_ortho(f32 left, f32 right, f32 bottom, f32 top, f32 near, f32 far);
SFR_FUNC sfrmat sfr_mat_mul(sfrmat a, sfrmat b);
SFR_FUNC sfrvec sfr_mat_mul_vec(sfrmat m, sfrvec v);
SFR_FUNC sfrmat sfr_mat_qinv(sfrmat m);
SFR_FUNC sfrmat sfr_mat_model_look_at(sfrvec pos, sfrvec target, sfrvec up); // used for when a 3D object (e.g. a missile or spotlight) should rotate to face a target
SFR_FUNC sfrmat sfr_mat_view_look_at(sfrvec pos, sfrvec target, sfrvec up);  // used only for cameras, computes the inverse translation required for view spaces
SFR_FUNC void sfr_mat_decompose(sfrmat m, sfrvec* pos, sfrvec* rot, sfrvec* scale);
SFR_FUNC sfrmat sfr_mat_from_quat(sfrvec q);
SFR_FUNC sfrvec sfr_quat_mul(sfrvec a, sfrvec b);
SFR_FUNC sfrvec sfr_quat_slerp(sfrvec a, sfrvec b, f32 t);
SFR_FUNC sfrvec sfr_quat_invert(sfrvec q);
SFR_FUNC sfrvec sfr_quat_norm(sfrvec q);

//: core functions
SFR_FUNC void sfr_init(i32 w, i32 h, f32 fovDeg,
    void* (*mallocFunc)(u64), void (*freeFunc)(void*), void* (*reallocFunc)(void*, u64));
SFR_FUNC void sfr_release(void);

// finish rendering triangles, must be called for any thread count
SFR_FUNC void sfr_present(void);

// shadow mapping
SFR_FUNC void sfr_shadow_pass_begin(
    SfrRenderTarget* target, sfrvec lightDir, sfrvec center, f32 orthoSize, f32 distance, f32 zFar);
SFR_FUNC void sfr_shadow_pass_end(void);
SFR_FUNC void sfr_shadow_push_state(void); // temporarily disable shadowmap and save its state
SFR_FUNC void sfr_shadow_pop_state(void);  // restores the previously stored shadowmap

// render targets
SFR_FUNC SfrRenderTarget* sfr_create_target(i32 width, i32 height, u8 hasPixels);
SFR_FUNC void sfr_release_target(SfrRenderTarget** target);
SFR_FUNC void sfr_set_render_target(SfrRenderTarget* target); // NULL resets to default

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

// other drawing functions, if mat is NULL sfrState.baseMat (white 1x1 texture) will be used
SFR_FUNC void sfr_point(f32 worldX, f32 worldY, f32 worldZ, i32 radius, u32 col);
SFR_FUNC void sfr_billboard(u32 col, const SfrMaterial* mat);
SFR_FUNC void sfr_cube(u32 col, const SfrMaterial* mat);
SFR_FUNC void sfr_cube_ex(u32 col[12]);
SFR_FUNC void sfr_cube_inv(u32 col, const SfrMaterial* mat);
SFR_FUNC void sfr_sphere(i32 rings, i32 slices, u32 col);
SFR_FUNC void sfr_capsule(sfrvec start, sfrvec end, f32 radius, i32 slices, i32 rings, u32 col);
SFR_FUNC void sfr_cylinder(sfrvec startPos, sfrvec endPos, f32 startRadius, f32 endRadius, i32 sides, u32 col);
SFR_FUNC void sfr_mesh(const SfrMesh* mesh, u32 col, const SfrMaterial* mat);
SFR_FUNC void sfr_string(const SfrFont* font, const char* s, i32 sLength, u32 col); // not yet implemented
SFR_FUNC void sfr_glyph(const SfrFont* font, u16 id, u32 col); // draw a single character

// dynamic mesh functions
SFR_FUNC SfrDynamicMesh* sfr_dmesh_request(i32 vertCount);
SFR_FUNC void sfr_dmesh_submit(const SfrDynamicMesh* dmesh, u32 col, const SfrMaterial* mat);

// static scene functions
SFR_FUNC SfrScene* sfr_scene_create(SfrSceneObject* objects, i32 count);
SFR_FUNC void sfr_scene_apply_lights(const SfrScene* scene);
SFR_FUNC void sfr_scene_draw(const SfrScene* scene);
SFR_FUNC SfrRayHit sfr_scene_raycast(const SfrScene* scene, f32 ox, f32 oy, f32 oz, f32 dx, f32 dy, f32 dz);
SFR_FUNC void sfr_scene_set_transform(SfrScene* scene, sfrvec pos, sfrvec rot, sfrvec scale);

// project the world position specified to screen coordinates
SFR_FUNC u8 sfr_world_to_screen(f32 x, f32 y, f32 z, i32* screenX, i32* screenY);

// update the camera with the new position and view
SFR_FUNC void sfr_set_camera(f32 x, f32 y, f32 z, f32 yaw, f32 pitch, f32 roll);
SFR_FUNC void sfr_set_fov(f32 fovDeg); // update projection matrix with new fov
SFR_FUNC void sfr_set_rendermode(i32 mode); // takes SFR_RENDERMODE_[mode] enum
SFR_FUNC void sfr_set_lightmap(SfrTexture* lightmap); // NULL => disable lightmap
SFR_FUNC void sfr_set_shadowmap(SfrRenderTarget* target, sfrmat matLightVP); // NULL => disable shadowmap

// helpers for adding and removing lights, only enabled when lighting is enabled
SFR_FUNC SfrLight* sfr_light_add_point(f32 posX, f32 posY, f32 posZ, f32 ambient, f32 intensity, f32 attenuation, f32 r, f32 g, f32 b);
SFR_FUNC SfrLight* sfr_light_add_directional(f32 dirX, f32 dirY, f32 dirZ, f32 ambient, f32 intensity, f32 r, f32 g, f32 b);
SFR_FUNC void sfr_light_remove(SfrLight* light);
SFR_FUNC void sfr_light_clear_all(void);

#ifdef SFR_USE_CGLTF
    SFR_FUNC void sfr_model_animate(SfrModel* model, i32 animInd, f32 time);
    SFR_FUNC SfrModel* sfr_load_gltf(const char* filename, i32 uvChannel, i32 texMaxWidth, i32 texMaxHeight);
    SFR_FUNC void sfr_model_draw(const SfrModel* model, sfrmat transform, const SfrMaterial* overrideMat);
    SFR_FUNC SfrScene* sfr_scene_from_model(const SfrModel* model);
    SFR_FUNC void sfr_release_model(SfrModel** model);
#endif

// material / texture helpers
SFR_FUNC void sfr_material_resize_textures(SfrMaterial* mat, i32 newW, i32 newH);
SFR_FUNC SfrTexture* sfr_load_texture_raw(i32 w, i32 h, const u32* pixels);

// things requiring stdio
#ifndef SFR_NO_STD
    SFR_FUNC SfrMesh* sfr_load_mesh(const char* filename); // load an obj file into a struct that sofren can use
    SFR_FUNC void sfr_release_mesh(SfrMesh** mesh);        // release loaded mesh's memory

    // wrappers of sfr_load_texture and sfr_release_texture
    SFR_FUNC SfrMaterial* sfr_load_material(const char* albedoPath, const char* metRoughPath);
    SFR_FUNC void sfr_release_material(SfrMaterial** mat);

    SFR_FUNC SfrTexture* sfr_load_texture(const char* filename); // load a BMP texture
    SFR_FUNC void sfr_release_texture(SfrTexture** texture);     // release loaded texture's memory

    SFR_FUNC SfrFont* sfr_load_font(const char* filename); // load a .srft (sofren font type) font, see 'sfr-fontmaker'
    SFR_FUNC void sfr_release_font(SfrFont** font);        // release loaded font's memory

    // NOTE: the editor to create .cmp files is not public yet, but there are some .cmp files in examples/res/maps/
    SFR_FUNC SfrScene* sfr_load_scene(const char* filename);           // load a .cmp (compiled map) file
    SFR_FUNC void sfr_release_scene(SfrScene** scene, u8 freeObjects); // release a loaded scene's memory
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
    static void sfr__process_geometry_jobs(void);
    static void sfr__process_raster_jobs(void* tData); // actually struct sfrThreadData* tData
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

// SIMD wrappers
#ifndef SFR_NO_SIMD
    #if defined(__AVX2__)
        #include <immintrin.h>
        #define SFR_SIMD_LANES 8
        #define SFR_SIMD_LANE_MAX_OFFSET 7.f
        #define SFR_SIMD_FULL_LANE_MASK  0xFF
        #define SFR_SIMD_FULL_BYTE_MASK  -1

        #define SFR_SIMD_SET_STEPS()      sfrvf_setr(0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f)
        #define SFR_SIMD_SET_HALF_STEPS() sfrvf_setr(0.5f, 1.5f, 2.5f, 3.5f, 4.5f, 5.5f, 6.5f, 7.5f)
        #define SFR_SIMD_SET_INDICES()    sfrvi_setr(0, 1, 2, 3, 4, 5, 6, 7)

        typedef __m256  vf32;  // 8 floats
        typedef __m128  vf32s; // 4 floats (for vector math)
        typedef __m256i vi32;  // 8 ints

    #elif defined(__ARM_NEON)
        #include <arm_neon.h>
        #define SFR_SIMD_LANES 4
        #define SFR_SIMD_LANE_MAX_OFFSET 3.f
        #define SFR_SIMD_FULL_LANE_MASK  0x0F
        #define SFR_SIMD_FULL_BYTE_MASK  0xFFFF

        #define SFR_SIMD_SET_STEPS()      sfrvf_setr(0.f, 1.f, 2.f, 3.f)
        #define SFR_SIMD_SET_HALF_STEPS() sfrvf_setr(0.5f, 1.5f, 2.5f, 3.5f)
        #define SFR_SIMD_SET_INDICES()    sfrvi_setr(0, 1, 2, 3)

        typedef float32x4_t vf32;  // 4 floats
        typedef float32x4_t vf32s; // 4 floats (same for NEON)
        typedef int32x4_t   vi32;  // 4 ints
    #else
        #define SFR_NO_SIMD
        #ifndef SFR_NO_WARNINGS
            #warning "SFR WARNING: SIMD explicitly supported architecture not found, SFR_NO_SIMD auto defined"
        #endif
    #endif
#else
    #define SFR_SIMD_LANE_MAX_OFFSET 7.f
#endif
#ifndef SFR_NO_SIMD
    enum sfrCmpMode {
        SFR_CMP_UNORD_Q = 0x03, // isnan(a) || isnan(b)

        SFR_CMP_LT_OQ   = 0x11, // a <  b
        SFR_CMP_LE_OQ   = 0x12, // a <= b

        SFR_CMP_GE_OQ   = 0x1D, // a >= b
        SFR_CMP_GT_OQ   = 0x1E, // a >  b
    };

    #define SFR_SHUFFLE(z, y, x, w) (((z) << 6) | ((y) << 4) | ((x) << 2) | (w))

    #if defined(__AVX2__)

        #define sfrvf_setr(e0, e1, e2, e3, e4, e5, e6, e7) _mm256_setr_ps(e0, e1, e2, e3, e4, e5, e6, e7)
        #define sfrvi_setr(e0, e1, e2, e3, e4, e5, e6, e7) _mm256_setr_epi32(e0, e1, e2, e3, e4, e5, e6, e7)
        
        #define sfrvi_gather(src, base, vindex, mask, scale) _mm256_mask_i32gather_epi32((src), (base), (vindex), (mask), (scale))
        #define sfrvf_gather(src, base, vindex, mask, scale) _mm256_mask_i32gather_ps((src), (base), (vindex), (mask), (scale))

        #define sfrvi_slli(a, imm) _mm256_slli_epi32((a), (imm))
        #define sfrvi_srli(a, imm) _mm256_srli_epi32((a), (imm))
        
        #define sfrvf_cmp(a, b, imm) _mm256_cmp_ps((a), (b), (imm))
        #define sfrvfs_shuffle(a, b, imm) _mm_shuffle_ps((a), (b), (imm))
        #define sfrvfs_dp(a, b, imm) _mm_dp_ps((a), (b), (imm))

    #elif defined(__ARM_NEON)

        #define sfrvf_setr(e0, e1, e2, e3) (float32x4_t){e0, e1, e2, e3}
        #define sfrvi_setr(e0, e1, e2, e3) (int32x4_t){e0, e1, e2, e3}

        #define sfrvi_gather(src, base, vindex, mask, scale) ({ \
            SFR_ALIGNED(16) int32_t ind[4]; \
            SFR_ALIGNED(16) uint32_t m[4];  \
            SFR_ALIGNED(16) int32_t out[4]; \
            \
            vst1q_s32(ind, (vindex)); \
            vst1q_u32(m, vreinterpretq_u32_s32(mask)); \
            vst1q_s32(out, (src)); \
            \
            const char* b = (const char*)(base); \
            \
            if (m[0] & 0x80000000) out[0] = *(const int32_t*)(b + ind[0] * (scale)); \
            if (m[1] & 0x80000000) out[1] = *(const int32_t*)(b + ind[1] * (scale)); \
            if (m[2] & 0x80000000) out[2] = *(const int32_t*)(b + ind[2] * (scale)); \
            if (m[3] & 0x80000000) out[3] = *(const int32_t*)(b + ind[3] * (scale)); \
            \
            vld1q_s32(out); \
        })

        #define sfrvf_gather(src, base, vindex, mask, scale) ({ \
            SFR_ALIGNED(16) int32_t ind[4]; \
            SFR_ALIGNED(16) uint32_t m[4];  \
            SFR_ALIGNED(16) float out[4];   \
            \
            vst1q_s32(ind, (vindex)); \
            vst1q_u32(m, vreinterpretq_u32_f32(mask)); \
            vst1q_f32(out, (src)); \
            \
            const char* b = (const char*)(base); \
            \
            if (m[0] & 0x80000000) out[0] = *(const float*)(b + ind[0] * (scale)); \
            if (m[1] & 0x80000000) out[1] = *(const float*)(b + ind[1] * (scale)); \
            if (m[2] & 0x80000000) out[2] = *(const float*)(b + ind[2] * (scale)); \
            if (m[3] & 0x80000000) out[3] = *(const float*)(b + ind[3] * (scale)); \
            \
            vld1q_f32(out); \
        })

        #define sfrvi_slli(a, imm) vshlq_s32((a), vdupq_n_s32((imm)))
        #define sfrvi_srli(a, imm) vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a), vdupq_n_s32(-(imm))))

        #define sfrvf_cmp(a, b, imm) \
            ( (imm) == SFR_CMP_UNORD_Q ? \
                vreinterpretq_f32_u32( \
                    vorrq_u32( \
                        vmvnq_u32(vceqq_f32((a), (a))), \
                        vmvnq_u32(vceqq_f32((b), (b))) \
                    ) \
                ) : \
            (imm) == SFR_CMP_LT_OQ ? \
                vreinterpretq_f32_u32(vcltq_f32((a), (b))) : \
            (imm) == SFR_CMP_LE_OQ ? \
                vreinterpretq_f32_u32(vcleq_f32((a), (b))) : \
            (imm) == SFR_CMP_GE_OQ ? \
                vreinterpretq_f32_u32(vcgeq_f32((a), (b))) : \
            (imm) == SFR_CMP_GT_OQ ? \
                vreinterpretq_f32_u32(vcgtq_f32((a), (b))) : \
            vdupq_n_f32(0.f) )

        #define sfrvfs_shuffle(a, b, imm) \
            (float32x4_t){ \
                vgetq_lane_f32((a), ((imm) >> 0) & 0x3), \
                vgetq_lane_f32((a), ((imm) >> 2) & 0x3), \
                vgetq_lane_f32((b), ((imm) >> 4) & 0x3), \
                vgetq_lane_f32((b), ((imm) >> 6) & 0x3)  \
            }

        // upper 4 bits of imm control which lanes are multiplied and accumulated,
        // lower 4 bits of imm control which lanes the result is broadcasted to
        #define sfrvfs_dp(a, b, imm) ({ \
            const float32x4_t m = vmulq_f32((a), (b)); \
            float sum = 0.f; \
            if ((imm) & 0x10) sum += vgetq_lane_f32(m, 0); \
            if ((imm) & 0x20) sum += vgetq_lane_f32(m, 1); \
            if ((imm) & 0x40) sum += vgetq_lane_f32(m, 2); \
            if ((imm) & 0x80) sum += vgetq_lane_f32(m, 3); \
            float32x4_t res = vdupq_n_f32(0.f); \
            if ((imm) & 0x01) res = vsetq_lane_f32(sum, res, 0); \
            if ((imm) & 0x02) res = vsetq_lane_f32(sum, res, 1); \
            if ((imm) & 0x04) res = vsetq_lane_f32(sum, res, 2); \
            if ((imm) & 0x08) res = vsetq_lane_f32(sum, res, 3); \
            res; \
        })

    #endif

    SFR_FUNC vf32 sfrvf_set1(f32 v);
    SFR_FUNC vi32 sfrvi_set1(i32 v);
    SFR_FUNC vf32 sfrvf_zero(void);
    SFR_FUNC vi32 sfrvi_zero(void);
    SFR_FUNC vf32 sfrvf_loadu(const f32* mem);
    SFR_FUNC void sfrvf_storeu(f32* mem, vf32 a);
    SFR_FUNC vi32 sfrvi_loadu(const vi32* mem);
    SFR_FUNC void sfrvi_storeu(vi32* mem, vi32 a);
    
    SFR_FUNC vf32s sfrvfs_set1(f32 v);
    SFR_FUNC f32 sfrvfs_cvtss_f32(vf32s a);
    SFR_FUNC vf32s sfrvfs_add(vf32s a, vf32s b);
    SFR_FUNC vf32s sfrvfs_sub(vf32s a, vf32s b);
    SFR_FUNC vf32s sfrvfs_mul(vf32s a, vf32s b);
    SFR_FUNC vf32s sfrvfs_div(vf32s a, vf32s b);
    SFR_FUNC vf32s sfrvfs_rsqrt(vf32s a);
    SFR_FUNC vf32s sfrvfs_sqrt_ss(vf32s a);

    SFR_FUNC vf32 sfrvf_add(vf32 a, vf32 b);
    SFR_FUNC vf32 sfrvf_sub(vf32 a, vf32 b);
    SFR_FUNC vf32 sfrvf_mul(vf32 a, vf32 b);
    SFR_FUNC vf32 sfrvf_fmadd(vf32 a, vf32 b, vf32 c);
    SFR_FUNC vf32 sfrvf_fnmadd(vf32 a, vf32 b, vf32 c);
    SFR_FUNC vf32 sfrvf_div(vf32 a, vf32 b);
    SFR_FUNC vf32 sfrvf_sqrt(vf32 a);
    SFR_FUNC vf32 sfrvf_rsqrt(vf32 a);
    SFR_FUNC vf32 sfrvf_rcp(vf32 a);
    SFR_FUNC vf32 sfrvf_floor(vf32 a);
    SFR_FUNC vf32 sfrvf_min(vf32 a, vf32 b);
    SFR_FUNC vf32 sfrvf_max(vf32 a, vf32 b);

    SFR_FUNC vf32 sfrvf_log2(vf32 x);
    SFR_FUNC vf32 sfrvf_exp2(vf32 x);
    SFR_FUNC vf32 sfrvf_pow(vf32 x, vf32 y);

    SFR_FUNC vi32 sfrvi_add(vi32 a, vi32 b);
    SFR_FUNC vi32 sfrvi_sub(vi32 a, vi32 b);
    SFR_FUNC vi32 sfrvi_mullo(vi32 a, vi32 b);
    SFR_FUNC vi32 sfrvi_min(vi32 a, vi32 b);
    SFR_FUNC vi32 sfrvi_and(vi32 a, vi32 b);
    SFR_FUNC vi32 sfrvi_or(vi32 a, vi32 b);
    SFR_FUNC vi32 sfrvi_cmpeq(vi32 a, vi32 b);
    SFR_FUNC vi32 sfrvi_cvtps(vf32 a);
    SFR_FUNC i32  sfrvi_movemask_epi8(vi32 a);

    SFR_FUNC vf32 sfrvf_cast_from_vi32(vi32 a);
    SFR_FUNC vi32 sfrvi_cast_from_vf32(vf32 a);
    SFR_FUNC vf32 sfrvf_cvtepi32(vi32 a);
    SFR_FUNC vi32 sfrvi_cvttps(vf32 a);
    SFR_FUNC vf32 sfrvf_and(vf32 a, vf32 b);

    SFR_FUNC i32  sfrvf_movemask(vf32 a);
    SFR_FUNC void sfrvi_maskstore(int* mem, vi32 mask, vi32 a);
    SFR_FUNC vf32 sfrvf_blendv(vf32 a, vf32 b, vf32 mask);
#endif


//================================================
//: PUBLIC MACROS
//================================================

#define SFR_PI (3.14159265358979323846)
#define SFR_EPSILON (1e-8)

#define SFR_ARRLEN(_arr)  (sizeof(_arr) / sizeof((_arr)[0]))

#define SFR_MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))
#define SFR_MAX(_a, _b) ((_a) > (_b) ? (_a) : (_b))
#define SFR_CLAMP(_x, _min, _max) ((_x) < (_min) ? (_min) : ((_x) > (_max) ? (_max) : (_x)))


//================================================
//: IMPLEMENTATION / TYPES
//================================================

#ifdef SFR_IMPL

#ifdef _MSC_VER
    #define SFR_ALIGNED(n) __declspec(align(n))
#else
    #define SFR_ALIGNED(n) __attribute__((aligned(n)))
#endif

#ifndef SFR_NO_SIMD
    typedef union SFR_ALIGNED(16) sfrvec {
        vf32s v;
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
    f32* lmUvs;    // lightmap uvs (2 floats per vert)
    f32* normals;  // vertex normals (3 floats per vert)
    f32* tangents; // vertex tangents (4 floats per vert (x, y, z, sign))
    i32 vertCount; // total number of floats in tris array
} SfrMesh;

typedef struct sfrdmesh {
    // pointing into sfrDynamicArena
    f32* positions;
    f32* uvs;
    f32* lmUvs;
    f32* normals;
    f32* tangents;
    i32 vertCount;
    SfrMesh _mesh; // underlying mesh passed to the rasterizer
} SfrDynamicMesh;

typedef struct sfrtex {
    u32* pixels;
    i32 w, h;

    // only allocated and used for lightmaps to avoid expensive filtering
    u32* quadR;
    u32* quadG;
    u32* quadB;

    // index 0 is base texture, index 1-11 are mipmaps
    u32* allPixels[14];
    i32 allW[14];
    i32 allH[14];
    
    i32 blocksW[14];     // how many 4x4 blocks across
    i32 strideShift[14]; // if POT, log2(blocksW) to avoid integer multiplication

    i32 mipLevels; // total levels including base
    u8 isPot;      // power of 2 flag
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

typedef struct sfrRenderTarget {
    i32 width, height;
    u32* pixels; // NULL if depth only
    f32* depth;
    f32 shadowBias; // bias only for shadowmaps
    f32 shadowStrength;
    u8 ownsMem;
} SfrRenderTarget;

struct sfrBounds {
    f32 minX, minY, minZ;
    f32 maxX, maxY, maxZ;
};

typedef struct sfrScene {
    SfrSceneObject* objects;
    i32 count;

    SfrTexture** textures;
    SfrMaterial** materials;
    i32 textureCount;

    SfrTexture* globalLightmap;
    SfrTexture* globalDirectionmap;

    SfrLight* lights;
    i32 lightCount;

    u32 skyColor;
    f32 ambientLight;

    sfrvec spawnPos;
    u8 hasSpawn;
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
    f32 u, v;            // barycentric coords of the hit

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

struct sfrVertexData {
    f32 invZ;
    u32 n;      // packed normal, 10-10-10-2
    u32 t;      // packed tangent + sign, 10-10-10-2
    f32 u, v;
    u16 lu, lv; // packed UNORM
};

// helper to track vertex attributes during clipping
struct sfrTexVert {
    sfrvec pos;      // position in view space
    f32 u, v;        // texture coords
    f32 lu, lv;      // lightmap coords
    sfrvec normal;   // world space normal for lighting
    sfrvec tangent;  // transformed tangent and handedness sign in w
    f32 viewZ;       // z in view space for perspective correction
};

struct sfrGeomTri {
    const f32* posA;
    const f32* uvA;
    const f32* lmUvA;
    const f32* normA;
    const f32* tanA;
    
    const f32* posB;
    const f32* uvB;
    const f32* lmUvB;
    const f32* normB;
    const f32* tanB;
    
    const f32* posC;
    const f32* uvC;
    const f32* lmUvC;
    const f32* normC;
    const f32* tanC;
    
    u32 col;
    const SfrMaterial* mat;
    
    i32 renderMode;

    struct sfrGeomClipBufs {
        struct sfrTexVert clipTris[16][3];
        struct sfrTexVert buffer[16][3];
    }* clipBuf;
};

// only used during rasterization
struct sfrRasterBin {
    i32 shadeId;

    // depth buffer equations (rasterizer)
    f32 dzdx, dzdy, zBase;

    // edge equations (rasterizer, barycentric weights)
    f32 A0, B0, C0;
    f32 A1, B1, C1;
    f32 A2, B2, C2;

    // bounding box (rasterizer)
    i16 minX, maxX;
    i16 minY, maxY;
};

enum sfrRenderPath {
    SFR_PATH_UNLIT_COLOR,
    SFR_PATH_UNLIT_TEX,
    
    SFR_PATH_LIGHTMAP_COLOR,
    SFR_PATH_LIGHTMAP_TEX,
    SFR_PATH_LIGHTMAP_PBR,
    SFR_PATH_LIGHTMAP_COLOR_SHADOW,
    SFR_PATH_LIGHTMAP_TEX_SHADOW,
    SFR_PATH_LIGHTMAP_PBR_SHADOW,

    SFR_PATH_DYNAMIC_COLOR,
    SFR_PATH_DYNAMIC_TEX,
    SFR_PATH_DYNAMIC_COLOR_SHADOW,
    SFR_PATH_DYNAMIC_TEX_SHADOW,
    
    SFR_PATH_LIGHTMAP_DYNAMIC_COLOR,
    SFR_PATH_LIGHTMAP_DYNAMIC_TEX,
    SFR_PATH_LIGHTMAP_DYNAMIC_PBR,
    SFR_PATH_LIGHTMAP_DYNAMIC_COLOR_SHADOW,
    SFR_PATH_LIGHTMAP_DYNAMIC_TEX_SHADOW,
    SFR_PATH_LIGHTMAP_DYNAMIC_PBR_SHADOW,

    SFR_PATH_DEBUG_NORMALS,
    SFR_PATH_DEBUG_FRAGPOS,
    SFR_PATH_COUNT
};

enum sfrRenderLightType {
    SFR_RLT_UNLIT,
    SFR_RLT_LIGHTMAP,
    SFR_RLT_DYNAMIC,
    SFR_RLT_LIGHTMAP_DYNAMIC,
    SFR_RLT_DEBUG_NORMALS,
    SFR_RLT_DEBUG_FRAGPOS
};

// only used once per visible pixel during the resolve phase
struct sfrShadeBin {
    // needed for barycentric weights
    f32 invDet;

    // the 3 verts and their attributes (resolver)
    struct sfrVertexData v0;
    struct sfrVertexData v1;
    struct sfrVertexData v2;

    u32 col;
    const SfrMaterial* mat;

    enum sfrRenderPath renderPathInd;
};

struct sfrTile {
    i32 minX, minY, maxX, maxY;
    f32 minInvZ; // furthest visible depth in the tile (stored natively as 1/Z)

    #ifdef SFR_MULTITHREADED
        SfrAtomic32 hasWork; // 1 if any thread binned a triangle here, 0 otherwise
    #endif
};

#ifdef SFR_MULTITHREADED
    struct sfrBinRef {
        i32 binIndex; // index into the thread's linear raster/shade bin arrays
        i32 nextRef;  // index of the next sfrBinRef for this tile (-1 if end of list)
    };

    struct SFR_ALIGNED(64) sfrThreadData {
        SfrThread handle;
        i32 threadInd;

        // flat arenas for bin data (thread local, grows as needed, resets every frame)
        struct sfrRasterBin* rasterBins;
        struct sfrShadeBin* shadeBins;
        i32 binCapacity;
        i32 binCount;

        // flat arena for tile references (linked list nodes)
        struct sfrBinRef* binRefs;
        i32 refCapacity;
        i32 refCount;

        // linked list heads/tails mapped to global tile indices
        i32* tileFirstRef; // size = sfrThreadBuf->tileCount
        i32* tileLastRef;  // size = sfrThreadBuf->tileCount

        // thread local rendering cache (moved out of sfrTile)
        SFR_ALIGNED(32) f32 localDepth[SFR_TILE_WIDTH * SFR_TILE_HEIGHT];
        SFR_ALIGNED(32) i32 localId[SFR_TILE_WIDTH * SFR_TILE_HEIGHT];
    };

    struct sfrMeshChunkJob {
        const f32* tris;
        const f32* uvs;
        const f32* lmUvs;
        const f32* normals;
        const f32* tangents;
        sfrmat matNormal;
        sfrmat matMVP;
        u32 col;
        const SfrMaterial* mat;
        i32 startTriangle;
        i32 triangleCount;
        i32 renderMode;
    };

    struct sfrThreadBuf {
        // tiling system data
        struct sfrTile* tiles;
        i32 tileCols, tileRows, tileCount, tileMax;

        // rasterizer work dispatch data
        i32* rasterWorkQueue;
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
        struct sfrThreadData threads[SFR_THREAD_COUNT + 1];
        SfrSemaphore geometryStartSem;
        SfrSemaphore geometryDoneSem;
        SfrSemaphore rasterStartSem;
        SfrSemaphore rasterDoneSem;
    };
#endif // SFR_MULTITHREADED

struct sfrState {
    sfrmat matInvView;
    sfrmat matNormal;
    u8 normalMatDirty;

    u32 randState;

    f32 halfWidth, halfHeight;
    sfrvec clipPlanes[4][2];

    SfrMaterial baseMat;

    SfrLight* lights;
    i32 lightCount, lightCap;

    SfrTexture* activeLightmap;
    SfrTexture* activeDirectionmap;

    SfrRenderTarget* activeShadowmap;
    SfrRenderTarget* savedShadowmap;
    sfrmat matLightVP;
    sfrmat savedMatLightVP;

    enum {
        SFR_RENDERMODE_UNLIT,        // no lighting, fullbright
        SFR_RENDERMODE_SHADED,       // dynamic lights or lightmap, lightmap takes priority
        SFR_RENDERMODE_SHADED_MIXED, // dynamic lights and lightmap
        SFR_RENDERMODE_NORMALS,      // unlit normals
        SFR_RENDERMODE_FRAGPOS,      // show frag position
        SFR_RENDERMODE_DEPTH_ONLY,   // for shadowmapping
        SFR_RENDERMODE_COUNT
    } renderMode;

#ifdef SFR_MULTITHREADED
    u8 shutdown;
#else
    struct sfrRasterBin* globalRasterBins;
    struct sfrShadeBin* globalShadeBins;
    i32 globalBinsCount, globalBinsCap;

    i32* idBuf;
    i32 idBufCap;
#endif
};

#ifndef SFR_NO_SIMD

struct sfrFragDataSimd {
    vf32 z;
    vf32 cu, cv;
    vf32 lu, lv;
    vf32 nx, ny, nz;
    vf32 fragX, fragY, fragZ;
    vf32 viewX, viewY, viewZ;
    vf32 shadow; // defaults to 1.f if shadows are disabled
};

struct sfrSurfacePropsSimd {
    vf32 albedoR, albedoG, albedoB;
    vf32 roughness, metallic;
    vf32 diffR, diffG, diffB;
    vf32 f0R, f0G, f0B;
    vf32 shininess;
    vf32 energyConservation;
};

struct sfrLightmapStateSimd {
    vf32 lmR, lmG, lmB;
    vf32 fx, fy;
    vi32 ind00;
};

struct sfrResolveChunkArgs {
    const struct sfrRasterBin* rBin;
    const struct sfrShadeBin* sBin;
    i32 globalX;
    i32 globalY;
    i32 globalInd;
    const vi32* pWriteMask;
};

#endif // !SFR_NO_SIMD

struct sfrFragData {
    f32 z;
    f32 cu, cv;
    f32 lu, lv;
    f32 shadow;
    sfrvec normal;
    sfrvec fragPos;
    sfrvec viewDir;
};

struct sfrSurfaceProps {
    f32 albedoR, albedoG, albedoB;
    f32 roughness, metallic;
    f32 diffR, diffG, diffB;
    f32 f0R, f0G, f0B;
    f32 shininess;
    f32 energyConservation;
};

struct sfrLightmapState {
    f32 lmR, lmG, lmB;
    i32 ind;
    f32 fx, fy;
};


//: global variables
static u32* sfrPixelBuf;
static f32* sfrDepthBuf;
static i32 sfrWidth, sfrHeight;

// for SfrDynamicMesh
static u8* sfrDynamicArena = NULL;
static u64 sfrDynamicArenaCap = 0;
static u64 sfrPendingArenaCap = 0; // tracks deferred growth to safely resize arena post present
static u64 sfrDynamicArenaOffset = 0;

static SfrRenderTarget sfrDefaultTarget = {0};
static SfrRenderTarget* sfrCurrTarget = &sfrDefaultTarget;
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

#define SFR__V(_xv, _yv, _zv, _wv) (sfrvec){ .x = (_xv), .y = (_yv), .z = (_zv), .w = (_wv) }
#define SFR__V0 SFR__V(0, 0, 0, 0)

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
        if (!(r)) { SFR__ERR_EXIT("malloc failed (%d bytes)\n", (i32)(s)); } \
        sfr_memset((r), 0, (s));

    #define SFR__FREE(p) \
        if (p) { sfrFree(p); (p) = NULL; }
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
        r.v = sfrvfs_add(a.v, b.v);
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
        r.v = sfrvfs_sub(a.v, b.v);
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
        const vf32s t = sfrvfs_set1(b);
        r.v = sfrvfs_mul(a.v, t);
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
        const vf32s t = sfrvfs_set1(b);
        r.v = sfrvfs_div(a.v, t);
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
        return sfrvfs_cvtss_f32(sfrvfs_dp(a.v, b.v, 0x71));
    #else
        return a.x * b.x + a.y * b.y + a.z * b.z;
    #endif
}

SFR_FUNC f32 sfr_vec_length(sfrvec v) {
    #ifndef SFR_NO_SIMD
        return sfrvfs_cvtss_f32(sfrvfs_sqrt_ss(sfrvfs_dp(v.v, v.v, 0x71)));
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
        const vf32s dp = sfrvfs_dp(v.v, v.v, 0x7F);
        return sfrvfs_cvtss_f32(dp) > SFR_EPSILON ?
            (sfrvec){ .v = sfrvfs_mul(v.v, sfrvfs_rsqrt(dp)) } :
            SFR__V0;
    #else
        const f32 l = sfr_vec_length(v);
        return l > SFR_EPSILON ? sfr_vec_mul(v, 1.f / l) : SFR__V0;
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

SFR_FUNC sfrvec sfr_vec_perpendicular(sfrvec v) {
    sfrvec res = {0};

    f32 min = sfr_fabsf(v.x);
    sfrvec cardinalAxis = { .x = 1.f, .y = 0.f, .z = 0.f };

    if (sfr_fabsf(v.y) < min) {
        min = sfr_fabsf(v.y);
        cardinalAxis = (sfrvec){ .x = 0.f, .y = 1.f, .z = 0.f };
    }

    if (sfr_fabsf(v.z) < min) {
        cardinalAxis = (sfrvec){ .x = 0.f, .y = 0.f, .z = 1.f };
    }

    res.x = v.y * cardinalAxis.z - v.z * cardinalAxis.y;
    res.y = v.z * cardinalAxis.x - v.x * cardinalAxis.z;
    res.z = v.x * cardinalAxis.y - v.y * cardinalAxis.x;

    return res;
}

// copied from raylib's Vector3RotateByQuaternion
SFR_FUNC sfrvec sfr_vec_rotate_by_quat(sfrvec v, sfrvec q) {
    sfrvec res = {0};
    res.x = v.x*(q.x*q.x + q.w*q.w - q.y*q.y - q.z*q.z) + v.y*(2*q.x*q.y - 2*q.w*q.z) + v.z*(2*q.x*q.z + 2*q.w*q.y);
    res.y = v.x*(2*q.w*q.z + 2*q.x*q.y) + v.y*(q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z) + v.z*(-2*q.w*q.x + 2*q.y*q.z);
    res.z = v.x*(-2*q.w*q.y + 2*q.x*q.z) + v.y*(2*q.w*q.x + 2*q.y*q.z)+ v.z*(q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    return res;
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

SFR_FUNC sfrmat sfr_mat_ortho(f32 left, f32 right, f32 bottom, f32 top, f32 near, f32 far) {
    sfrmat r = {0};
    r.m[0][0] = 2.f / (right - left);
    r.m[1][1] = 2.f / (top - bottom);
    r.m[2][2] = 1.f / (far - near);
    r.m[3][0] = -(right + left) / (right - left);
    r.m[3][1] = -(top + bottom) / (top - bottom);
    r.m[3][2] = -near / (far - near);
    r.m[3][3] = 1.f;
    return r;
}

SFR_FUNC sfrmat sfr_mat_mul(sfrmat a, sfrmat b) {
    sfrmat r;
    #ifndef SFR_NO_SIMD
        for (int i = 0; i < 4; i += 1) {
            const vf32s ar = a.rows[i].v;
            const vf32s x = sfrvfs_shuffle(ar, ar, SFR_SHUFFLE(0, 0, 0, 0));
            const vf32s y = sfrvfs_shuffle(ar, ar, SFR_SHUFFLE(1, 1, 1, 1));
            const vf32s z = sfrvfs_shuffle(ar, ar, SFR_SHUFFLE(2, 2, 2, 2));
            const vf32s w = sfrvfs_shuffle(ar, ar, SFR_SHUFFLE(3, 3, 3, 3));

            const vf32s r0 = sfrvfs_mul(x, b.rows[0].v);
            const vf32s r1 = sfrvfs_mul(y, b.rows[1].v);
            const vf32s r2 = sfrvfs_mul(z, b.rows[2].v);
            const vf32s r3 = sfrvfs_mul(w, b.rows[3].v);

            const vf32s sum01 = sfrvfs_add(r0, r1);
            const vf32s sum23 = sfrvfs_add(r2, r3);
            r.rows[i].v = sfrvfs_add(sum01, sum23);
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
        const vf32s vx = sfrvfs_shuffle(v.v, v.v, SFR_SHUFFLE(0, 0, 0, 0));
        const vf32s vy = sfrvfs_shuffle(v.v, v.v, SFR_SHUFFLE(1, 1, 1, 1));
        const vf32s vz = sfrvfs_shuffle(v.v, v.v, SFR_SHUFFLE(2, 2, 2, 2));
        const vf32s vw = sfrvfs_shuffle(v.v, v.v, SFR_SHUFFLE(3, 3, 3, 3));

        const vf32s mul0 = sfrvfs_mul(vx, m.rows[0].v);
        const vf32s mul1 = sfrvfs_mul(vy, m.rows[1].v);
        const vf32s mul2 = sfrvfs_mul(vz, m.rows[2].v);
        const vf32s mul3 = sfrvfs_mul(vw, m.rows[3].v);

        r.v = sfrvfs_add(sfrvfs_add(mul0, mul1),  sfrvfs_add(mul2, mul3));
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

SFR_FUNC sfrmat sfr_mat_model_look_at(sfrvec pos, sfrvec target, sfrvec up) {
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

SFR_FUNC sfrmat sfr_mat_view_look_at(sfrvec pos, sfrvec target, sfrvec up) {
    const sfrvec forward = sfr_vec_norm(sfr_vec_sub(target, pos));
    
    // safety check for looking straight down
    if (sfr_fabsf(forward.x) < 0.001f && sfr_fabsf(forward.z) < 0.001f) {
        up = SFR__V(0.f, 0.f, -1.f, 0.f);
    }

    up = sfr_vec_norm(sfr_vec_sub(up, sfr_vec_mul(forward, sfr_vec_dot(up, forward))));
    const sfrvec right = sfr_vec_cross(up, forward);

    sfrmat r;
    // row 1
    r.m[0][0] = right.x;
    r.m[0][1] = up.x;      // transposed
    r.m[0][2] = forward.x; // transposed
    r.m[0][3] = 0.f;
    // row 2
    r.m[1][0] = right.y;   // transposed
    r.m[1][1] = up.y;
    r.m[1][2] = forward.y; // transposed
    r.m[1][3] = 0.f;
    // row 3
    r.m[2][0] = right.z;   // transposed
    r.m[2][1] = up.z;      // transposed
    r.m[2][2] = forward.z;
    r.m[2][3] = 0.f;
    // row 4 (translation inverse)
    r.m[3][0] = -sfr_vec_dot(pos, right);
    r.m[3][1] = -sfr_vec_dot(pos, up);
    r.m[3][2] = -sfr_vec_dot(pos, forward);
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
    m.m[0][1] = xy - wz;
    m.m[0][2] = xz + wy;
    m.m[0][3] = 0.f;

    m.m[1][0] = xy + wz;
    m.m[1][1] = 1.f - (xx + zz);
    m.m[1][2] = yz - wx;
    m.m[1][3] = 0.f;

    m.m[2][0] = xz - wy;
    m.m[2][1] = yz + wx;
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

SFR_FUNC sfrvec sfr_quat_invert(sfrvec q) {
    sfrvec res = q;

    const f32 len2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (0.f != len2) {
        const f32 invLength = 1.f / len2;
        res.x *= -invLength;
        res.y *= -invLength;
        res.z *= -invLength;
        res.w *= invLength;
    }

    return res;
}

SFR_FUNC sfrvec sfr_quat_norm(sfrvec q) {
    const f32 len2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (0.f != len2) {
        const f32 invLen = 1.f / sfr_sqrtf(len2);
        q.x *= invLen;
        q.y *= invLen;
        q.z *= invLen;
        q.w *= invLen;
    }
    return q;
}


//================================================
//: PRIVATE FUNCTIONS
//================================================

// map [-1.0, 1.0] -> [0, 1023]
static inline u32 sfr__pack_vec3(f32 x, f32 y, f32 z) {
    const u32 px = (u32)((x * 0.5f + 0.5f) * 1023.f);
    const u32 py = (u32)((y * 0.5f + 0.5f) * 1023.f);
    const u32 pz = (u32)((z * 0.5f + 0.5f) * 1023.f);
    return px | (py << 10) | (pz << 20);
}
// map [0, 1023] -> [-1.0, 1.0]
static inline void sfr__unpack_vec3(u32 packed, f32* x, f32* y, f32* z) {
    *x = (f32)((packed >> 00) & 1023) * (2.f / 1023.f) - 1.f;
    *y = (f32)((packed >> 10) & 1023) * (2.f / 1023.f) - 1.f;
    *z = (f32)((packed >> 20) & 1023) * (2.f / 1023.f) - 1.f;
}

// tangent packing (10-10-10-1)
static inline u32 sfr__pack_tangent(f32 x, f32 y, f32 z, f32 w) {
    const u32 px = (u32)((x * 0.5f + 0.5f) * 1023.f);
    const u32 py = (u32)((y * 0.5f + 0.5f) * 1023.f);
    const u32 pz = (u32)((z * 0.5f + 0.5f) * 1023.f);
    const u32 pw = (w > 0.f) ? 1 : 0; // 1 for positive, 0 for negative
    return px | (py << 10) | (pz << 20) | (pw << 30);
}
static inline void sfr__unpack_tangent(u32 packed, f32* x, f32* y, f32* z, f32* w) {
    *x = (f32)(packed & 1023) * (2.f / 1023.f) - 1.f;
    *y = (f32)((packed >> 10) & 1023) * (2.f / 1023.f) - 1.f;
    *z = (f32)((packed >> 20) & 1023) * (2.f / 1023.f) - 1.f;
    *w = ((packed >> 30) & 1) ? 1.f : -1.f;
}

// uv packing (u16 UNORM)
static inline u16 sfr__pack_uv(f32 val) {
    // f32 clamped = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
    // return (u16)(clamped * 65535.0f + 0.5f);
    return (u16)(val * 65535.0f + 0.5f);
}

// uv unpacking (u16 UNORM)
static inline f32 sfr__unpack_uv(u16 val) {
    return (f32)val * 0.0000152590218f; 
}

static inline void sfr__mesh_swap_tri(SfrMesh* mesh, i32 a, i32 b) {
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

    // swap lm uvs
    if (mesh->lmUvs) {
        for (i32 i = 0; i < 6; i += 1) {
            SFR__SWAP(f32, mesh->lmUvs[a * 6 + i], mesh->lmUvs[b * 6 + i]);
        }
    }

    // swap normals
    if (mesh->normals) {
        for (i32 i = 0; i < 9; i += 1) {
            SFR__SWAP(f32, mesh->normals[a * 9 + i], mesh->normals[b * 9 + i]);
        }
    }

    // swap tangents
    if (mesh->tangents) {
        for (i32 i = 0; i < 12; i += 1) {
            SFR__SWAP(f32, mesh->tangents[a * 12 + i], mesh->tangents[b * 12 + i]);
        }
    }
}

static void sfr__mesh_ensure_channels(SfrMesh* mesh) {
    const i32 vertexCount = mesh->vertCount / 3;

    if (!mesh->uvs) {
        SFR__MALLOC(mesh->uvs, sizeof(f32) * vertexCount * 2);
    }

    if (!mesh->lmUvs) {
        SFR__MALLOC(mesh->lmUvs, sizeof(f32) * vertexCount * 2);
    }

    if (!mesh->normals) {
        SFR__MALLOC(mesh->normals, sizeof(f32) * vertexCount * 3);
    }

    if (!mesh->tangents) {
        SFR__MALLOC(mesh->tangents, sizeof(f32) * vertexCount * 4);
    }
}

static void sfr__mesh_compute_tangents(SfrMesh* mesh) {
    if (!mesh->uvs || !mesh->normals) {
        return;
    }

    const i32 vertexCount = mesh->vertCount / 3;
    if (!mesh->tangents) {
        SFR__MALLOC(mesh->tangents, sizeof(f32) * vertexCount * 4);
    }

    // arrays to accumulate tangents and bitangents before orthonormalization
    sfrvec *tan1, *tan2;
    SFR__MALLOC(tan1, sizeof(sfrvec) * vertexCount);
    SFR__MALLOC(tan2, sizeof(sfrvec) * vertexCount);
    sfr_memset(tan1, 0, sizeof(sfrvec) * vertexCount);
    sfr_memset(tan2, 0, sizeof(sfrvec) * vertexCount);

    for (i32 i = 0; i < vertexCount; i += 3) {
        // get positions
        const sfrvec p0 = SFR__V(mesh->tris[(i + 0) * 3 + 0], mesh->tris[(i + 0) * 3 + 1], mesh->tris[(i + 0) * 3 + 2], 0.f);
        const sfrvec p1 = SFR__V(mesh->tris[(i + 1) * 3 + 0], mesh->tris[(i + 1) * 3 + 1], mesh->tris[(i + 1) * 3 + 2], 0.f);
        const sfrvec p2 = SFR__V(mesh->tris[(i + 2) * 3 + 0], mesh->tris[(i + 2) * 3 + 1], mesh->tris[(i + 2) * 3 + 2], 0.f);

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

        const sfrvec t = SFR__V(
            f * (deltaV2 * edge1.x - deltaV1 * edge2.x),
            f * (deltaV2 * edge1.y - deltaV1 * edge2.y),
            f * (deltaV2 * edge1.z - deltaV1 * edge2.z),
            0.f
        );

        const sfrvec b = SFR__V(
            f * (-deltaU2 * edge1.x + deltaU1 * edge2.x),
            f * (-deltaU2 * edge1.y + deltaU1 * edge2.y),
            f * (-deltaU2 * edge1.z + deltaU1 * edge2.z),
            0.f
        );

        tan1[i + 0] = sfr_vec_add(tan1[i + 0], t);
        tan1[i + 1] = sfr_vec_add(tan1[i + 1], t);
        tan1[i + 2] = sfr_vec_add(tan1[i + 2], t);

        tan2[i + 0] = sfr_vec_add(tan2[i + 0], b);
        tan2[i + 1] = sfr_vec_add(tan2[i + 1], b);
        tan2[i + 2] = sfr_vec_add(tan2[i + 2], b);
    }

    // gram-schmidt orthogonalize
    for (i32 i = 0; i < vertexCount; i += 1) {
        const sfrvec n = SFR__V(mesh->normals[i * 3 + 0], mesh->normals[i * 3 + 1], mesh->normals[i * 3 + 2], 0.f);
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

    SFR__FREE(tan1);
    SFR__FREE(tan2);
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
static inline i32 sfr__fast_log2(f32 x) {
    union { f32 f; u32 u; } temp = { .f = x };
    return (i32)((temp.u >> 23) & 0xFF) - 127;
}

static inline f32 sfr__fast_inv_sqrt(f32 number) {
    union { f32 f; u32 i; } conv = { .f = number };
    const f32 x2 = number * 0.5f;
    conv.i  = 0x5f3759df - (conv.i >> 1);
    conv.f *= (1.5f - (x2 * conv.f * conv.f)); // 1st iteration
    conv.f *= (1.5f - (x2 * conv.f * conv.f)); // 2nd iteration for specular precision
    return conv.f;
}

// scalar equivalent of the SIMD version
static inline f32 sfr__fast_pow(f32 x, f32 y) {
    // clamp base to prevent log2 of zero/negatives
    const f32 cx = sfr_fmaxf(x, SFR_EPSILON);
    
    // fast log2
    union { f32 f; i32 i; } u_log = { .f = cx };
    const f32 log2x = (f32)(u_log.i - 0x3F800000) * 1.1920928955078125e-7f;
    
    // fast exp2
    f32 exp2Val = y * log2x;
    exp2Val = sfr_fmaxf(exp2Val, -126.f); // prevent underflow
    
    union { f32 f; i32 i; } uexp;
    uexp.i = (i32)(exp2Val * 8388608.f) + 0x3F800000;
    
    return uexp.f;
}

static inline enum sfrRenderPath sfr__get_render_path(const SfrMaterial* mat, i32 renderMode) {
    if (SFR_RENDERMODE_NORMALS == renderMode) return SFR_PATH_DEBUG_NORMALS;
    if (SFR_RENDERMODE_FRAGPOS == renderMode) return SFR_PATH_DEBUG_FRAGPOS;

    const u8 hasTex = (mat && NULL != mat->albedoTex);
    const u8 hasPbr = (hasTex && NULL != mat->metallicRoughnessTex);
    const u8 hasShadows = (sfrState.activeShadowmap && NULL != sfrState.activeShadowmap->depth);

    if (SFR_RENDERMODE_UNLIT == renderMode) {
        return hasTex ? SFR_PATH_UNLIT_TEX : SFR_PATH_UNLIT_COLOR;
    }

    if (sfrState.activeLightmap) {
        if (SFR_RENDERMODE_SHADED_MIXED == renderMode && sfrState.lightCount > 0) {
            if (hasShadows) {
                if (sfrState.activeDirectionmap && hasPbr) return SFR_PATH_LIGHTMAP_DYNAMIC_PBR_SHADOW;
                if (hasTex) return SFR_PATH_LIGHTMAP_DYNAMIC_TEX_SHADOW;
                return SFR_PATH_LIGHTMAP_DYNAMIC_COLOR_SHADOW;
            } else {
                if (sfrState.activeDirectionmap && hasPbr) return SFR_PATH_LIGHTMAP_DYNAMIC_PBR;
                if (hasTex) return SFR_PATH_LIGHTMAP_DYNAMIC_TEX;
                return SFR_PATH_LIGHTMAP_DYNAMIC_COLOR;
            }
        } else {
            if (hasShadows) {
                if (sfrState.activeDirectionmap && hasPbr) return SFR_PATH_LIGHTMAP_PBR_SHADOW;
                if (hasTex) return SFR_PATH_LIGHTMAP_TEX_SHADOW;
                return SFR_PATH_LIGHTMAP_COLOR_SHADOW;
            } else {
                if (sfrState.activeDirectionmap && hasPbr) return SFR_PATH_LIGHTMAP_PBR;
                if (hasTex) return SFR_PATH_LIGHTMAP_TEX;
                return SFR_PATH_LIGHTMAP_COLOR;
            }
        }
    } else if (SFR_RENDERMODE_SHADED == renderMode && sfrState.lightCount > 0) {
        if (hasShadows) {
            return hasTex ? SFR_PATH_DYNAMIC_TEX_SHADOW : SFR_PATH_DYNAMIC_COLOR_SHADOW;
        } else {
            return hasTex ? SFR_PATH_DYNAMIC_TEX : SFR_PATH_DYNAMIC_COLOR;
        }
    }
    
    return hasTex ? SFR_PATH_UNLIT_TEX : SFR_PATH_UNLIT_COLOR;
}

static inline struct sfrTexVert sfr__lerp_vert(struct sfrTexVert a, struct sfrTexVert b, f32 t) {
    return (struct sfrTexVert){
        .pos = sfr_vec_lerp(a.pos, b.pos, t),
        .viewZ = SFR__LERPF(a.viewZ, b.viewZ, t),
        .u = SFR__LERPF(a.u, b.u, t),
        .v = SFR__LERPF(a.v, b.v, t),
        .lu = SFR__LERPF(a.lu, b.lu, t),
        .lv = SFR__LERPF(a.lv, b.lv, t),
        .normal = sfr_vec_lerp(a.normal, b.normal, t),
        .tangent = sfr_vec_lerp(a.tangent, b.tangent, t)
    };
}

static inline i32 sfr__clip_tri_homogeneous(struct sfrTexVert out[restrict 2][3], sfrvec plane, const struct sfrTexVert in[3]) {
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

static void sfr__rasterize_bin(const struct sfrRasterBin* bin, struct sfrTile* tile, f32* targetDepthBuf, i32* targetIdBuf) {
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

    // vv precompute max offsets for trivial rejection, min calculated lower for scalar vv
    const f32 e0MaxOffset = sfr_fmaxf(0.f, A0 * SFR_SIMD_LANE_MAX_OFFSET) + sfr_fmaxf(0.f, B0 * SFR_SIMD_LANE_MAX_OFFSET);
    const f32 e1MaxOffset = sfr_fmaxf(0.f, A1 * SFR_SIMD_LANE_MAX_OFFSET) + sfr_fmaxf(0.f, B1 * SFR_SIMD_LANE_MAX_OFFSET);
    const f32 e2MaxOffset = sfr_fmaxf(0.f, A2 * SFR_SIMD_LANE_MAX_OFFSET) + sfr_fmaxf(0.f, B2 * SFR_SIMD_LANE_MAX_OFFSET);

#ifndef SFR_NO_SIMD
    if (maxX - minX <= 4 && maxY - minY <= 4) {
        goto SCALAR; // SIMD will do too much wasted work so use scalar for tiny triangles
    }

    // align minX to 8 for SIMD
    const i32 alignedMinX = minX & ~7;

    // vv setup vectors vv
    const vf32 vA0 = sfrvf_set1(A0), vB0 = sfrvf_set1(B0), vC0 = sfrvf_set1(C0);
    const vf32 vA1 = sfrvf_set1(A1), vB1 = sfrvf_set1(B1), vC1 = sfrvf_set1(C1);
    const vf32 vA2 = sfrvf_set1(A2), vB2 = sfrvf_set1(B2), vC2 = sfrvf_set1(C2);

    const vf32 vZBase = sfrvf_set1(zBase), vdzdx = sfrvf_set1(dzdx), vdzdy = sfrvf_set1(dzdy);

    const vf32 vXOffsets = SFR_SIMD_SET_HALF_STEPS();
    const vi32 vIndex = SFR_SIMD_SET_INDICES();

    const vi32 vShadeId = sfrvi_set1(bin->shadeId);

    for (i32 yBase = minY; yBase < maxY; yBase += SFR_SIMD_LANES) {
        const f32 fy = (f32)(yBase - bin->minY) + 0.5f;
        const vf32 vy = sfrvf_set1(fy);

        // trivial rejection test
        for (i32 xBase = alignedMinX; xBase < maxX; xBase += SFR_SIMD_LANES) {
            const f32 fx = (f32)(xBase - bin->minX);

            // trivial rejection
            if (C0 + A0 * (fx + 0.5f) + B0 * fy + e0MaxOffset < 0.f) continue;
            if (C1 + A1 * (fx + 0.5f) + B1 * fy + e1MaxOffset < 0.f) continue;
            if (C2 + A2 * (fx + 0.5f) + B2 * fy + e2MaxOffset < 0.f) continue;

            const vf32 vXBlock = sfrvf_add(sfrvf_set1(fx), vXOffsets);

            // start values at (xBase, yBase)
            vf32 vE0 = sfrvf_add(vC0, sfrvf_add(sfrvf_mul(vA0, vXBlock), sfrvf_mul(vB0, vy)));
            vf32 vE1 = sfrvf_add(vC1, sfrvf_add(sfrvf_mul(vA1, vXBlock), sfrvf_mul(vB1, vy)));
            vf32 vE2 = sfrvf_add(vC2, sfrvf_add(sfrvf_mul(vA2, vXBlock), sfrvf_mul(vB2, vy)));

            // vz is completely native 1/Z now
            vf32 vz  = sfrvf_add(vZBase, sfrvf_add(sfrvf_mul(vdzdx, vXBlock), sfrvf_mul(vdzdy, vy)));

            // constants to step y by 1 for the inner 8 lines
            const vf32 vB0Step = vB0, vB1Step = vB1, vB2Step = vB2;
            const vf32 vdZdyStep = vdzdy;

            // limit y to tile bounds
            const i32 yLimit = SFR_MIN(yBase + SFR_SIMD_LANES, maxY);

            for (i32 y = yBase; y < yLimit; y += 1) {
                // determine coverage
                vf32 mask = sfrvf_and(
                    sfrvf_cmp(vE0, sfrvf_zero(), SFR_CMP_GE_OQ),
                    sfrvf_and(
                        sfrvf_cmp(vE1, sfrvf_zero(), SFR_CMP_GE_OQ),
                        sfrvf_cmp(vE2, sfrvf_zero(), SFR_CMP_GE_OQ)
                    )
                );

                // alignment check
                if (xBase < minX || xBase + SFR_SIMD_LANES > maxX) {
                    const vi32 vGlobalX = sfrvi_add(sfrvi_set1(xBase), vIndex);
                    const vf32 vBoundMask = sfrvf_and(
                        sfrvf_cmp(sfrvf_cvtepi32(vGlobalX), sfrvf_set1((f32)minX), SFR_CMP_GE_OQ),
                        sfrvf_cmp(sfrvf_cvtepi32(vGlobalX), sfrvf_set1((f32)maxX), SFR_CMP_LT_OQ)
                    );
                    mask = sfrvf_and(mask, vBoundMask);
                }

                i32 maskInt = sfrvf_movemask(mask);
                if (maskInt) {
                    #ifdef SFR_MULTITHREADED
                        const i32 localY = y - tile->minY;
                        const i32 localX = xBase - tile->minX;
                        const i32 pixelInd = localY * SFR_TILE_WIDTH + localX;
                    #else
                        const i32 pixelInd = y * sfrWidth + xBase;
                    #endif
                    const vf32 vOldDepth = sfrvf_loadu(&targetDepthBuf[pixelInd]);

                    // closer objects have larger 1/Z values
                    const vf32 depthMask = sfrvf_cmp(vz, vOldDepth, SFR_CMP_GT_OQ);
                    mask = sfrvf_and(mask, depthMask);
                    maskInt = sfrvf_movemask(mask);

                    if (SFR_SIMD_FULL_LANE_MASK == maskInt) {
                        // 100% coverage so skip the load/blend and store
                        sfrvf_storeu(&targetDepthBuf[pixelInd], vz);
                        if (targetIdBuf) {
                            sfrvi_storeu((vi32*)&targetIdBuf[pixelInd], vShadeId);
                        }
                    } else if (maskInt) {
                        const vf32 vNewDepth = sfrvf_blendv(vOldDepth, vz, mask);
                        sfrvf_storeu(&targetDepthBuf[pixelInd], vNewDepth);
                        if (targetIdBuf) {
                            sfrvi_maskstore((int*)&targetIdBuf[pixelInd], sfrvi_cast_from_vf32(mask), vShadeId);
                        }
                    }
                }

                // step y by 1 for the next row in this block
                vE0 = sfrvf_add(vE0, vB0Step);
                vE1 = sfrvf_add(vE1, vB1Step);
                vE2 = sfrvf_add(vE2, vB2Step);
                vz = sfrvf_add(vz, vdZdyStep);
            }
        }
    }

    return;

    SCALAR:;
#endif

    // precompute max/min offsets for 8x8 block
    // since evaluated at top left pixel (offset 0), furthest pixel is at +7x +7y
    const f32 e0MinOffset = sfr_fminf(0.f, A0 * 7.f) + sfr_fminf(0.f, B0 * 7.f);
    const f32 e1MinOffset = sfr_fminf(0.f, A1 * 7.f) + sfr_fminf(0.f, B1 * 7.f);
    const f32 e2MinOffset = sfr_fminf(0.f, A2 * 7.f) + sfr_fminf(0.f, B2 * 7.f);

    // iterate in 8x8 blocks
    for (i32 yBase = minY; yBase < maxY; yBase += 8) {
        const f32 fyBase = (f32)(yBase - bin->minY) + 0.5f;
        const i32 yLimit = SFR_MIN(yBase + 8, maxY);

        for (i32 xBase = minX; xBase < maxX; xBase += 8) {
            const f32 fxBase = (f32)(xBase - bin->minX) + 0.5f;
            const i32 xLimit = SFR_MIN(xBase + 8, maxX);

            // base edge equations at the top left pixel of this block
            const f32 E0base = C0 + A0 * fxBase + B0 * fyBase;
            const f32 E1base = C1 + A1 * fxBase + B1 * fyBase;
            const f32 E2base = C2 + A2 * fxBase + B2 * fyBase;

            // trivial rejection, if the max possible value inside the block is < 0, the whole block is outside
            if (E0base + e0MaxOffset < 0.f || 
                E1base + e1MaxOffset < 0.f || 
                E2base + e2MaxOffset < 0.f
            ) {
                continue; 
            }

            // trivial Accept, if the min possible value inside the block is >= 0, the whole block is inside.
            const u8 trivialAccept =
                E0base + e0MinOffset >= 0.f && 
                E1base + e1MinOffset >= 0.f && 
                E2base + e2MinOffset >= 0.f;

            f32 zRow = zBase + dzdx * fxBase + dzdy * fyBase;

            if (trivialAccept) {
                // fast path, no edge equations evaluated per pixel just Z testing
                for (i32 y = yBase; y < yLimit; y += 1) {
                    f32 z = zRow;

                    #ifdef SFR_MULTITHREADED
                        const i32 localY = y - tile->minY;
                        i32 pixelInd = localY * SFR_TILE_WIDTH + (xBase - tile->minX);
                    #else
                        i32 pixelInd = y * sfrWidth + xBase;
                    #endif

                    for (i32 x = xBase; x < xLimit; x += 1, z += dzdx, pixelInd += 1) {
                        if (z > targetDepthBuf[pixelInd]) {
                            targetDepthBuf[pixelInd] = z;
                            if (targetIdBuf) {
                                targetIdBuf[pixelInd] = bin->shadeId;
                            }
                        }
                    }
                    zRow += dzdy;
                }
            } else {
                // partial path, evaluate edge equations but only for the pixels inside this specific 8x8 block
                f32 E0row = E0base;
                f32 E1row = E1base;
                f32 E2row = E2base;

                for (i32 y = yBase; y < yLimit; y += 1) {
                    f32 E0 = E0row;
                    f32 E1 = E1row;
                    f32 E2 = E2row;
                    f32 z = zRow;

                    #ifdef SFR_MULTITHREADED
                        const i32 localY = y - tile->minY;
                        i32 pixelInd = localY * SFR_TILE_WIDTH + (xBase - tile->minX);
                    #else
                        i32 pixelInd = y * sfrWidth + xBase;
                    #endif

                    for (i32 x = xBase; x < xLimit; x += 1, E0 += A0, E1 += A1, E2 += A2, z += dzdx, pixelInd += 1) {
                        if (E0 >= 0.f && E1 >= 0.f && E2 >= 0.f) {
                            if (z > targetDepthBuf[pixelInd]) {
                                targetDepthBuf[pixelInd] = z;
                                if (targetIdBuf) {
                                    targetIdBuf[pixelInd] = bin->shadeId;
                                }
                            }
                        }
                    }
                    E0row += B0;
                    E1row += B1;
                    E2row += B2;
                    zRow += dzdy;
                }
            }
        }
    }
}

static void sfr__bin_triangle(
    f32 ax, f32 ay, f32 bx, f32 by, f32 cx, f32 cy,
    const struct sfrVertexData* v0,
    const struct sfrVertexData* v1,
    const struct sfrVertexData* v2,
    u32 col, const SfrMaterial* mat,
    i32 renderMode
) {
    const f32 det = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
    #ifndef SFR_NO_CULLING
        // cull backfaces and sliver triangles from both fast and slow paths
        if (det <= 0.01f) { 
            return;
        }
    #else
        // cull slivers
        if (sfr_fabsf(det) < 0.01f) {
            return;
        }
        // if culling is disabled, swap the winding order of backfaces
        // or edge equations invert and rasterize the bounding box
        if (det < 0.f) {
            SFR__SWAP(f32, bx, cx);
            SFR__SWAP(f32, by, cy);
            const struct sfrVertexData* temp = v1;
            v1 = v2;
            v2 = temp;
            det = -det; // flip det to positive
        }
    #endif
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
    struct sfrThreadData* tData = &sfrThreadBuf->threads[sfrTlsThreadInd];

    if (tData->binCount >= tData->binCapacity) {
        tData->binCapacity = (0 == tData->binCapacity) ? 4096 : tData->binCapacity * 2;
        tData->rasterBins = (struct sfrRasterBin*)sfrRealloc(tData->rasterBins, sizeof(struct sfrRasterBin) * tData->binCapacity);
        tData->shadeBins  = (struct sfrShadeBin*)sfrRealloc(tData->shadeBins,  sizeof(struct sfrShadeBin)  * tData->binCapacity);
    }

    const i32 globalInd = tData->binCount;
    tData->binCount += 1;

    struct sfrRasterBin* rBin = &tData->rasterBins[globalInd];
    struct sfrShadeBin* sBin = &tData->shadeBins[globalInd];
#else
    if (sfrState.globalBinsCount >= sfrState.globalBinsCap) {
        sfrState.globalBinsCap = (i32)(sfrState.globalBinsCap * 1.5f);
        sfrState.globalRasterBins = (struct sfrRasterBin*)sfrRealloc(sfrState.globalRasterBins,
            sizeof(struct sfrRasterBin) * (u64)sfrState.globalBinsCap);
        sfrState.globalShadeBins = (struct sfrShadeBin*)sfrRealloc(sfrState.globalShadeBins,
            sizeof(struct sfrShadeBin) * (u64)sfrState.globalBinsCap);
    }
    struct sfrRasterBin* rBin = &sfrState.globalRasterBins[sfrState.globalBinsCount];
    struct sfrShadeBin* sBin = &sfrState.globalShadeBins[sfrState.globalBinsCount];
    
    // assign singlethreaded global id
    const i32 globalInd = sfrState.globalBinsCount;
    sfrState.globalBinsCount += 1;
#endif

    // populate rasterizer data
    *rBin = (struct sfrRasterBin){
        .shadeId =
        #ifdef SFR_MULTITHREADED
            (sfrTlsThreadInd << 24) | globalInd,
        #else
            globalInd,
        #endif
        .dzdx = dzdx, .dzdy = dzdy, .zBase = zBase,
        .A0 = A0, .B0 = B0, .C0 = C0,
        .A1 = A1, .B1 = B1, .C1 = C1,
        .A2 = A2, .B2 = B2, .C2 = C2,
        .minX = (i16)minX, .maxX = (i16)maxX,
        .minY = (i16)minY, .maxY = (i16)maxY
    };

    // populate shading data
    *sBin = (struct sfrShadeBin){
        .invDet = invDet,
        .v0 = *v0, .v1 = *v1, .v2 = *v2,
        .col = (mat ? (mat->albedoTex ? col : mat->baseColor) : col),
        .mat = mat,
        .renderPathInd = sfr__get_render_path(mat, renderMode)
    };

#ifdef SFR_MULTITHREADED

    const i32 xStart = (i32)minX / SFR_TILE_WIDTH;
    const i32 xEnd   = (i32)maxX / SFR_TILE_WIDTH;
    const i32 yStart = (i32)minY / SFR_TILE_HEIGHT;
    const i32 yEnd   = (i32)maxY / SFR_TILE_HEIGHT;

    for (i32 ty = yStart; ty <= yEnd; ty += 1) {
        for (i32 tx = xStart; tx <= xEnd; tx += 1) {
            const i32 tileInd = ty * sfrThreadBuf->tileCols + tx;

            // thread local resize for the linked list reference arena
            if (tData->refCount >= tData->refCapacity) {
                tData->refCapacity = (0 == tData->refCapacity) ? 4096 : tData->refCapacity * 2;
                tData->binRefs = (struct sfrBinRef*)sfrRealloc(tData->binRefs, sizeof(struct sfrBinRef) * tData->refCapacity);
            }

            // allocate a reference node
            const i32 refInd = tData->refCount;
            tData->refCount += 1;
            
            tData->binRefs[refInd].binIndex = globalInd;
            tData->binRefs[refInd].nextRef = -1;

            // link it into this thread's chain for this specific tile
            const i32 lastRef = tData->tileLastRef[tileInd];
            if (-1 != lastRef) {
                tData->binRefs[lastRef].nextRef = refInd;
            } else {
                tData->tileFirstRef[tileInd] = refInd;
            }
            tData->tileLastRef[tileInd] = refInd;
        }
    }
#else
    sfrRasterCount += 1;
    struct sfrTile fullTile = {
        .minX = 0, .minY = 0,
        .maxX = sfrWidth, .maxY = sfrHeight,
        .minInvZ = 0.f
    };
    i32* idBuf = (SFR_RENDERMODE_DEPTH_ONLY == sfrState.renderMode) ? NULL : sfrState.idBuf;
    sfr__rasterize_bin(rBin, &fullTile, sfrDepthBuf, idBuf);
#endif
}

static inline void sfr__process_and_bin_triangle(
    const sfrmat* restrict matMVP, const sfrmat* restrict matNormal,
    const struct sfrGeomTri* gt
) {
    const sfrvec ap = SFR__V(gt->posA[0], gt->posA[1], gt->posA[2], 1.f);
    const sfrvec bp = SFR__V(gt->posB[0], gt->posB[1], gt->posB[2], 1.f);
    const sfrvec cp = SFR__V(gt->posC[0], gt->posC[1], gt->posC[2], 1.f);

    const sfrvec aClip = sfr_mat_mul_vec(*matMVP, ap);
    const sfrvec bClip = sfr_mat_mul_vec(*matMVP, bp);
    const sfrvec cClip = sfr_mat_mul_vec(*matMVP, cp);

    const SfrMaterial* matToUse = gt->mat ? gt->mat : &sfrState.baseMat;

    // if (aClip.w > SFR_EPSILON && bClip.w > SFR_EPSILON && cClip.w > SFR_EPSILON) {
    //  doesn't work because when only checking W > 0 triangles that intersect the near plane (clipZ < 0)
    //  bypass the clipper and go straight to the fast path as long as they're still in front of the
    //  camera eye (W > 0), but Z >= 0 enforces the near plane, if a triangle intersects the near plane
    //  it fails this check and is routed to the slow path, and Z <= W enforces the far plane, with
    //  X and Y intentionally omitted because they're already hanlded during the guard band clipping
    if ((aClip.z >= 0.f) & (aClip.z <= aClip.w) &
        (bClip.z >= 0.f) & (bClip.z <= bClip.w) &
        (cClip.z >= 0.f) & (cClip.z <= cClip.w)
    ) {
        #ifndef SFR_NO_CULLING
            // homogeneous backface culling before the perspective divide
            const f32 homoArea = 
                (bClip.x * aClip.w - aClip.x * bClip.w) * (cClip.y * aClip.w - aClip.y * cClip.w) - 
                (bClip.y * aClip.w - aClip.y * bClip.w) * (cClip.x * aClip.w - aClip.x * cClip.w);

            if (homoArea >= 0.f) {
                return;
            }
        #endif

        // Now we only pay for the division if the triangle is facing the camera!
        const f32 aInvW = 1.f / aClip.w, bInvW = 1.f / bClip.w, cInvW = 1.f / cClip.w;

        const f32 sax =  (aClip.x * aInvW + 1.f) * sfrState.halfWidth;
        const f32 say = (-aClip.y * aInvW + 1.f) * sfrState.halfHeight;
        const f32 sbx =  (bClip.x * bInvW + 1.f) * sfrState.halfWidth;
        const f32 sby = (-bClip.y * bInvW + 1.f) * sfrState.halfHeight;
        const f32 scx =  (cClip.x * cInvW + 1.f) * sfrState.halfWidth;
        const f32 scy = (-cClip.y * cInvW + 1.f) * sfrState.halfHeight;

        // screen space culling
        const f32 area = (sbx - sax) * (scy - say) - (sby - say) * (scx - sax);

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

        // transform normals
        const sfrvec na = sfr_mat_mul_vec(*matNormal, SFR__V(gt->normA[0], gt->normA[1], gt->normA[2], 0.f));
        const sfrvec nb = sfr_mat_mul_vec(*matNormal, SFR__V(gt->normB[0], gt->normB[1], gt->normB[2], 0.f));
        const sfrvec nc = sfr_mat_mul_vec(*matNormal, SFR__V(gt->normC[0], gt->normC[1], gt->normC[2], 0.f));

        // transform tangents and preserve w
        sfrvec ta = {0}, tb = {0}, tc = {0};
        ta = sfr_mat_mul_vec(*matNormal, SFR__V(gt->tanA[0], gt->tanA[1], gt->tanA[2], 0.f)); ta.w = gt->tanA[3];
        tb = sfr_mat_mul_vec(*matNormal, SFR__V(gt->tanB[0], gt->tanB[1], gt->tanB[2], 0.f)); tb.w = gt->tanB[3];
        tc = sfr_mat_mul_vec(*matNormal, SFR__V(gt->tanC[0], gt->tanC[1], gt->tanC[2], 0.f)); tc.w = gt->tanC[3];

        // if projection matrix m[3][3] is 1, it is orthogrpahic
        f32 aDepth = aInvW, bDepth = bInvW, cDepth = cInvW;
        if (1.f == sfrMatProj.m[3][3]) {
            aDepth = 1.f - aClip.z;
            bDepth = 1.f - bClip.z;
            cDepth = 1.f - cClip.z;
        }

        const struct sfrVertexData v0 = {
            .invZ = aDepth,
            .u = gt->uvA[0], .v = gt->uvA[1],
            .lu = sfr__pack_uv(gt->lmUvA[0]), .lv = sfr__pack_uv(gt->lmUvA[1]),
            .n = sfr__pack_vec3(na.x, na.y, na.z),
            .t = sfr__pack_tangent(ta.x, ta.y, ta.z, ta.w)
        };
        const struct sfrVertexData v1 = {
            .invZ = bDepth,
            .u = gt->uvB[0], .v = gt->uvB[1],
            .lu = sfr__pack_uv(gt->lmUvB[0]), .lv = sfr__pack_uv(gt->lmUvB[1]),
            .n = sfr__pack_vec3(nb.x, nb.y, nb.z),
            .t = sfr__pack_tangent(tb.x, tb.y, tb.z, tb.w)
        };
        const struct sfrVertexData v2 = {
            .invZ = cDepth,
            .u = gt->uvC[0], .v = gt->uvC[1],
            .lu = sfr__pack_uv(gt->lmUvC[0]), .lv = sfr__pack_uv(gt->lmUvC[1]),
            .n = sfr__pack_vec3(nc.x, nc.y, nc.z),
            .t = sfr__pack_tangent(tc.x, tc.y, tc.z, tc.w)
        };

        sfr__bin_triangle(sax, say, sbx, sby, scx, scy, &v0, &v1, &v2, gt->col, matToUse, gt->renderMode);
        return;
    }

    // slow path, intersects near plane and needs clipping
    struct sfrTexVert (*input)[3] = gt->clipBuf->clipTris;
    i32 inputCount = 1;

    // transform normals
    const sfrvec na = sfr_mat_mul_vec(*matNormal, SFR__V(gt->normA[0], gt->normA[1], gt->normA[2], 0.f));
    const sfrvec nb = sfr_mat_mul_vec(*matNormal, SFR__V(gt->normB[0], gt->normB[1], gt->normB[2], 0.f));
    const sfrvec nc = sfr_mat_mul_vec(*matNormal, SFR__V(gt->normC[0], gt->normC[1], gt->normC[2], 0.f));

    // transform tangents and preserve w
    sfrvec ta = {0}, tb = {0}, tc = {0};
    ta = sfr_mat_mul_vec(*matNormal, SFR__V(gt->tanA[0], gt->tanA[1], gt->tanA[2], 0.f)); ta.w = gt->tanA[3];
    tb = sfr_mat_mul_vec(*matNormal, SFR__V(gt->tanB[0], gt->tanB[1], gt->tanB[2], 0.f)); tb.w = gt->tanB[3];
    tc = sfr_mat_mul_vec(*matNormal, SFR__V(gt->tanC[0], gt->tanC[1], gt->tanC[2], 0.f)); tc.w = gt->tanC[3];

    input[0][0] = (struct sfrTexVert){
        .pos = aClip,
        .u = gt->uvA[0], .v = gt->uvA[1],
        .lu = gt->lmUvA[0], .lv = gt->lmUvA[1],
        .normal = na, .tangent = ta,
        .viewZ = aClip.w
    };
    input[0][1] = (struct sfrTexVert){
        .pos = bClip,
        .u = gt->uvB[0], .v = gt->uvB[1],
        .lu = gt->lmUvB[0], .lv = gt->lmUvB[1],
        .normal = nb, .tangent = tb,
        .viewZ = bClip.w
    };
    input[0][2] = (struct sfrTexVert){
        .pos = cClip,
        .u = gt->uvC[0], .v = gt->uvC[1],
        .lu = gt->lmUvC[0], .lv = gt->lmUvC[1],
        .normal = nc, .tangent = tc,
        .viewZ = cClip.w
    };

    // frustum planes in homogeneous clip space
    const sfrvec frustumPlanes[6] = {
        SFR__V(0.f, 0.f, 1.f, 0.f),  // near
        SFR__V(0.f, 0.f, -1.f, 1.f), // far
        SFR__V(1.f, 0.f, 0.f, 1.f),  // left
        SFR__V(-1.f, 0.f, 0.f, 1.f), // right
        SFR__V(0.f, 1.f, 0.f, 1.f),  // bottom
        SFR__V(0.f, -1.f, 0.f, 1.f)  // top
    };

    struct sfrTexVert (*output)[3] = gt->clipBuf->buffer;

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
                skip = 1;
                break;
            }
            const f32 iw = 1.f / tri[j].pos.w;
            screen[j].pos.x =  ((tri[j].pos.x * iw) + 1.f) * sfrState.halfWidth;
            screen[j].pos.y = (-(tri[j].pos.y * iw) + 1.f) * sfrState.halfHeight;
            screen[j].u = tri[j].u;
            screen[j].v = tri[j].v;
            screen[j].lu = tri[j].lu;
            screen[j].lv = tri[j].lv;
            screen[j].viewZ = tri[j].viewZ;
            screen[j].normal = tri[j].normal;
            screen[j].tangent = tri[j].tangent;
        }

        if (skip) {
            continue;
        }

        const f32 aInvZ = 1.f / screen[0].viewZ, bInvZ = 1.f / screen[1].viewZ, cInvZ = 1.f / screen[2].viewZ;

        f32 aDepth = aInvZ, bDepth = bInvZ, cDepth = cInvZ;
        if (1.f == sfrMatProj.m[3][3]) {
            aDepth = 1.f - aClip.z;
            bDepth = 1.f - bClip.z;
            cDepth = 1.f - cClip.z;
        }

        const struct sfrVertexData v0 = {
            .invZ = aDepth,
            .u = screen[0].u, .v = screen[0].v,
            .lu = sfr__pack_uv(screen[0].lu), .lv = sfr__pack_uv(screen[0].lv),
            .n = sfr__pack_vec3(screen[0].normal.x, screen[0].normal.y, screen[0].normal.z),
            .t = sfr__pack_tangent(screen[0].tangent.x, screen[0].tangent.y, screen[0].tangent.z, screen[0].tangent.w)
        };
        const struct sfrVertexData v1 = {
            .invZ = bDepth,
            .u = screen[1].u, .v = screen[1].v,
            .lu = sfr__pack_uv(screen[1].lu), .lv = sfr__pack_uv(screen[1].lv),
            .n = sfr__pack_vec3(screen[1].normal.x, screen[1].normal.y, screen[1].normal.z),
            .t = sfr__pack_tangent(screen[1].tangent.x, screen[1].tangent.y, screen[1].tangent.z, screen[1].tangent.w)
        };
        const struct sfrVertexData v2 = {
            .invZ = cDepth,
            .u = screen[2].u, .v = screen[2].v,
            .lu = sfr__pack_uv(screen[2].lu), .lv = sfr__pack_uv(screen[2].lv),
            .n = sfr__pack_vec3(screen[2].normal.x, screen[2].normal.y, screen[2].normal.z),
            .t = sfr__pack_tangent(screen[2].tangent.x, screen[2].tangent.y, screen[2].tangent.z, screen[2].tangent.w)
        };

        sfr__bin_triangle(
            screen[0].pos.x, screen[0].pos.y,
            screen[1].pos.x, screen[1].pos.y,
            screen[2].pos.x, screen[2].pos.y,
            &v0, &v1, &v2, gt->col, matToUse,
            gt->renderMode
        );
    }
}

static inline void sfr__get_bins(u32 id, const struct sfrRasterBin** outRasterBin, const struct sfrShadeBin** outShadeBin) {
    #ifdef SFR_MULTITHREADED
        // unpack thread index (high 8 bits) and local bin index (low 24 bits)
        const i32 threadInd = (id >> 24) & 0xFF;
        const i32 binIndex  = id & 0xFFFFFF;

        const struct sfrThreadData* sourceThread = &sfrThreadBuf->threads[threadInd];
        *outRasterBin = &sourceThread->rasterBins[binIndex];
        *outShadeBin = &sourceThread->shadeBins[binIndex];
    #else
        *outRasterBin = &sfrState.globalRasterBins[id];
        *outShadeBin = &sfrState.globalShadeBins[id];
    #endif
}

static inline i32 sfr__calc_lod(
    const struct sfrRasterBin* rBin,
    const struct sfrShadeBin* sBin,
    i32 globalX, i32 globalY
) {
    const SfrMaterial* mat = sBin->mat;
    if (!mat || !mat->albedoTex) {
        return 0;
    }

    // centers
    #ifndef SFR_NO_SIMD
        const f32 ccx = (f32)(globalX - rBin->minX) + 0.5f;
    #else
        const f32 ccx = (f32)(globalX - rBin->minX) + (SFR_SIMD_LANE_MAX_OFFSET * 0.5f);
    #endif
    const f32 ccy = (f32)(globalY - rBin->minY) + 0.5f;

    // base barycentric weights
    const f32 cw0 = (rBin->C1 + rBin->A1 * ccx + rBin->B1 * ccy) * sBin->invDet;
    const f32 cw1 = (rBin->C2 + rBin->A2 * ccx + rBin->B2 * ccy) * sBin->invDet;
    const f32 cw2 = (rBin->C0 + rBin->A0 * ccx + rBin->B0 * ccy) * sBin->invDet;

    // offset barycentric weights for screen space derivatives
    const f32 cw0x = cw0 + rBin->A1 * sBin->invDet, cw1x = cw1 + rBin->A2 * sBin->invDet, cw2x = cw2 + rBin->A0 * sBin->invDet;
    const f32 cw0y = cw0 + rBin->B1 * sBin->invDet, cw1y = cw1 + rBin->B2 * sBin->invDet, cw2y = cw2 + rBin->B0 * sBin->invDet;

    // UVs
    const f32 v0u = sBin->v0.u, v0v = sBin->v0.v;
    const f32 v1u = sBin->v1.u, v1v = sBin->v1.v;
    const f32 v2u = sBin->v2.u, v2v = sBin->v2.v;

    // perspective correction (base)
    const f32 cinvZ = cw0 * sBin->v0.invZ + cw1 * sBin->v1.invZ + cw2 * sBin->v2.invZ;
    const f32 cz = 1.f / cinvZ;
    const f32 cpW0 = cw0 * sBin->v0.invZ * cz;
    const f32 cpW1 = cw1 * sBin->v1.invZ * cz;
    const f32 cpW2 = cw2 * sBin->v2.invZ * cz;
    const f32 ccu = cpW0 * v0u + cpW1 * v1u + cpW2 * v2u;
    const f32 ccv = cpW0 * v0v + cpW1 * v1v + cpW2 * v2v;

    // perspective correction (X + 1)
    const f32 cinvZX = cw0x * sBin->v0.invZ + cw1x * sBin->v1.invZ + cw2x * sBin->v2.invZ;
    const f32 czX = 1.f / cinvZX;
    const f32 cpW0x = cw0x * sBin->v0.invZ * czX;
    const f32 cpW1x = cw1x * sBin->v1.invZ * czX;
    const f32 cpW2x = cw2x * sBin->v2.invZ * czX;
    const f32 cuX = cpW0x * v0u + cpW1x * v1u + cpW2x * v2u;
    const f32 cvX = cpW0x * v0v + cpW1x * v1v + cpW2x * v2v;

    // perspective correction (Y + 1)
    const f32 cinvZY = cw0y * sBin->v0.invZ + cw1y * sBin->v1.invZ + cw2y * sBin->v2.invZ;
    const f32 czY = 1.f / cinvZY;
    const f32 cpW0y = cw0y * sBin->v0.invZ * czY;
    const f32 cpW1y = cw1y * sBin->v1.invZ * czY;
    const f32 cpW2y = cw2y * sBin->v2.invZ * czY;
    const f32 cuY = cpW0y * v0u + cpW1y * v1u + cpW2y * v2u;
    const f32 cvY = cpW0y * v0v + cpW1y * v1v + cpW2y * v2v;

    // normalized uv gradients
    const f32 dudx = cuX - ccu, dvdx = cvX - ccv;
    const f32 dudy = cuY - ccu, dvdy = cvY - ccv;

    const f32 ux = dudx * (f32)mat->albedoTex->w, vx = dvdx * (f32)mat->albedoTex->h;
    const f32 uy = dudy * (f32)mat->albedoTex->w, vy = dvdy * (f32)mat->albedoTex->h;
    const f32 rhoMax = sfr_fmaxf(sfr_fmaxf(sfr_fabsf(ux), sfr_fabsf(vx)), sfr_fmaxf(sfr_fabsf(uy), sfr_fabsf(vy)));
    
    i32 lod = 0;
    if (rhoMax > 1.f) {
        lod = sfr__fast_log2(rhoMax);
    }
    return SFR_CLAMP(lod, 0, mat->albedoTex->mipLevels - 1);
}

#ifndef SFR_NO_SIMD

// handles barycentrics, interpolation, and vector math for the SIMD chunk
static inline void sfr__process_geometry_simd(
    const struct sfrRasterBin* rBin,
    const struct sfrShadeBin* sBin,
    i32 globalX, i32 globalY, 
    struct sfrFragDataSimd* fd
) {
    const vf32 vInvDet = sfrvf_set1(sBin->invDet);
    const vf32 vSteps = SFR_SIMD_SET_STEPS();

    // compute scalar base values for the start of the vector chunk
    const f32 baseX = (f32)(globalX - rBin->minX) + 0.5f;
    const f32 baseY = (f32)(globalY - rBin->minY) + 0.5f;
    const f32 w0base = rBin->A1 * baseX + rBin->B1 * baseY + rBin->C1;
    const f32 w1base = rBin->A2 * baseX + rBin->B2 * baseY + rBin->C2;
    const f32 w2base = rBin->A0 * baseX + rBin->B0 * baseY + rBin->C0;

    const vf32 vw0 = sfrvf_mul(sfrvf_fmadd(sfrvf_set1(rBin->A1), vSteps, sfrvf_set1(w0base)), vInvDet);
    const vf32 vw1 = sfrvf_mul(sfrvf_fmadd(sfrvf_set1(rBin->A2), vSteps, sfrvf_set1(w1base)), vInvDet);
    const vf32 vw2 = sfrvf_mul(sfrvf_fmadd(sfrvf_set1(rBin->A0), vSteps, sfrvf_set1(w2base)), vInvDet);

    const vf32 vInvZ0 = sfrvf_set1(sBin->v0.invZ);
    const vf32 vInvZ1 = sfrvf_set1(sBin->v1.invZ);
    const vf32 vInvZ2 = sfrvf_set1(sBin->v2.invZ);

    const vf32 vInvZ_val = sfrvf_fmadd(vw0, vInvZ0, sfrvf_fmadd(vw1, vInvZ1, sfrvf_mul(vw2, vInvZ2)));

    // approximation (~11 bits) refined to double precision
    const vf32 vZapprox = sfrvf_rcp(vInvZ_val);
    fd->z = sfrvf_mul(vZapprox, sfrvf_fnmadd(vInvZ_val, vZapprox, sfrvf_set1(2.f)));

    // calculate SIMD perspective weights (pW)
    const vf32 pW0 = sfrvf_mul(sfrvf_mul(vw0, vInvZ0), fd->z);
    const vf32 pW1 = sfrvf_mul(sfrvf_mul(vw1, vInvZ1), fd->z);
    const vf32 pW2 = sfrvf_mul(sfrvf_mul(vw2, vInvZ2), fd->z);

    // unpack UVs and broadcast
    const vf32 vv0u = sfrvf_set1(sBin->v0.u), vv0v = sfrvf_set1(sBin->v0.v);
    const vf32 vv1u = sfrvf_set1(sBin->v1.u), vv1v = sfrvf_set1(sBin->v1.v);
    const vf32 vv2u = sfrvf_set1(sBin->v2.u), vv2v = sfrvf_set1(sBin->v2.v);
    const vf32 v0lu = sfrvf_set1(sfr__unpack_uv(sBin->v0.lu)), v0lv = sfrvf_set1(sfr__unpack_uv(sBin->v0.lv));
    const vf32 v1lu = sfrvf_set1(sfr__unpack_uv(sBin->v1.lu)), v1lv = sfrvf_set1(sfr__unpack_uv(sBin->v1.lv));
    const vf32 v2lu = sfrvf_set1(sfr__unpack_uv(sBin->v2.lu)), v2lv = sfrvf_set1(sfr__unpack_uv(sBin->v2.lv));

    // interpolate UVs directly into fragment struct
    fd->cu = sfrvf_fmadd(pW0, vv0u, sfrvf_fmadd(pW1, vv1u, sfrvf_mul(pW2, vv2u)));
    fd->cv = sfrvf_fmadd(pW0, vv0v, sfrvf_fmadd(pW1, vv1v, sfrvf_mul(pW2, vv2v)));
    fd->lu = sfrvf_fmadd(pW0, v0lu, sfrvf_fmadd(pW1, v1lu, sfrvf_mul(pW2, v2lu)));
    fd->lv = sfrvf_fmadd(pW0, v0lv, sfrvf_fmadd(pW1, v1lv, sfrvf_mul(pW2, v2lv)));

    // unpack normals and broadcast
    f32 nx, ny, nz;
    sfr__unpack_vec3(sBin->v0.n, &nx, &ny, &nz);
    const vf32 v0nx = sfrvf_set1(nx), v0ny = sfrvf_set1(ny), v0nz = sfrvf_set1(nz);
    sfr__unpack_vec3(sBin->v1.n, &nx, &ny, &nz);
    const vf32 v1nx = sfrvf_set1(nx), v1ny = sfrvf_set1(ny), v1nz = sfrvf_set1(nz);
    sfr__unpack_vec3(sBin->v2.n, &nx, &ny, &nz);
    const vf32 v2nx = sfrvf_set1(nx), v2ny = sfrvf_set1(ny), v2nz = sfrvf_set1(nz);

    // interpolate and normalize normals
    fd->nx = sfrvf_fmadd(pW0, v0nx, sfrvf_fmadd(pW1, v1nx, sfrvf_mul(pW2, v2nx)));
    fd->ny = sfrvf_fmadd(pW0, v0ny, sfrvf_fmadd(pW1, v1ny, sfrvf_mul(pW2, v2ny)));
    fd->nz = sfrvf_fmadd(pW0, v0nz, sfrvf_fmadd(pW1, v1nz, sfrvf_mul(pW2, v2nz)));

    const vf32 vnDot = sfrvf_fmadd(fd->nx, fd->nx, sfrvf_fmadd(fd->ny, fd->ny, sfrvf_mul(fd->nz, fd->nz)));
    const vf32 vnLen = sfrvf_sqrt(vnDot);
    const vf32 vnInvLen = sfrvf_div(sfrvf_set1(1.f), sfrvf_max(vnLen, sfrvf_set1(0.0001f)));
    fd->nx = sfrvf_mul(fd->nx, vnInvLen);
    fd->ny = sfrvf_mul(fd->ny, vnInvLen);
    fd->nz = sfrvf_mul(fd->nz, vnInvLen);

    // reconstruct frag pos
    const vf32 vGlobalX = sfrvf_add(sfrvf_set1((f32)globalX), vSteps);
    const vf32 vGlobalY = sfrvf_set1((f32)globalY);

    const vf32 vNdcX = sfrvf_sub(sfrvf_div(vGlobalX, sfrvf_set1(sfrState.halfWidth)), sfrvf_set1(1.f));
    const vf32 vNdcY = sfrvf_sub(sfrvf_set1(1.f), sfrvf_div(vGlobalY, sfrvf_set1(sfrState.halfHeight)));

    const vf32 vViewX = sfrvf_mul(sfrvf_mul(vNdcX, fd->z), sfrvf_set1(1.f / sfrMatProj.m[0][0]));
    const vf32 vViewY = sfrvf_mul(sfrvf_mul(vNdcY, fd->z), sfrvf_set1(1.f / sfrMatProj.m[1][1]));
    const vf32 vViewZ = fd->z;

    const vf32 mI00 = sfrvf_set1(sfrState.matInvView.m[0][0]), mI10 = sfrvf_set1(sfrState.matInvView.m[1][0]);
    const vf32 mI20 = sfrvf_set1(sfrState.matInvView.m[2][0]), mI30 = sfrvf_set1(sfrState.matInvView.m[3][0]);
    const vf32 mI01 = sfrvf_set1(sfrState.matInvView.m[0][1]), mI11 = sfrvf_set1(sfrState.matInvView.m[1][1]);
    const vf32 mI21 = sfrvf_set1(sfrState.matInvView.m[2][1]), mI31 = sfrvf_set1(sfrState.matInvView.m[3][1]);
    const vf32 mI02 = sfrvf_set1(sfrState.matInvView.m[0][2]), mI12 = sfrvf_set1(sfrState.matInvView.m[1][2]);
    const vf32 mI22 = sfrvf_set1(sfrState.matInvView.m[2][2]), mI32 = sfrvf_set1(sfrState.matInvView.m[3][2]);

    fd->fragX = sfrvf_fmadd(mI00, vViewX, sfrvf_fmadd(mI10, vViewY, sfrvf_fmadd(mI20, vViewZ, mI30)));
    fd->fragY = sfrvf_fmadd(mI01, vViewX, sfrvf_fmadd(mI11, vViewY, sfrvf_fmadd(mI21, vViewZ, mI31)));
    fd->fragZ = sfrvf_fmadd(mI02, vViewX, sfrvf_fmadd(mI12, vViewY, sfrvf_fmadd(mI22, vViewZ, mI32)));

    // view vector calculation
    fd->viewX = sfrvf_sub(sfrvf_set1(sfrCamPos.x), fd->fragX);
    fd->viewY = sfrvf_sub(sfrvf_set1(sfrCamPos.y), fd->fragY);
    fd->viewZ = sfrvf_sub(sfrvf_set1(sfrCamPos.z), fd->fragZ);
    const vf32 vViewDot = sfrvf_fmadd(fd->viewX, fd->viewX, sfrvf_fmadd(fd->viewY, fd->viewY, sfrvf_mul(fd->viewZ, fd->viewZ)));
    const vf32 vViewLen = sfrvf_sqrt(vViewDot);
    const vf32 vViewInvLen = sfrvf_div(sfrvf_set1(1.f), sfrvf_max(vViewLen, sfrvf_set1(0.0001f)));
    fd->viewX = sfrvf_mul(fd->viewX, vViewInvLen);
    fd->viewY = sfrvf_mul(fd->viewY, vViewInvLen);
    fd->viewZ = sfrvf_mul(fd->viewZ, vViewInvLen);
}

// evaluates the 2x2 PCF shadow kernel
static inline void sfr__sample_shadowmap_simd(
    struct sfrFragDataSimd* fd, 
    const vi32* pWriteMask
) {
    fd->shadow = sfrvf_set1(1.f);
    
    if (!sfrState.activeShadowmap || !sfrState.activeShadowmap->depth) {
        return;
    }

    // matrix multiply (fragPos * matLightVP)
    const vf32 vM00 = sfrvf_set1(sfrState.matLightVP.m[0][0]);
    const vf32 vM10 = sfrvf_set1(sfrState.matLightVP.m[1][0]);
    const vf32 vM20 = sfrvf_set1(sfrState.matLightVP.m[2][0]);
    const vf32 vM30 = sfrvf_set1(sfrState.matLightVP.m[3][0]);
    const vf32 vLightX = sfrvf_fmadd(vM00, fd->fragX, sfrvf_fmadd(vM10, fd->fragY, sfrvf_fmadd(vM20, fd->fragZ, vM30)));

    const vf32 vM01 = sfrvf_set1(sfrState.matLightVP.m[0][1]);
    const vf32 vM11 = sfrvf_set1(sfrState.matLightVP.m[1][1]);
    const vf32 vM21 = sfrvf_set1(sfrState.matLightVP.m[2][1]);
    const vf32 vM31 = sfrvf_set1(sfrState.matLightVP.m[3][1]);
    const vf32 vLightY = sfrvf_fmadd(vM01, fd->fragX, sfrvf_fmadd(vM11, fd->fragY, sfrvf_fmadd(vM21, fd->fragZ, vM31)));

    const vf32 vM02 = sfrvf_set1(sfrState.matLightVP.m[0][2]);
    const vf32 vM12 = sfrvf_set1(sfrState.matLightVP.m[1][2]);
    const vf32 vM22 = sfrvf_set1(sfrState.matLightVP.m[2][2]);
    const vf32 vM32 = sfrvf_set1(sfrState.matLightVP.m[3][2]);
    const vf32 vLightZ = sfrvf_fmadd(vM02, fd->fragX, sfrvf_fmadd(vM12, fd->fragY, sfrvf_fmadd(vM22, fd->fragZ, vM32)));

    const vf32 vM03 = sfrvf_set1(sfrState.matLightVP.m[0][3]);
    const vf32 vM13 = sfrvf_set1(sfrState.matLightVP.m[1][3]);
    const vf32 vM23 = sfrvf_set1(sfrState.matLightVP.m[2][3]);
    const vf32 vM33 = sfrvf_set1(sfrState.matLightVP.m[3][3]);
    const vf32 vLightW = sfrvf_fmadd(vM03, fd->fragX, sfrvf_fmadd(vM13, fd->fragY, sfrvf_fmadd(vM23, fd->fragZ, vM33)));

    // perspective divide and UV mapping
    const vf32 vInvW = sfrvf_div(sfrvf_set1(1.f), vLightW);
    const vf32 vHalf = sfrvf_set1(0.5f);
    const vf32 vUvX = sfrvf_fmadd(sfrvf_mul(vLightX, vInvW), vHalf, vHalf);
    const vf32 vUvY = sfrvf_fnmadd(sfrvf_mul(vLightY, vInvW), vHalf, vHalf); // 0.5 - (y * 0.5)
    
    vf32 vFragDepth = sfrvf_sub(sfrvf_set1(1.f), sfrvf_mul(vLightZ, vInvW));
    vFragDepth = sfrvf_add(vFragDepth, sfrvf_set1(sfrState.activeShadowmap->shadowBias)); // add bias

    // bounds checking
    const vf32 vZero = sfrvf_zero();
    const vf32 vOne = sfrvf_set1(1.f);
    const vf32 vBoundsMask = sfrvf_and(
        sfrvf_and(sfrvf_cmp(vUvX, vZero, SFR_CMP_GE_OQ), sfrvf_cmp(vUvX, vOne, SFR_CMP_LE_OQ)),
        sfrvf_and(sfrvf_cmp(vUvY, vZero, SFR_CMP_GE_OQ), sfrvf_cmp(vUvY, vOne, SFR_CMP_LE_OQ))
    );

    // calculate base indices for the 2x2 PCF grid
    const vi32 vWidthM1 = sfrvi_set1(sfrState.activeShadowmap->width - 1);
    const vi32 vHeightM1 = sfrvi_set1(sfrState.activeShadowmap->height - 1);
    
    // calculate base coordinates
    const vi32 vSmX0 = sfrvi_cvttps(sfrvf_mul(vUvX, sfrvf_cvtepi32(vWidthM1)));
    const vi32 vSmY0 = sfrvi_cvttps(sfrvf_mul(vUvY, sfrvf_cvtepi32(vHeightM1)));
    
    // calculate +1 coordinates and clamp
    const vi32 vSmX1 = sfrvi_min(sfrvi_add(vSmX0, sfrvi_set1(1)), vWidthM1);
    const vi32 vSmY1 = sfrvi_min(sfrvi_add(vSmY0, sfrvi_set1(1)), vHeightM1);
    
    const vi32 vWidthInt = sfrvi_set1(sfrState.activeShadowmap->width);
    
    // base Index (x, y)
    const vi32 vInd00 = sfrvi_add(sfrvi_mullo(vSmY0, vWidthInt), vSmX0);
    const vi32 vInd10 = sfrvi_add(sfrvi_mullo(vSmY0, vWidthInt), vSmX1);
    const vi32 vInd01 = sfrvi_add(sfrvi_mullo(vSmY1, vWidthInt), vSmX0);
    const vi32 vInd11 = sfrvi_add(sfrvi_mullo(vSmY1, vWidthInt), vSmX1);

    const vi32 vWriteMask = *pWriteMask;

    // explicitly copy the mask because the gather destroys it
    const vf32 vGatherMask = sfrvf_and(vBoundsMask, sfrvf_cast_from_vi32(vWriteMask));

    const vf32 vDepth00 = sfrvf_gather(vOne, sfrState.activeShadowmap->depth, vInd00, vGatherMask, 4);
    const vf32 vDepth10 = sfrvf_gather(vOne, sfrState.activeShadowmap->depth, vInd10, vGatherMask, 4);
    const vf32 vDepth01 = sfrvf_gather(vOne, sfrState.activeShadowmap->depth, vInd01, vGatherMask, 4);
    const vf32 vDepth11 = sfrvf_gather(vOne, sfrState.activeShadowmap->depth, vInd11, vGatherMask, 4);

    // compare depths (fragDepth < closestDepth means it's in shadow)
    const vf32 vShadowVal = sfrvf_set1(1.f - sfrState.activeShadowmap->shadowStrength);
    
    const vf32 vRes00 = sfrvf_blendv(vOne, vShadowVal, sfrvf_cmp(vFragDepth, vDepth00, SFR_CMP_LT_OQ));
    const vf32 vRes10 = sfrvf_blendv(vOne, vShadowVal, sfrvf_cmp(vFragDepth, vDepth10, SFR_CMP_LT_OQ));
    const vf32 vRes01 = sfrvf_blendv(vOne, vShadowVal, sfrvf_cmp(vFragDepth, vDepth01, SFR_CMP_LT_OQ));
    const vf32 vRes11 = sfrvf_blendv(vOne, vShadowVal, sfrvf_cmp(vFragDepth, vDepth11, SFR_CMP_LT_OQ));

    // average the 4 samples
    const vf32 vSum1 = sfrvf_add(vRes00, vRes10);
    const vf32 vSum2 = sfrvf_add(vRes01, vRes11);
    const vf32 vShadowIntensity = sfrvf_mul(sfrvf_add(vSum1, vSum2), sfrvf_set1(0.25f));
    
    // if outside bounds keep 1.0, otherwise apply blurred intensity
    fd->shadow = sfrvf_blendv(vOne, vShadowIntensity, vBoundsMask);
}

static inline void sfr__sample_material_base_simd(
    const struct sfrShadeBin* sBin, 
    const SfrMaterial* mat, 
    struct sfrSurfacePropsSimd* props
) {
    props->albedoR = sfrvf_set1((f32)((sBin->col >> 16) & 0xFF) / 255.f);
    props->albedoG = sfrvf_set1((f32)((sBin->col >> 8)  & 0xFF) / 255.f);
    props->albedoB = sfrvf_set1((f32)((sBin->col >> 0)  & 0xFF) / 255.f);

    props->metallic = sfrvf_set1(mat->metallicFactor);
    props->roughness = sfrvf_max(sfrvf_set1(0.001f), sfrvf_set1(mat->roughnessFactor));
}

static inline void sfr__sample_material_tex_simd(
    const SfrMaterial* mat, 
    const struct sfrFragDataSimd* fd, 
    i32 aLod, 
    const vi32* pWriteMask, 
    struct sfrSurfacePropsSimd* props
) {
    if (!mat->albedoTex) {
        return;
    }

    const SfrTexture* tex = mat->albedoTex;
    const u32* const mipPixels = tex->allPixels[aLod];
    const i32 currW = tex->allW[aLod];
    const i32 currH = tex->allH[aLod];

    vf32 vau = sfrvf_sub(fd->cu, sfrvf_floor(fd->cu));
    vf32 vav = sfrvf_sub(fd->cv, sfrvf_floor(fd->cv));

    vau = sfrvf_mul(vau, sfrvf_set1((f32)currW));
    vav = sfrvf_mul(vav, sfrvf_set1((f32)currH));

    vi32 vatx, vaty;
    vi32 vTexInds;

    if (tex->isPot) {
        vatx = sfrvi_and(sfrvi_cvttps(vau), sfrvi_set1(currW - 1));
        vaty = sfrvi_and(sfrvi_cvttps(vav), sfrvi_set1(currH - 1));

        // block coords (x / 4, y / 4)
        const vi32 vBlockX = sfrvi_srli(vatx, 2);
        const vi32 vBlockY = sfrvi_srli(vaty, 2);
        
        // inner block coords (x % 4, y % 4)
        const vi32 vInBlockX = sfrvi_and(vatx, sfrvi_set1(3));
        const vi32 vInBlockY = sfrvi_and(vaty, sfrvi_set1(3));

        // block inds
        const vi32 vBlockInd = sfrvi_add(sfrvi_slli(vBlockY, tex->strideShift[aLod]), vBlockX);
        const vi32 vInnerInd = sfrvi_add(sfrvi_slli(vInBlockY, 2), vInBlockX);
        
        // final ind
        vTexInds = sfrvi_add(sfrvi_slli(vBlockInd, 4), vInnerInd);
    } else {
        // fallback
        vatx = sfrvi_min(sfrvi_cvttps(vau), sfrvi_set1(currW - 1));
        vaty = sfrvi_min(sfrvi_cvttps(vav), sfrvi_set1(currH - 1));

        const vi32 vBlockX = sfrvi_srli(vatx, 2);
        const vi32 vBlockY = sfrvi_srli(vaty, 2);
        const vi32 vInBlockX = sfrvi_and(vatx, sfrvi_set1(3));
        const vi32 vInBlockY = sfrvi_and(vaty, sfrvi_set1(3));

        // must use integer mul for NPOT row striding
        const vi32 vBlocksW = sfrvi_set1(tex->blocksW[aLod]);
        const vi32 vBlockInd = sfrvi_add(sfrvi_mullo(vBlockY, vBlocksW), vBlockX);
        
        const vi32 vInnerInd = sfrvi_add(sfrvi_slli(vInBlockY, 2), vInBlockX);
        vTexInds = sfrvi_add(sfrvi_slli(vBlockInd, 4), vInnerInd);
    }

    const vi32 vWriteMask = *pWriteMask;

    // only fetch active pixels using the mask passed from the chunk resolver
    vi32 vCol = sfrvi_gather(sfrvi_zero(), (const int*)mipPixels, vTexInds, vWriteMask, 4);

    const vf32 vInv255 = sfrvf_set1(1.f / 255.f);
    const vi32 v0xFF  = sfrvi_set1(0xFF);
    
    props->albedoR = sfrvf_mul(props->albedoR, sfrvf_mul(sfrvf_cvtepi32(sfrvi_and(sfrvi_srli(vCol, 16), v0xFF)), vInv255));
    props->albedoG = sfrvf_mul(props->albedoG, sfrvf_mul(sfrvf_cvtepi32(sfrvi_and(sfrvi_srli(vCol, 8), v0xFF)), vInv255));
    props->albedoB = sfrvf_mul(props->albedoB, sfrvf_mul(sfrvf_cvtepi32(sfrvi_and(vCol, v0xFF)), vInv255));

    if (mat->metallicRoughnessTex) {
        const SfrTexture* mrTex = mat->metallicRoughnessTex;
        const u32* const mrMipPixels = mrTex->allPixels[aLod];
        
        const vi32 vMrCol = sfrvi_gather(sfrvi_zero(), (const int*)mrMipPixels, vTexInds, vWriteMask, 4);
        vf32 vTexRoughness = sfrvf_mul(sfrvf_cvtepi32(sfrvi_and(sfrvi_srli(vMrCol, 8), v0xFF)), vInv255);
        props->roughness = sfrvf_max(sfrvf_set1(0.001f), vTexRoughness);
        props->metallic = sfrvf_mul(sfrvf_cvtepi32(sfrvi_and(vMrCol, v0xFF)), vInv255);
    }
}

static inline void sfr__calc_surface_props_simd(struct sfrSurfacePropsSimd* props) {
    const vf32 vOne = sfrvf_set1(1.f);
    const vf32 vInvMetallic = sfrvf_sub(vOne, props->metallic);

    props->diffR = sfrvf_mul(props->albedoR, vInvMetallic);
    props->diffG = sfrvf_mul(props->albedoG, vInvMetallic);
    props->diffB = sfrvf_mul(props->albedoB, vInvMetallic);

    const vf32 vF0Base = sfrvf_mul(sfrvf_set1(0.04f), vInvMetallic);
    props->f0R = sfrvf_fmadd(props->albedoR, props->metallic, vF0Base);
    props->f0G = sfrvf_fmadd(props->albedoG, props->metallic, vF0Base);
    props->f0B = sfrvf_fmadd(props->albedoB, props->metallic, vF0Base);

    // convert PBR roughness to blinn-phong shininess
    const vf32 vAlpha = sfrvf_mul(props->roughness, props->roughness);
    const vf32 vAlphaSq = sfrvf_mul(vAlpha, vAlpha);
    vf32 vShininess = sfrvf_sub(sfrvf_div(sfrvf_set1(2.f), vAlphaSq), sfrvf_set1(2.f));
    props->shininess = sfrvf_max(sfrvf_set1(1.f), sfrvf_min(vShininess, sfrvf_set1(4096.f)));

    props->energyConservation = sfrvf_mul(sfrvf_add(sfrvf_set1(8.f), props->shininess), sfrvf_set1(1.f / (8.f * SFR_PI)));
}

static inline void sfr__apply_lightmap_simd(
    const struct sfrFragDataSimd* fd, 
    const struct sfrSurfacePropsSimd* props, 
    const vi32* pWriteMask,
    struct sfrLightmapStateSimd* lmState, 
    vf32* vFinalR, vf32* vFinalG, vf32* vFinalB
) {
    const SfrTexture* lm = sfrState.activeLightmap;

    const vf32 vlmW = sfrvf_set1((f32)lm->w);
    const vf32 vlmH = sfrvf_set1((f32)lm->h);
    const vf32 vHalf = sfrvf_set1(0.5f);
    
    vf32 vpx = sfrvf_sub(sfrvf_mul(fd->lu, vlmW), vHalf);
    vf32 vpy = sfrvf_sub(sfrvf_mul(fd->lv, vlmH), vHalf);

    const vf32 vZero = sfrvf_zero();
    const vf32 vMaxX = sfrvf_set1((f32)(lm->w - 1) - 0.001f);
    const vf32 vMaxY = sfrvf_set1((f32)(lm->h - 1) - 0.001f);
    vpx = sfrvf_max(vZero, sfrvf_min(vpx, vMaxX));
    vpy = sfrvf_max(vZero, sfrvf_min(vpy, vMaxY));

    const vi32 vtx0 = sfrvi_cvttps(vpx);
    const vi32 vty0 = sfrvi_cvttps(vpy);
    
    lmState->fx = sfrvf_sub(vpx, sfrvf_cvtepi32(vtx0));
    lmState->fy = sfrvf_sub(vpy, sfrvf_cvtepi32(vty0));

    const vi32 vPitch = sfrvi_set1(lm->w);
    lmState->ind00 = sfrvi_add(sfrvi_mullo(vty0, vPitch), vtx0);

    const vi32 vWriteMask = *pWriteMask;
    const vi32 vZeroInt = sfrvi_zero();
    const vi32 vQuadR = sfrvi_gather(vZeroInt, (const int*)lm->quadR, lmState->ind00, vWriteMask, 4);
    const vi32 vQuadG = sfrvi_gather(vZeroInt, (const int*)lm->quadG, lmState->ind00, vWriteMask, 4);
    const vi32 vQuadB = sfrvi_gather(vZeroInt, (const int*)lm->quadB, lmState->ind00, vWriteMask, 4);

    const vi32 v0xFF = sfrvi_set1(0xFF);

    #define BLEND_PLANAR(vQuad) \
        __extension__ ({ \
            vf32 c00 = sfrvf_cvtepi32(sfrvi_and(vQuad, v0xFF)); \
            vf32 c10 = sfrvf_cvtepi32(sfrvi_and(sfrvi_srli(vQuad, 8), v0xFF)); \
            vf32 c01 = sfrvf_cvtepi32(sfrvi_and(sfrvi_srli(vQuad, 16), v0xFF)); \
            vf32 c11 = sfrvf_cvtepi32(sfrvi_srli(vQuad, 24)); \
            vf32 cTop = sfrvf_fmadd(lmState->fx, sfrvf_sub(c10, c00), c00); \
            vf32 cBot = sfrvf_fmadd(lmState->fx, sfrvf_sub(c11, c01), c01); \
            sfrvf_fmadd(lmState->fy, sfrvf_sub(cBot, cTop), cTop); \
        })

    lmState->lmR = BLEND_PLANAR(vQuadR);
    lmState->lmG = BLEND_PLANAR(vQuadG);
    lmState->lmB = BLEND_PLANAR(vQuadB);

    const vf32 vInv255 = sfrvf_set1(1.f / 255.f);
    *vFinalR = sfrvf_mul(props->albedoR, sfrvf_mul(lmState->lmR, vInv255));
    *vFinalG = sfrvf_mul(props->albedoG, sfrvf_mul(lmState->lmG, vInv255));
    *vFinalB = sfrvf_mul(props->albedoB, sfrvf_mul(lmState->lmB, vInv255));
}

static inline void sfr__apply_directionmap_simd(
    const struct sfrFragDataSimd* fd, 
    const struct sfrSurfacePropsSimd* props, 
    const struct sfrLightmapStateSimd* lmState, 
    const vi32* pWriteMask,
    vf32* vFinalR, vf32* vFinalG, vf32* vFinalB
) {
    const SfrTexture* dm = sfrState.activeDirectionmap;
    const vi32 vZeroInt = sfrvi_zero();
    const vi32 v0xFF = sfrvi_set1(0xFF);

    const vi32 vWriteMask = *pWriteMask;
    const vi32 vDirQuadR = sfrvi_gather(vZeroInt, (const int*)dm->quadR, lmState->ind00, vWriteMask, 4);
    const vi32 vDirQuadG = sfrvi_gather(vZeroInt, (const int*)dm->quadG, lmState->ind00, vWriteMask, 4);
    const vi32 vDirQuadB = sfrvi_gather(vZeroInt, (const int*)dm->quadB, lmState->ind00, vWriteMask, 4);

    const vf32 vDirRraw = BLEND_PLANAR(vDirQuadR);
    const vf32 vDirGraw = BLEND_PLANAR(vDirQuadG);
    const vf32 vDirBraw = BLEND_PLANAR(vDirQuadB);

    #undef BLEND_PLANAR

    // remap [0, 255] to [-1.0, 1.0]
    const vf32 vRemapScale = sfrvf_set1(2.f / 255.f);
    const vf32 vRemapOffset = sfrvf_set1(-1.f);
    vf32 vLDirX = sfrvf_fmadd(vDirRraw, vRemapScale, vRemapOffset);
    vf32 vLDirY = sfrvf_fmadd(vDirGraw, vRemapScale, vRemapOffset);
    vf32 vLDirZ = sfrvf_fmadd(vDirBraw, vRemapScale, vRemapOffset);

    const vf32 vDirLenSq = sfrvf_fmadd(vLDirX, vLDirX, sfrvf_fmadd(vLDirY, vLDirY, sfrvf_mul(vLDirZ, vLDirZ)));
    const vf32 vDirLen = sfrvf_sqrt(vDirLenSq);
    
    vf32 vDirRsqrt = sfrvf_rsqrt(vDirLenSq);
    vLDirX = sfrvf_mul(vLDirX, vDirRsqrt);
    vLDirY = sfrvf_mul(vLDirY, vDirRsqrt);
    vLDirZ = sfrvf_mul(vLDirZ, vDirRsqrt);

    vf32 vhx = sfrvf_add(vLDirX, fd->viewX);
    vf32 vhy = sfrvf_add(vLDirY, fd->viewY);
    vf32 vhz = sfrvf_add(vLDirZ, fd->viewZ);
    const vf32 vhDot = sfrvf_fmadd(vhx, vhx, sfrvf_fmadd(vhy, vhy, sfrvf_mul(vhz, vhz)));
    const vf32 vhLen = sfrvf_sqrt(vhDot);
    const vf32 vhInvLen = sfrvf_div(sfrvf_set1(1.f), sfrvf_max(vhLen, sfrvf_set1(0.0001f)));
    vhx = sfrvf_mul(vhx, vhInvLen);
    vhy = sfrvf_mul(vhy, vhInvLen);
    vhz = sfrvf_mul(vhz, vhInvLen);

    const vf32 vnDotL = sfrvf_fmadd(fd->nx, vLDirX, sfrvf_fmadd(fd->ny, vLDirY, sfrvf_mul(fd->nz, vLDirZ)));
    const vf32 vDiffMask = sfrvf_cmp(vnDotL, sfrvf_zero(), SFR_CMP_GT_OQ);

    vf32 vnDotH = sfrvf_fmadd(fd->nx, vhx, sfrvf_fmadd(fd->ny, vhy, sfrvf_mul(fd->nz, vhz)));
    vnDotH = sfrvf_max(sfrvf_set1(0.00001f), sfrvf_min(vnDotH, sfrvf_set1(1.f)));
    
    vf32 vSpec = sfrvf_pow(vnDotH, props->shininess);

    vf32 vIsNaN = sfrvf_cmp(vSpec, vSpec, SFR_CMP_UNORD_Q);
    vSpec = sfrvf_blendv(vSpec, sfrvf_zero(), vIsNaN);
    vSpec = sfrvf_min(vSpec, sfrvf_set1(1000.f));

    // mask out specular completely if the pixel is shadowed/facing away
    vSpec = sfrvf_blendv(sfrvf_zero(), vSpec, vDiffMask);

    const vf32 vSpecIntensity = sfrvf_mul(vSpec, sfrvf_mul(vDirLen, props->energyConservation));
    const vf32 vInv255 = sfrvf_set1(1.f / 255.f);
    
    const vf32 vSpecR = sfrvf_mul(vSpecIntensity, sfrvf_mul(lmState->lmR, vInv255));
    const vf32 vSpecG = sfrvf_mul(vSpecIntensity, sfrvf_mul(lmState->lmG, vInv255));
    const vf32 vSpecB = sfrvf_mul(vSpecIntensity, sfrvf_mul(lmState->lmB, vInv255));

    const vf32 vDirMask = sfrvf_cmp(vDirLen, sfrvf_set1(0.01f), SFR_CMP_GT_OQ);
    *vFinalR = sfrvf_blendv(*vFinalR, sfrvf_fmadd(vSpecR, props->f0R, *vFinalR), vDirMask);
    *vFinalG = sfrvf_blendv(*vFinalG, sfrvf_fmadd(vSpecG, props->f0G, *vFinalG), vDirMask);
    *vFinalB = sfrvf_blendv(*vFinalB, sfrvf_fmadd(vSpecB, props->f0B, *vFinalB), vDirMask);
}

static inline void sfr__apply_dynamic_lights_simd(
    const struct sfrFragDataSimd* fd, 
    const struct sfrSurfacePropsSimd* props, 
    vf32* vFinalR, vf32* vFinalG, vf32* vFinalB
) {
    const vf32 vOne = sfrvf_set1(1.f);
    const vf32 vZero = sfrvf_zero();

    for (i32 i = 0; i < sfrState.lightCount; i += 1) {
        const SfrLight* const light = &sfrState.lights[i];

        vf32 vLDirX, vLDirY, vLDirZ;
        vf32 vAtten = vOne;

        if (SFR_LIGHT_DIRECTIONAL == light->type) {
            vLDirX = sfrvf_set1(-light->x);
            vLDirY = sfrvf_set1(-light->y);
            vLDirZ = sfrvf_set1(-light->z);
        } else {
            const vf32 vDiffX = sfrvf_sub(sfrvf_set1(light->x), fd->fragX);
            const vf32 vDiffY = sfrvf_sub(sfrvf_set1(light->y), fd->fragY);
            const vf32 vDiffZ = sfrvf_sub(sfrvf_set1(light->z), fd->fragZ);

            const vf32 vDistSq = sfrvf_add(sfrvf_mul(vDiffX, vDiffX), sfrvf_add(sfrvf_mul(vDiffY, vDiffY), sfrvf_mul(vDiffZ, vDiffZ)));
            const vf32 vDistRsqrt = sfrvf_rsqrt(vDistSq);
            const vf32 vDist = sfrvf_mul(vDistSq, vDistRsqrt);

            vLDirX = sfrvf_mul(vDiffX, vDistRsqrt);
            vLDirY = sfrvf_mul(vDiffY, vDistRsqrt);
            vLDirZ = sfrvf_mul(vDiffZ, vDistRsqrt);

            const vf32 vAttenCore = sfrvf_max(vZero, sfrvf_sub(vOne, sfrvf_mul(vDist, sfrvf_set1(1.f / light->attenuation))));
            vAtten = sfrvf_mul(vAttenCore, vAttenCore);
        }

        const vf32 vnDotL = sfrvf_add(sfrvf_mul(fd->nx, vLDirX), sfrvf_add(sfrvf_mul(fd->ny, vLDirY), sfrvf_mul(fd->nz, vLDirZ)));
        const vf32 vDiff = sfrvf_max(vZero, vnDotL);

        vf32 vhx = sfrvf_add(vLDirX, fd->viewX);
        vf32 vhy = sfrvf_add(vLDirY, fd->viewY);
        vf32 vhz = sfrvf_add(vLDirZ, fd->viewZ);
        const vf32 vhDot = sfrvf_add(sfrvf_mul(vhx, vhx), sfrvf_add(sfrvf_mul(vhy, vhy), sfrvf_mul(vhz, vhz)));
        const vf32 vhRsqrt = sfrvf_rsqrt(vhDot);
        vhx = sfrvf_mul(vhx, vhRsqrt);
        vhy = sfrvf_mul(vhy, vhRsqrt);
        vhz = sfrvf_mul(vhz, vhRsqrt);

        const vf32 vnDotH = sfrvf_max(vZero, sfrvf_add(sfrvf_mul(fd->nx, vhx), sfrvf_add(sfrvf_mul(fd->ny, vhy), sfrvf_mul(fd->nz, vhz))));

        vf32 vSpec = sfrvf_pow(vnDotH, props->shininess);

        vSpec = sfrvf_mul(vSpec, props->energyConservation);
        vSpec = sfrvf_blendv(vZero, vSpec, sfrvf_cmp(vDiff, vZero, SFR_CMP_GT_OQ));

        const vf32 vIntensity = sfrvf_mul(sfrvf_set1(light->intensity), vAtten);
        const vf32 vDiffuseIntensity = sfrvf_mul(vDiff, vIntensity);
        const vf32 vSpecularIntensity = sfrvf_mul(vSpec, vDiffuseIntensity);

        const vf32 vLightR = sfrvf_set1(light->r);
        const vf32 vLightG = sfrvf_set1(light->g);
        const vf32 vLightB = sfrvf_set1(light->b);
        const vf32 vAmbient = sfrvf_set1(light->ambient);

        const vf32 vDiffTerm = sfrvf_add(vAmbient, vDiffuseIntensity);
        const vf32 vSpecTerm = sfrvf_add(vAmbient, vSpecularIntensity);

        const vf32 vLightDiffR = sfrvf_mul(vLightR, props->diffR);
        const vf32 vLightSpecR = sfrvf_mul(vLightR, props->f0R);
        *vFinalR = sfrvf_fmadd(vDiffTerm, vLightDiffR, *vFinalR);
        *vFinalR = sfrvf_fmadd(vSpecTerm, vLightSpecR, *vFinalR);

        const vf32 vLightDiffG = sfrvf_mul(vLightG, props->diffG);
        const vf32 vLightSpecG = sfrvf_mul(vLightG, props->f0G);
        *vFinalG = sfrvf_fmadd(vDiffTerm, vLightDiffG, *vFinalG);
        *vFinalG = sfrvf_fmadd(vSpecTerm, vLightSpecG, *vFinalG);

        const vf32 vLightDiffB = sfrvf_mul(vLightB, props->diffB);
        const vf32 vLightSpecB = sfrvf_mul(vLightB, props->f0B);
        *vFinalB = sfrvf_fmadd(vDiffTerm, vLightDiffB, *vFinalB);
        *vFinalB = sfrvf_fmadd(vSpecTerm, vLightSpecB, *vFinalB);
    }
}

static inline void sfr__pack_and_write_shaded_simd(
    i32 globalInd, const vi32* pWriteMask, 
    const vf32* pFinalR, const vf32* pFinalG, const vf32* pFinalB, const vf32* pShadow
) {
    vf32 vFinalR = *pFinalR;
    vf32 vFinalG = *pFinalG;
    vf32 vFinalB = *pFinalB;
    vf32 vShadow = *pShadow;

    #ifndef SFR_NO_GAMMA_CORRECT
        // sqrt used as a fast approximation for the gamma LUT
        #define SFR_GAMMA_CORRECT(v) sfrvf_sqrt(v)
    #else
        #define SFR_GAMMA_CORRECT(v) (v)
    #endif

    vFinalR = SFR_GAMMA_CORRECT(sfrvf_mul(vFinalR, vShadow));
    vFinalG = SFR_GAMMA_CORRECT(sfrvf_mul(vFinalG, vShadow));
    vFinalB = SFR_GAMMA_CORRECT(sfrvf_mul(vFinalB, vShadow));

    #undef SFR_GAMMA_CORRECT

    const vf32 v255 = sfrvf_set1(255.f);
    const vf32 vfr = sfrvf_min(sfrvf_mul(vFinalR, v255), v255);
    const vf32 vfg = sfrvf_min(sfrvf_mul(vFinalG, v255), v255);
    const vf32 vfb = sfrvf_min(sfrvf_mul(vFinalB, v255), v255);

    const vi32 vFinalPixel = sfrvi_or(sfrvi_set1(0xFF000000),
        sfrvi_or(sfrvi_slli(sfrvi_cvttps(vfr), 16),
        sfrvi_or(sfrvi_slli(sfrvi_cvttps(vfg), 8), sfrvi_cvttps(vfb))));

    const vi32 vWriteMask = *pWriteMask;
    if (sfrvi_movemask_epi8(vWriteMask) == SFR_SIMD_FULL_BYTE_MASK) {
        sfrvi_storeu((vi32*)&sfrPixelBuf[globalInd], vFinalPixel);
    } else {
        sfrvi_maskstore((int*)&sfrPixelBuf[globalInd], vWriteMask, vFinalPixel);
    }
}

static inline void sfr__pack_and_write_normals_simd(
    i32 globalInd, const vi32* pWriteMask, 
    const struct sfrFragDataSimd* fd
) {
    const vf32 vOne = sfrvf_set1(1.f);
    const vf32 vHalf = sfrvf_set1(0.5f);
    const vf32 vZero = sfrvf_zero();

    const vf32 vFinalR = sfrvf_max(vZero, sfrvf_min(sfrvf_add(sfrvf_mul(fd->nx, vHalf), vHalf), vOne));
    const vf32 vFinalG = sfrvf_max(vZero, sfrvf_min(sfrvf_add(sfrvf_mul(fd->ny, vHalf), vHalf), vOne));
    const vf32 vFinalB = sfrvf_max(vZero, sfrvf_min(sfrvf_add(sfrvf_mul(fd->nz, vHalf), vHalf), vOne));

    const vf32 v255 = sfrvf_set1(255.f);
    const vi32 vFinalPixel = sfrvi_or(sfrvi_set1(0xFF000000),
        sfrvi_or(sfrvi_slli(sfrvi_cvttps(sfrvf_mul(vFinalR, v255)), 16),
        sfrvi_or(sfrvi_slli(sfrvi_cvttps(sfrvf_mul(vFinalG, v255)), 8), sfrvi_cvttps(sfrvf_mul(vFinalB, v255)))));

    const vi32 vWriteMask = *pWriteMask;
    sfrvi_maskstore((int*)&sfrPixelBuf[globalInd], vWriteMask, vFinalPixel);
}

static inline void sfr__pack_and_write_fragpos_simd(
    i32 globalInd, const vi32* pWriteMask, 
    const struct sfrFragDataSimd* fd
) {
    const vf32 vFinalR = sfrvf_sub(fd->fragX, sfrvf_floor(fd->fragX));
    const vf32 vFinalG = sfrvf_sub(fd->fragY, sfrvf_floor(fd->fragY));
    const vf32 vFinalB = sfrvf_sub(fd->fragZ, sfrvf_floor(fd->fragZ));

    const vf32 v255 = sfrvf_set1(255.f);
    const vi32 vFinalPixel = sfrvi_or(sfrvi_set1(0xFF000000),
        sfrvi_or(sfrvi_slli(sfrvi_cvttps(sfrvf_mul(vFinalR, v255)), 16),
        sfrvi_or(sfrvi_slli(sfrvi_cvttps(sfrvf_mul(vFinalG, v255)), 8), sfrvi_cvttps(sfrvf_mul(vFinalB, v255)))));

    const vi32 vWriteMask = *pWriteMask;
    sfrvi_maskstore((int*)&sfrPixelBuf[globalInd], vWriteMask, vFinalPixel);
}

#define SFR__DEF_CHUNK_RESOLVER(NAME, HAS_TEX, CALC_SURFACE, HAS_SHADOW, LIGHT_MODE) \
static void _sfr__resolve_chunk_##NAME(const struct sfrResolveChunkArgs* args) { \
    const struct sfrRasterBin* rBin = args->rBin; \
    const struct sfrShadeBin* sBin = args->sBin; \
    const i32 globalX = args->globalX; \
    const i32 globalY = args->globalY; \
    const i32 globalInd = args->globalInd; \
    const vi32* pWriteMask = args->pWriteMask; \
    \
    const i32 aLod = HAS_TEX ? sfr__calc_lod(rBin, sBin, globalX, globalY) : 0; \
    \
    static SFR_THREAD_LOCAL struct sfrFragDataSimd fd; \
    sfr__process_geometry_simd(rBin, sBin, globalX, globalY, &fd); \
    if (LIGHT_MODE == SFR_RLT_DEBUG_NORMALS) { \
        sfr__pack_and_write_normals_simd(globalInd, pWriteMask, &fd); \
        return; \
    } \
    if (LIGHT_MODE == SFR_RLT_DEBUG_FRAGPOS) { \
        sfr__pack_and_write_fragpos_simd(globalInd, pWriteMask, &fd); \
        return; \
    } \
    if (HAS_SHADOW) sfr__sample_shadowmap_simd(&fd, pWriteMask); \
    \
    static SFR_THREAD_LOCAL struct sfrSurfacePropsSimd props; \
    sfr__sample_material_base_simd(sBin, sBin->mat, &props); \
    if (HAS_TEX) sfr__sample_material_tex_simd(sBin->mat, &fd, aLod, pWriteMask, &props); \
    if (CALC_SURFACE) sfr__calc_surface_props_simd(&props); \
    \
    vf32 vFinalR, vFinalG, vFinalB; \
    if (LIGHT_MODE == SFR_RLT_LIGHTMAP || LIGHT_MODE == SFR_RLT_LIGHTMAP_DYNAMIC) { \
        static SFR_THREAD_LOCAL struct sfrLightmapStateSimd lmState; \
        vFinalR = sfrvf_zero(), vFinalG = sfrvf_zero(), vFinalB = sfrvf_zero(); \
        sfr__apply_lightmap_simd(&fd, &props, pWriteMask, &lmState, &vFinalR, &vFinalG, &vFinalB); \
        /* if (CALC_SURFACE) sfr__apply_directionmap_simd(&fd, &props, &lmState, pWriteMask, &vFinalR, &vFinalG, &vFinalB); */ \
        if (LIGHT_MODE == SFR_RLT_LIGHTMAP_DYNAMIC) sfr__apply_dynamic_lights_simd(&fd, &props, &vFinalR, &vFinalG, &vFinalB); \
    } else if (LIGHT_MODE == SFR_RLT_DYNAMIC) { \
        vFinalR = sfrvf_zero(), vFinalG = sfrvf_zero(), vFinalB = sfrvf_zero(); \
        sfr__apply_dynamic_lights_simd(&fd, &props, &vFinalR, &vFinalG, &vFinalB); \
    } else /* LIGHT_MODE == SFR_RLT_UNLIT */ { \
        vFinalR = props.albedoR, vFinalG = props.albedoG, vFinalB = props.albedoB; \
    } \
    \
    const vf32 vShadow = HAS_SHADOW ? fd.shadow : sfrvf_set1(1.f); \
    sfr__pack_and_write_shaded_simd(globalInd, pWriteMask, &vFinalR, &vFinalG, &vFinalB, &vShadow); \
}

// (NAME, HAS_TEX, CALC_SURFACE, HAS_SHADOW, LIGHT_MODE)
SFR__DEF_CHUNK_RESOLVER(unlit_color,            0, 0, 0, SFR_RLT_UNLIT)
SFR__DEF_CHUNK_RESOLVER(unlit_tex,              1, 0, 0, SFR_RLT_UNLIT)

SFR__DEF_CHUNK_RESOLVER(lightmap_color,         0, 0, 0, SFR_RLT_LIGHTMAP)
SFR__DEF_CHUNK_RESOLVER(lightmap_tex,           1, 0, 0, SFR_RLT_LIGHTMAP)
SFR__DEF_CHUNK_RESOLVER(lightmap_color_shadow,  0, 0, 1, SFR_RLT_LIGHTMAP)
SFR__DEF_CHUNK_RESOLVER(lightmap_tex_shadow,    1, 0, 1, SFR_RLT_LIGHTMAP)
SFR__DEF_CHUNK_RESOLVER(lightmap_pbr,           1, 1, 0, SFR_RLT_LIGHTMAP)
SFR__DEF_CHUNK_RESOLVER(lightmap_pbr_shadow,    1, 1, 1, SFR_RLT_LIGHTMAP)

SFR__DEF_CHUNK_RESOLVER(dynamic_color,          0, 1, 0, SFR_RLT_DYNAMIC)
SFR__DEF_CHUNK_RESOLVER(dynamic_tex,            1, 1, 0, SFR_RLT_DYNAMIC)
SFR__DEF_CHUNK_RESOLVER(dynamic_color_shadow,   0, 1, 1, SFR_RLT_DYNAMIC)
SFR__DEF_CHUNK_RESOLVER(dynamic_tex_shadow,     1, 1, 1, SFR_RLT_DYNAMIC)

SFR__DEF_CHUNK_RESOLVER(lightmap_dynamic_color,         0, 1, 0, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_CHUNK_RESOLVER(lightmap_dynamic_tex,           1, 1, 0, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_CHUNK_RESOLVER(lightmap_dynamic_color_shadow,  0, 1, 1, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_CHUNK_RESOLVER(lightmap_dynamic_tex_shadow,    1, 1, 1, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_CHUNK_RESOLVER(lightmap_dynamic_pbr,           1, 1, 0, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_CHUNK_RESOLVER(lightmap_dynamic_pbr_shadow,    1, 1, 1, SFR_RLT_LIGHTMAP_DYNAMIC)

SFR__DEF_CHUNK_RESOLVER(debug_normals,          0, 0, 0, SFR_RLT_DEBUG_NORMALS)
SFR__DEF_CHUNK_RESOLVER(debug_fragpos,          0, 0, 0, SFR_RLT_DEBUG_FRAGPOS)

#endif // !SFR_NO_SIMD

// handles barycentrics, perspective correction, interpolation, and world/view vector math
static inline void sfr__process_geometry(
    const struct sfrRasterBin* rBin, 
    const struct sfrShadeBin* sBin, 
    i32 globalX, 
    i32 globalY, 
    struct sfrFragData* fd
) {
    // pixel centers
    const f32 cx = (f32)(globalX - rBin->minX) + 0.5f;
    const f32 cy = (f32)(globalY - rBin->minY) + 0.5f;

    // base barycentric weights
    const f32 w0 = (rBin->C1 + rBin->A1 * cx + rBin->B1 * cy) * sBin->invDet;
    const f32 w1 = (rBin->C2 + rBin->A2 * cx + rBin->B2 * cy) * sBin->invDet;
    const f32 w2 = (rBin->C0 + rBin->A0 * cx + rBin->B0 * cy) * sBin->invDet;

    // perspective correction (base)
    const f32 invZ = w0 * sBin->v0.invZ + w1 * sBin->v1.invZ + w2 * sBin->v2.invZ;
    fd->z = 1.f / invZ;
    
    // calculate perspective correct weights (sum to 1.0)
    const f32 pW0 = w0 * sBin->v0.invZ * fd->z;
    const f32 pW1 = w1 * sBin->v1.invZ * fd->z;
    const f32 pW2 = w2 * sBin->v2.invZ * fd->z;

    // unpack UVs
    const f32 v0u = sBin->v0.u, v0v = sBin->v0.v;
    const f32 v1u = sBin->v1.u, v1v = sBin->v1.v;
    const f32 v2u = sBin->v2.u, v2v = sBin->v2.v;
    const f32 v0lu = sfr__unpack_uv(sBin->v0.lu), v0lv = sfr__unpack_uv(sBin->v0.lv);
    const f32 v1lu = sfr__unpack_uv(sBin->v1.lu), v1lv = sfr__unpack_uv(sBin->v1.lv);
    const f32 v2lu = sfr__unpack_uv(sBin->v2.lu), v2lv = sfr__unpack_uv(sBin->v2.lv);

    // interpolate raw attributes directly into fragment data
    fd->cu = pW0 * v0u + pW1 * v1u + pW2 * v2u;
    fd->cv = pW0 * v0v + pW1 * v1v + pW2 * v2v;
    fd->lu = pW0 * v0lu + pW1 * v1lu + pW2 * v2lu;
    fd->lv = pW0 * v0lv + pW1 * v1lv + pW2 * v2lv;

    // unpack normals
    f32 v0nx, v0ny, v0nz;
    f32 v1nx, v1ny, v1nz;
    f32 v2nx, v2ny, v2nz;
    sfr__unpack_vec3(sBin->v0.n, &v0nx, &v0ny, &v0nz);
    sfr__unpack_vec3(sBin->v1.n, &v1nx, &v1ny, &v1nz);
    sfr__unpack_vec3(sBin->v2.n, &v2nx, &v2ny, &v2nz);

    // interpolate normals
    fd->normal = sfr_vec_normf(
        pW0 * v0nx + pW1 * v1nx + pW2 * v2nx,
        pW0 * v0ny + pW1 * v1ny + pW2 * v2ny,
        pW0 * v0nz + pW1 * v1nz + pW2 * v2nz
    );

    // convert pixel coordinates to NDC [-1, 1]
    const f32 ndcX = ((f32)globalX / sfrState.halfWidth) - 1.f;
    const f32 ndcY = 1.f - ((f32)globalY / sfrState.halfHeight);

    // un project NDC back to veiw space using the projection mat diagonals
    const f32 viewX = ndcX * fd->z / sfrMatProj.m[0][0];
    const f32 viewY = ndcY * fd->z / sfrMatProj.m[1][1];
    const f32 viewZ = fd->z; 

    // convert view space to world space using the inverse view mat
    fd->fragPos = sfr_mat_mul_vec(sfrState.matInvView, SFR__V(viewX, viewY, viewZ, 1.f));

    // fast view vector calculation
    const f32 vx = sfrCamPos.x - fd->fragPos.x;
    const f32 vy = sfrCamPos.y - fd->fragPos.y;
    const f32 vz = sfrCamPos.z - fd->fragPos.z;
    const f32 viewInvDist = sfr__fast_inv_sqrt(vx*vx + vy*vy + vz*vz + 0.0001f);
    fd->viewDir = SFR__V(vx * viewInvDist, vy * viewInvDist, vz * viewInvDist, 0.f);
}

// updates the shadow multiplier in the fragment data struct
static inline void sfr__sample_shadowmap(struct sfrFragData* fd) {
    fd->shadow = 1.f; // 1.0 = fully lit, 0.0 = fully in shadow

    if (!sfrState.activeShadowmap || !sfrState.activeShadowmap->depth) {
        return;
    }

    // transform world position to light NDC space [-1, 1]
    const sfrvec lightSpacePos = sfr_mat_mul_vec(sfrState.matLightVP,
        SFR__V(fd->fragPos.x, fd->fragPos.y, fd->fragPos.z, 1.f));

    // perspective divide (not needed for ortho?)
    const f32 invW = 1.f / lightSpacePos.w;
    const f32 ndcX = lightSpacePos.x * invW;
    const f32 ndcY = lightSpacePos.y * invW;
    const f32 ndcZ = lightSpacePos.z * invW;

    // map NDC to UV space [-1, 1] -> [0, 1]
    const f32 uvX = ndcX * 0.5f + 0.5f;
    const f32 uvY = 0.5f - ndcY * 0.5f; // -ndcY
    
    // ensure inside shadowmap bounds
    if (uvX >= 0.f && uvX <= 1.f && uvY >= 0.f && uvY <= 1.f) {
        const f32 texelX = 1.f / sfrState.activeShadowmap->width;
        const f32 texelY = 1.f / sfrState.activeShadowmap->height;
        const f32 fragDepth = 1.f - ndcZ;
        const f32 bias = sfrState.activeShadowmap->shadowBias;
        const f32 intensity = 1.f - sfrState.activeShadowmap->shadowStrength;

        f32 pcfShadow = 0.f;
        
        // 3x3 PCF kernel
        for (i32 y = -1; y <= 1; y += 1) {
            for (i32 x = -1; x <= 1; x += 1) {
                const f32 offsetX = uvX + x * texelX;
                const f32 offsetY = uvY + y * texelY;
                
                if (offsetX >= 0.f && offsetX <= 1.f && offsetY >= 0.f && offsetY <= 1.f) {
                    const i32 smX = (i32)(offsetX * (sfrState.activeShadowmap->width - 1));
                    const i32 smY = (i32)(offsetY * (sfrState.activeShadowmap->height - 1));
                    const i32 di = smY * sfrState.activeShadowmap->width + smX;
                    
                    const f32 closestDepth = sfrState.activeShadowmap->depth[di];
                    
                    // if the pixel is behind the shadow map depth it's in shadow, otherwise lit
                    pcfShadow += (fragDepth + bias < closestDepth) ? intensity : 1.f;
                } else {
                    // outside the shadow map is fully lit
                    pcfShadow += 1.f; 
                }
            }
        }
        // average the 9 samples
        fd->shadow = pcfShadow / 9.f;
    }
}

// extracts the base vertex/material color and default PBR values.
static inline void sfr__sample_material_base(
    const struct sfrShadeBin* sBin, const SfrMaterial* mat,
    struct sfrSurfaceProps* props
) {
    props->albedoR = (f32)((sBin->col >> 16) & 0xFF) / 255.f;
    props->albedoG = (f32)((sBin->col >> 8)  & 0xFF) / 255.f;
    props->albedoB = (f32)((sBin->col >> 0)  & 0xFF) / 255.f;

    props->metallic = mat->metallicFactor;
    // clamped to avoid division by zero later
    props->roughness = sfr_fmaxf(0.001f, mat->roughnessFactor);
}

// multiplies the base material properties by the sampled texture values
static inline void sfr__sample_material_tex(
    const SfrMaterial* mat, const struct sfrFragData* fd, i32 aLod,
    struct sfrSurfaceProps* props
) {
    if (!mat->albedoTex) {
        return;
    }

    const SfrTexture* tex = mat->albedoTex;

    // LOD precalculated
    const u32* const aMipPixels = tex->allPixels[aLod];
    const i32 currW = tex->allW[aLod];
    const i32 currH = tex->allH[aLod];

    // sample albedo map
    f32 acu = fd->cu - sfr_floorf(fd->cu);
    f32 acv = fd->cv - sfr_floorf(fd->cv);
    i32 atx, aty;
    i32 texInd;

    if (tex->isPot) {
        acu *= (f32)currW;
        acv *= (f32)currH;
        atx = (i32)acu & (currW - 1);
        aty = (i32)acv & (currH - 1);
        const i32 blockX = atx / 4;
        const i32 blockY = aty / 4;
        const i32 inBlockX = atx % 4;
        const i32 inBlockY = aty % 4;
        const i32 blockInd = (blockY << tex->strideShift[aLod]) + blockX;
        const i32 innerInd = inBlockY * 4 + inBlockX;
        texInd = blockInd * 16 + innerInd;
    } else {
        atx = (i32)(acu * currW);
        aty = (i32)(acv * currH);
        if (atx >= currW) atx = currW - 1;
        if (aty >= currH) aty = currH - 1;
        const i32 blockX = atx / 4;
        const i32 blockY = aty / 4;
        const i32 inBlockX = atx % 4;
        const i32 inBlockY = aty % 4;
        const i32 blocksW = tex->blocksW[aLod];
        const i32 blockInd = blockY * blocksW + blockX;
        const i32 innerInd = inBlockY * 4 + inBlockX;
        texInd = blockInd * 16 + innerInd;
    }

    const u32 col = aMipPixels[texInd];
    const f32 i255 = 1.f / 255.f;
    props->albedoR *= (f32)((col >> 16) & 0xFF) * i255;
    props->albedoG *= (f32)((col >> 8)  & 0xFF) * i255;
    props->albedoB *= (f32)((col >> 0)  & 0xFF) * i255;

    // assume metallicRoughnessTex and albedoTex have the same dimensions
    if (mat->metallicRoughnessTex) {
        const SfrTexture* mrTex = mat->metallicRoughnessTex;
        const u32* const mrMipPixels = mrTex->allPixels[aLod];

        // g = roughness, b = metallic
        const u32 mrCol = mrMipPixels[texInd];
        props->roughness = sfr_fmaxf(0.001f, (f32)((mrCol >> 8) & 0xFF) * i255);
        props->metallic = (f32)((mrCol >> 0) & 0xFF) * i255;
    }
}

// pre calculates the finalized diffuse, specular (F0), and shininess values 
static inline void sfr__calc_surface_props(struct sfrSurfaceProps* props) {
    // metals have no diffuse light, dielectrics keep their albedo
    props->diffR = props->albedoR * (1.f - props->metallic);
    props->diffG = props->albedoG * (1.f - props->metallic);
    props->diffB = props->albedoB * (1.f - props->metallic);

    // F0 is the base specular reflectance
    // dielectrics use 4%, metals use their base albedo
    props->f0R = 0.04f * (1.f - props->metallic) + props->albedoR * props->metallic;
    props->f0G = 0.04f * (1.f - props->metallic) + props->albedoG * props->metallic;
    props->f0B = 0.04f * (1.f - props->metallic) + props->albedoB * props->metallic;

    // convert PBR roughness to blinn-phong shininess
    const f32 alpha = props->roughness * props->roughness;
    f32 shininess = (2.f / (alpha * alpha)) - 2.f;
    props->shininess = sfr_fmaxf(1.f, sfr_fminf(shininess, 4096.f));

    props->energyConservation = (8.f + props->shininess) * (1.f / (8.f * SFR_PI));
}

// evaluates the base lightmap
static inline void sfr__apply_lightmap(
    const struct sfrFragData* fd, 
    const struct sfrSurfaceProps* props, 
    struct sfrLightmapState* lmState, 
    f32* finalR, f32* finalG, f32* finalB
) {
    const SfrTexture* lm = sfrState.activeLightmap;
    
    // map to pixel centers (-0.5 to sample the middle)
    f32 px = fd->lu * lm->w - 0.5f;
    f32 py = fd->lv * lm->h - 0.5f;
    
    // clamp to prevent reading outside atlas boundaries
    if (px < 0.f) px = 0.f;
    if (py < 0.f) py = 0.f;
    if (px >= lm->w - 1) px = (f32)(lm->w - 1) - 0.001f;
    if (py >= lm->h - 1) py = (f32)(lm->h - 1) - 0.001f;
    
    // get the base pixel coordinate
    const i32 tx0 = (i32)px;
    const i32 ty0 = (i32)py;
    
    // calculate the fractional blend weights (saved for direction map)
    lmState->fx = px - (f32)tx0;
    lmState->fy = py - (f32)ty0;
    
    // single index calculation for the quad
    lmState->ind = ty0 * lm->w + tx0;
    
    // fetch the pre-packed 2x2 quads
    const u32 qR = lm->quadR[lmState->ind];
    const u32 qG = lm->quadG[lmState->ind];
    const u32 qB = lm->quadB[lmState->ind];
    
    // unpack and blend red
    const f32 r00 = (f32)(qR & 0xFF);
    const f32 r10 = (f32)((qR >> 8) & 0xFF);
    const f32 r01 = (f32)((qR >> 16) & 0xFF);
    const f32 r11 = (f32)(qR >> 24);
    const f32 rTop = r00 + lmState->fx * (r10 - r00);
    const f32 rBot = r01 + lmState->fx * (r11 - r01);
    lmState->lmR = rTop + lmState->fy * (rBot - rTop);
    
    // unpack and blend green
    const f32 g00 = (f32)(qG & 0xFF);
    const f32 g10 = (f32)((qG >> 8) & 0xFF);
    const f32 g01 = (f32)((qG >> 16) & 0xFF);
    const f32 g11 = (f32)(qG >> 24);
    const f32 gTop = g00 + lmState->fx * (g10 - g00);
    const f32 gBot = g01 + lmState->fx * (g11 - g01);
    lmState->lmG = gTop + lmState->fy * (gBot - gTop);
    
    // unpack and blend blue
    const f32 b00 = (f32)(qB & 0xFF);
    const f32 b10 = (f32)((qB >> 8) & 0xFF);
    const f32 b01 = (f32)((qB >> 16) & 0xFF);
    const f32 b11 = (f32)(qB >> 24);
    const f32 bTop = b00 + lmState->fx * (b10 - b00);
    const f32 bBot = b01 + lmState->fx * (b11 - b01);
    lmState->lmB = bTop + lmState->fy * (bBot - bTop);

    // multiply base albedo and scale
    const f32 i255 = 1.f / 255.f;
    *finalR = props->albedoR * (lmState->lmR * i255);
    *finalG = props->albedoG * (lmState->lmG * i255);
    *finalB = props->albedoB * (lmState->lmB * i255);
}

// evaluates the specular highlight from a directional lightmap atlas
static inline void sfr__apply_directionmap(
    const struct sfrFragData* fd, 
    const struct sfrSurfaceProps* props, 
    const struct sfrLightmapState* lmState, 
    f32* finalR, f32* finalG, f32* finalB
) {
    const SfrTexture* dm = sfrState.activeDirectionmap;
    
    const u32 qR = dm->quadR[lmState->ind];
    const u32 qG = dm->quadG[lmState->ind];
    const u32 qB = dm->quadB[lmState->ind];
    
    const f32 rTop = (f32)(qR & 0xFF) + lmState->fx * ((f32)((qR >> 8) & 0xFF) - (f32)(qR & 0xFF));
    const f32 rBot = (f32)((qR >> 16) & 0xFF) + lmState->fx * ((f32)(qR >> 24) - (f32)((qR >> 16) & 0xFF));
    const f32 dirR = rTop + lmState->fy * (rBot - rTop);
    
    const f32 gTop = (f32)(qG & 0xFF) + lmState->fx * ((f32)((qG >> 8) & 0xFF) - (f32)(qG & 0xFF));
    const f32 gBot = (f32)((qG >> 16) & 0xFF) + lmState->fx * ((f32)(qG >> 24) - (f32)((qG >> 16) & 0xFF));
    const f32 dirG = gTop + lmState->fy * (gBot - gTop);
    
    const f32 bTop = (f32)(qB & 0xFF) + lmState->fx * ((f32)((qB >> 8) & 0xFF) - (f32)(qB & 0xFF));
    const f32 bBot = (f32)((qB >> 16) & 0xFF) + lmState->fx * ((f32)(qB >> 24) - (f32)((qB >> 16) & 0xFF));
    const f32 dirB = bTop + lmState->fy * (bBot - bTop);

    const f32 i255 = 1.f / 255.f;
    const f32 dx = (dirR * i255) * 2.f - 1.f;
    const f32 dy = (dirG * i255) * 2.f - 1.f;
    const f32 dz = (dirB * i255) * 2.f - 1.f;
    const f32 dirLen = sfr_sqrtf(dx*dx + dy*dy + dz*dz);
    
    if (dirLen > 0.01f) {
        const f32 invLen = 1.f / dirLen;
        const sfrvec lDir = SFR__V(dx * invLen, dy * invLen, dz * invLen, 0.f);

        const sfrvec halfDir = sfr_vec_norm(sfr_vec_add(lDir, fd->viewDir));

        const f32 ndotl = sfr_fmaxf(0.f, sfr_vec_dot(fd->normal, lDir));
        if (ndotl > 0.f) {
            const f32 ndoth = sfr_fmaxf(0.0001f, sfr_vec_dot(fd->normal, halfDir));
            f32 spec = sfr__fast_pow(ndoth, props->shininess);
            spec = sfr_fminf(spec, 1000.f);
            const f32 specIntensity = spec * props->energyConservation * dirLen;
            
            *finalR += specIntensity * (lmState->lmR * i255) * props->f0R;
            *finalG += specIntensity * (lmState->lmG * i255) * props->f0G;
            *finalB += specIntensity * (lmState->lmB * i255) * props->f0B;
        }
    }
}

// accumulates all active dynamic lights into the final color.
static inline void sfr__apply_dynamic_lights(
    const struct sfrFragData* fd, 
    const struct sfrSurfaceProps* props, 
    f32* finalR, f32* finalG, f32* finalB
) {
    for (i32 i = 0; i < sfrState.lightCount; i += 1) {
        const SfrLight* const light = &sfrState.lights[i];

        sfrvec lightDir;
        f32 attenuation = 1.f;

        if (SFR_LIGHT_DIRECTIONAL == light->type) {
            lightDir = sfr_vec_normf(-light->x, -light->y, -light->z);
        } else /* if (SFR_LIGHT_POINT == light->type) */ {
            const sfrvec diffPos = SFR__V(light->x - fd->fragPos.x, light->y - fd->fragPos.y, light->z - fd->fragPos.z, 0.f);
            const f32 dist = sfr_vec_length(diffPos);
            lightDir = sfr_vec_mul(diffPos, 1.f / dist);

            // quadratic falloff
            attenuation = sfr_fmaxf(0.f, 1.f - (dist / light->attenuation));
            attenuation *= attenuation;
        }

        // diffuse term
        const f32 diff = sfr_fmaxf(0.f, sfr_vec_dot(fd->normal, lightDir));

        // specular term
        f32 spec = 0.f;
        if (diff > 0.f) {
            sfrvec halfwayDir = sfr_vec_norm(sfr_vec_add(lightDir, fd->viewDir));

            const f32 ndoth = sfr_fmaxf(0.0001f, sfr_vec_dot(fd->normal, halfwayDir));
            spec = sfr__fast_pow(ndoth, props->shininess);
            spec = sfr_fminf(spec, 1000.f);
            spec *= props->energyConservation;
        }

        const f32 diffuseIntensity = diff * light->intensity * attenuation;
        const f32 specularIntensity = spec * diffuseIntensity;

        *finalR += (light->ambient + diffuseIntensity)  * light->r * props->diffR +
                   (light->ambient + specularIntensity) * light->r * props->f0R;

        *finalG += (light->ambient + diffuseIntensity)  * light->g * props->diffG +
                   (light->ambient + specularIntensity) * light->g * props->f0G;

        *finalB += (light->ambient + diffuseIntensity)  * light->b * props->diffB +
                   (light->ambient + specularIntensity) * light->b * props->f0B;
    }
}

static inline void sfr__pack_and_write_shaded(i32 globalInd, f32 finalR, f32 finalG, f32 finalB, f32 shadow) {
    finalR = SFR_CLAMP(finalR * shadow, 0.f, 1.f);
    finalG = SFR_CLAMP(finalG * shadow, 0.f, 1.f);
    finalB = SFR_CLAMP(finalB * shadow, 0.f, 1.f);
    
    #ifndef SFR_NO_GAMMA_CORRECT
        const u8 fr = sfrGammaLUT[(i32)(finalR * 255.f)];
        const u8 fg = sfrGammaLUT[(i32)(finalG * 255.f)];
        const u8 fb = sfrGammaLUT[(i32)(finalB * 255.f)];
    #else
        const u8 fr = finalR * 255.f;
        const u8 fg = finalG * 255.f;
        const u8 fb = finalB * 255.f;
    #endif

    sfrPixelBuf[globalInd] = (0xFF << 24) | (fr << 16) | (fg << 8) | fb;
}

static inline void sfr__pack_and_write_normals(i32 globalInd, const sfrvec* normal) {
    const u8 fr = (u8)(sfr_fmaxf(0.f, sfr_fminf(normal->x * 0.5f + 0.5f, 1.f)) * 255.f);
    const u8 fg = (u8)(sfr_fmaxf(0.f, sfr_fminf(normal->y * 0.5f + 0.5f, 1.f)) * 255.f);
    const u8 fb = (u8)(sfr_fmaxf(0.f, sfr_fminf(normal->z * 0.5f + 0.5f, 1.f)) * 255.f);

    sfrPixelBuf[globalInd] = (0xFF << 24) | (fr << 16) | (fg << 8) | fb;
}

static inline void sfr__pack_and_write_fragpos(i32 globalInd, const sfrvec* fragPos) {
    const f32 fx = fragPos->x - sfr_floorf(fragPos->x);
    const f32 fy = fragPos->y - sfr_floorf(fragPos->y);
    const f32 fz = fragPos->z - sfr_floorf(fragPos->z);
    
    const u8 fr = (u8)(fx * 255.f);
    const u8 fg = (u8)(fy * 255.f);
    const u8 fb = (u8)(fz * 255.f);

    sfrPixelBuf[globalInd] = (0xFF << 24) | (fr << 16) | (fg << 8) | fb;
}

#define SFR__DEF_PIXEL_RESOLVER(NAME, HAS_TEX, CALC_SURFACE, HAS_SHADOW, LIGHT_MODE) \
static void _sfr__resolve_pixel_##NAME(const struct sfrRasterBin* rBin, const struct sfrShadeBin* sBin, i32 globalX, i32 globalY, i32 globalInd, i32 aLod) { \
    (void)aLod; \
    \
    struct sfrFragData fd; \
    sfr__process_geometry(rBin, sBin, globalX, globalY, &fd); \
    if (LIGHT_MODE == SFR_RLT_DEBUG_NORMALS) { \
        sfr__pack_and_write_normals(globalInd, &fd.normal); \
        return; \
    } \
    if (LIGHT_MODE == SFR_RLT_DEBUG_FRAGPOS) { \
        sfr__pack_and_write_fragpos(globalInd, &fd.fragPos); \
        return; \
    } \
    if (HAS_SHADOW) sfr__sample_shadowmap(&fd); \
    \
    struct sfrSurfaceProps props; \
    sfr__sample_material_base(sBin, sBin->mat, &props); \
    if (HAS_TEX) sfr__sample_material_tex(sBin->mat, &fd, aLod, &props); \
    if (CALC_SURFACE) sfr__calc_surface_props(&props); \
    \
    f32 finalR, finalG, finalB; \
    if (LIGHT_MODE == SFR_RLT_LIGHTMAP || LIGHT_MODE == SFR_RLT_LIGHTMAP_DYNAMIC) { \
        struct sfrLightmapState lmState; \
        finalR = 0.f; finalG = 0.f; finalB = 0.f; \
        sfr__apply_lightmap(&fd, &props, &lmState, &finalR, &finalG, &finalB); \
        if (CALC_SURFACE) sfr__apply_directionmap(&fd, &props, &lmState, &finalR, &finalG, &finalB); \
        if (LIGHT_MODE == SFR_RLT_LIGHTMAP_DYNAMIC) sfr__apply_dynamic_lights(&fd, &props, &finalR, &finalG, &finalB); \
    } else if (LIGHT_MODE == SFR_RLT_DYNAMIC) { \
        finalR = 0.f; finalG = 0.f; finalB = 0.f; \
        sfr__apply_dynamic_lights(&fd, &props, &finalR, &finalG, &finalB); \
    } else /* LIGHT_MODE == SFR_RLT_UNLIT */ { \
        finalR = props.albedoR, finalG = props.albedoG, finalB = props.albedoB; \
    } \
    \
    sfr__pack_and_write_shaded(globalInd, finalR, finalG, finalB, HAS_SHADOW ? fd.shadow : 1.f); \
}

// (NAME, HAS_TEX, CALC_SURFACE, HAS_SHADOW, LIGHT_MODE)
SFR__DEF_PIXEL_RESOLVER(unlit_color,            0, 0, 0, SFR_RLT_UNLIT)
SFR__DEF_PIXEL_RESOLVER(unlit_tex,              1, 0, 0, SFR_RLT_UNLIT)

SFR__DEF_PIXEL_RESOLVER(lightmap_color,         0, 0, 0, SFR_RLT_LIGHTMAP)
SFR__DEF_PIXEL_RESOLVER(lightmap_tex,           1, 0, 0, SFR_RLT_LIGHTMAP)
SFR__DEF_PIXEL_RESOLVER(lightmap_color_shadow,  0, 0, 1, SFR_RLT_LIGHTMAP)
SFR__DEF_PIXEL_RESOLVER(lightmap_tex_shadow,    1, 0, 1, SFR_RLT_LIGHTMAP)
SFR__DEF_PIXEL_RESOLVER(lightmap_pbr,           1, 1, 0, SFR_RLT_LIGHTMAP)
SFR__DEF_PIXEL_RESOLVER(lightmap_pbr_shadow,    1, 1, 1, SFR_RLT_LIGHTMAP)

SFR__DEF_PIXEL_RESOLVER(dynamic_color,          0, 1, 0, SFR_RLT_DYNAMIC)
SFR__DEF_PIXEL_RESOLVER(dynamic_tex,            1, 1, 0, SFR_RLT_DYNAMIC)
SFR__DEF_PIXEL_RESOLVER(dynamic_color_shadow,   0, 1, 1, SFR_RLT_DYNAMIC)
SFR__DEF_PIXEL_RESOLVER(dynamic_tex_shadow,     1, 1, 1, SFR_RLT_DYNAMIC)

SFR__DEF_PIXEL_RESOLVER(lightmap_dynamic_color,         0, 1, 0, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_PIXEL_RESOLVER(lightmap_dynamic_tex,           1, 1, 0, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_PIXEL_RESOLVER(lightmap_dynamic_color_shadow,  0, 1, 1, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_PIXEL_RESOLVER(lightmap_dynamic_tex_shadow,    1, 1, 1, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_PIXEL_RESOLVER(lightmap_dynamic_pbr,           1, 1, 0, SFR_RLT_LIGHTMAP_DYNAMIC)
SFR__DEF_PIXEL_RESOLVER(lightmap_dynamic_pbr_shadow,    1, 1, 1, SFR_RLT_LIGHTMAP_DYNAMIC)

SFR__DEF_PIXEL_RESOLVER(debug_normals,          0, 0, 0, SFR_RLT_DEBUG_NORMALS)
SFR__DEF_PIXEL_RESOLVER(debug_fragpos,          0, 0, 0, SFR_RLT_DEBUG_FRAGPOS)

static void sfr__resolve_tile(const struct sfrTile* tile, i32* targetIdBuf) {
    const i32 w = tile->maxX - tile->minX;
    const i32 h = tile->maxY - tile->minY;

    typedef void (*pixelResolverFunc)(
        const struct sfrRasterBin* rBin, const struct sfrShadeBin* sBin,
        i32 globalX, i32 globalY, i32 globalInd, i32 aLod);
    const pixelResolverFunc pixelResolvers[SFR_PATH_COUNT] = {
        [SFR_PATH_UNLIT_COLOR] = _sfr__resolve_pixel_unlit_color,
        [SFR_PATH_UNLIT_TEX] = _sfr__resolve_pixel_unlit_tex,
        [SFR_PATH_LIGHTMAP_COLOR] = _sfr__resolve_pixel_lightmap_color,
        [SFR_PATH_LIGHTMAP_TEX] = _sfr__resolve_pixel_lightmap_tex,
        [SFR_PATH_LIGHTMAP_PBR] = _sfr__resolve_pixel_lightmap_pbr,
        [SFR_PATH_LIGHTMAP_COLOR_SHADOW] = _sfr__resolve_pixel_lightmap_color_shadow,
        [SFR_PATH_LIGHTMAP_TEX_SHADOW] = _sfr__resolve_pixel_lightmap_tex_shadow,
        [SFR_PATH_LIGHTMAP_PBR_SHADOW] = _sfr__resolve_pixel_lightmap_pbr_shadow,
        [SFR_PATH_DYNAMIC_COLOR] = _sfr__resolve_pixel_dynamic_color,
        [SFR_PATH_DYNAMIC_TEX] = _sfr__resolve_pixel_dynamic_tex,
        [SFR_PATH_DYNAMIC_COLOR_SHADOW] = _sfr__resolve_pixel_dynamic_color_shadow,
        [SFR_PATH_DYNAMIC_TEX_SHADOW] = _sfr__resolve_pixel_dynamic_tex_shadow,
        [SFR_PATH_LIGHTMAP_DYNAMIC_COLOR] = _sfr__resolve_pixel_lightmap_dynamic_color,
        [SFR_PATH_LIGHTMAP_DYNAMIC_TEX] = _sfr__resolve_pixel_lightmap_dynamic_tex,
        [SFR_PATH_LIGHTMAP_DYNAMIC_PBR] = _sfr__resolve_pixel_lightmap_dynamic_pbr,
        [SFR_PATH_LIGHTMAP_DYNAMIC_COLOR_SHADOW] = _sfr__resolve_pixel_lightmap_dynamic_color_shadow,
        [SFR_PATH_LIGHTMAP_DYNAMIC_TEX_SHADOW] = _sfr__resolve_pixel_lightmap_dynamic_tex_shadow,
        [SFR_PATH_LIGHTMAP_DYNAMIC_PBR_SHADOW] = _sfr__resolve_pixel_lightmap_dynamic_pbr_shadow,
        [SFR_PATH_DEBUG_NORMALS] = _sfr__resolve_pixel_debug_normals,
        [SFR_PATH_DEBUG_FRAGPOS] = _sfr__resolve_pixel_debug_fragpos,
    };

#ifndef SFR_NO_SIMD

    typedef void (*chunkResolverFunc)(const struct sfrResolveChunkArgs* args);
    const chunkResolverFunc chunkResolvers[SFR_PATH_COUNT] = {
        [SFR_PATH_UNLIT_COLOR] = _sfr__resolve_chunk_unlit_color,
        [SFR_PATH_UNLIT_TEX] = _sfr__resolve_chunk_unlit_tex,
        [SFR_PATH_LIGHTMAP_COLOR] = _sfr__resolve_chunk_lightmap_color,
        [SFR_PATH_LIGHTMAP_TEX] = _sfr__resolve_chunk_lightmap_tex,
        [SFR_PATH_LIGHTMAP_PBR] = _sfr__resolve_chunk_lightmap_pbr,
        [SFR_PATH_LIGHTMAP_COLOR_SHADOW] = _sfr__resolve_chunk_lightmap_color_shadow,
        [SFR_PATH_LIGHTMAP_TEX_SHADOW] = _sfr__resolve_chunk_lightmap_tex_shadow,
        [SFR_PATH_LIGHTMAP_PBR_SHADOW] = _sfr__resolve_chunk_lightmap_pbr_shadow,
        [SFR_PATH_DYNAMIC_COLOR] = _sfr__resolve_chunk_dynamic_color,
        [SFR_PATH_DYNAMIC_TEX] = _sfr__resolve_chunk_dynamic_tex,
        [SFR_PATH_DYNAMIC_COLOR_SHADOW] = _sfr__resolve_chunk_dynamic_color_shadow,
        [SFR_PATH_DYNAMIC_TEX_SHADOW] = _sfr__resolve_chunk_dynamic_tex_shadow,
        [SFR_PATH_LIGHTMAP_DYNAMIC_COLOR] = _sfr__resolve_chunk_lightmap_dynamic_color,
        [SFR_PATH_LIGHTMAP_DYNAMIC_TEX] = _sfr__resolve_chunk_lightmap_dynamic_tex,
        [SFR_PATH_LIGHTMAP_DYNAMIC_PBR] = _sfr__resolve_chunk_lightmap_dynamic_pbr,
        [SFR_PATH_LIGHTMAP_DYNAMIC_COLOR_SHADOW] = _sfr__resolve_chunk_lightmap_dynamic_color_shadow,
        [SFR_PATH_LIGHTMAP_DYNAMIC_TEX_SHADOW] = _sfr__resolve_chunk_lightmap_dynamic_tex_shadow,
        [SFR_PATH_LIGHTMAP_DYNAMIC_PBR_SHADOW] = _sfr__resolve_chunk_lightmap_dynamic_pbr_shadow,
        [SFR_PATH_DEBUG_NORMALS] = _sfr__resolve_chunk_debug_normals,
        [SFR_PATH_DEBUG_FRAGPOS] = _sfr__resolve_chunk_debug_fragpos,
    };

    for (i32 y = 0; y < h; y += 1) {
        const i32 globalY = tile->minY + y;
        const i32 globalIndBase = globalY * sfrWidth + tile->minX;

        #ifdef SFR_MULTITHREADED
            const i32 bufIndBase = y * SFR_TILE_WIDTH;
        #else
            const i32 bufIndBase = globalIndBase;
        #endif

        i32 x = 0;

        for (; x <= w - SFR_SIMD_LANES; x += SFR_SIMD_LANES) {
            const i32 bufInd = bufIndBase + x;
            const i32 globalInd = globalIndBase + x;
            const i32 globalX = tile->minX + x;

            const vi32 vIds = sfrvi_loadu((vi32*)&targetIdBuf[bufInd]);

            i32 processedMask = 0;
            const i32 laneMask = (1 << SFR_SIMD_LANES) - 1;

            // loop until all valid lanes in this chunk have been resolved
            while ((processedMask & laneMask) != laneMask) {
                const i32 remainingBits = (~processedMask) & laneMask;
                
                // prevent bitscan on 0
                if (0 == remainingBits) {
                    break; 
                }

                // find the id of the first pixel not yet processed
                i32 firstUnprocessedBit = 0;
                #if defined(_MSC_VER) && !defined(__clang__)
                    u64 bitInd;
                    _BitScanForward(&bitInd, remainingBits);
                    firstUnprocessedBit = (i32)bitInd;
                #else
                    firstUnprocessedBit = __builtin_ctz(remainingBits);
                #endif
                
                const i32 currId = targetIdBuf[bufInd + firstUnprocessedBit];

                // compare all pixels in chunk to the current ID
                const vi32 vCmpMask = sfrvi_cmpeq(vIds, sfrvi_set1(currId));
                
                // force clamp the movemask to valid lanes
                const i32 currLaneMask = sfrvf_movemask(sfrvf_cast_from_vi32(vCmpMask)) & laneMask;

                // if garbage memory caused zero matches, force progress
                if (0 == currLaneMask) {
                    processedMask |= (1 << firstUnprocessedBit);
                    continue;
                }

                processedMask |= currLaneMask;

                // only resolve if it's an actual triangle (not empty background)
                if (currId >= 0) {
                    // vCmpMask is already vi32, no need to cast it back
                    const struct sfrRasterBin* rBin;
                    const struct sfrShadeBin* sBin;
                    sfr__get_bins(currId, &rBin, &sBin);

                    const struct sfrResolveChunkArgs args = {
                        .rBin = rBin, .sBin = sBin,
                        .globalX = globalX,
                        .globalY = tile->minY + y,
                        .globalInd = globalInd,
                        .pWriteMask = &vCmpMask
                    };

                    chunkResolvers[sBin->renderPathInd](&args);
                }
            }
        }

        // tail loop fallback for non aligned chunks
        for (; x < w; x += 1) {
            const i32 bufInd = bufIndBase + x;
            const i32 globalInd = globalIndBase + x;
            const i32 globalX = tile->minX + x;

            const i32 id = targetIdBuf[bufInd];
            if (-1 != id) {
                const struct sfrRasterBin* rBin;
                const struct sfrShadeBin* sBin;
                sfr__get_bins(id, &rBin, &sBin);
                pixelResolvers[sBin->renderPathInd](rBin, sBin, globalX, globalY, globalInd, sfr__calc_lod(rBin, sBin, globalX, globalY));
            }
        }
    }

#else // !SFR_NO_SIMD

    // step by 2 vertically and horizontally
    for (i32 y = 0; y < h; y += 2) {
        const i32 yLimit = SFR_MIN(y + 2, h);

        for (i32 x = 0; x < w; x += 2) {
            const i32 xLimit = SFR_MIN(x + 2, w);

            i32 ids[4] = {-1, -1, -1, -1};
            i32 idCount = 0;
            i32 firstId = -1;
            u8 allSame = 1;

            // gather all target IDs in the 2x2 quad
            for (i32 qy = y; qy < yLimit; qy += 1) {
                #ifdef SFR_MULTITHREADED
                    const i32 rowBase = qy * SFR_TILE_WIDTH;
                #else
                    const i32 globalY = tile->minY + qy;
                    const i32 rowBase = globalY * sfrWidth;
                #endif

                for (i32 qx = x; qx < xLimit; qx += 1) {
                    #ifdef SFR_MULTITHREADED
                        const i32 bufInd = rowBase + qx;
                    #else
                        const i32 bufInd = rowBase + (tile->minX + qx);
                    #endif

                    const i32 id = targetIdBuf[bufInd];
                    ids[(qy - y) * 2 + (qx - x)] = id;
                    
                    if (-1 != id) {
                        idCount += 1;
                        if (-1 == firstId) {
                            firstId = id;
                        } else if (firstId != id) {
                            allSame = 0;
                        }
                    }
                }
            }

            if (0 == idCount) {
                continue; // Empty block
            }

            // coarse LOD, evaluate once if the whole quad belongs to the same triangle
            i32 sharedLod = 0;
            if (allSame) {
                const struct sfrRasterBin* rBin;
                const struct sfrShadeBin* sBin;
                sfr__get_bins(firstId, &rBin, &sBin);
                sharedLod = sfr__calc_lod(rBin, sBin, tile->minX + x, tile->minY + y);
            }

            // resolve the pixels
            for (i32 qy = y; qy < yLimit; qy += 1) {
                const i32 globalY = tile->minY + qy;
                for (i32 qx = x; qx < xLimit; qx += 1) {
                    const i32 id = ids[(qy - y) * 2 + (qx - x)];
                    if (-1 == id) {
                        continue;
                    }

                    const i32 globalX = tile->minX + qx;
                    const i32 globalInd = globalY * sfrWidth + globalX;

                    const struct sfrRasterBin* rBin;
                    const struct sfrShadeBin* sBin;
                    sfr__get_bins(id, &rBin, &sBin);

                    // fall back to individual evaluation if on a triangle edge
                    i32 aLod = sharedLod;
                    if (!allSame) {
                        aLod = sfr__calc_lod(rBin, sBin, globalX, globalY);
                    }

                    pixelResolvers[sBin->renderPathInd](rBin, sBin, globalX, globalY, globalInd, aLod);
                }
            }
        }
    }
#endif // SFR_NO_SIMD
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

    struct sfrGeomClipBufs clipBuf;
    struct sfrGeomTri gt = {
        .posA  = (f32[3]){ax, ay, az},
        .uvA   = (f32[2]){au, av},
        .lmUvA = (f32[2]){0.f, 0.f},
        .normA = (f32[3]){anx, any, anz},
        .tanA  = (f32[4]){0.f, 0.f, 0.f, 0.f},

        .posB  = (f32[3]){bx, by, bz},
        .uvB   = (f32[2]){bu, bv},
        .lmUvB = (f32[2]){0.f, 0.f},
        .normB = (f32[3]){bnx, bny, bnz},
        .tanB  = (f32[4]){0.f, 0.f, 0.f, 0.f},

        .posC  = (f32[3]){cx, cy, cz},
        .uvC   = (f32[2]){cu, cv},
        .lmUvC = (f32[2]){0.f, 0.f},
        .normC = (f32[3]){cnx, cny, cnz},
        .tanC  = (f32[4]){0.f, 0.f, 0.f, 0.f},

        .col = col,
        .mat = mat,

        .renderMode = sfrState.renderMode,

        .clipBuf = &clipBuf
    };

    const sfrmat matMVP = sfr_mat_mul(sfr_mat_mul(sfrMatModel, sfrMatView), sfrMatProj);

    sfr__process_and_bin_triangle(&matMVP, &sfrState.matNormal, &gt);
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
    tex->isPot = (0 == (tex->w & (tex->w - 1))) && (0 == (tex->h & (tex->h - 1)));
    
    // calculate total memory needed for all levels (including 0) padded to 4x4
    i32 cw = tex->w, ch = tex->h;
    i32 totalPaddedPixels = 0;
    i32 levels = 0;
    
    while (levels < (i32)SFR_ARRLEN(tex->allPixels)) {
        // pad to multiple of 4
        const i32 pw = (cw + 3) & ~3;
        const i32 ph = (ch + 3) & ~3;
        totalPaddedPixels += pw * ph;
        
        levels += 1;
        if (1 == cw && 1 == ch) {
            break;
        }
        
        cw = (cw > 1) ? cw / 2 : 1;
        ch = (ch > 1) ? ch / 2 : 1;
    }
    tex->mipLevels = levels;
    
    // allocate one contiguous block for all swizzled mips
    u32* mipMemory;
    SFR__MALLOC(mipMemory, sizeof(u32) * totalPaddedPixels);
    u32* currSwizzleDest = mipMemory;

    // helper macro to swizzle linear data into the 4x4 block layout
    #define DO_SWIZZLE(lvl, srcW, srcH, linearSrc) { \
        const i32 pw = (srcW + 3) & ~3; \
        const i32 ph = (srcH + 3) & ~3; \
        const i32 bw = pw / 4; \
        tex->allPixels[lvl] = currSwizzleDest; \
        tex->allW[lvl] = srcW; \
        tex->allH[lvl] = srcH; \
        tex->blocksW[lvl] = bw; \
        tex->strideShift[lvl] = tex->isPot ? sfr__fast_log2(bw) : 0; \
        for (i32 y = 0; y < srcH; y += 1) { \
            for (i32 x = 0; x < srcW; x += 1) { \
                const i32 bX = x / 4, bY = y / 4, inX = x % 4, inY = y % 4; \
                const i32 dInd = (bY * bw + bX) * 16 + (inY * 4) + inX; \
                currSwizzleDest[dInd] = linearSrc[y * srcW + x]; \
            } \
        } \
        currSwizzleDest += (pw * ph); \
    }

    // swizzle level 0
    DO_SWIZZLE(0, tex->w, tex->h, tex->pixels);

    // generate subsequent mips using linear temp buffers
    cw = tex->w; ch = tex->h;
    u32* srcLinear = tex->pixels;
    u32* tempLinear = NULL;

    for (i32 currLevel = 1; currLevel < tex->mipLevels; currLevel += 1) {
        const i32 nw = (cw > 1) ? cw / 2 : 1;
        const i32 nh = (ch > 1) ? ch / 2 : 1;

        u32* destLinear;
        SFR__MALLOC(destLinear, sizeof(u32) * nw * nh);

        // tent filter
        for (i32 y = 0; y < nh; y += 1) {
            for (i32 x = 0; x < nw; x += 1) {
                const i32 cx = x * 2;
                const i32 cy = y * 2;

                u32 r = 0, g = 0, b = 0, a = 0;
                u32 wsum = 0;

                for (i32 ky = -1; ky <= 2; ky += 1) {
                    const i32 sy = SFR_CLAMP(cy + ky, 0, ch - 1);

                    const u32 wy = (-1 == ky || 2 == ky) ? 1 : 2;

                    for (i32 kx = -1; kx <= 2; kx += 1) {
                        const i32 sx = SFR_CLAMP(cx + kx, 0, cw - 1);

                        const u32 wx = (-1 == kx || 2 == kx) ? 1 : 2;

                        const u32 w = wx * wy;
                        const u32 c = srcLinear[sy * cw + sx];

                        r += ((c >> 16) & 255) * w;
                        g += ((c >> 8)  & 255) * w;
                        b += ((c >> 0)  & 255) * w;
                        a += ((c >> 24) & 255) * w;

                        wsum += w;
                    }
                }

                r /= wsum;
                g /= wsum;
                b /= wsum;
                a /= wsum;

                destLinear[y * nw + x] = (a << 24) | (r << 16) | (g << 8) | b;
            }
        }

        // swizzle generated mip and prep for next iteration
        DO_SWIZZLE(currLevel, nw, nh, destLinear);

        if (tempLinear) {
            SFR__FREE(tempLinear);
        }
        tempLinear = destLinear;
        srcLinear = tempLinear;
        cw = nw;
        ch = nh;
    }

    if (tempLinear) {
        SFR__FREE(tempLinear);
    }
    
    #undef DO_SWIZZLE
}

// resize a single texture in place
static void sfr__resize_texture_in_place(SfrTexture* tex, i32 newW, i32 newH) {
    if (!tex || (tex->w == newW && tex->h == newH)) {
        return;
    }

    u32* newPixels;
    SFR__MALLOC(newPixels, sizeof(u32) * newW * newH);

    // ratio of source pixels to dest pixels
    const f32 scaleX = (f32)tex->w / (f32)newW;
    const f32 scaleY = (f32)tex->h / (f32)newH;

    for (i32 dy = 0; dy < newH; dy += 1) {
        // vertical footprint of this dest pixel
        const f32 srcY0 = dy * scaleY;
        const f32 srcY1 = (dy + 1) * scaleY;
        
        const i32 syMin = (i32)srcY0;
        i32 syMax = (i32)srcY1 + ((srcY1 > (f32)(i32)srcY1) ? 1 : 0);
        if (syMax > tex->h) syMax = tex->h;

        for (i32 dx = 0; dx < newW; dx += 1) {
            // horizontal footprint of this dest pixel
            const f32 srcX0 = dx * scaleX;
            const f32 srcX1 = (dx + 1) * scaleX;
            
            const i32 sxMin = (i32)srcX0;
            i32 sxMax = (i32)srcX1 + ((srcX1 > (f32)(i32)srcX1) ? 1 : 0);
            if (sxMax > tex->w) sxMax = tex->w;

            f32 rSum = 0.f, gSum = 0.f, bSum = 0.f, aSum = 0.f;
            f32 weightSum = 0.f;

            // loop over every source pixel that touches this footprint
            for (i32 sy = syMin; sy < syMax; sy += 1) {
                // vertical fractional coverage
                f32 yWeight = 1.f;
                if (sy < srcY0) yWeight = (sy + 1.f) - srcY0;    // partial top edge
                else if (sy + 1.f > srcY1) yWeight = srcY1 - sy; // partial bottom edge

                for (i32 sx = sxMin; sx < sxMax; sx += 1) {
                    // horizontal fractional coverage
                    f32 xWeight = 1.f;
                    if (sx < srcX0) { // partial left edge
                        xWeight = (sx + 1.f) - srcX0;
                    } else if (sx + 1.f > srcX1) { // partial right edge
                        xWeight = srcX1 - sx;
                    }

                    const f32 weight = xWeight * yWeight;
                    const u32 col = tex->pixels[sy * tex->w + sx];

                    // accumulate weighted colors
                    rSum += ((col >> 16) & 0xFF) * weight;
                    gSum += ((col >> 8)  & 0xFF) * weight;
                    bSum += ((col >> 0)  & 0xFF) * weight;
                    aSum += ((col >> 24) & 0xFF) * weight;
                    weightSum += weight;
                }
            }

            // normalize
            const f32 invWeight = 1.f / weightSum;
            u32 r = (u32)(rSum * invWeight + 0.5f);
            u32 g = (u32)(gSum * invWeight + 0.5f);
            u32 b = (u32)(bSum * invWeight + 0.5f);
            u32 a = (u32)(aSum * invWeight + 0.5f);

            // clamp to avoid overflow
            if (r > 255) r = 255;
            if (g > 255) g = 255;
            if (b > 255) b = 255;
            if (a > 255) a = 255;

            newPixels[dy * newW + dx] = (a << 24) | (r << 16) | (g << 8) | b;
        }
    }

    SFR__FREE(tex->pixels);
    if (tex->mipLevels > 0) {
        SFR__FREE(tex->allPixels[0]);
    }

    tex->pixels = newPixels;
    tex->w = newW;
    tex->h = newH;

    sfr__generate_mipmaps(tex);
}

#ifdef SFR_MULTITHREADED

static void sfr__ensure_tile_cap(i32 width, i32 height) {
    const i32 cols = (width + SFR_TILE_WIDTH - 1) / SFR_TILE_WIDTH;
    const i32 rows = (height + SFR_TILE_HEIGHT - 1) / SFR_TILE_HEIGHT;
    const i32 count = cols * rows;

    if (count <= sfrThreadBuf->tileMax) {
        return;
    }

    sfrThreadBuf->tiles = (struct sfrTile*)sfrRealloc(sfrThreadBuf->tiles, sizeof(struct sfrTile) * count);
    sfrThreadBuf->rasterWorkQueue = (i32*)sfrRealloc(sfrThreadBuf->rasterWorkQueue, sizeof(i32) * count);

    for (i32 i = 0; i <= SFR_THREAD_COUNT; i += 1) {
        struct sfrThreadData* tData = &sfrThreadBuf->threads[i];
        tData->tileFirstRef = (i32*)sfrRealloc(tData->tileFirstRef, sizeof(i32) * count);
        tData->tileLastRef = (i32*)sfrRealloc(tData->tileLastRef, sizeof(i32) * count);
    }

    sfrThreadBuf->tileMax = count;
}

#endif // SFR_MULTITHREADED


//================================================
//: PUBLIC API FUNCTION DEFINITIONS
//================================================

SFR_FUNC void sfr_init(i32 w, i32 h, f32 fovDeg, void* (*mallocFunc)(u64), void (*freeFunc)(void*), void* (*reallocFunc)(void*, u64)) {
    if (!mallocFunc || !freeFunc || !reallocFunc) {
        SFR__ERR_EXIT("malloc, free, and realloc must be provided\n");
    }
    sfrMalloc = mallocFunc, sfrFree = freeFunc, sfrRealloc = reallocFunc;

    sfrDefaultTarget.width = w, sfrDefaultTarget.height = h;
    SFR__MALLOC(sfrDefaultTarget.pixels, sizeof(u32) * w * h);
    SFR__MALLOC(sfrDefaultTarget.depth, sizeof(u32) * w * h);
    sfrDefaultTarget.ownsMem = 0;

    sfrDynamicArenaCap = 1024 * 8;
    SFR__MALLOC(sfrDynamicArena, sfrDynamicArenaCap);

    #ifdef SFR_MULTITHREADED
        SFR__MALLOC(sfrThreadBuf, sizeof(struct sfrThreadBuf));

        sfrThreadBuf->meshJobPoolCapacity = 1024;
        SFR__MALLOC(sfrThreadBuf->meshJobPool, sizeof(struct sfrMeshChunkJob) * sfrThreadBuf->meshJobPoolCapacity);

        sfrThreadBuf->geometryWorkQueueCapacity = 1024;
        SFR__MALLOC(sfrThreadBuf->geometryWorkQueue, sizeof(i32) * sfrThreadBuf->geometryWorkQueueCapacity);

        if (!sfrThreadBuf->meshJobPool || !sfrThreadBuf->geometryWorkQueue) {
            SFR__ERR_EXIT("failed to allocate dynamic buffers\n");
        }

        sfrState.shutdown = 0;
        sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueCount, 0);
        sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueHead, 0);

        sfr_atomic_set(&sfrThreadBuf->meshJobAllocator, 0);
        sfr_atomic_set(&sfrThreadBuf->geometryWorkQueueCount, 0);
        sfr_atomic_set(&sfrThreadBuf->geometryWorkQueueHead, 0);

        sfr_mutex_init(&sfrThreadBuf->geometryMutex);

        sfr_semaphore_init(&sfrThreadBuf->geometryStartSem, 0);
        sfr_semaphore_init(&sfrThreadBuf->geometryDoneSem, 0);
        sfr_semaphore_init(&sfrThreadBuf->rasterStartSem, 0);
        sfr_semaphore_init(&sfrThreadBuf->rasterDoneSem, 0);

        for (i32 i = 0; i <= SFR_THREAD_COUNT; i += 1) {
            struct sfrThreadData* tData = &sfrThreadBuf->threads[i];
            tData->threadInd = i;
            
            // allocate thread local flat bin arrays
            tData->binCapacity = 1024;
            tData->binCount = 0;
            SFR__MALLOC(tData->rasterBins, sizeof(struct sfrRasterBin) * tData->binCapacity);
            SFR__MALLOC(tData->shadeBins, sizeof(struct sfrShadeBin) * tData->binCapacity);
            
            // allocate thread local bin reference nodes
            tData->refCapacity = 1024;
            tData->refCount = 0;
            SFR__MALLOC(tData->binRefs, sizeof(struct sfrBinRef) * tData->refCapacity);

            const i32 threadStack = 1024 * 1024 * 2; // 2MB stack
            if (i < SFR_THREAD_COUNT) {
                #ifdef _WIN32
                    const u64 handle = _beginthreadex(NULL, threadStack, sfr__worker_thread_func, tData, STACK_SIZE_PARAM_IS_A_RESERVATION, NULL);
                    tData->handle = (SfrThread)handle;
                #else
                    pthread_attr_t attr;
                    pthread_attr_init(&attr);
                    pthread_attr_setstacksize(&attr, threadStack);
                    pthread_create(&tData->handle, &attr, sfr__worker_thread_func, tData);
                    pthread_attr_destroy(&attr);
                #endif
            }
        }
    #else
        sfrState.idBufCap = w * h;
        SFR__MALLOC(sfrState.idBuf, sizeof(i32) * w * h);

        sfrState.globalBinsCap = 1024;
        sfrState.globalBinsCount = 0;
        SFR__MALLOC(sfrState.globalRasterBins, sizeof(struct sfrRasterBin) * sfrState.globalBinsCap);
        SFR__MALLOC(sfrState.globalShadeBins, sizeof(struct sfrShadeBin) * sfrState.globalBinsCap);
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
    sfrState.baseMat.metallicFactor = 0.1f;
    sfr__generate_mipmaps(&baseTex);

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
    SFR__FREE(sfrPixelBuf);
    SFR__FREE(sfrDepthBuf);
    SFR__FREE(sfrDynamicArena);
    #ifdef SFR_MULTITHREADED
        sfr__shutdown();
        SFR__FREE(sfrThreadBuf);
    #endif
}

#ifdef SFR_MULTITHREADED

SFR_FUNC void sfr__shutdown(void) {
    sfr_present();

    sfrState.shutdown = 1;
    // post to both semaphores to ensure threads wake up from either wait state
    sfr_semaphore_post(&sfrThreadBuf->geometryStartSem, SFR_THREAD_COUNT);
    sfr_semaphore_post(&sfrThreadBuf->rasterStartSem, SFR_THREAD_COUNT);

    for (i32 i = 0; i <= SFR_THREAD_COUNT; i += 1) {
        struct sfrThreadData* tData = &sfrThreadBuf->threads[i];
        
        if (i < SFR_THREAD_COUNT) {
            #ifdef _WIN32
                WaitForSingleObject(tData->handle, INFINITE);
                CloseHandle(tData->handle);
            #else
                pthread_join(tData->handle, NULL);
            #endif
        }

        SFR__FREE(tData->rasterBins);
        SFR__FREE(tData->shadeBins);
        SFR__FREE(tData->binRefs);
        SFR__FREE(tData->tileFirstRef);
        SFR__FREE(tData->tileLastRef);
    }

    SFR__FREE(sfrThreadBuf->meshJobPool);
    SFR__FREE(sfrThreadBuf->geometryWorkQueue);

    sfr_mutex_destroy(&sfrThreadBuf->geometryMutex);

    sfr_semaphore_destroy(&sfrThreadBuf->geometryStartSem);
    sfr_semaphore_destroy(&sfrThreadBuf->geometryDoneSem);
    sfr_semaphore_destroy(&sfrThreadBuf->rasterStartSem);
    sfr_semaphore_destroy(&sfrThreadBuf->rasterDoneSem);
}

#endif // SFR_MULTITHREADED

// dispatches jobs to workers and waits for them to complete
SFR_FUNC void sfr_present(void) {
#ifdef SFR_MULTITHREADED
    // dispatch geometry
    sfr_semaphore_post(&sfrThreadBuf->geometryStartSem, SFR_THREAD_COUNT);

    // let the main thread process geometry jobs while waiting
    sfrTlsBinStart = 0;
    sfrTlsBinEnd = 0;
    sfr__process_geometry_jobs();

    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        sfr_semaphore_wait(&sfrThreadBuf->geometryDoneSem);
    }

    // build raster queues
    sfr_atomic_set(&sfrThreadBuf->meshJobAllocator, 0);
    sfr_atomic_set(&sfrThreadBuf->geometryWorkQueueCount, 0);
    sfr_atomic_set(&sfrThreadBuf->geometryWorkQueueHead, 0);

    i32 workCount = 0;

    for (i32 i = 0; i < sfrThreadBuf->tileCount; i += 1) {
        u8 tileHasWork = 0;

        // determine if any thread binned a triangle into this tile
        for (i32 t = 0; t <= SFR_THREAD_COUNT; t += 1) {
            if (-1 != sfrThreadBuf->threads[t].tileFirstRef[i]) {
                tileHasWork = 1;
                break;
            }
        }

        if (tileHasWork) {
            sfrThreadBuf->rasterWorkQueue[workCount++] = i;
        }
    }

    for (i32 t = 0; t <= SFR_THREAD_COUNT; t += 1) {
        sfrRasterCount += sfrThreadBuf->threads[t].binCount;
    }

    sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueCount, workCount);
    sfr_atomic_set(&sfrThreadBuf->rasterWorkQueueHead, 0);

    // dispatch rasterization
    sfr_semaphore_post(&sfrThreadBuf->rasterStartSem, SFR_THREAD_COUNT);

    // let the main thread process raster jobs while waiting
    sfr__process_raster_jobs(&sfrThreadBuf->threads[sfrTlsThreadInd]);

    for (i32 i = 0; i < SFR_THREAD_COUNT; i += 1) {
        sfr_semaphore_wait(&sfrThreadBuf->rasterDoneSem);
    }

    for (i32 i = 0; i <= SFR_THREAD_COUNT; i += 1) {
        struct sfrThreadData* tData = &sfrThreadBuf->threads[i];
        tData->binCount = 0;
        tData->refCount = 0;
        
        sfr_memset(tData->tileFirstRef, -1, sizeof(i32) * sfrThreadBuf->tileCount);
        sfr_memset(tData->tileLastRef, -1, sizeof(i32) * sfrThreadBuf->tileCount);
    }
#else
    if (0 == sfrState.globalBinsCount) {
        return;
    }

    if (SFR_RENDERMODE_DEPTH_ONLY != sfrState.renderMode) {
        const struct sfrTile fullTile = {
            .minX = 0, .minY = 0, .maxX = sfrWidth, .maxY = sfrHeight,
            .minInvZ = 0.f
        };
        sfr__resolve_tile(&fullTile, sfrState.idBuf);
    }
    
    // reset global bins and ID buffer so they don't bleed into the next flush
    sfrState.globalBinsCount = 0;
    if (sfrState.idBuf) {
        for (i32 i = 0; i < sfrWidth * sfrHeight; i += 1) {
            sfrState.idBuf[i] = -1;
        }
    }
#endif

    if (sfrPendingArenaCap > sfrDynamicArenaCap) {
        u8* newArena = (u8*)sfrRealloc(sfrDynamicArena, sfrPendingArenaCap);
        if (!newArena) {
            SFR__ERR_EXIT("failed to reallocate sfrDynamicArena (%u bytes)\n", (u32)sfrPendingArenaCap);
        }

        // ensure unused memory is zerod
        sfr_memset(newArena + sfrDynamicArenaCap, 0, sfrPendingArenaCap - sfrDynamicArenaCap);
        
        sfrDynamicArena = newArena;
        sfrDynamicArenaCap = sfrPendingArenaCap;
        sfrPendingArenaCap = 0;
    }

    sfrDynamicArenaOffset = 0;
}

SFR_FUNC void sfr_shadow_pass_begin(
    SfrRenderTarget* target, sfrvec lightDir, sfrvec center,
    f32 orthoSize, f32 distance, f32 zFar
) {
    if (!target) {
        return;
    }

    if (0.f == target->shadowBias) {
        target->shadowBias = 0.0006f;
    }

    // prepare target and engine state
    sfr_set_render_target(target);
    sfr_set_rendermode(SFR_RENDERMODE_DEPTH_ONLY);
    sfr_clear_depth();

    // math setup
    lightDir = sfr_vec_norm(lightDir);
    const sfrvec up = (sfr_fabsf(lightDir.y) > 0.99f)
        ? SFR__V(0.f, 0.f, 1.f, 0.f)
        : SFR__V(0.f, 1.f, 0.f, 0.f);

    // build initial view
    sfrvec lightPos = sfr_vec_sub(center, sfr_vec_mul(lightDir, distance));
    const sfrmat lightView = sfr_mat_view_look_at(lightPos, center, up);

    // transform to light space for texel snapping
    sfrvec centerLS = sfr_mat_mul_vec(lightView, center);
    const f32 worldUnitsPerTexel = (orthoSize * 2.f) / (f32)target->width;
    centerLS.x = sfr_floorf(centerLS.x / worldUnitsPerTexel) * worldUnitsPerTexel;
    centerLS.y = sfr_floorf(centerLS.y / worldUnitsPerTexel) * worldUnitsPerTexel;

    // fast rigidbody inverse of view matrix
    const sfrmat invLightView = sfr_mat_qinv(lightView);

    // transform snapped center back to world space
    const sfrvec snappedCenter = sfr_mat_mul_vec(invLightView, centerLS);

    // rebuild stabilized matrices
    lightPos = sfr_vec_sub(snappedCenter, sfr_vec_mul(lightDir, distance));
    sfrMatView = sfr_mat_view_look_at(lightPos, snappedCenter, up);
    sfrState.matInvView = sfr_mat_qinv(sfrMatView);
    sfrMatProj = sfr_mat_ortho(-orthoSize, orthoSize, -orthoSize, orthoSize, 1.f, zFar);

    // cache the data for pass 2
    sfr_set_shadowmap(target, sfr_mat_mul(sfrMatView, sfrMatProj));
}

SFR_FUNC void sfr_shadow_pass_end(void) {
    // reset the pipeline for the main rendering pass
    sfr_set_render_target(NULL);
    sfr_set_rendermode(SFR_RENDERMODE_SHADED);
}

SFR_FUNC void sfr_shadow_push_state(void) {
    sfrState.savedShadowmap = sfrState.activeShadowmap;
    sfrState.savedMatLightVP = sfrState.matLightVP;
    sfrState.activeShadowmap = NULL;
}

SFR_FUNC void sfr_shadow_pop_state(void) {
    sfrState.activeShadowmap = sfrState.savedShadowmap;
    sfrState.matLightVP = sfrState.savedMatLightVP;
}

SFR_FUNC SfrRenderTarget* sfr_create_target(i32 width, i32 height, u8 hasPixels) {
    SfrRenderTarget* t;
    SFR__MALLOC(t, sizeof(SfrRenderTarget));

    t->width = width;
    t->height = height;
    if (hasPixels) {
        SFR__MALLOC(t->pixels, sizeof(u32) * width * height);
    } else {
        t->pixels = NULL;
    }

    SFR__MALLOC(t->depth, sizeof(f32) * width * height);

    t->ownsMem = 1;

    // default shadowmap values
    t->shadowBias = 0.0006f;
    t->shadowStrength = 0.5f;

    return t;
}

SFR_FUNC void sfr_release_target(SfrRenderTarget** target) {
    if (!target || !(*target)) {
        return;
    }

    if ((*target)->ownsMem) {
        SFR__FREE((*target)->pixels);
        SFR__FREE((*target)->depth);
    }
    SFR__FREE(*target);
}

SFR_FUNC void sfr_set_render_target(SfrRenderTarget* target) {
    sfr_present(); // flush geometry to the current target before switching

    if (!target) {
        target = &sfrDefaultTarget;
    }
    sfrCurrTarget = target;

    sfrWidth = target->width, sfrHeight = target->height;
    sfrPixelBuf = target->pixels, sfrDepthBuf = target->depth;

    #ifndef SFR_MULTITHREADED
        const i32 neededPixels = sfrWidth * sfrHeight;
        if (neededPixels > sfrState.idBufCap) {
            sfrState.idBufCap = neededPixels;
            sfrState.idBuf = (i32*)sfrRealloc(sfrState.idBuf, sizeof(i32) * sfrState.idBufCap);
        }
    #endif

    sfrState.halfWidth = sfrWidth / 2.f;
    sfrState.halfHeight = sfrHeight / 2.f;

    sfrState.clipPlanes[0][0] = SFR__V(0.f, 0.5f, 0.f, 1.f);
    sfrState.clipPlanes[0][1] = SFR__V(0.f, 1.f,  0.f, 1.f);
    sfrState.clipPlanes[1][0] = SFR__V(0.f, (f32)sfrHeight, 0.f, 1.f);
    sfrState.clipPlanes[1][1] = SFR__V(0.f, -1.f,   0.f, 1.f);
    sfrState.clipPlanes[2][0] = SFR__V(0.5f, 0.f, 0.f, 1.f);
    sfrState.clipPlanes[2][1] = SFR__V(1.f,  0.f, 0.f, 1.f);
    sfrState.clipPlanes[3][0] = SFR__V((f32)sfrWidth, 0.f, 0.f, 1.f);
    sfrState.clipPlanes[3][1] = SFR__V(-1.f,  0.f, 0.f, 1.f);

    #ifdef SFR_MULTITHREADED
        sfr__ensure_tile_cap(sfrWidth, sfrHeight);
        sfrThreadBuf->tileCols = (sfrWidth + SFR_TILE_WIDTH - 1) / SFR_TILE_WIDTH;
        sfrThreadBuf->tileRows = (sfrHeight + SFR_TILE_HEIGHT - 1) / SFR_TILE_HEIGHT;
        sfrThreadBuf->tileCount = sfrThreadBuf->tileCols * sfrThreadBuf->tileRows;

        for (i32 y = 0, i = 0; y < sfrThreadBuf->tileRows; y += 1) {
            for (i32 x = 0; x < sfrThreadBuf->tileCols; x += 1, i += 1) {
                struct sfrTile* tile = &sfrThreadBuf->tiles[i];
                tile->minX = x * SFR_TILE_WIDTH;
                tile->minY = y * SFR_TILE_HEIGHT;
                tile->maxX = SFR_MIN((x + 1) * SFR_TILE_WIDTH, sfrWidth);
                tile->maxY = SFR_MIN((y + 1) * SFR_TILE_HEIGHT, sfrHeight);
                tile->minInvZ = 0.f;
                sfr_atomic_set(&tile->hasWork, 0);
            }
        }

        for (i32 i = 0; i <= SFR_THREAD_COUNT; i += 1) {
            sfr_memset(sfrThreadBuf->threads[i].tileFirstRef, -1, sfrThreadBuf->tileCount * sizeof(i32));
            sfr_memset(sfrThreadBuf->threads[i].tileLastRef, -1, sfrThreadBuf->tileCount * sizeof(i32));
        }
    #endif
}

SFR_FUNC void sfr_resize(i32 width, i32 height) {
    sfr_present();

    // reallocate the default screen buffer
    sfrDefaultTarget.pixels = (u32*)sfrRealloc(sfrDefaultTarget.pixels, sizeof(u32) * width * height);
    sfrDefaultTarget.depth = (f32*)sfrRealloc(sfrDefaultTarget.depth, sizeof(f32) * width * height);
    sfrDefaultTarget.width = width, sfrDefaultTarget.height = height;

    if (&sfrDefaultTarget == sfrCurrTarget) {
        // re trigger bounding box recalculations
        sfr_set_render_target(NULL);
    }

    // update projection matrix
    sfr_set_fov(sfrCamFov);
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
    const sfrvec up = SFR__V(0.f, 1.f, 0.f, 1.f);
    const sfrmat view = sfr_mat_model_look_at(sfrCamPos, SFR__V(x, y, z, 1.f), up);
    sfrMatView = sfr_mat_qinv(view);
    sfrState.matInvView = view;
}

SFR_FUNC void sfr_clear(u32 clearCol) {
    if (SFR_RENDERMODE_DEPTH_ONLY == sfrState.renderMode || !sfrPixelBuf) {
        sfr_clear_depth();
        return;
    }

    #ifdef SFR_MULTITHREADED
        sfr_present();
    #endif

    const i32 count = sfrWidth * sfrHeight;

    #if !defined(SFR_NO_SIMD) && !defined(SFR_NO_STRING)
        const vi32 vCol = sfrvi_set1(clearCol);
        const vi32 vDepth = sfrvi_set1(0);

        #ifndef SFR_MULTITHREADED
            const vi32 vId = sfrvi_set1(-1);
        #endif

        i32 i = 0;
        for (; i <= count - SFR_SIMD_LANES; i += SFR_SIMD_LANES) {
            sfrvi_storeu((vi32*)&sfrPixelBuf[i], vCol);
            sfrvi_storeu((vi32*)&sfrDepthBuf[i], vDepth);
            #ifndef SFR_MULTITHREADED
                sfrvi_storeu((vi32*)&sfrState.idBuf[i], vId);
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
    sfr_present();

    const i32 count = sfrWidth * sfrHeight;

    #if !defined(SFR_NO_SIMD) && !defined(SFR_NO_STRING)
        const vi32 vDepth = sfrvi_set1(0);

        i32 i = 0;
        for (; i <= count - SFR_SIMD_LANES; i += SFR_SIMD_LANES) {
            sfrvi_storeu((vi32*)&sfrDepthBuf[i], vDepth);
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
    const i32 savedRenderMode = sfrState.renderMode;
    const u8 savedDirty = sfrState.normalMatDirty;

    sfrState.renderMode = SFR_RENDERMODE_UNLIT;

    sfrMatModel = SFR__MAT_IDENTITY;
    const f32 scale = sfrFarDist;
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
    sfrState.renderMode = savedRenderMode;
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
    sfr_present();

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

    const sfrvec center = SFR__V(savedModel.m[3][0], savedModel.m[3][1], savedModel.m[3][2], 1.f);

    const f32 sx = 0.5f * sfr_sqrtf(
        savedModel.m[0][0] * savedModel.m[0][0] +
        savedModel.m[0][1] * savedModel.m[0][1] +
        savedModel.m[0][2] * savedModel.m[0][2]);
    const f32 sy = 0.5f * sfr_sqrtf(
        savedModel.m[1][0] * savedModel.m[1][0] +
        savedModel.m[1][1] * savedModel.m[1][1] +
        savedModel.m[1][2] * savedModel.m[1][2]);

    const sfrvec right = SFR__V(sx * sfrMatView.m[0][0], sx * sfrMatView.m[1][0], sx * sfrMatView.m[2][0], 0.f);
    const sfrvec up    = SFR__V(sy * sfrMatView.m[0][1], sy * sfrMatView.m[1][1], sy * sfrMatView.m[2][1], 0.f);

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

// essentially copied from raylib source: rmodels.c, DrawSphereEx
SFR_FUNC void sfr_sphere(i32 rings, i32 slices, u32 col) {
    // angle between latitudinal parallels
    // const f32 ringAngle = DEG2RAD * (180.f / (rings + 1));
    const f32 ringAngle = SFR_PI / (rings + 1); // simplified from above

    // angle between longitudinal meridians
    // const f32 sliceAngle = DEG2RAD * (360.f / (slices);
    const f32 sliceAngle = 2 * SFR_PI / slices; // simplified from above

    const f32 cosRing = sfr_cosf(ringAngle);
    const f32 sinRing = sfr_sinf(ringAngle);
    const f32 cosSlice = sfr_cosf(sliceAngle);
    const f32 sinSlice = sfr_sinf(sliceAngle);

    // required to store face vertices
    sfrvec verts[4] = {0};
    verts[2] = SFR__V(0.f, 1.f, 0.f, 0.f);
    verts[3] = SFR__V(sinRing, cosRing, 0.f, 0.f);

    for (i32 i = 0; i <= rings; i += 1) {
        for (i32 j = 0; j < slices; j += 1) {
            verts[0] = verts[2];
            verts[1] = verts[3];
            // rotation matrix around y axis
            verts[2] = SFR__V(cosSlice*verts[2].x - sinSlice*verts[2].z, verts[2].y, sinSlice*verts[2].x + cosSlice*verts[2].z, 0.f);
            verts[3] = SFR__V(cosSlice*verts[3].x - sinSlice*verts[3].z, verts[3].y, sinSlice*verts[3].x + cosSlice*verts[3].z, 0.f);

            sfr__triangle_tex_norm(
                verts[0].x, verts[0].y, verts[0].z, 0.f, 0.f, verts[0].x, verts[0].y, verts[0].z,
                verts[3].x, verts[3].y, verts[3].z, 0.f, 0.f, verts[3].x, verts[3].y, verts[3].z,
                verts[1].x, verts[1].y, verts[1].z, 0.f, 0.f, verts[1].x, verts[1].y, verts[1].z,
                col, &sfrState.baseMat);

            sfr__triangle_tex_norm(
                verts[0].x, verts[0].y, verts[0].z, 0.f, 0.f, verts[0].x, verts[0].y, verts[0].z,
                verts[2].x, verts[2].y, verts[2].z, 0.f, 0.f, verts[2].x, verts[2].y, verts[2].z,
                verts[3].x, verts[3].y, verts[3].z, 0.f, 0.f, verts[3].x, verts[3].y, verts[3].z,
                col, &sfrState.baseMat);
        }

        // rotate around z axis to set up starting verts for next ring
        verts[2] = verts[3];
        // rotation matrix around z axis
        verts[3] = SFR__V(cosRing*verts[3].x + sinRing*verts[3].y, -sinRing*verts[3].x + cosRing*verts[3].y, verts[3].z, 0.f);
    }
}

// essentially copied from raylib source: rmodels.c, DrawCapsule
SFR_FUNC void sfr_capsule(sfrvec startPos, sfrvec endPos, f32 radius, i32 slices, i32 rings, u32 col) {
    if (slices < 3) {
        slices = 3;
    }

    sfrvec dir = sfr_vec_sub(endPos, startPos);
    
    // draw a sphere if start and end points are the same
    const u8 sphereCase = (0.f == dir.x && 0.f == dir.y && 0.f == dir.z);
    if (sphereCase) {
        dir = SFR__V(0.f, 1.f, 0.f, 0.f);
    }

    // construct a basic of the base and the caps
    sfrvec b0 = sfr_vec_norm(dir);
    const sfrvec b1 = sfr_vec_norm(sfr_vec_perpendicular(dir));
    const sfrvec b2 = sfr_vec_norm(sfr_vec_cross(b1, dir));
    sfrvec capCenter = endPos;

    const f32 baseSliceAngle = 2.f * SFR_PI / slices;
    const f32 baseRingAngle = 0.5f * SFR_PI / rings;

    // render both caps
    for (i32 c = 0; c < 2; c += 1) {
        for (i32 i = 0; i < rings; i += 1) {
            for (i32 j = 0; j < slices; j += 1) {
                // building up the rings from capCenter in the direction of the 'dir' vector

                // compute the four vertices
                const f32 ringSin1 = sinf(baseSliceAngle*(j + 0))*cosf(baseRingAngle*( i + 0 ));
                const f32 ringCos1 = cosf(baseSliceAngle*(j + 0))*cosf(baseRingAngle*( i + 0 ));
                const sfrvec w1 = SFR__V(
                    capCenter.x + (sinf(baseRingAngle*( i + 0 ))*b0.x + ringSin1*b1.x + ringCos1*b2.x)*radius,
                    capCenter.y + (sinf(baseRingAngle*( i + 0 ))*b0.y + ringSin1*b1.y + ringCos1*b2.y)*radius,
                    capCenter.z + (sinf(baseRingAngle*( i + 0 ))*b0.z + ringSin1*b1.z + ringCos1*b2.z)*radius,
                    0.f);
                const f32 ringSin2 = sinf(baseSliceAngle*(j + 1))*cosf(baseRingAngle*( i + 0 ));
                const f32 ringCos2 = cosf(baseSliceAngle*(j + 1))*cosf(baseRingAngle*( i + 0 ));
                const sfrvec w2 = SFR__V(
                    capCenter.x + (sinf(baseRingAngle*( i + 0 ))*b0.x + ringSin2*b1.x + ringCos2*b2.x)*radius,
                    capCenter.y + (sinf(baseRingAngle*( i + 0 ))*b0.y + ringSin2*b1.y + ringCos2*b2.y)*radius,
                    capCenter.z + (sinf(baseRingAngle*( i + 0 ))*b0.z + ringSin2*b1.z + ringCos2*b2.z)*radius,
                    0.f);

                const f32 ringSin3 = sinf(baseSliceAngle*(j + 0))*cosf(baseRingAngle*( i + 1 ));
                const f32 ringCos3 = cosf(baseSliceAngle*(j + 0))*cosf(baseRingAngle*( i + 1 ));
                const sfrvec w3 = SFR__V(
                    capCenter.x + (sinf(baseRingAngle*( i + 1 ))*b0.x + ringSin3*b1.x + ringCos3*b2.x)*radius,
                    capCenter.y + (sinf(baseRingAngle*( i + 1 ))*b0.y + ringSin3*b1.y + ringCos3*b2.y)*radius,
                    capCenter.z + (sinf(baseRingAngle*( i + 1 ))*b0.z + ringSin3*b1.z + ringCos3*b2.z)*radius,
                    0.f);
                const f32 ringSin4 = sinf(baseSliceAngle*(j + 1))*cosf(baseRingAngle*( i + 1 ));
                const f32 ringCos4 = cosf(baseSliceAngle*(j + 1))*cosf(baseRingAngle*( i + 1 ));
                const sfrvec w4 = SFR__V(
                    capCenter.x + (sinf(baseRingAngle*( i + 1 ))*b0.x + ringSin4*b1.x + ringCos4*b2.x)*radius,
                    capCenter.y + (sinf(baseRingAngle*( i + 1 ))*b0.y + ringSin4*b1.y + ringCos4*b2.y)*radius,
                    capCenter.z + (sinf(baseRingAngle*( i + 1 ))*b0.z + ringSin4*b1.z + ringCos4*b2.z)*radius,
                    0.f);

                // make sure cap triangle normals are facing outwards
                if (0 == c) {
                    sfr_triangle(
                        w1.x, w1.y, w1.z,
                        w2.x, w2.y, w2.z,
                        w3.x, w3.y, w3.z,
                        col);
                    sfr_triangle(
                        w2.x, w2.y, w2.z,
                        w4.x, w4.y, w4.z,
                        w3.x, w3.y, w3.z,
                        col);
                } else {
                    sfr_triangle(
                        w1.x, w1.y, w1.z,
                        w3.x, w3.y, w3.z,
                        w2.x, w2.y, w2.z,
                        col);
                    sfr_triangle(
                        w2.x, w2.y, w2.z,
                        w3.x, w3.y, w3.z,
                        w4.x, w4.y, w4.z,
                        col);
                }
            }
        }
        capCenter = startPos;
        b0 = sfr_vec_mul(b0, -1.f);
    }

    // render middle
    if (!sphereCase) {
        for (int j = 0; j < slices; j += 1) {
            // compute the four vertices
            const f32 ringSin1 = sinf(baseSliceAngle*(j + 0))*radius;
            const f32 ringCos1 = cosf(baseSliceAngle*(j + 0))*radius;
            const sfrvec w1 = SFR__V(
                startPos.x + ringSin1*b1.x + ringCos1*b2.x,
                startPos.y + ringSin1*b1.y + ringCos1*b2.y,
                startPos.z + ringSin1*b1.z + ringCos1*b2.z,
                0.f);
            const f32 ringSin2 = sinf(baseSliceAngle*(j + 1))*radius;
            const f32 ringCos2 = cosf(baseSliceAngle*(j + 1))*radius;
            const sfrvec w2 = SFR__V(
                startPos.x + ringSin2*b1.x + ringCos2*b2.x,
                startPos.y + ringSin2*b1.y + ringCos2*b2.y,
                startPos.z + ringSin2*b1.z + ringCos2*b2.z,
                0.f);

            const f32 ringSin3 = sinf(baseSliceAngle*(j + 0))*radius;
            const f32 ringCos3 = cosf(baseSliceAngle*(j + 0))*radius;
            const sfrvec w3 = SFR__V(
                endPos.x + ringSin3*b1.x + ringCos3*b2.x,
                endPos.y + ringSin3*b1.y + ringCos3*b2.y,
                endPos.z + ringSin3*b1.z + ringCos3*b2.z,
                0.f);
            const f32 ringSin4 = sinf(baseSliceAngle*(j + 1))*radius;
            const f32 ringCos4 = cosf(baseSliceAngle*(j + 1))*radius;
            const sfrvec w4 = SFR__V(
                endPos.x + ringSin4*b1.x + ringCos4*b2.x,
                endPos.y + ringSin4*b1.y + ringCos4*b2.y,
                endPos.z + ringSin4*b1.z + ringCos4*b2.z,
                0.f);

            sfr_triangle(
                w1.x, w1.y, w1.z,
                w2.x, w2.y, w2.z,
                w3.x, w3.y, w3.z,
                col);
            sfr_triangle(
                w2.x, w2.y, w2.z,
                w4.x, w4.y, w4.z,
                w3.x, w3.y, w3.z,
                col);
        }
    }
}

// essentially copied from raylib source: rmodels.c, DrawCylinderEx
SFR_FUNC void sfr_cylinder(sfrvec startPos, sfrvec endPos, f32 startRadius, f32 endRadius, i32 sides, u32 col) {
    if (sides < 3) {
        sides = 3;
    }

    const sfrvec dir = sfr_vec_sub(endPos, startPos);
    if (0 == dir.x && 0 == dir.y && 0 == dir.z) {
        return; // security check
    }

    // construct a basis of the base and the top faces
    const sfrvec b1 = sfr_vec_norm(sfr_vec_perpendicular(dir));
    const sfrvec b2 = sfr_vec_norm(sfr_vec_cross(b1, dir));

    const f32 baseAngle = (2.f * SFR_PI) / sides;

    for (i32 i = 0; i < sides; i += 1) {
        // the four vertices
        const f32 s1 = sfr_sinf(baseAngle * (i + 0)) * startRadius;
        const f32 c1 = sfr_cosf(baseAngle * (i + 0)) * startRadius;
        const sfrvec w1 = {
            .x = startPos.x + s1*b1.x + c1*b2.x,
            .y = startPos.y + s1*b1.y + c1*b2.y,
            .z = startPos.z + s1*b1.z + c1*b2.z
        };
        const f32 s2 = sfr_sinf(baseAngle * (i + 1)) * startRadius;
        const f32 c2 = sfr_cosf(baseAngle * (i + 1)) * startRadius;
        const sfrvec w2 = {
            .x = startPos.x + s2*b1.x + c2*b2.x,
            .y = startPos.y + s2*b1.y + c2*b2.y,
            .z = startPos.z + s2*b1.z + c2*b2.z
        };
        const f32 s3 = sfr_sinf(baseAngle * (i + 0)) * endRadius;
        const f32 c3 = sfr_cosf(baseAngle * (i + 0)) * endRadius;
        const sfrvec w3 = {
            .x = endPos.x + s3*b1.x + c3*b2.x,
            .y = endPos.y + s3*b1.y + c3*b2.y,
            .z = endPos.z + s3*b1.z + c3*b2.z
        };
        const f32 s4 = sfr_sinf(baseAngle * (i + 1)) * endRadius;
        const f32 c4 = sfr_cosf(baseAngle * (i + 1)) * endRadius;
        const sfrvec w4 = {
            .x = endPos.x + s4*b1.x + c4*b2.x,
            .y = endPos.y + s4*b1.y + c4*b2.y,
            .z = endPos.z + s4*b1.z + c4*b2.z
        };

        if (startRadius > 0) {
            sfr_triangle(
                startPos.x, startPos.y, startPos.z,
                w2.x, w2.y, w2.z,
                w1.x, w1.y, w1.z,
                col);
        }

        sfr_triangle(
            w1.x, w1.y, w1.z,
            w2.x, w2.y, w2.z,
            w3.x, w3.y, w3.z,
            col);

        sfr_triangle(
            w2.x, w2.y, w2.z,
            w4.x, w4.y, w4.z,
            w3.x, w3.y, w3.z,
            col);
        
        if (endRadius > 0) {
            sfr_triangle(
                endPos.x, endPos.y, endPos.z,
                w3.x, w3.y, w3.z,
                w4.x, w4.y, w4.z,
                col);
        }
    }
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
            job->uvs = &mesh->uvs[i * 6];
            job->lmUvs = &mesh->lmUvs[i * 6];
            job->normals = &mesh->normals[i * 9];
            job->tangents = &mesh->tangents[i * 12];
            job->matNormal = sfrState.matNormal;
            job->matMVP = matMVP;
            job->col = col;
            job->mat = mat;
            job->startTriangle = i;
            job->triangleCount = SFR_MIN(SFR_GEOMETRY_JOB_SIZE, triangleCount - i);
            job->renderMode = sfrState.renderMode;

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
        struct sfrGeomClipBufs clipBuf;
        struct sfrGeomTri gt = { .clipBuf = &clipBuf };
        const i32 renderMode = sfrState.renderMode;

        for (i32 i = 0; i < mesh->vertCount; i += 9) {
            const i32 uvInd = (i / 9) * 6;
            const i32 tanInd = (i / 9) * 12;

            gt.posA = &mesh->tris[i + 0];
            gt.uvA = &mesh->uvs[uvInd + 0];
            gt.lmUvA = &mesh->lmUvs[uvInd + 0];
            gt.normA = &mesh->normals[i + 0];
            gt.tanA = &mesh->tangents[tanInd + 0];

            gt.posB = &mesh->tris[i + 3];
            gt.uvB = &mesh->uvs[uvInd + 2];
            gt.lmUvB = &mesh->lmUvs[uvInd + 2];
            gt.normB = &mesh->normals[i + 3];
            gt.tanB = &mesh->tangents[tanInd + 4];
            
            gt.posC = &mesh->tris[i + 6];
            gt.uvC = &mesh->uvs[uvInd + 4];
            gt.lmUvC = &mesh->lmUvs[uvInd + 4];
            gt.normC = &mesh->normals[i + 6];
            gt.tanC = &mesh->tangents[tanInd + 8];
            
            gt.renderMode = renderMode;

            gt.col = col;
            gt.mat = mat;

            sfr__process_and_bin_triangle(&matMVP, &sfrState.matNormal, &gt);
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

SFR_FUNC SfrDynamicMesh* sfr_dmesh_request(i32 vertCount) {
    if (vertCount <= 0) {
        return NULL;
    }

    // memory requirements per vertex:
    // positions (3) + uvs (2) + lmUvs (2) + normals (3) + tangents (4) = 14 floats
    const u64 bytesNeeded = (u64)vertCount * 14 * sizeof(f32);
    
    // include the size of SfrDynamicMesh struct and 32 byte alignment padding
    const u64 totalNeeded = ((bytesNeeded + sizeof(SfrDynamicMesh)) + 31) & ~31;

    // grow the arena if the requested frame data exceeds capacity
    if (sfrDynamicArenaOffset + totalNeeded > sfrDynamicArenaCap) {
        u64 newCap = (0 == sfrDynamicArenaCap) ? (1024 * 8) : sfrDynamicArenaCap * 2;
        while (sfrDynamicArenaOffset + totalNeeded > newCap) {
            newCap *= 2;
        }

        sfrPendingArenaCap = newCap;

        return NULL;
    }

    // carve out the SfrDynamicMesh handle
    SfrDynamicMesh* dmesh = (SfrDynamicMesh*)(sfrDynamicArena + sfrDynamicArenaOffset);
    u8* dataPtr = (u8*)dmesh + sizeof(SfrDynamicMesh);

    // map internal pointers linearly across the allocated memory slice
    dmesh->positions = (f32*)dataPtr;
    dmesh->uvs       = dmesh->positions + (vertCount * 3);
    dmesh->lmUvs     = dmesh->uvs       + (vertCount * 2);
    dmesh->normals   = dmesh->lmUvs     + (vertCount * 2);
    dmesh->tangents  = dmesh->normals   + (vertCount * 3);
    dmesh->vertCount = vertCount;

    // map properties back to SfrMesh
    dmesh->_mesh.tris      = dmesh->positions;
    dmesh->_mesh.uvs       = dmesh->uvs;
    dmesh->_mesh.lmUvs     = dmesh->lmUvs;
    dmesh->_mesh.normals   = dmesh->normals;
    dmesh->_mesh.tangents  = dmesh->tangents;
    dmesh->_mesh.vertCount = vertCount * 3;

    // advance the frame allocation offset
    sfrDynamicArenaOffset += totalNeeded;

    return dmesh;
}

SFR_FUNC void sfr_dmesh_submit(const SfrDynamicMesh* dmesh, u32 col, const SfrMaterial* mat) {
    if (!dmesh) {
        return;
    }
    sfr_mesh(&dmesh->_mesh, col, mat);
}

SFR_FUNC SfrScene* sfr_scene_create(SfrSceneObject* objects, i32 count) {
    SfrScene* scene;
    SFR__MALLOC(scene, sizeof(SfrScene));
    sfr_memset(scene, 0, sizeof(SfrScene));
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

SFR_FUNC void sfr_scene_apply_lights(const SfrScene* scene) {
    if (!scene) {
        return;
    }

    sfrState.lightCount = 0; // clear old lights and replace them with scene's
    for (i32 i = 0; i < scene->lightCount; i += 1) {
        const SfrLight* const l = &scene->lights[i];
        if (SFR_LIGHT_POINT == l->type) {
            sfr_light_add_point(l->x, l->y, l->z, l->ambient, l->intensity, l->attenuation, l->r, l->g, l->b);
        } else {
            sfr_light_add_directional(l->x, l->y, l->z, l->ambient, l->intensity, l->r, l->g, l->b);
        }
    }
}

// TODO don't fully overwrite sfrMatModel (see sfr_model_draw)
SFR_FUNC void sfr_scene_draw(const SfrScene* scene) {
    const sfrmat savedModel = sfrMatModel;
    const sfrmat savedNormal = sfrState.matNormal;
    const u8 savedDirty = sfrState.normalMatDirty;
    SfrTexture* const savedLightmap = sfrState.activeLightmap;
    SfrTexture* const savedDirectionmap = sfrState.activeDirectionmap;

    sfrState.normalMatDirty = 0;
    sfrState.activeLightmap = scene->globalLightmap;
    sfrState.activeDirectionmap = scene->globalDirectionmap;

    for (i32 i = 0; i < scene->count; i += 1) {
        sfrMatModel = scene->objects[i]._model;
        sfrState.matNormal = scene->objects[i]._normal;

        sfr_mesh(scene->objects[i].mesh, scene->objects[i].col, scene->objects[i].mat);
    }

    sfr_present();

    sfrMatModel = savedModel;
    sfrState.matNormal = savedNormal;
    sfrState.normalMatDirty = savedDirty;
    sfrState.activeLightmap = savedLightmap;
    sfrState.activeDirectionmap = savedDirectionmap;
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

    const sfrvec rayOrigin = SFR__V(ox, oy, oz, 1.f);
    const sfrvec rayDirRaw = SFR__V(dx, dy, dz, 0.f);
    const sfrvec rayDir = sfr_vec_norm(rayDirRaw);


    for (i32 i = 0; i < scene->count; i += 1) {
        SfrSceneObject* const obj = &scene->objects[i];
        if (!obj->mesh || !obj->_bvhNodes) {
            continue;
        }

        // faster to transform the ray to the object than the box to world
        // so raycast in local space

        const sfrvec localOrigin = sfr_mat_mul_vec(obj->_invModel, rayOrigin);
        const sfrvec localDir = sfr_mat_mul_vec(obj->_invModel, SFR__V(dx, dy, dz, 0.f));
        const sfrvec localDirInv = SFR__V(
            1.f / (sfr_fabsf(localDir.x) > SFR_EPSILON ? localDir.x : SFR_EPSILON),
            1.f / (sfr_fabsf(localDir.y) > SFR_EPSILON ? localDir.y : SFR_EPSILON),
            1.f / (sfr_fabsf(localDir.z) > SFR_EPSILON ? localDir.z : SFR_EPSILON),
            0.f
        );

        i32 stack[512];
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

                    sfrvec v0 = SFR__V(obj->mesh->tris[baseInd + 0], obj->mesh->tris[baseInd + 1], obj->mesh->tris[baseInd + 2], 1.f);
                    sfrvec v1 = SFR__V(obj->mesh->tris[baseInd + 3], obj->mesh->tris[baseInd + 4], obj->mesh->tris[baseInd + 5], 1.f);
                    sfrvec v2 = SFR__V(obj->mesh->tris[baseInd + 6], obj->mesh->tris[baseInd + 7], obj->mesh->tris[baseInd + 8], 1.f);

                    // transform to world space for the actual hit check
                    v0 = sfr_mat_mul_vec(obj->_model, v0);
                    v1 = sfr_mat_mul_vec(obj->_model, v1);
                    v2 = sfr_mat_mul_vec(obj->_model, v2);

                    f32 u, v;
                    if (sfr__intersect_triangle(rayOrigin, rayDir, v0, v1, v2, &hit.distance, &u, &v)) {
                        hit.hit = 1;
                        hit.u = u, hit.v = v;
                        hit.objectInd = i;
                        hit.triangleInd = node->leftFirst + t;
                        hit.obj = obj;
                        hit.pos = sfr_vec_add(rayOrigin, sfr_vec_mul(rayDir, hit.distance));
                        hit.normal = sfr_vec_face_normal(v0, v1, v2);
                    }
                }
            } else { // internal
                if (stackPtr >= 510) {
                    continue;
                }

                stack[stackPtr++] = node->leftFirst;
                stack[stackPtr++] = node->leftFirst + 1;
            }
        }
    }

    return hit;
}

SFR_FUNC void sfr_scene_set_transform(SfrScene* scene, sfrvec pos, sfrvec rot, sfrvec scale) {
    if (!scene) {
        return;
    }

    // build global scene model
    sfrmat sceneModel = sfr_mat_scale(scale.x, scale.y, scale.z);
    sceneModel = sfr_mat_mul(sceneModel, sfr_mat_rot_x(rot.x));
    sceneModel = sfr_mat_mul(sceneModel, sfr_mat_rot_y(rot.y));
    sceneModel = sfr_mat_mul(sceneModel, sfr_mat_rot_z(rot.z));
    sceneModel = sfr_mat_mul(sceneModel, sfr_mat_translate(pos.x, pos.y, pos.z));

    // build global scene inverse model (T^-1 * R^-1 * S^-1)
    sfrmat sceneInvModel = sfr_mat_translate(-pos.x, -pos.y, -pos.z);
    sceneInvModel = sfr_mat_mul(sceneInvModel, sfr_mat_rot_z(-rot.z));
    sceneInvModel = sfr_mat_mul(sceneInvModel, sfr_mat_rot_y(-rot.y));
    sceneInvModel = sfr_mat_mul(sceneInvModel, sfr_mat_rot_x(-rot.x));
    sceneInvModel = sfr_mat_mul(sceneInvModel, sfr_mat_scale(1.f / scale.x, 1.f / scale.y, 1.f / scale.z));

    for (i32 i = 0; i < scene->count; i += 1) {
        SfrSceneObject* const obj = &scene->objects[i];

        // reconstruct local model from the object's base coordinates
        sfrmat localModel = sfr_mat_scale(obj->scale.x, obj->scale.y, obj->scale.z);
        localModel = sfr_mat_mul(localModel, sfr_mat_rot_x(obj->rot.x));
        localModel = sfr_mat_mul(localModel, sfr_mat_rot_y(obj->rot.y));
        localModel = sfr_mat_mul(localModel, sfr_mat_rot_z(obj->rot.z));
        localModel = sfr_mat_mul(localModel, sfr_mat_translate(obj->pos.x, obj->pos.y, obj->pos.z));

        // reconstruct local inverse model
        sfrmat localInvModel = sfr_mat_translate(-obj->pos.x, -obj->pos.y, -obj->pos.z);
        localInvModel = sfr_mat_mul(localInvModel, sfr_mat_rot_z(-obj->rot.z));
        localInvModel = sfr_mat_mul(localInvModel, sfr_mat_rot_y(-obj->rot.y));
        localInvModel = sfr_mat_mul(localInvModel, sfr_mat_rot_x(-obj->rot.x));
        localInvModel = sfr_mat_mul(localInvModel, sfr_mat_scale(1.f / obj->scale.x, 1.f / obj->scale.y, 1.f / obj->scale.z));

        // apply global transform (World = Local * Scene)
        obj->_model = sfr_mat_mul(localModel, sceneModel);
        
        // apply global inverse transform ((A*B)^-1 = B^-1 * A^-1)
        obj->_invModel = sfr_mat_mul(sceneInvModel, localInvModel); 
        
        // calculate shading normal matrix (handles nonuniform scaling)
        obj->_normal = sfr__calc_normal_mat(obj->_model);
    }
}

SFR_FUNC u8 sfr_world_to_screen(f32 x, f32 y, f32 z, i32* screenX, i32* screenY) {
    sfrvec p = SFR__V(x, y, z, 1.f);
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
    sfrCamPos = SFR__V(x, y, z, 1.f);
    sfrCamUp = SFR__V(0.f, 1.f, 0.f, 1.f);
    sfrCamTarget = SFR__V(0.f, 0.f, 1.f, 1.f);

    const sfrmat rotX = sfr_mat_rot_x(pitch);
    const sfrmat rotY = sfr_mat_rot_y(yaw);
    const sfrmat rotZ = sfr_mat_rot_z(roll);

    sfrCamUp = sfr_mat_mul_vec(rotZ, sfrCamUp);
    sfrCamTarget = sfr_mat_mul_vec(rotX, sfrCamTarget);
    sfrCamTarget = sfr_mat_mul_vec(rotY, sfrCamTarget);
    sfrCamTarget = sfr_vec_add(sfrCamPos, sfrCamTarget);

    const sfrmat view = sfr_mat_model_look_at(sfrCamPos, sfrCamTarget, sfrCamUp);
    sfrMatView = sfr_mat_qinv(view);
    sfrState.matInvView = view;
}

SFR_FUNC void sfr_set_fov(f32 fovDeg) {
    const f32 aspect = (f32)sfrHeight / sfrWidth;
    sfrMatProj = sfr_mat_proj(fovDeg, aspect, sfrNearDist, sfrFarDist);
    sfrCamFov = fovDeg;
}

SFR_FUNC void sfr_set_rendermode(i32 mode) {
    if (mode < SFR_RENDERMODE_UNLIT || mode >= SFR_RENDERMODE_COUNT) {
        SFR__ERR_RET(, "invalid mode (%d), expected in the range [%d, %d)\n", mode, SFR_RENDERMODE_UNLIT, SFR_RENDERMODE_COUNT);
    }
    sfrState.renderMode = mode;
}

SFR_FUNC void sfr_set_lightmap(SfrTexture* lightmap) {
    sfrState.activeLightmap = lightmap;
}

SFR_FUNC void sfr_set_shadowmap(SfrRenderTarget* target, sfrmat matLightVP) {
    sfrState.activeShadowmap = target;
    sfrState.matLightVP = matLightVP;
}

SFR_FUNC SfrLight* sfr_light_add_point(f32 posX, f32 posY, f32 posZ, f32 ambient, f32 intensity, f32 attenuation, f32 r, f32 g, f32 b) {
    if (sfrState.lightCount >= sfrState.lightCap) {
        const i32 newCap = (i32)(sfrState.lightCap * 1.5f);
        SfrLight* newLights = (SfrLight*)sfrRealloc(
            sfrState.lights, sizeof(SfrLight) * newCap);

        if (!newLights) {
            SFR__ERR_RET(NULL, "failed to realloc lights\n");
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
            SFR__ERR_RET(NULL, "failed to realloc lights\n");
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

SFR_FUNC void sfr_light_clear_all(void) {
    sfrState.lightCount = 0;
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

SFR_FUNC SfrModel* sfr_load_gltf(const char* filename, i32 uvChannel, i32 texMaxWidth, i32 texMaxHeight) {
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
    u8* fileContent = _fileContent;
    if (fread(fileContent, 1, fileSize, file) != (u64)fileSize) {
        fclose(file);
        SFR__FREE(fileContent);
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
        SFR__FREE(fileContent);
        SFR__ERR_RET(NULL, "parse error %d\n", parseRes);
    }

    // vv load buffers vv
    const cgltf_result loadBufRes = cgltf_load_buffers(&options, data, filename);
    if (cgltf_result_success != loadBufRes) {
        cgltf_free(data);
        SFR__FREE(fileContent);
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

        for (i32 i = 0; i < (i32)data->textures_count; i += 1) {
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
                        SFR__FREE(buf);
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
                
                if (w > texMaxWidth || h > texMaxHeight) {
                    sfr__resize_texture_in_place(tex, texMaxWidth, texMaxHeight);
                } else {
                    // avoid generating mipmaps twice for no reason (called in resize anyway)
                    sfr__generate_mipmaps(tex);
                }
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

        model->_allTextures = loadedTextures;
        model->_texCount = (i32)data->textures_count;
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
            tnode->localPos = SFR__V(tnode->localMatrix.m[3][0], tnode->localMatrix.m[3][1], tnode->localMatrix.m[3][2], 1.f);
            tnode->localRot = SFR__V(0.f, 0.f, 0.f, 1.f);
            tnode->localScale = SFR__V(
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
            );
        } else {
            // init transforms from TRS
            if (cnode->has_translation) {
                tnode->localPos = SFR__V(cnode->translation[0], cnode->translation[1], -cnode->translation[2], 1.f);
            } else {
                tnode->localPos = SFR__V(0.f, 0.f, 0.f, 1.f);
            }
            if (cnode->has_rotation) {
                tnode->localRot = SFR__V(-cnode->rotation[0], -cnode->rotation[1], cnode->rotation[2], cnode->rotation[3]);
            } else {
                tnode->localRot = SFR__V(0.f, 0.f, 0.f, 1.f);
            }
            if (cnode->has_scale) {
                tnode->localScale = SFR__V(cnode->scale[0], cnode->scale[1], cnode->scale[2], 1.f);
            } else {
                tnode->localScale = SFR__V(1.f, 1.f, 1.f, 1.f);
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
        SFR__FREE(computed);
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
            sfr_memset(sm, 0, sizeof(SfrMesh));
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
                sfr__mesh_compute_tangents(sm);
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

            sfr__mesh_ensure_channels(sm);

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

                SFR__FREE(patchedSamplers);
            }
        }
    } else {
        model->animations = NULL;
    }

    cgltf_free(data);
    SFR__FREE(fileContent);

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
    if (!model || !(*model)) {
        return;
    }

    SfrModel* m = *model;

    // meshes
    for (i32 i = 0; i < m->_meshCount; i += 1) {
        sfr_release_mesh(&m->_allMeshes[i]);
    }
    SFR__FREE(m->_allMeshes);

    // textures
    if (m->_allTextures) {
        for (i32 i = 0; i < m->_texCount; i += 1) {
            sfr_release_texture(&m->_allTextures[i]);
        }
        SFR__FREE(m->_allTextures);
    }

    // materials
    if (m->_allMaterials) {
        for (i32 i = 0; i < m->_matCount; i += 1) {
            if (m->_allMaterials[i]) {
                m->_allMaterials[i]->albedoTex = m->_allMaterials[i]->metallicRoughnessTex = NULL;
                sfr_release_material(&m->_allMaterials[i]);
            }
        }
        SFR__FREE(m->_allMaterials);
    }

    // animation data
    if (m->animations) {
        for (i32 i = 0; i < m->animCount; i += 1){
            SFR__FREE(m->animations[i].samplers->inputs);
            SFR__FREE(m->animations[i].samplers->outputs);
            SFR__FREE(m->animations[i].samplers);
            SFR__FREE(m->animations[i].channels);
        }
        SFR__FREE(m->animations);
    }

    // hierarchy
    SFR__FREE(m->transforms);
    SFR__FREE(m->nodes);

    SFR__FREE(m);
    *model = NULL;
}

#endif // SFR_USE_CGLTF

SFR_FUNC void sfr_material_resize_textures(SfrMaterial* mat, i32 newW, i32 newH) {
    if (!mat) {
        return;
    }

    if (mat->albedoTex) {
        sfr__resize_texture_in_place(mat->albedoTex, newW, newH);
    }
    
    if (mat->metallicRoughnessTex) {
        sfr__resize_texture_in_place(mat->metallicRoughnessTex, newW, newH);
    }
}

SFR_FUNC SfrTexture* sfr_load_texture_raw(i32 w, i32 h, const u32* pixels) {
    SfrTexture* tex;
    SFR__MALLOC(tex, sizeof(SfrTexture));
    sfr_memset(tex, 0, sizeof(SfrTexture));
    tex->w = w;
    tex->h = h;
    SFR__MALLOC(tex->pixels, sizeof(u32) * w * h);
    sfr_memcpy(tex->pixels, pixels, w * h * 4);
    sfr__generate_mipmaps(tex); 
    return tex;
}

#ifndef SFR_NO_STD

SFR_FUNC SfrMesh* sfr_load_mesh(const char* filename) {
    SfrMesh* mesh;
    SFR__MALLOC(mesh, sizeof(SfrMesh));
    *mesh = (SfrMesh){0};

    FILE* objFile = fopen(filename, "rb");
    if (!objFile) {
        SFR__FREE(mesh);
        SFR__ERR_RET(NULL, "failed to open file '%s'\n", filename);
    }

    // read all raw data into temporary dynamic arrays
    // TODO make dynamic array macros
    sfrvec* tempVerts = NULL; i32 tempVertsCap = 0; i32 vertCount = 0;
    sfrvec* tempUVs = NULL; i32 tempUVsCap = 0; i32 uvCount = 0;
    sfrvec* tempNormals = NULL; i32 tempNormalsCap = 0; i32 normalCount = 0;

    // store face data as indices
    struct sfrMeshFaceInds { i32 v[4], vt[4], vn[4], count; };
    struct sfrMeshFaceInds* tempFaces = NULL; i32 tempFacesCap = 0; i32 faceCount = 0;

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
                tempFaces = (struct sfrMeshFaceInds*)realloc(tempFaces, tempFacesCap * sizeof(struct sfrMeshFaceInds));
                if (!tempFaces) {
                    SFR__ERR_EXIT("realloc failed for tempFaces\n");
                }
            }
            struct sfrMeshFaceInds* f = &tempFaces[faceCount];
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
            const struct sfrMeshFaceInds* f = &tempFaces[i];
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
        const struct sfrMeshFaceInds* f = &tempFaces[i];
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
    SFR__FREE(tempVerts);
    SFR__FREE(tempUVs);
    SFR__FREE(tempNormals);
    SFR__FREE(tempFaces);
    SFR__FREE(computedNormals);

    // tangents aren't stored anywhere in obj files
    sfr__mesh_compute_tangents(mesh);
    sfr__mesh_ensure_channels(mesh);

    return mesh;
}

SFR_FUNC void sfr_release_mesh(SfrMesh** mesh) {
    if (!mesh || !(*mesh)) {
        return;
    }

    SFR__FREE((*mesh)->tris);
    SFR__FREE((*mesh)->uvs);
    SFR__FREE((*mesh)->lmUvs);
    SFR__FREE((*mesh)->normals);
    SFR__FREE((*mesh)->tangents);

    SFR__FREE(*mesh);
}

SFR_FUNC SfrMaterial* sfr_load_material(const char* albedoPath, const char* metRoughPath) {
    SfrMaterial* mat;
    SFR__MALLOC(mat, sizeof(SfrMaterial));

    SfrTexture* albedo = NULL;
    if (albedoPath) {
        albedo = sfr_load_texture(albedoPath);
    }

    SfrTexture* metRough = NULL;
    if (metRoughPath) {
        metRough = sfr_load_texture(metRoughPath);
    }

    mat->albedoTex = albedo;
    mat->metallicRoughnessTex = metRough;
    mat->baseColor = 0xFFFFFFFF;
    mat->roughnessFactor = metRough ? 0.f : 0.8f;
    mat->metallicFactor  = metRough ? 0.f : 0.1f;

    return mat;
}

SFR_FUNC void sfr_release_material(SfrMaterial** mat) {
    if (!(*mat)) {
        return;
    }

    if ((*mat)->albedoTex == (*mat)->metallicRoughnessTex) {
        (*mat)->metallicRoughnessTex = NULL;
    }

    sfr_release_texture(&(*mat)->albedoTex);
    sfr_release_texture(&(*mat)->metallicRoughnessTex);

    SFR__FREE(*mat);
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
        if (infoSize >= 40 && (i32)fread(palette, 1, paletteSize, file) != paletteSize) {
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
    if ((i32)fread(pixelData, 1, dataSize, file) != dataSize) {
        SFR__FREE(pixelData);
        fclose(file);
        SFR__ERR_RET(NULL, "failed to read pixel data\n");
    }
    fclose(file);

    // create texture
    SfrTexture* tex;
    SFR__MALLOC(tex, sizeof(SfrTexture));
    if (!tex) {
        SFR__FREE(pixelData);
        SFR__ERR_RET(NULL, "failed to allocate texture struct\n");
    }
    sfr_memset(tex, 0, sizeof(SfrTexture));

    tex->w = width;
    tex->h = height;
    SFR__MALLOC(tex->pixels, sizeof(u32) * width * height);
    if (!tex->pixels) {
        SFR__FREE(pixelData);
        SFR__FREE(tex);
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
                    SFR__FREE(pixelData);
                    SFR__FREE(tex->pixels);
                    SFR__FREE(tex);
                    SFR__ERR_RET(NULL, "unsupported bit depth: %d\n", bpp);
                } break;
            }

            tex->pixels[i] = col;
        }
    }

    SFR__FREE(pixelData);

    sfr__generate_mipmaps(tex);

    return tex;
#endif
}

SFR_FUNC void sfr_release_texture(SfrTexture** tex) {
    if (!tex || !(*tex)) {
        return;
    }

    SFR__FREE((*tex)->pixels);
    SFR__FREE((*tex)->quadR);
    SFR__FREE((*tex)->quadG);
    SFR__FREE((*tex)->quadB);

    SFR__FREE((*tex)->allPixels[0]);

    SFR__FREE(*tex);
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
            SFR__FREE(font);
            SFR__ERR_RET(NULL, "error reading from file\n");
        }
        if (vertCount >= SFR_FONT_VERT_MAX) {
            fclose(file);
            SFR__FREE(font);
            SFR__ERR_RET(NULL,
                "vert count out of bounds for glyph '%c' (%d) (%d >= %d)\n",
                i, i, vertCount, SFR_FONT_VERT_MAX);
        }

        if (vertCount && vertCount != fread(font->verts[i], 4, vertCount, file)) {
            fclose(file);
            SFR__FREE(font);
            SFR__ERR_RET(NULL, "error reading vertices from file\n");
        }
    }

    return font;
}

SFR_FUNC void sfr_release_font(SfrFont** font) {
    if (!font || !(*font)) {
        return;
    }

    SFR__FREE(*font);
}

SFR_FUNC SfrScene* sfr_load_scene(const char* filename) {
    FILE* file = fopen(filename, "rb");
    if (!file) SFR__ERR_RET(NULL, "failed to open '%s'\n", filename);

    char magic[4];
    if (4 != fread(magic, 1, 4, file) ||
        'S' != magic[0] || 'F' != magic[1] || 'R' != magic[2] || 'M' != magic[3]
    ) {
        fclose(file);
        SFR__ERR_RET(NULL, "not a valid SFRM map file ('%s')\n", filename);
    }

    u32 version;
    fread(&version, 4, 1, file);

    SfrScene* scene;
    SFR__MALLOC(scene, sizeof(SfrScene));
    sfr_memset(scene, 0, sizeof(SfrScene));

    fread(&scene->skyColor, 4, 1, file);
    fread(&scene->ambientLight, 4, 1, file);
    fread(&scene->hasSpawn, 1, 1, file);
    fread(&scene->spawnPos.x, 4, 1, file);
    fread(&scene->spawnPos.y, 4, 1, file);
    fread(&scene->spawnPos.z, 4, 1, file);

    fread(&scene->lightCount, 4, 1, file);
    if (scene->lightCount > 0) {
        SFR__MALLOC(scene->lights, sizeof(SfrLight) * scene->lightCount);
        for (i32 i = 0; i < scene->lightCount; i += 1) {
            i32 type;
            fread(&type, 4, 1, file);
            fread(&scene->lights[i].x, 4, 1, file);
            fread(&scene->lights[i].y, 4, 1, file);
            fread(&scene->lights[i].z, 4, 1, file);
            fread(&scene->lights[i].r, 4, 1, file);
            fread(&scene->lights[i].g, 4, 1, file);
            fread(&scene->lights[i].b, 4, 1, file);
            fread(&scene->lights[i].ambient, 4, 1, file);
            fread(&scene->lights[i].intensity, 4, 1, file);
            fread(&scene->lights[i].attenuation, 4, 1, file);
            scene->lights[i].type = (1 == type) ? SFR_LIGHT_POINT : SFR_LIGHT_DIRECTIONAL;
        }
    }

    fread(&scene->textureCount, 4, 1, file);
    if (scene->textureCount > 0) {
        SFR__MALLOC(scene->textures, sizeof(SfrTexture*) * scene->textureCount);
        SFR__MALLOC(scene->materials, sizeof(SfrMaterial*) * scene->textureCount);

        for (i32 i = 0; i < scene->textureCount; i += 1) {
            i32 w, h;
            fread(&w, 4, 1, file);
            fread(&h, 4, 1, file);

            SFR__MALLOC(scene->textures[i], sizeof(SfrTexture));
            sfr_memset(scene->textures[i], 0, sizeof(SfrTexture));
            
            scene->textures[i]->w = w;
            scene->textures[i]->h = h;
            scene->textures[i]->mipLevels = 1;
            
            const i32 total_pixels = w * h;
            SFR__MALLOC(scene->textures[i]->pixels, sizeof(u32) * total_pixels);

            // decode RLE pixel stream
            i32 decoded = 0;
            while (decoded < total_pixels) {
                u8 count, r, g, b, a;
                fread(&count, 1, 1, file);
                fread(&r, 1, 1, file);
                fread(&g, 1, 1, file);
                fread(&b, 1, 1, file);
                fread(&a, 1, 1, file);

                const u32 pixel_color = ((u32)a << 24) | ((u32)r << 16) | ((u32)g << 8) | (u32)b;
                
                for (i32 p = 0; p < count && decoded < total_pixels; p += 1) {
                    scene->textures[i]->pixels[decoded++] = pixel_color;
                }
            }

            sfr__generate_mipmaps(scene->textures[i]);

            SFR__MALLOC(scene->materials[i], sizeof(SfrMaterial));
            *scene->materials[i] = (SfrMaterial){
                .albedoTex = scene->textures[i],
                .metallicRoughnessTex = NULL,
                .baseColor = 0xFFFFFFFF,
                .metallicFactor = 0.2f,
                .roughnessFactor = 0.8f
            };
        }
    }

    fread(&scene->count, 4, 1, file);
    if (scene->count > 0) {
        SFR__MALLOC(scene->objects, sizeof(SfrSceneObject) * scene->count);

        for (i32 i = 0; i < scene->count; i += 1) {
            i32 texID, pbrTexID;
            f32 metallic, roughness;
            u32 col;
            i32 vertCount;
            fread(&texID, 4, 1, file);
            fread(&pbrTexID, 4, 1, file);
            fread(&metallic, 4, 1, file);
            fread(&roughness, 4, 1, file);
            fread(&col, 4, 1, file);
            fread(&vertCount, 4, 1, file);

            SfrMesh* mesh;
            SFR__MALLOC(mesh, sizeof(SfrMesh));
            
            mesh->vertCount = vertCount * 3; 
            
            SFR__MALLOC(mesh->tris, sizeof(f32) * vertCount * 3);
            SFR__MALLOC(mesh->uvs, sizeof(f32) * vertCount * 2);
            SFR__MALLOC(mesh->lmUvs, sizeof(f32) * vertCount * 2);
            SFR__MALLOC(mesh->normals, sizeof(f32) * vertCount * 3);
            mesh->tangents = NULL;

            if (vertCount > 0) {
                fread(mesh->tris, sizeof(f32), vertCount * 3, file);
                fread(mesh->uvs, sizeof(f32), vertCount * 2, file);
                fread(mesh->lmUvs, sizeof(f32), vertCount * 2, file);
                fread(mesh->normals, sizeof(f32), vertCount * 3, file);
                sfr__mesh_compute_tangents(mesh);
                sfr__mesh_ensure_channels(mesh);
            }

            SfrSceneObject* obj = &scene->objects[i];
            obj->mesh = mesh;
            obj->pos = SFR__V(0.f, 0.f, 0.f, 1.f);
            obj->rot = SFR__V(0.f, 0.f, 0.f, 1.f);
            obj->scale = SFR__V(1.f, 1.f, 1.f, 1.f);
            obj->col = col;

            // allocate a dedicated material for this object
            SfrMaterial* mat;
            SFR__MALLOC(mat, sizeof(SfrMaterial));
            mat->albedoTex = (texID >= 0 && texID < scene->textureCount) ? scene->textures[texID] : NULL;
            mat->metallicRoughnessTex = (pbrTexID >= 0 && pbrTexID < scene->textureCount) ? scene->textures[pbrTexID] : NULL;
            mat->baseColor = col;
            mat->metallicFactor = metallic;
            mat->roughnessFactor = roughness;
            obj->mat = mat;

            obj->_model = sfr_mat_identity();
            obj->_invModel = sfr_mat_identity();
            obj->_normal = sfr_mat_identity();

            const i32 triCount = vertCount / 3;
            if (triCount > 0) {
                SFR__MALLOC(obj->_bvhNodes, sizeof(struct sfrBvhNode) * (2 * triCount - 1));
                obj->_bvhRoot = 0;
                obj->_bvhNodes[0].leftFirst = 0;
                obj->_bvhNodes[0].count = triCount;

                i32 nodePtr = 1;
                sfr__bvh_update_bounds(0, obj->_bvhNodes, mesh, triCount);
                sfr__bvh_subdivide(obj->_bvhNodes, 0, &nodePtr, obj->mesh);
                obj->_bvhNodeCount = nodePtr;
            } else {
                obj->_bvhNodes = NULL;
                obj->_bvhNodeCount = 0;
            }
        }
    }

    i32 atlasDim = 0;
    if (1 == fread(&atlasDim, 4, 1, file) && atlasDim > 0) {
        const i32 pixelCount = atlasDim * atlasDim;
        
        SFR__MALLOC(scene->globalLightmap, sizeof(SfrTexture));
        scene->globalLightmap->w = atlasDim;
        scene->globalLightmap->h = atlasDim;
        scene->globalLightmap->mipLevels = 1;
        
        // allocate the 3 planar quad arrays instead of the normal pixel array
        SFR__MALLOC(scene->globalLightmap->quadR, sizeof(u32) * pixelCount);
        SFR__MALLOC(scene->globalLightmap->quadG, sizeof(u32) * pixelCount);
        SFR__MALLOC(scene->globalLightmap->quadB, sizeof(u32) * pixelCount);
        
        u8* rawRGB;
        SFR__MALLOC(rawRGB, pixelCount * 3);
        
        if (fread(rawRGB, 1, pixelCount * 3, file) == (u64)(pixelCount * 3)) {
            for (i32 y = 0; y < atlasDim; y += 1) {
                for (i32 x = 0; x < atlasDim; x += 1) {
                    // clamp adjacent pixels to the edge of the texture
                    const i32 x1 = (x + 1 < atlasDim) ? x + 1 : x;
                    const i32 y1 = (y + 1 < atlasDim) ? y + 1 : y;
                    
                    // base inds for the 4 corners in the raw array
                    const i32 i00 = (y * atlasDim + x) * 3;
                    const i32 i10 = (y * atlasDim + x1) * 3;
                    const i32 i01 = (y1 * atlasDim + x) * 3;
                    const i32 i11 = (y1 * atlasDim + x1) * 3;
                    
                    const i32 outInd = y * atlasDim + x;

                    // pack the channels
                    scene->globalLightmap->quadR[outInd] = 
                        ((u32)rawRGB[i00 + 0])       | 
                        ((u32)rawRGB[i10 + 0] << 8)  | 
                        ((u32)rawRGB[i01 + 0] << 16) | 
                        ((u32)rawRGB[i11 + 0] << 24);
                    scene->globalLightmap->quadG[outInd] = 
                        ((u32)rawRGB[i00 + 1])       | 
                        ((u32)rawRGB[i10 + 1] << 8)  | 
                        ((u32)rawRGB[i01 + 1] << 16) | 
                        ((u32)rawRGB[i11 + 1] << 24);
                    scene->globalLightmap->quadB[outInd] = 
                        ((u32)rawRGB[i00 + 2])       | 
                        ((u32)rawRGB[i10 + 2] << 8)  | 
                        ((u32)rawRGB[i01 + 2] << 16) | 
                        ((u32)rawRGB[i11 + 2] << 24);
                }
            }
        }
        SFR__FREE(rawRGB);

        // vv load the direction map vv
        SFR__MALLOC(scene->globalDirectionmap, sizeof(SfrTexture));
        scene->globalDirectionmap->w = atlasDim;
        scene->globalDirectionmap->h = atlasDim;
        scene->globalDirectionmap->mipLevels = 1;
        
        SFR__MALLOC(scene->globalDirectionmap->quadR, sizeof(u32) * pixelCount);
        SFR__MALLOC(scene->globalDirectionmap->quadG, sizeof(u32) * pixelCount);
        SFR__MALLOC(scene->globalDirectionmap->quadB, sizeof(u32) * pixelCount);
        
        u8* rawDirRGB;
        SFR__MALLOC(rawDirRGB, pixelCount * 3);

        if (fread(rawDirRGB, 1, pixelCount * 3, file) == (u64)(pixelCount * 3)) {
            for (i32 y = 0; y < atlasDim; y += 1) {
                for (i32 x = 0; x < atlasDim; x += 1) {
                    const i32 x1 = (x + 1 < atlasDim) ? x + 1 : x;
                    const i32 y1 = (y + 1 < atlasDim) ? y + 1 : y;
                    const i32 i00 = (y * atlasDim + x) * 3;
                    const i32 i10 = (y * atlasDim + x1) * 3;
                    const i32 i01 = (y1 * atlasDim + x) * 3;
                    const i32 i11 = (y1 * atlasDim + x1) * 3;
                    const i32 outInd = y * atlasDim + x;

                    scene->globalDirectionmap->quadR[outInd] = 
                        ((u32)rawDirRGB[i00 + 0])       |
                        ((u32)rawDirRGB[i10 + 0] << 8)  | 
                        ((u32)rawDirRGB[i01 + 0] << 16) |
                        ((u32)rawDirRGB[i11 + 0] << 24);
                    scene->globalDirectionmap->quadG[outInd] = 
                        ((u32)rawDirRGB[i00 + 1])       |
                        ((u32)rawDirRGB[i10 + 1] << 8)  | 
                        ((u32)rawDirRGB[i01 + 1] << 16) |
                        ((u32)rawDirRGB[i11 + 1] << 24);
                    scene->globalDirectionmap->quadB[outInd] = 
                        ((u32)rawDirRGB[i00 + 2])       |
                        ((u32)rawDirRGB[i10 + 2] << 8)  | 
                        ((u32)rawDirRGB[i01 + 2] << 16) |
                        ((u32)rawDirRGB[i11 + 2] << 24);
                }
            }
        }
        SFR__FREE(rawDirRGB);
    } else {
        scene->globalLightmap = NULL;
        scene->globalDirectionmap = NULL;
    }

    fclose(file);
    return scene;
}

SFR_FUNC void sfr_release_scene(SfrScene** scene, u8 freeObjects) {
    if (!scene || !(*scene)) {
        return;
    }

    if ((*scene)->objects) {
        for (i32 i = 0; i < (*scene)->count; i += 1) {
            SFR__FREE((*scene)->objects[i]._bvhNodes);
            if (freeObjects && (*scene)->objects[i].mesh) {
                sfr_release_mesh(&(*scene)->objects[i].mesh);
            }
        }
        if (freeObjects) {
            SFR__FREE((*scene)->objects);
        }
    }

    if ((*scene)->materials) {
        for (i32 i = 0; i < (*scene)->textureCount; i += 1) {
            SFR__FREE((*scene)->materials[i]);
        }
        SFR__FREE((*scene)->materials);
    }
    if ((*scene)->textures) {
        for (i32 i = 0; i < (*scene)->textureCount; i += 1) {
            if ((*scene)->textures[i]) {
                sfr_release_texture(&(*scene)->textures[i]);
            }
        }
        SFR__FREE((*scene)->textures);
    }

    SFR__FREE((*scene)->lights);

    if ((*scene)->globalLightmap) {
        sfr_release_texture(&(*scene)->globalLightmap);
    }
    if ((*scene)->globalDirectionmap) {
        sfr_release_texture(&(*scene)->globalDirectionmap);
    }

    SFR__FREE(*scene);
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

static void sfr__process_geometry_jobs(void) {
    // allocate once per thread lifecycle
    struct sfrGeomClipBufs clipBuf;
    struct sfrGeomTri gt = { .clipBuf = &clipBuf };

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

            gt.posA = &job->tris[triInd + 0];
            gt.uvA = &job->uvs[uvInd + 0];
            gt.lmUvA = &job->lmUvs[uvInd + 0];
            gt.normA = &job->normals[triInd + 0];
            gt.tanA = &job->tangents[tanInd + 0];

            gt.posB = &job->tris[triInd + 3];
            gt.uvB = &job->uvs[uvInd + 2];
            gt.lmUvB = &job->lmUvs[uvInd + 2];
            gt.normB = &job->normals[triInd + 3];
            gt.tanB = &job->tangents[tanInd + 4];
            
            gt.posC = &job->tris[triInd + 6];
            gt.uvC = &job->uvs[uvInd + 4];
            gt.lmUvC = &job->lmUvs[uvInd + 4];
            gt.normC = &job->normals[triInd + 6];
            gt.tanC = &job->tangents[tanInd + 8];
            
            gt.renderMode = job->renderMode;

            gt.col = job->col;
            gt.mat = job->mat;

            sfr__process_and_bin_triangle(&job->matMVP, &job->matNormal, &gt);
        }
    }
}

static void sfr__process_raster_jobs(void* data) {
    struct sfrThreadData* tData = (struct sfrThreadData*)data;

    while (1) {
        // get the next tile index from the work queue
        const i32 head = sfr_atomic_add(&sfrThreadBuf->rasterWorkQueueHead, 1) - 1;
        if (head >= sfr_atomic_get(&sfrThreadBuf->rasterWorkQueueCount)) {
            break; // no more raster work
        }

        const i32 tileInd = sfrThreadBuf->rasterWorkQueue[head];
        struct sfrTile* const tile = &sfrThreadBuf->tiles[tileInd];

        tile->minInvZ = 0.f;

        // only this thread is touching this tile so this is fine
        // load existing depth buffer to preserve previous frames
        for (i32 y = 0; y < tile->maxY - tile->minY; y += 1) {
            const i32 globalY = tile->minY + y;
            const i32 w = tile->maxX - tile->minX;
            sfr_memcpy(&tData->localDepth[y * SFR_TILE_WIDTH], &sfrDepthBuf[globalY * sfrWidth + tile->minX], sizeof(f32) * w);
        }

        sfr_memset(tData->localId, -1, sizeof(i32) * SFR_TILE_WIDTH * SFR_TILE_HEIGHT);

        // pull bins from all threads that touched this tile
        for (i32 t = 0; t <= SFR_THREAD_COUNT; t += 1) {
            const struct sfrThreadData* const sourceThread = &sfrThreadBuf->threads[t];
            i32 currentRefInd = sourceThread->tileFirstRef[tileInd];

            while (currentRefInd != -1) {
                const struct sfrBinRef* const ref = &sourceThread->binRefs[currentRefInd];
                const struct sfrRasterBin* const rBin = &sourceThread->rasterBins[ref->binIndex];

                // pass thread's local buffers to be written into
                sfr__rasterize_bin(rBin, tile, tData->localDepth, tData->localId);

                currentRefInd = ref->nextRef;
            }
        }

        // depth writeback
        for (i32 y = 0; y < tile->maxY - tile->minY; y += 1) {
            const i32 globalY = tile->minY + y;
            const i32 w = tile->maxX - tile->minX;
            sfr_memcpy(&sfrDepthBuf[globalY * sfrWidth + tile->minX], &tData->localDepth[y * SFR_TILE_WIDTH], sizeof(f32) * w);
        }

        if (SFR_RENDERMODE_DEPTH_ONLY != sfrState.renderMode) {
            sfr__resolve_tile(tile, tData->localId);
        }

        sfr_atomic_set(&tile->hasWork, 0);
    }
}

#ifdef _WIN32
static unsigned __stdcall sfr__worker_thread_func(void* arg) {
#else
static void* sfr__worker_thread_func(void* arg) {
#endif
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

        sfr__process_geometry_jobs();

        sfr_semaphore_post(&sfrThreadBuf->geometryDoneSem, 1);

        // vv rasterization phase vv
        sfr_semaphore_wait(&sfrThreadBuf->rasterStartSem);
        if (sfrState.shutdown) {
            break;
        }

        struct sfrThreadData* tData = &sfrThreadBuf->threads[sfrTlsThreadInd];

        sfr__process_raster_jobs(tData);

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

#ifndef SFR_NO_SIMD

SFR_FUNC vf32 sfrvf_set1(f32 v) {
#if defined(__AVX2__)
    return _mm256_set1_ps(v);
#elif defined(__ARM_NEON)
    return vdupq_n_f32(v);
#endif
}

SFR_FUNC vi32 sfrvi_set1(i32 v) {
#if defined(__AVX2__)
    return _mm256_set1_epi32(v);
#elif defined(__ARM_NEON)
    return vdupq_n_s32(v);
#endif
}

SFR_FUNC vf32 sfrvf_zero(void) {
#if defined(__AVX2__)
    return _mm256_setzero_ps();
#elif defined(__ARM_NEON)
    return vdupq_n_f32(0.f);
#endif
}

SFR_FUNC vi32 sfrvi_zero(void) {
#if defined(__AVX2__)
    return _mm256_setzero_si256();
#elif defined(__ARM_NEON)
    return vdupq_n_s32(0);
#endif
}

SFR_FUNC vf32 sfrvf_loadu(const f32* mem) {
#if defined(__AVX2__)
    #if SFR_SIMD_LANES == 4
        // load 128 bits and cast up to 256
        return _mm256_castps128_ps256(_mm_loadu_ps(mem));
    #else
        return _mm256_loadu_ps(mem);
    #endif
#elif defined(__ARM_NEON)
    return vld1q_f32(mem);
#endif
}

SFR_FUNC void sfrvf_storeu(f32* mem, vf32 a) {
#if defined(__AVX2__)
    #if SFR_SIMD_LANES == 4
        // extract lower 128 bits and store
        _mm_storeu_ps(mem, _mm256_castps256_ps128(a));
    #else
        _mm256_storeu_ps(mem, a);
    #endif
#elif defined(__ARM_NEON)
    vst1q_f32(mem, a);
#endif
}

SFR_FUNC vi32 sfrvi_loadu(const vi32* mem) {
#if defined(__AVX2__)
    #if SFR_SIMD_LANES == 4
        return _mm256_castsi128_si256(_mm_loadu_si128((const __m128i*)mem));
    #else
        return _mm256_loadu_si256((const __m256i*)mem);
    #endif
#elif defined(__ARM_NEON)
    return vld1q_s32((const int32_t*)mem);
#endif
}

SFR_FUNC void sfrvi_storeu(vi32* mem, vi32 a) {
#if defined(__AVX2__)
    #if SFR_SIMD_LANES == 4
        // extract lower 128 bits and store
        _mm_storeu_si128((__m128i*)mem, _mm256_castsi256_si128(a));
    #else
        _mm256_storeu_si256((__m256i*)mem, a);
    #endif
#elif defined(__ARM_NEON)
    vst1q_s32((int32_t*)mem, a);
#endif
}

SFR_FUNC vf32s sfrvfs_set1(f32 v) {
#if defined(__AVX2__)
    return _mm_set1_ps(v);
#elif defined(__ARM_NEON)
    return vdupq_n_f32(v);
#endif
}

SFR_FUNC f32 sfrvfs_cvtss_f32(vf32s a) {
#if defined(__AVX2__)
    return _mm_cvtss_f32(a);
#elif defined(__ARM_NEON)
    return vgetq_lane_f32(a, 0); // extract first lane
#endif
}

SFR_FUNC vf32s sfrvfs_add(vf32s a, vf32s b) {
#if defined(__AVX2__)
    return _mm_add_ps(a, b);
#elif defined(__ARM_NEON)
    return vaddq_f32(a, b);
#endif
}

SFR_FUNC vf32s sfrvfs_sub(vf32s a, vf32s b) {
#if defined(__AVX2__)
    return _mm_sub_ps(a, b);
#elif defined(__ARM_NEON)
    return vsubq_f32(a, b);
#endif
}

SFR_FUNC vf32s sfrvfs_mul(vf32s a, vf32s b) {
#if defined(__AVX2__)
    return _mm_mul_ps(a, b);
#elif defined(__ARM_NEON)
    return vmulq_f32(a, b);
#endif
}

SFR_FUNC vf32s sfrvfs_div(vf32s a, vf32s b) {
#if defined(__AVX2__)
    return _mm_div_ps(a, b);
#elif defined(__ARM_NEON)
    return vdivq_f32(a, b);
#endif
}

SFR_FUNC vf32s sfrvfs_rsqrt(vf32s a) {
#if defined(__AVX2__)
    return _mm_rsqrt_ps(a);
#elif defined(__ARM_NEON)
    return vrsqrteq_f32(a);
#endif
}

SFR_FUNC vf32s sfrvfs_sqrt_ss(vf32s a) {
#if defined(__AVX2__)
    return _mm_sqrt_ss(a);
#elif defined(__ARM_NEON)
    return vsetq_lane_f32(vgetq_lane_f32(vsqrtq_f32(a), 0), a, 0);
#endif
}

SFR_FUNC vf32 sfrvf_add(vf32 a, vf32 b) {
#if defined(__AVX2__)
    return _mm256_add_ps(a, b);
#elif defined(__ARM_NEON)
    return vaddq_f32(a, b);
#endif
}

SFR_FUNC vf32 sfrvf_sub(vf32 a, vf32 b) {
#if defined(__AVX2__)
    return _mm256_sub_ps(a, b);
#elif defined(__ARM_NEON)
    return vsubq_f32(a, b);
#endif
}

SFR_FUNC vf32 sfrvf_mul(vf32 a, vf32 b) {
#if defined(__AVX2__)
    return _mm256_mul_ps(a, b);
#elif defined(__ARM_NEON)
    return vmulq_f32(a, b);
#endif
}

SFR_FUNC vf32 sfrvf_fmadd(vf32 a, vf32 b, vf32 c) {
#if defined(__AVX2__)
    return _mm256_fmadd_ps(a, b, c);
#elif defined(__ARM_NEON)
    return vfmaq_f32(c, a, b); // NEON fma accumulates into the first arg
#endif
}

SFR_FUNC vf32 sfrvf_fnmadd(vf32 a, vf32 b, vf32 c) {
#if defined(__AVX2__)
    return _mm256_fnmadd_ps(a, b, c); // -(a * b) + c
#elif defined(__ARM_NEON)
    return vmlsq_f32(c, a, b); // c - (a * b)
#endif
}

SFR_FUNC vf32 sfrvf_div(vf32 a, vf32 b) {
#if defined(__AVX2__)
    return _mm256_div_ps(a, b);
#elif defined(__ARM_NEON)
    return vdivq_f32(a, b); // requires AArch64
#endif
}

SFR_FUNC vf32 sfrvf_sqrt(vf32 a) {
#if defined(__AVX2__)
    return _mm256_sqrt_ps(a);
#elif defined(__ARM_NEON)
    return vsqrtq_f32(a); // native AArch64 sqrt
#endif
}

SFR_FUNC vf32 sfrvf_rsqrt(vf32 a) {
#if defined(__AVX2__)
    return _mm256_rsqrt_ps(a);
#elif defined(__ARM_NEON)
    return vrsqrteq_f32(a); 
#endif
}

SFR_FUNC vf32 sfrvf_rcp(vf32 a) {
#if defined(__AVX2__)
    return _mm256_rcp_ps(a);
#elif defined(__ARM_NEON)
    return vrecpeq_f32(a);
#endif
}

SFR_FUNC vf32 sfrvf_floor(vf32 a) {
#if defined(__AVX2__)
    return _mm256_floor_ps(a);
#elif defined(__ARM_NEON)
    return vrndmq_f32(a); // native AArch64 round to minus infinity
#endif
}

SFR_FUNC vf32 sfrvf_min(vf32 a, vf32 b) {
#if defined(__AVX2__)
    return _mm256_min_ps(a, b);
#elif defined(__ARM_NEON)
    return vminq_f32(a, b);
#endif
}

SFR_FUNC vf32 sfrvf_max(vf32 a, vf32 b) {
#if defined(__AVX2__)
    return _mm256_max_ps(a, b);
#elif defined(__ARM_NEON)
    return vmaxq_f32(a, b);
#endif
}

SFR_FUNC vf32 sfrvf_log2(vf32 x) {
    const vi32 i = sfrvi_cast_from_vf32(x);
    const vf32 e = sfrvf_cvtepi32(sfrvi_sub(i, sfrvi_set1(0x3F800000)));
    return sfrvf_mul(e, sfrvf_set1(1.1920928955078125e-7f));
}

SFR_FUNC vf32 sfrvf_exp2(vf32 x) {
    // clamp x to avoid underflowing the exponent into negative numbers/NaNs
    x = sfrvf_max(x, sfrvf_set1(-126.f));

    const vi32 i = sfrvi_cvtps(sfrvf_mul(x, sfrvf_set1(8388608.f)));
    return sfrvf_cast_from_vi32(sfrvi_add(i, sfrvi_set1(0x3F800000)));
}

// fast pow(x, y)
SFR_FUNC vf32 sfrvf_pow(vf32 x, vf32 y) {
    // clamp x to avoid taking the log of zero or negative numbers (NaN/inf)
    const vf32 cx = sfrvf_max(x, sfrvf_set1(SFR_EPSILON));
    return sfrvf_exp2(sfrvf_mul(y, sfrvf_log2(cx)));
}

SFR_FUNC vi32 sfrvi_add(vi32 a, vi32 b) {
#if defined(__AVX2__)
    return _mm256_add_epi32(a, b);
#elif defined(__ARM_NEON)
    return vaddq_s32(a, b);
#endif
}

SFR_FUNC vi32 sfrvi_sub(vi32 a, vi32 b) {
#if defined(__AVX2__)
    return _mm256_sub_epi32(a, b);
#elif defined(__ARM_NEON)
    return vsubq_s32(a, b);
#endif
}

SFR_FUNC vi32 sfrvi_mullo(vi32 a, vi32 b) {
#if defined(__AVX2__)
    return _mm256_mullo_epi32(a, b);
#elif defined(__ARM_NEON)
    return vmulq_s32(a, b);
#endif
}

SFR_FUNC vi32 sfrvi_min(vi32 a, vi32 b) {
#if defined(__AVX2__)
    return _mm256_min_epi32(a, b);
#elif defined(__ARM_NEON)
    return vminq_s32(a, b);
#endif
}

SFR_FUNC vi32 sfrvi_and(vi32 a, vi32 b) {
#if defined(__AVX2__)
    return _mm256_and_si256(a, b);
#elif defined(__ARM_NEON)
    return vandq_s32(a, b);
#endif
}

SFR_FUNC vi32 sfrvi_or(vi32 a, vi32 b) {
#if defined(__AVX2__)
    return _mm256_or_si256(a, b);
#elif defined(__ARM_NEON)
    return vorrq_s32(a, b);
#endif
}

SFR_FUNC vi32 sfrvi_cmpeq(vi32 a, vi32 b) {
#if defined(__AVX2__)
    return _mm256_cmpeq_epi32(a, b);
#elif defined(__ARM_NEON)
    // NEON comparison returns uint32 mask, safely cast back to int32 
    return vreinterpretq_s32_u32(vceqq_s32(a, b)); 
#endif
}

SFR_FUNC vi32 sfrvi_cvtps(vf32 a) {
#if defined(__AVX2__)
    return _mm256_cvtps_epi32(a);
#elif defined(__ARM_NEON)
    return vcvtnq_s32_f32(a);
#endif
}

SFR_FUNC i32 sfrvi_movemask_epi8(vi32 a) {
#if defined(__AVX2__)
    #if SFR_SIMD_LANES == 4
        return _mm256_movemask_epi8(a) & 0x0000FFFF; // 4 lanes = 16 bytes
    #else
        return _mm256_movemask_epi8(a);
    #endif
#elif defined(__ARM_NEON)
    const int8x16_t input = vreinterpretq_s8_s32(a);
    const uint8x16_t msbMask = vcltzq_s8(input);
    const uint8x16_t bitWeights = { 1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128 };
    const uint8x16_t weighted = vandq_u8(msbMask, bitWeights);
    const uint8x8_t p0 = vpadd_u8(vget_low_u8(weighted), vget_high_u8(weighted));
    const uint8x8_t p1 = vpadd_u8(p0, p0);
    const uint8x8_t p2 = vpadd_u8(p1, p1);
    return (vget_lane_u8(p2, 1) << 8) | vget_lane_u8(p2, 0);
#endif
}

SFR_FUNC vf32 sfrvf_cast_from_vi32(vi32 a) {
#if defined(__AVX2__)
    return _mm256_castsi256_ps(a);
#elif defined(__ARM_NEON)
    return vreinterpretq_f32_s32(a);
#endif
}

SFR_FUNC vi32 sfrvi_cast_from_vf32(vf32 a) {
#if defined(__AVX2__)
    return _mm256_castps_si256(a);
#elif defined(__ARM_NEON)
    return vreinterpretq_s32_f32(a);
#endif
}

SFR_FUNC vf32 sfrvf_cvtepi32(vi32 a) {
#if defined(__AVX2__)
    return _mm256_cvtepi32_ps(a);
#elif defined(__ARM_NEON)
    return vcvtq_f32_s32(a);
#endif
}

SFR_FUNC vi32 sfrvi_cvttps(vf32 a) {
#if defined(__AVX2__)
    return _mm256_cvttps_epi32(a);
#elif defined(__ARM_NEON)
    return vcvtq_s32_f32(a);
#endif
}

SFR_FUNC vf32 sfrvf_and(vf32 a, vf32 b) {
#if defined(__AVX2__)
    return _mm256_and_ps(a, b);
#elif defined(__ARM_NEON)
    return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b)));
#endif
}

SFR_FUNC i32 sfrvf_movemask(vf32 a) {
#if defined(__AVX2__)
    #if SFR_SIMD_LANES == 4
        return _mm256_movemask_ps(a) & 0x0F; // kill upper 4 lanes
    #else
        return _mm256_movemask_ps(a);
    #endif
#elif defined(__ARM_NEON)
    // extract the sign bit of each float to create a 4 bit mask
    uint32x4_t m = vreinterpretq_u32_f32(a);
    m = vshrq_n_u32(m, 31);
    const uint32x4_t shifts = {1, 2, 4, 8};
    m = vmulq_u32(m, shifts);
    return vgetq_lane_u32(m, 0) + vgetq_lane_u32(m, 1) + 
           vgetq_lane_u32(m, 2) + vgetq_lane_u32(m, 3);
#endif
}

SFR_FUNC void sfrvi_maskstore(int* mem, vi32 mask, vi32 a) {
#if defined(__AVX2__)
    #if SFR_SIMD_LANES == 4
        // kill the upper 4 lanes
        const __m256i simMask = _mm256_setr_epi32(-1, -1, -1, -1, 0, 0, 0, 0);
        mask = _mm256_and_si256(mask, simMask);
    #endif
    _mm256_maskstore_epi32(mem, mask, a);
#elif defined(__ARM_NEON)
    const uint32x4_t m = vreinterpretq_u32_s32(mask);
    if (vgetq_lane_u32(m, 0) & 0x80000000) mem[0] = vgetq_lane_s32(a, 0);
    if (vgetq_lane_u32(m, 1) & 0x80000000) mem[1] = vgetq_lane_s32(a, 1);
    if (vgetq_lane_u32(m, 2) & 0x80000000) mem[2] = vgetq_lane_s32(a, 2);
    if (vgetq_lane_u32(m, 3) & 0x80000000) mem[3] = vgetq_lane_s32(a, 3);
#endif
}

SFR_FUNC vf32 sfrvf_blendv(vf32 a, vf32 b, vf32 mask) {
#if defined(__AVX2__)
    return _mm256_blendv_ps(a, b, mask);
#elif defined(__ARM_NEON)
    const uint32x4_t msbMask = vcltzq_s32(vreinterpretq_s32_f32(mask));
    return vreinterpretq_f32_u32(vbslq_u32(
        msbMask,
        vreinterpretq_u32_f32(b),
        vreinterpretq_u32_f32(a)
    ));
#endif
}

#endif // !SFR_NO_SIMD

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
    #undef vf32
    #undef vf32s
    #undef vi32
#endif

#ifdef __cplusplus
}
#endif

#endif // SFR_H
