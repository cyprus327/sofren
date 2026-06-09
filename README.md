# sofren - Single File Software Renderer

### An efficiently multithreaded, vectorized, single file, cross platform, deferred renderer

---

Realtime Blinn-Phong shading

<p align="center">
  <img src="https://github.com/user-attachments/assets/692a2758-875d-4657-9dd9-6c13cb042ff3"
      width="800">
</p>

---

Sponza at 720p, 73 FPS, with realtime dynamic shadows

<p align="center">
  <img src="https://github.com/user-attachments/assets/04cce14f-651a-4267-8511-58edd713559f"
      width="800">
</p>

---

Baked lighting

<details>
  <summary>About...</summary>

  Everything in this scene is baked, even the specular, no lights are enabled in this screenshot.

  This is exported from an editor designed around outputting to sofren that I've been working on between classes.

  The editor isn't public because the code quality isn't good, and I'm not sure it will ever be public.

  However, sofren supports baked lighting generated from other software like Blender via glb/gltf models.

  Textures from [ambientcg](https://ambientcg.com/)
</details>

<p align="center">
  <img src="https://github.com/user-attachments/assets/93e7a30c-a026-4a5f-aec6-70038767173a"
      width="800">
</p>

---

For examples and good starting points rendering to an SDL2 window or console window, see the examples folder

## Features
- Single file with as few a 0 other headers, can be 100% standalone
- Cross platform multithreading (Windows or pthreads)
- Deferred rendering with a visibility buffer
- SIMD rasterizing and geometry pipelines (or scalar fallback if SIMD is unavailable)
- SIMD wrappers supporting AVX/AVX2 (`immintrin.h`) and ARM NEON (`arm_neon.h`) intrinsics
- Blinn-Phong shading with point and directional lights
- Realtime dynamic and static shadowmaps
- Baked lighting support (if UVs are unwrapped already)
- Smooth mipmaps auto generated and used for all loaded textures
- Perspective correct texture mapping (currently only .bmp image support unless `optional/stb_image.h` is used)
- Custom font format (.srft, see [sfr-fontmaker]([https://g](https://github.com/cyprus327/sfr-fontmaker)))
- OBJ mesh loading (requires `stdio.h`)
- GLB / GLTF model loading (requires `optional/cgltf.h` (with `optional/stb_image.h` for textures))
- Customizable math implementations (system or bundled)
- Primitive drawing (triangles, cubes, spheres, cylinders, billboards, etc.)
- Skyboxes (cubemaps not spheremaps)
- Efficient static scene rendering with fast raycasting via a BVH
- ARGB8888 color format support (alpha currently unused)
- Backface culling, depth buffering, and clipping
- Left handed and row major

## Windows Compilation (with SIMD)

If you're compiling on Windows using MinGW/GCC with SIMD enabled (the default) and optimization flags
like `-O2` or `-O3`, you must include specific compiler flags to prevent segfaults.

The Windows x64 ABI mandates a 16 byte stack alignment, but GCC's AVX implementation strictly requires a
32 byte alignment. When GCC omits the frame pointer in release mode, its internal stack offset math can
break across function pointer boundaries, overwriting return addresses and corrupting the stack.
Also, GCC can aggressively optimize unaligned SIMD memory stores into strict aligned instructions,
which will instantly crash if your heap isn't 32 byte aligned.

This isn't technically needed if run your program through some debugger like gdb since when Windows
detects a debugger it will replace malloc/free with the debug heap, which adds padding around each allocation.

**Required Compiler Flags for Windows Release Builds:**
```bash
-fno-omit-frame-pointer
```

## Windows Memory Allocation

Standard `malloc` on Windows only guarantees 16 byte alignment, and since sofren uses 256 bit SIMD intrinsics
(unless you're using an older ARM processor, in which case it will use 128 bit), passing standard malloc
to sfr_init will cause a crash on Windows. You must provide 32 byte aligned allocators on Windows:

```c
#ifdef _WIN32
    #include <malloc.h>
    static void* sfr_malloc(u64 size) { return _aligned_malloc(size, 32); }
    static void* sfr_realloc(void* ptr, u64 size) { return _aligned_realloc(ptr, size, 32); }
    static void sfr_free(void* ptr) { _aligned_free(ptr); }
#else
    #include <stdlib.h>
    static void* sfr_malloc(u64 size) { return malloc(size); }
    static void* sfr_realloc(void* ptr, u64 size) { return realloc(ptr, size); }
    static void sfr_free(void* ptr) { free(ptr); }
#endif

// ... later

sfr_init(width, height, fov, sfr_malloc, sfr_free, sfr_realloc);
```

## Pre Processing Configuration 
```c
// the default values are the opposite of the macro unless otherwise stated,
// e.g. functions aren't inline by default and SIMD is enabled by default

// indicate declaration of functions, only define this in one file
#define SFR_IMPL

#define SFR_NO_STD    // don't include 'stdio' and 'stdlib.h'
#define SFR_NO_MATH   // don't include 'math.h' and use bundled math
#define SFR_NO_STRING // don't include 'string.h'
#define SFR_NO_STDINT // don't include 'stdint.h'

// disable culling triangles
#define SFR_NO_CULLING

// tiny triangles < 0.1 pixels will be culled if this isn't defined
// on highly detailed models this can make them sort of fade out at distance
#define SFR_NO_SMALL_CULLING

// whether or not to use SIMD (immintrin.h)
#define SFR_NO_SIMD

// types are defined as things like 'i32' or 'u64' internally,
// if this causes some issue define this for types
// to now be defined as 'sfri32_t', 'sfru64_t', etc.
#define SFR_PREFIXED_TYPES

// don't use #warning for reporting potential comp time problems
#define SFR_NO_WARNINGS

// if you don't want functions to be static define this
// overwrites SFR_USE_INLINE, if it is defined
#define SFR_FUNC

// defines SFR_FUNC as 'static inline' default is 'static'
#define SFR_USE_INLINE

// only applicable when SFR_NO_MATH is defined, their values
// dictate the accuracy of the bundled math functions
#define SFR_SQRT_ACCURACY // default of 20
#define SFR_TRIG_ACCURACY // default of 10

// for text rendering
#define SFR_FONT_GLYPH_MAX // max glyphs per font, default of 512
#define SFR_FONT_VERT_MAX  // max verts per glyph, default of 72 (12 tris)

// when SFR_THREAD_COUNT isn't 1, below is applicable
#define SFR_THREAD_COUNT       // default of 8, i.e. multithreaded
#define SFR_TILE_WIDTH         // default of 64, in pixels
#define SFR_TILE_HEIGHT        // default of 64, in pixels
#define SFR_GEOMETRY_JOB_SIZE  // default of 64, number of tris per geometry job
#define SFR_BIN_PAGE_SIZE      // default of 4096, number of tris per page for tiles

// glb / gltf related
#define SFR_USE_CGLTF      // have model loading via cgltf
#define SFR_USE_STB_IMAGE  // support all image types via stb_image (not just .bmp)
#define SFR_CGLTF_PATH     // where cgltf.h is (used as #include SFR_CGLTF_PATH)
#define SFR_STB_IMAGE_PATH // where stb_image.h is (used as #include SFR_STB_IMAGE_PATH)
```

## Global Variables

There are some global variables in sofren.c, those in the public API are:

```c
// how many triangles have been rasterized since the last call to clear
// atomic since it is updated by multiple threads when multithreading is enabled
// if multithreading isn't enabled, SfrAtomic32 == i32 == int32_t
extern SfrAtomic32 sfrRasterCount;

// current matrices being used for rendering,
// use all the provided functions when changing
extern sfrmat sfrMatModel, sfrMatView, sfrMatProj;

// use 'sfr_set_camera' when changing
extern sfrvec sfrCamPos, sfrCamUp, sfrCamTarget;
// use 'sfr_set_fov' when changing
extern f32 sfrCamFov;

// can be freely changed, just call 'sfr_set_fov(sfrCamFov)' to
// update 'sfrMatProj' to account for the change
extern f32 sfrNearDist, sfrFarDist;
```

## Simple Example

**For more in depth examples, see the examples folder**

### Red Rotating Cube
```c
// this is the correct order of operations (scale, rotate, translate), sofren is row major
sfr_reset();                   // reset model matrix to identity
sfr_scale(1.f, 3.f, 0.5f);     // scale by x y z
sfr_rotate_y(time);            // rotate about y, radians not degrees
sfr_rotate_x(time * 0.5f);     // rotate about x
sfr_translate(0.f, 0.f, 2.5f); // move to x y z
sfr_cube(0xFFFF0000);          // draw pure red cube (ARGB colors, but A currently unused)
```

## TODO / Upcoming Features / Known Bugs
- Skeletal animation / some improved animation system
- Get normal maps working
- Improved shadows

## Gallery

Some more baked lighting images, these are much older than the one at the top of this README,
and contain lots of noise and splotchyness that has since been fixed.

<img width="1280" height="720" alt="baked room 2" src="https://github.com/user-attachments/assets/abd6ebf6-047c-4fee-bacd-22878eadbe9e" />

<img width="1280" height="720" alt="baked pillars" src="https://github.com/user-attachments/assets/cb407585-8b76-48c1-a057-ad870a713fce" />

<img width="1280" height="720" alt="baked corridor" src="https://github.com/user-attachments/assets/9f184b37-2587-403b-9103-a0749fbee154" />

<img width="1280" height="720" alt="baked room 1" src="https://github.com/user-attachments/assets/03d9c19f-22f9-4ba1-addf-06198509289b" />

---

<details>
  <summary>About the animation...</summary>
  This is (for now) faked / hacked animation. I made a python script to export each frame of an animated
  .fbx model to many individual .glb models, where I then load each frame in an array of
  SfrModel* and render them sequentially (like a spritesheet but with full models).
  This is obviously terrible and won't be done in the future when skinned mesh rendering is supported.
</details>

https://github.com/user-attachments/assets/8ad46e35-616c-48bd-bc51-4779224e59b1

---

# License
This project is under the MIT license, do with that what you will
