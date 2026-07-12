# sofren - Single File Software Renderer

### An efficiently multithreaded, vectorized, single file, cross platform, deferred renderer

---

Blinn-Phong shading

<p align="center">
  <img src="https://github.com/user-attachments/assets/692a2758-875d-4657-9dd9-6c13cb042ff3"
      width="800">
</p>

---

Realtime point light shadows demo at 720p

<div align="center">
  <video src="https://github.com/user-attachments/assets/94d075f4-d50e-463e-84c0-6cd493a7cfb8" width="100%" autoplay loop muted playsinline controls></video>
</div>

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

### Core & Architecture
- Single file library (`stdio.h` and `stdlib.h` are the only strictly required headers)
- Cross-platform multithreading (Windows API or pthreads)
- SIMD wrappers supporting AVX/AVX2 (`immintrin.h`) and ARM NEON (`arm_neon.h`) intrinsics, with scalar fallbacks
- Customizable math implementations (use the bundled math or your system's)
- Left handed coordinate system using row major matrices
### Rendering Pipeline
- Deferred rendering with a visibility buffer
- SIMD accelerated rasterization and geometry pipelines
- Perspective correct texture mapping with auto generated, smooth mipmaps
- Per triangle transparency, backface culling, depth buffering, and clipping
- Efficient static scene rendering with fast raycasting via a BVH
- ARGB8888 color format
### Lighting & Shadows
- Blinn-Phong shading with point and directional lights
- Realtime dynamic and static shadowmaps with point and directional lights
- Baked lighting and lightmap support (for unwrapped UVs)
### Assets & Animation
- GLB / GLTF model loading (requires `optional/cgltf.h` and `optional/stb_image.h`)
- Skeletal animation via CPU skinning with hierarchical node transforms
- Animation blending and crossfading, supporting Cubic Spline, Step, and Linear interpolation
- OBJ mesh loading
- Texture loading (native `.bmp` support, or universal image support via `optional/stb_image.h`)
- Built-in primitive generation (triangles, cubes, spheres, cylinders, batched billboards, etc.)
- Skyboxes (cubemap format)
- Custom `.srft` font format support (see [sfr-fontmaker]([https://g](https://github.com/cyprus327/sfr-fontmaker)))

## Pre Processing Configuration 
```c
// the default values are the opposite of the macro unless otherwise stated,
// e.g. functions aren't inline by default and SIMD is enabled by default

// indicate declaration of functions, only define this in one file
#define SFR_IMPL

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
#define SFR_THREAD_COUNT      // default of 7 worker threads
#define SFR_TILE_WIDTH        // default of 64, in pixels
#define SFR_TILE_HEIGHT       // default of 64, in pixels
#define SFR_GEOMETRY_JOB_SIZE // default of 64, number of tris per geometry job

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
sfr_cube(0xFFFF0000);          // draw pure red cube (ARGB colors)
```

## TODO / Upcoming Features / Known Bugs
- Get normal maps working
- Improved shadows
- Indexed rendering rewrite (maybe)

## Gallery

<p align="center">
  <img src="https://github.com/user-attachments/assets/04cce14f-651a-4267-8511-58edd713559f"
      width="800">
</p>

Some more baked lighting images, these are much older than the one at the top of this README,
and contain lots of noise and splotchyness that has since been fixed.

<img width="1280" height="720" alt="baked room 2" src="https://github.com/user-attachments/assets/abd6ebf6-047c-4fee-bacd-22878eadbe9e" />

<img width="1280" height="720" alt="baked pillars" src="https://github.com/user-attachments/assets/cb407585-8b76-48c1-a057-ad870a713fce" />

<img width="1280" height="720" alt="baked corridor" src="https://github.com/user-attachments/assets/9f184b37-2587-403b-9103-a0749fbee154" />

<img width="1280" height="720" alt="baked room 1" src="https://github.com/user-attachments/assets/03d9c19f-22f9-4ba1-addf-06198509289b" />

---

<details>
  <summary>About the animation...</summary>
  This animation is now natively possible without the hack below just by calling sfr_model_animate

  However this clip is faked / hacked animation. I made a python script to export each frame of an animated
  .fbx model to many individual .glb models, where I then load each frame in an array of
  SfrModel* and render them sequentially (like a spritesheet but with full models).
  This is obviously terrible and as mentioned doesn't need to be done anymore.
</details>

https://github.com/user-attachments/assets/8ad46e35-616c-48bd-bc51-4779224e59b1

---

# License
This project is under the MIT license, do with that what you will
