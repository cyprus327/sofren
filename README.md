# sofren - Single File Software Renderer

### An efficiently multithreaded, single file, cross platform software renderer

---

<details>
  <summary>About...</summary>
  Lighting baked in Blender, exported to .glb, and loaded using sofren's cgltf wrapper (sfr_load_model).
  
  This is meant to show that lightmaps work in sofren, allowing for much better lighting than would otherwise be possible when rasterizing at high resolutions.
  
  Background skybox from [here](https://bumbadida.itch.io/skybox-textures-sdr).

  The scene is being rendered at 1280x720 235fps.
</details>
<img width="1274" height="714" alt="cornellbox" src="https://github.com/user-attachments/assets/137517ac-d80d-4d9d-bb74-eccc85dca6a8" />

---

Sponza at 720p 60fps on an older laptop while recording
<details>
  <summary>About...</summary>
  The model was loading using raylib's LoadModel function and then converted to sofren's structs,
  after which an SfrScene was created to allow for raycasting.

  In newer versions, this model would've been loaded with the built in function that uses cgltf.
  Also currently phong shading used in this video is removed because of the SIMD additions, but I might add it back.

  Additionally, textures were resized down to 128x128 from 1024x1024 (from profiling, like 40% of the
  program's time was just from cache misses so this was the "fix", mipmapping is on the TODO list).
  If you're wondering why the triangle count is changing when nothing is moving, it changes because
  the text showing FPS and the triangle count is comprised of triangles (.srft font format), and
  these triangles are still taken into account when counting triangles rendered.
</details>

https://github.com/user-attachments/assets/c11733d0-74b1-42f1-ad72-7a49711e4004

[*examples/font-starter-sdl2.c*](https://github.com/cyprus327/sofren/blob/main/examples/font-starter-sdl2.c)

![sfrfontdemo1](https://github.com/user-attachments/assets/87f62598-b39e-4d04-b19d-0f97ddba1622)

[*examples/starter-console.c*](https://github.com/cyprus327/sofren/blob/main/examples/tex-starter-sdl2.c)

![sfrconsoledemo1](https://github.com/user-attachments/assets/36b51566-7893-4729-a498-b18c6569ea83)

---

For examples and good starting points rendering to an SDL2 window or console window, see the examples folder

## Features
- Single file with as few a 0 other headers, can be 100% standalone
- Cross platform multithreading (Windows or pthreads)
- Perspective correct texture mapping (currently only .bmp image support unless `optional/stb_image.h` is used)
- Efficient static scene rendering with fast raycasting (BVH)
- SIMD rasterizing loop (or scalar fallback if SIMD is unavailable)
- Gouraud shading with a directional light
- Custom font format (.srft, see [sfr-fontmaker]([https://g](https://github.com/cyprus327/sfr-fontmaker)))
- OBJ mesh loading (requires `stdio.h`)
- GLB / GLTF model loading (requires `optional/cgltf.h` (with `optional/stb_image.h` for textures))
- Customizable math implementations (system or bundled)
- Primitive drawing (triangles, cubes, billboards)
- Skyboxes (cubemaps not spheremaps)
- ARGB8888 color format support (alpha currently unused)
- Backface culling, depth buffering, and clipping
- Simplistic design, quick to learn and use

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

// max width and height of the window to avoid reallocations when resizing
#define SFR_MAX_WIDTH  // default of 1920
#define SFR_MAX_HEIGHT // default of 1080

// only applicable when SFR_NO_MATH is defined, their values
// dictate the accuracy of the bundled math functions
#define SFR_SQRT_ACCURACY // default of 20
#define SFR_TRIG_ACCURACY // default of 10

// for text rendering
#define SFR_FONT_GLYPH_MAX // max glyphs per font, default of 512
#define SFR_FONT_VERT_MAX  // max verts per glyph, default of 72 (12 tris)

// when SFR_THREAD_COUNT isn't 1, below is applicable
#define SFR_THREAD_COUNT       // default of 1, i.e. single threaded
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
// pixel and depth buffer's dimensions
// set on init but can be changed if your window resizes or something,
// just remember to call realloc on sfrPixelBuf and sfrDepthBuf
extern i32 sfrWidth, sfrHeight;

// sofren isn't double buffered, you could easily make it though
extern u32* sfrPixelBuf;
extern f32* sfrDepthBuf;

// directional light for gouraud shading
extern SfrLight sfrLight;

// how many triangles have been rasterized since the last call to clear
// atomic since it is updated by multiple threads when multithreading is enabled
// if multithreading isn't enabled, SfrAtomic32 == i32 == int32_t
extern SfrAtomic32 sfrRasterCount;

// current matrices being used for rendering,
// use all the provided functions when changing
extern sfrmat sfrMatModel, sfrMatView, sfrMatProj;

extern sfrvec sfrCamPos; // use 'sfr_set_camera' when changing
extern f32    sfrCamFov; // use 'sfr_set_fov' when changing

// can be freely changed, just call 'sfr_set_fov(sfrCamFov)' to
// update 'sfrMatProj' to account for the change
extern f32 sfrNearDist, sfrFarDist;
```

## Simple Example

**For more in depth examples, see the examples folder**

### Red Rotating Cube
```c
// this is the correct order of operations (rotate, scale, translate)
sfr_reset();                   // reset model matrix to identity
sfr_rotate_y(time);            // rotate about y, radians not degrees
sfr_rotate_x(time * 0.5f);     // rotate about x
sfr_scale(1.f, 3.f, 0.5f);     // scale by x y z
sfr_translate(0.f, 0.f, 2.5f); // move to x y z
sfr_cube(0xFFFF0000);          // draw pure red cube (ARGB colors, but A currently unused)
``` 

## TODO / Upcoming Features / Known Bugs
- Fix overdraw (greenish pixels) on areas with many small / overlapping triangles
<img width="1273" height="714" alt="overdraw visual" src="https://github.com/user-attachments/assets/bfd0f9f1-b8d7-44d3-86e7-f33601f37e3d" />

- Skeletal animation / some improved animation system
- Mipmapping / memory optimizations, complex scenes are currently memory bound
- Shadowmapping for static scenes
- Further optimized rendering/rasterizing
- Change to fixed point math, maybe
- Dynamic shadows, maybe maybe

# License
This project is under the MIT license, do with that what you will
