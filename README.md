# sofren - Single File Software Renderer

### A minimalistic, efficient, single file, cross platform software renderer

---

Sponza at 720p 60fps on an older laptop while recording
<details>
  <summary>About...</summary>
  The model was loading using raylib's LoadModel function and then converted to sofren's structs,
  and the textures were resized down to 128x128 from 1024x1024 (from profiling, like 40% of the
  program's time was just from cache misses so this was the "fix", mipmapping is on the TODO list).
</details>

https://github.com/user-attachments/assets/c11733d0-74b1-42f1-ad72-7a49711e4004

*All of the following were originally at 1280x720, however the GIF quality is forced to be lower*

[*examples/full-sdl2.c*](https://github.com/cyprus327/sofren/blob/main/examples/full-sdl2.c)

![birds demo](https://github.com/user-attachments/assets/73646581-9351-4320-b029-6f31cd42f79f)

[*examples/font-starter-sdl2.c*](https://github.com/cyprus327/sofren/blob/main/examples/font-starter-sdl2.c)

![sfrfontdemo1](https://github.com/user-attachments/assets/87f62598-b39e-4d04-b19d-0f97ddba1622)

[*examples/starter-console.c*](https://github.com/cyprus327/sofren/blob/main/examples/tex-starter-sdl2.c)

![sfrconsoledemo1](https://github.com/user-attachments/assets/36b51566-7893-4729-a498-b18c6569ea83)

---

For examples and good starting points rendering to an SDL2 window or console window, see the examples folder

## Features
- Single file with as few a 0 other headers, can be 100% standalone
- Cross platform multithreading (Windows or pthreads)
- Perspective correct texture mapping (currently only .bmp image support)
- Phong shading, with directional and sphere lights
- Custom font format (.srft, see [sfr-fontmaker]([https://g](https://github.com/cyprus327/sfr-fontmaker)))
- Transparency (known limitation / bug, see the bottom of this README)
- OBJ mesh loading (requires `stdio.h`)
- Customizable math implementations (system or bundled)
- Primitive drawing (triangles, cubes, billboards)
- ARGB8888 color format support
- Backface culling, depth buffering, and clipping
- Simplistic design, quick to learn and use

## Pre Processing Configuration 
```c
// the default values are the opposite of the macro unless otherwise stated,
// e.g. functions aren't inline by default

// indicate declaration of functions, only define this in one file
#define SFR_IMPL

#define SFR_NO_STD    // don't include 'stdio' and 'stdlib.h'
#define SFR_NO_MATH   // don't include 'math.h' and use bundled math
#define SFR_NO_STRING // don't include 'string.h'
#define SFR_NO_STDINT // don't include 'stdint.h'

// disable culling triangles
#define SFR_NO_CULLING

// disable transparency, big FPS boost
#define SFR_NO_ALPHA

// whether or not to use SSE 4.1 intrinsics
#define SFR_USE_SIMD

// whether or not to use expensive but pretty phong shading
// defaults to goraud shading using one directional light (sfrLights[0]) otherwise
#define SFR_USE_PHONG

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
#define SFR_MAX_BINS_PER_TILE  // default of 8128, max tris that can be rendered on one screen tile 
#define SFR_MAX_BINS_PER_FRAME // default of 256k, max tris that can be rendered per frame
#define SFR_GEOMETRY_JOB_SIZE  // default of 64, number of tris per geometry job
#define SFR_MAX_GEOMETRY_JOBS  // default of 8128, max mesh chunks that can be processed per frame
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

// of size SFR_MAX_LIGHTS, just an array of all the light sources
extern SfrLight* sfrLights;

// how many triangles have been rasterized since the last call to clear
extern i32 sfrRasterCount;

// current matrices being used for rendering,
// use all the provided functions when changing
extern Mat sfrMatModel, sfrMatView, sfrMatProj;

extern Vec sfrCamPos; // use 'sfr_set_camera' when changing
extern f32 sfrCamFov; // use 'sfr_set_fov' when changing

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
sfr_cube(0xFFFF0000);          // draw pure red cube (ARGB colors)
``` 

## TODO / Upcoming Features / Known Bugs
- Mipmapping / memory optimizations, complex scenes are currently memory bound
- Fix bug with transparency where if there are two transparent objects separated by a solid object and the scene is viewed so the transparent objects overlap one another through the solid object, the object behind the wall will be visible inside of the closer object, e.g.. the following scene 0 | 0 in 2D viewed from a side where the 0s overlap one another, the second 0 is visible inside the first one
- Shadowmapping for static scenes
- Further optimized rendering/rasterizing
- Change to fixed point math, maybe
- AA, maybe
- Dynamic shadows, maybe maybe

# License
This project is under the MIT license, do with that what you will
