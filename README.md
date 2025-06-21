# sofren - Single File Software Renderer

### A minimalistic, efficient, single file, cross platform software renderer

---

*All of the following were originally at 1280x720, however the GIF quality is forced to be lower*

[*examples/full-sdl2.c*](https://github.com/cyprus327/sofren/blob/main/examples/tex-starter-sdl2.c)

![main demo](https://github.com/user-attachments/assets/73646581-9351-4320-b029-6f31cd42f79f)

[*examples/font-starter-sdl2.c*](https://github.com/cyprus327/sofren/blob/main/examples/font-starter-sdl2.c)

![sfrfontdemo1](https://github.com/user-attachments/assets/87f62598-b39e-4d04-b19d-0f97ddba1622)

[*examples/starter-console.c*](https://github.com/cyprus327/sofren/blob/main/examples/tex-starter-sdl2.c)

![sfrconsoledemo1](https://github.com/user-attachments/assets/36b51566-7893-4729-a498-b18c6569ea83)

---

For examples and good starting points rendering to an SDL2 window or console window, see the examples folder

## Features
- As few a 0 other headers, can be 100% standalone
- Single file, just `#include "sofren.c"`
- Cross platform multithreading (Windows or pthreads)
- Perspective correct texture mapping (currently only .bmp image support)
- Transparency (known limitation / bug, see the bottom of this README)
- Gouraud shading with a directional light
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

#define SFR_IMPL // indicate declaration of functions

#define SFR_NO_STD    // don't include 'stdio' and 'stdlib.h'
#define SFR_NO_MATH   // don't include 'math.h' and use bundled math
#define SFR_NO_STRING // don't include 'string.h'
#define SFR_NO_STDINT // don't include 'stdint.h'

// disable culling triangles
#define SFR_NO_CULLING

// disable transparency, big FPS boost
#define SFR_NO_ALPHA

// types are defined as things like 'i32' or 'u64' internally,
// if this causes some issue define this for types
// to now be defined as 'sfri32_t', 'sfru64_t', etc.
#define SFR_PREFIXED_TYPES

// don't use #warning for reporting potential comp time problems
#define SFR_NO_WARNINGS

// if you don't want functions to be static define this
// overwrites SFR_USE_INLINE, if it is defined
#define SFR_FUNC

// defines SFR_FUNC as either 'static inline' default is 'static'
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
#define SFR_THREAD_COUNT      // default of 1, i.e. single threaded
#define SFR_TILE_WIDTH        // default of 64, in pixels
#define SFR_TILE_HEIGHT       // default of 64, in pixels
#define SFR_MAX_BINS_PER_TILE // default of 4096, max tris that can be rendered on one screen tile 
#define SFR_MAX_BINNED_TRIS   // default of 256k, max tris that can be rendered per frame
```

## Global Variables

There are some global variables in sofren.c, those in the public API are:

```c
// pixel and depth buffer's dimensions
// set on init but can be changed if your window resizes or something,
// just remember to call realloc on sfrPixelBuf and sfrDepthBuf
extern i32 sfrWidth, sfrHeight;

// all buffers, i.e. pixel, depth, accumulation, as well as threading
// data (tiling system, work dispatch, and thread management)
extern SfrBuffers* sfrBuffers;

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
sfr_reset();                   // reset model matrix to identity
sfr_rotate_y(time);            // rotate about y, radians not degrees
sfr_rotate_x(time * 0.5f);     // rotate about x
sfr_scale(1.f, 3.f, 0.5f);     // scale by x y z
sfr_translate(0.f, 0.f, 2.5f); // move to x y z
sfr_cube(0xFFFF0000);          // draw pure red cube (ARGB colors)
``` 

## TODO / Upcoming Features / Known Bugs
- Fix bug with transparency where if there are two transparent objects separated by a solid object and the scene is viewed so the transparent objects overlap one another through the solid object, the object behind the wall will be visible inside of the closer object, e.g.. the following scene 0 | 0 in 2D viewed from a side where the 0s overlap one another, the second 0 is visible inside the first one
- Further optimized rendering/rasterizing
- Per vertex colors
- Change to fixed point math
- AA, maybe
- Stencil buffer, maybe
- Shadows, maybe maybe

# License
This project is under the MIT license, do with that what you will
