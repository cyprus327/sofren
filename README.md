# sofren - Single File Software Renderer

### A minimalistic, efficient, single file, cross platform software renderer

---

[*examples/tex-starter-sdl2.c*](https://github.com/cyprus327/sofren/blob/main/examples/tex-starter-sdl2.c)

![simple textures demo](https://github.com/user-attachments/assets/83dbc4ed-2fad-42e1-839e-ae6d80d4919b)

[*examples/starter-console.c*](https://github.com/cyprus327/sofren/blob/main/examples/tex-starter-sdl2.c)

![sfrconsoledemo1](https://github.com/user-attachments/assets/36b51566-7893-4729-a498-b18c6569ea83)

---

For examples and good starting points rendering to an SDL2 window or console window, see the examples folder

## Features
- As few a 0 other headers included, can be 100% standalone
- Single file - just `#include "sofren.c"`
- Primitive drawing (triangles, cubes)
- Perspective correct texture mapping
- OBJ mesh loading (requires `stdio.h`)
- Flat shading with directional lighting
- Customizable math implementations (system or bundled)
- Simplistic design, quick to learn and use
- ARGB8888 color format support
- Backface culling, depth buffering, and clipping

## Pre Processing Configuration 
```c
#define SFR_IMPL // indicate declaration of functions

#define SFR_NO_STD    // don't include 'stdio' and 'stdlib.h'
#define SFR_NO_MATH   // don't include 'math.h' and use bundled math
#define SFR_NO_STRING // don't include 'string.h'
#define SFR_NO_STDINT // don't include 'stdint.h'

#define SFR_NO_CULLING // disable culling triangles

// types are defined as things like 'i32' or 'Vec' internally,
// if this causes some issue define this for types
// to now be defined as 'sfri32_t', 'sfrvec_t', etc.
#define SFR_PREFIXED_TYPES

// don't use #warning for reporting potential comp time problems
#define SFR_NO_WARNINGS

// if you don't want functions to be static define this
// overwrites SFR_USE_INLINE, if it is defined
#define SFR_FUNC

// defines SFR_FUNC as either 'static inline' default is 'static'
#define SFR_USE_INLINE

// only applicable when SFR_NO_MATH is defined, their values
// dictate the accuracy of the bundled math functions
#define SFR_SQRT_ACCURACY // defaults to 20 if not defined
#define SFR_TRIG_ACCURACY // defaults to 10 if not defined
```

## Global Variables

There are some global variables in sofren.c, those in the public API are:

```c
// pixel and depth buffer's dimensions
// set on init but can be changed if your window resizes or something,
// just remember to call realloc on sfrPixelBuf and sfrDepthBuf
extern i32 sfrWidth, sfrHeight;

// the pixel and depth buffers, used interally but memory managed by you
extern u32* sfrPixelBuf; // ARGB8888, i.e. 0xFF0000 == RED, 0x0000FF == BLUE
extern f32* sfrDepthBuf;

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

## Examples

**For more in depth examples, see the examples folder**

### Red Rotating Cube
```c
sfr_reset();                   // prepare to draw
sfr_rotate_y(time);            // rotate about y
sfr_rotate_x(time * 0.5f);     // rotate about x
sfr_scale(1.f, 3.f, 0.5f);     // scale by x y z
sfr_translate(0.f, 0.f, 2.5f); // move to x y z
sfr_cube(0xFF0000);            // draw pure red cube
```

### OBJ Loading
```c
Mesh* mesh = sfr_load_mesh("model.obj"); // allocate and load obj file
...
mesh->rot.y = time;       // rotate about y, radians not degrees
mesh->pos.y = sinf(time); // move up and down
sfr_reset();              // reset model matrix
sfr_mesh(mesh);           // draw mesh using its properties
...
sfr_release_mesh(&mesh); // free mesh's memory
``` 

## TODO / Upcoming
- More advanced lighting
- Per vertex colors
- Support for more color formats
- MSAA, maybe
- Stencil buffer, maybe
- Config flag for multithreading, very maybe

# License
This project is under the MIT license, do with that what you will
