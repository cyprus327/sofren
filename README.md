# sofren - Single File Software Renderer

**A minimalistic, efficient, single file, cross platform software renderer**

For a fairly featureful example project using SDL2 for rendering, see `examples/demo1.c`

## Features
- As few a 0 other headers included
- Single file - just `#include "sofren.c"`
- Primitive drawing (triangles, cubes)
- OBJ mesh loading (requires `stdio.h`)
- Flat shading with directional lighting
- Matrix transformations (model/view/projection)
- Customizable math implementations (system or bundled)
- ARGB8888 color format support
- Backface culling & depth buffering

## Pre Processing Configuration 
```c
SFR_IMPL       // indicate declaration of functions
SFR_NO_STD     // disable 'stdio' and 'stdlib.h'
SFR_NO_MATH    // disable 'math.h' and use bundled math
SFR_NO_STRING  // disable 'string.h'
SFR_NO_CULLING // disable backface culling
SFR_USE_DOUBLE // use double precision floats
SFR_USE_INLINE // force inline functions

// only applicable when SFR_NO_MATH is defined, their values
// dictate the accuracy of the bundled math functions
SFR_SQRT_ACCURACY // defaults to 20 if not defined
SFR_TRIG_ACCURACY // defaults to 10 if not defined
``` 

## Setup
```c
#define SFR_IMPL
// other configuration flags, e.g.
// #define SFR_USE_DOUBLE
#include "sofren.c"
```

## Examples
**For more in depth examples, see the examples folder**
### Rotating Shaded Cube
```c
sfr_set_lighting(1, sfr_vec_normf(1.0, 1.0, 0.0), 0.2);
sfr_reset();
sfr_rotate_y(time * 0.8);
sfr_rotate_x(time * 0.5);
sfr_scale(1.0, 3.0, 0.5);
sfr_translate(0, 0, 2.5);
sfr_cube(0xFF0000); // red cube
```

### OBJ Loading  
```c
sfrmesh_t* mesh = sfr_load_mesh("model.obj"); // allocate and load obj file
...
mesh->rot.y = time; // rotate about y
mesh->pos.y = 0.3 + sinf(time * 2.0) * 0.2; // move up and down slowly
sfr_reset();    // reset sofren matrices
sfr_mesh(mesh); // draw mesh using its properties
...
sfr_release_mesh(&mesh); // free mesh's memory
``` 

## TODO / Upcoming
- Texture mapping
- Per vertex colors
- Support for more color formats
- More advanced lighting
- MSAA, maybe
- Stencil buffer, maybe
- Config flag for multithreading, very maybe

# License
This project is under the MIT license, do with that what you will
