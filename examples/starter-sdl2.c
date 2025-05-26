/* SOFREN EXAMPLES, starter-sdl2.c

demonstrating:
    resolution scaling,
    handling a window resizing,
    basic camera usage,
    how to draw primitives

*/

#define SFR_IMPL
#define SFR_NO_ALPHA
#include "../sofren.c"

#include <SDL2/SDL.h>

// rasterizing is expensive, easiest fix is to just use lower resolution
#define RES_SCALE 1.0

i32 main() {
    { // initialize sofren
        const i32 w = 1280 * RES_SCALE, h = 720 * RES_SCALE;
        u32* pixelBuf = (u32*)malloc(sizeof(u32) * w * h);
        f32* depthBuf = (f32*)malloc(sizeof(f32) * w * h);
        sfr_init(pixelBuf, depthBuf, w, h, 50.f);

        // super cheap function, can be set before rendering every individual
        // triangle if you want, but it isn't reset so may as well just set it once now
        sfr_set_lighting(1, sfr_vec_normf(-0.5f, 0.1f, -1.f), 0.4f);
    }

    SDL_Window* window; SDL_Renderer* renderer; SDL_Texture* texture;
    { // initialize SDL
        SDL_Init(SDL_INIT_VIDEO);
        window = SDL_CreateWindow("Window", SDL_WINDOWPOS_CENTERED, 
            SDL_WINDOWPOS_CENTERED, sfrWidth / RES_SCALE, sfrHeight / RES_SCALE, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
        texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, 
            SDL_TEXTUREACCESS_STREAMING, sfrWidth, sfrHeight);
    }

    // main loop
    for (i32 shouldQuit = 0; !shouldQuit;) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (SDL_QUIT == e.type || (SDL_KEYDOWN == e.type && SDL_SCANCODE_ESCAPE == e.key.keysym.scancode)) {
                shouldQuit = 1;
            }
        }

        // elapsed and frame time calculations
        static f32 time = 0.f;
        f32 currTime = SDL_GetTicks() / 1000.f, delta = currTime - time;
        time = currTime;

        { // handle resizing
            i32 width, height;
            SDL_GetWindowSize(window, &width, &height);
            
            width *= RES_SCALE;
            height *= RES_SCALE;

            if (width != sfrWidth || height != sfrHeight) {
                // update sfr internals
                sfrWidth = width;
                sfrHeight = height;
                sfrPixelBuf = (u32*)realloc(sfrPixelBuf, sizeof(u32) * width * height);
                sfrDepthBuf = (f32*)realloc(sfrDepthBuf, sizeof(f32) * width * height);
                
                // update projection matrix
                sfr_set_fov(sfrCamFov);

                // update render texture
                SDL_DestroyTexture(texture);
                texture = SDL_CreateTexture(renderer,
                    SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, width, height);
            }
        }

        // clear previous frame
        sfr_clear(0x000000);

        // make camera look around at the scene
        sfr_set_camera(0.f, 0.f, -1.f, sinf(time) * 0.5f, 0.f, cosf(time) * 0.5f);
        
        { // render scene
            sfr_rand_seed(5);
            for (i32 i = 0; i < 500; i += 1) {
                f32 rt = sfr_rand_flt(0.f, 500.f) + time;
                sfr_reset();
                sfr_scale(
                    (1.4f + sinf(rt * 0.6f)) / 2.f,
                    (1.2f + sinf(rt * 0.8f)) / 2.f,
                    (1.8f + sinf(rt * 0.3f)) / 2.f);
                sfr_rotate_x(rt * SFR_PI * 0.2f);
                sfr_rotate_y(rt * SFR_PI * 0.3f);
                sfr_rotate_z(rt * SFR_PI * 0.6f);
                sfr_translate(sfr_rand_flt(-6.f, 6.f), sfr_rand_flt(-3, 3) + sinf(rt), sfr_rand_flt(4.f, 12.f));
                sfr_cube(sfr_rand_int(0, 0xFFFFFF));
            }
        }

        { // update SDL window
            SDL_UpdateTexture(texture, NULL, sfrPixelBuf, sfrWidth * 4);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, NULL, NULL);
            SDL_RenderPresent(renderer);
        }
    }

    { // cleanup
        free(sfrPixelBuf);
        free(sfrDepthBuf);
        
        SDL_DestroyTexture(texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
    
    return 0;
}
