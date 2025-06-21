/* SOFREN EXAMPLES, font-starter-sdl2.c

small example showing usage of a SfrFont struct and .srft file

.srft file format:
    [0..4] header:
        - [0..4] = ['s']['r']['f']['t'] = 1936877172
    [5...] data:
        - {
            vert count, u16, 2 bytes, max 
            vert data,  f32, 4 bytes, [x0][y0][x1][y1][x2][...]
        }

*/

#define SFR_IMPL
#define SFR_NO_ALPHA
#include "../sofren.c"

#include <stdio.h>
#include <string.h>

#include <SDL2/SDL.h>

#define RES_SCALE 1.0

i32 main() {
    { // initialize sofren
        const i32 w = 1280 * RES_SCALE, h = 720 * RES_SCALE;
        sfr_init(malloc(sizeof(SfrBuffers)), w, h, 50.f);
        sfr_set_lighting(1, sfr_vec_normf(-0.5f, 0.1f, -1.f), 0.4f);
    }

    SDL_Window* window; SDL_Renderer* renderer; SDL_Texture* texture;
    { // initialize SDL
        SDL_Init(SDL_INIT_VIDEO);
        window = SDL_CreateWindow("Window", SDL_WINDOWPOS_CENTERED, 
            SDL_WINDOWPOS_CENTERED, sfrWidth / RES_SCALE, sfrHeight / RES_SCALE, SDL_WINDOW_SHOWN);
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
        texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, 
            SDL_TEXTUREACCESS_STREAMING, sfrWidth, sfrHeight);
    }

    // load font
    SfrFont* font = sfr_load_font("examples/res/basic-font.srft");
    if (!font) {
        SFR_ERR_RET(1, "Failed to load font\n");
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

        // clear previous frame
        sfr_clear(0x000000);

        { // render scene
            // first line
            sfr_reset();
            sfr_scale(0.125f, 0.125f, 0.125f);
            sfr_translate(-7.f, 1.5f, 10.f);

            const char* text1 = "text rendering";
            for (i32 i = 0; i < strlen(text1); i += 1) {
                sfr_glyph(font, text1[i], 0xBAFA9A);
                sfr_translate(sinf(time) * 0.1f + 0.9f, 0.f, 0.f);
            }

            // second line
            sfr_reset();
            sfr_scale(0.08f, 0.08f, 0.08f);
            sfr_translate(-6.f, -0.5f, 10.f);

            const char* text2 = "this font sucks";
            for (i32 i = 0; i < strlen(text2); i += 1) {
                sfr_glyph(font, text2[i], 0x808080);
                sfr_translate(0.6f, 0.f, 0.f);
            }

            // third line
            sfr_reset();
            sfr_scale(0.04f, 0.04f, 0.04f);
            sfr_translate(-6.f, -1.5f, 10.f);

            const char* text3 = "abcdefghijklmnopqrstuvwxyz0123456789";
            for (i32 i = 0; i < strlen(text3); i += 1) {
                sfr_glyph(font, text3[i], 0x606060);
                sfr_translate(0.3f, 0.f, 0.f);
            }
        }

        { // update SDL window
            SDL_UpdateTexture(texture, NULL, sfrBuffers->pixel, sfrWidth * 4);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, NULL, NULL);
            SDL_RenderPresent(renderer);
        }
    }

    { // cleanup
        free(sfrBuffers);
        sfr_release_font(&font);
        
        SDL_DestroyTexture(texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
    
    return 0;
}
