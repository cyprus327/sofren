/* SOFREN EXAMPLES, tex-starter-sdl2.c

demonstrates:
    loading textures and meshes,
    drawing textured meshes and cubes,
    ui with an FPS counter

*/

#define SFR_IMPL
#include "../sofren.c"

#include <SDL2/SDL.h>

i32 main() {    
    Mesh* mesh; Texture* meshTex; Texture* cubeTex; Font* font;
    SDL_Window* window; SDL_Renderer* renderer; SDL_Texture* sdlTex;
    { // init
        // load mesh and texture
        mesh = sfr_load_mesh("examples/res/hawk.obj");
        if (!mesh) {
            return 1;
        }

        meshTex = sfr_load_texture("examples/res/hawk.bmp");
        if (!meshTex) {
            return 1;
        }

        cubeTex = sfr_load_texture("examples/res/test.bmp");
        if (!cubeTex) {
            return 1;
        }

        font = sfr_load_font("examples/res/basic-font.srft");
        if (!font) {
            return 1;
        }

        const i32 w = 1280, h = 720;

        // initialize sofren
        u32* pixelBuf = malloc(sizeof(u32) * w * h);
        f32* depthBuf = malloc(sizeof(f32) * w * h);
        sfr_init(pixelBuf, depthBuf, w, h, 50.0f);
        sfr_set_lighting(1, sfr_vec_normf(0.6f, 1.f, -1.f), 0.4f);
        
        // initialize SDL
        SDL_Init(SDL_INIT_VIDEO);
        window = SDL_CreateWindow("Window", SDL_WINDOWPOS_CENTERED, 
                                            SDL_WINDOWPOS_CENTERED, w, h, 0);
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        sdlTex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
                                              SDL_TEXTUREACCESS_STREAMING, w, h);
    }

    f32 camX = 0.f;
    f32 moveMult = 0;
    char asdfasdf = 'a';

    for (i32 shouldQuit = 0; !shouldQuit;) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (SDL_QUIT == e.type) {
                shouldQuit = 1;
            } else if (SDL_KEYDOWN == e.type || SDL_KEYUP == e.type) {
                switch (e.key.keysym.scancode) {
                    case SDL_SCANCODE_ESCAPE: shouldQuit = 1; break;
                    case SDL_SCANCODE_D: moveMult =  2.2f * e.key.state; break;
                    case SDL_SCANCODE_A: moveMult = -2.2f * e.key.state; break;
                    default: moveMult = 0.f; break;
                }
            }
        }

        // get frame time
        static f32 prevTime = 0.f;
        const f32 time = SDL_GetTicks() / 1000.f;
        const f32 frameTime = time - prevTime;
        prevTime = time;

        // move and update camera
        camX += moveMult * frameTime;
        sfr_set_camera(camX, 0.f, 0.f, 0.f, 0.f, 0.f);

        // clear buffers
        sfr_clear();

        { // draw scene
            // textured bunny
            sfr_reset();
            sfr_rotate_x(SFR_PI * 1.5f);
            sfr_rotate_y(time * 1.5f);
            sfr_scale(0.125f, 0.125f, 0.125f);
            sfr_translate(-2.5f, -1.7f, 8.f);
            sfr_mesh_tex(mesh, meshTex);

            // flat shaded bunny
            sfr_reset();
            sfr_rotate_x(SFR_PI * 1.5f);
            sfr_rotate_y(time * 1.5f);
            sfr_scale(0.125f, 0.125f, 0.125f);
            sfr_translate(2.5f, -1.7f, 8.f);
            sfr_mesh(mesh, 0xFFFFAA);

            // textured cube
            sfr_reset();
            sfr_rotate_y(sinf(time) * 0.2f + 0.5f);
            sfr_rotate_x(cosf(time) * 0.125f + 2.5f);
            sfr_scale(10.f, 10.f, 10.f);
            sfr_translate(0.f, 0.f, 20.f);
            sfr_cube_tex(cubeTex);
        }

        // reset depth buffer before drawing ui
        memset(sfrDepthBuf, 0x7F, sizeof(f32) * sfrWidth * sfrHeight);

        { // draw FPS counter
            static f32 fpsTimer = 0.f;
            static i32 fpsCounter = 0;
            fpsTimer += frameTime;
            fpsCounter += 1;

            static char buf[8] = "999";
            static i32 len = 3;

            if (fpsTimer >= 1.f) {
                len = 0;
                snprintf(buf, sizeof(buf), "%d", fpsCounter);
                while ('\0' != buf[len]) {
                    len += 1;
                }

                fpsTimer -= 1.f;
                fpsCounter = 0;
            }
            
            sfr_set_camera(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
            sfr_reset();
            sfr_scale(0.01f, 0.01f, 0.01f);
            sfr_translate(-0.87f, 0.43f, 1.f);
            for (i32 i = 0; i < len; i += 1) {
                sfr_translate(0.07f, 0.f, 0.f);
                sfr_glyph(font, buf[i], 0x77FF77);    
            }
        }

        // update SDL window
        SDL_UpdateTexture(sdlTex, NULL, sfrPixelBuf, sfrWidth * 4);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdlTex, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

    { // cleanup
        sfr_release_font(&font);
        sfr_release_texture(&cubeTex);
        sfr_release_texture(&meshTex);
        sfr_release_mesh(&mesh);
        free(sfrPixelBuf);
        free(sfrDepthBuf);
        
        SDL_DestroyTexture(sdlTex);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
    
    return 0;
}