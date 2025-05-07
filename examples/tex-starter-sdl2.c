/* SOFREN EXAMPLES, starter-sdl2.c

a small and digestable example demonstrating:
    loading textures and meshes,
    drawing textured meshes and cubes

*/

#define SFR_IMPL
#include "../sofren.c"

#include <SDL2/SDL.h>

i32 main() {    
    Mesh* mesh; Texture* meshTex;
    SDL_Window* window; SDL_Renderer* renderer; SDL_Texture* sdlTex;
    { // init
        // load mesh and texture
        mesh = sfr_load_mesh("examples/res/bunny.obj");
        if (!mesh) {
            return 1;
        }

        mesh->col = 0xFFFFAA;
        mesh->pos.y = -0.2f;
        mesh->pos.z = 1.6f;
        mesh->scale = (Vec){3.f, 3.f, 3.f};

        meshTex = sfr_load_texture("examples/res/test.bmp");
        if (!meshTex) {
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
            mesh->rot.y = time * 1.5f;
            
            // textured bunny
            mesh->pos.x = -0.6f;
            sfr_reset();
            sfr_mesh_tex(mesh, meshTex);

            // lit, flat shaded, bunny
            mesh->pos.x = 0.6f;
            sfr_reset();
            sfr_mesh(mesh);

            // textured cube
            sfr_reset();
            sfr_rotate_x(time * 0.5f);
            sfr_rotate_y(time * 1.1f);
            sfr_translate(0.f, 0.f, 8.f);
            sfr_cube_tex(meshTex);
        }

        // update SDL window
        SDL_UpdateTexture(sdlTex, NULL, sfrPixelBuf, sfrWidth * 4);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdlTex, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

    { // cleanup
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