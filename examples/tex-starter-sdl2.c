/* SOFREN EXAMPLES, tex-starter-sdl2.c

demonstrates:
    transparent drawing (alpha in use),
    loading textures and meshes,
    drawing textured meshes and cubes,
    ui with an FPS counter

*/

#define SFR_IMPL
// #define SFR_NO_ALPHA // ~15 more FPS when defined but no transparency
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
        #ifdef SFR_NO_ALPHA // accumBuf is only needed when alpha blending is enabled
            sfr_init(pixelBuf, depthBuf, w, h, 50.f);
        #else
            SfrAccumCol* accumBuf = malloc(sizeof(SfrAccumCol) * w * h);
            sfr_init(accumBuf, pixelBuf, depthBuf, w, h, 50.0f);
        #endif
        
        // initialize SDL
        SDL_Init(SDL_INIT_VIDEO);
        window = SDL_CreateWindow("Window", SDL_WINDOWPOS_CENTERED, 
                                            SDL_WINDOWPOS_CENTERED, w, h, 0);
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        sdlTex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, w, h);
    }

    f32 camX = 0.f;
    f32 moveMult = 0;

    for (u8 shouldQuit = 0; !shouldQuit;) {
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
        sfr_clear(0xFF000000);

        { // draw scene
            static f32 colTimer = 0.f;
            colTimer -= frameTime;
            
            u32 col;
            if (colTimer <= -0.5f) {
                colTimer += 3.5f;
            } else if (colTimer <= 0.f) {
                col = 0x00FFFFFF;
            } else if (colTimer <= 0.5f) {
                col = 0x00FF7777;
            } else if (colTimer <= 1.f) {
                col = 0x0077FF77;
            } else if (colTimer <= 1.5f) {
                col = 0x007777FF;
            } else if (colTimer <= 2.f) {
                col = 0x00FF0000;
            } else if (colTimer <= 2.5f) {
                col = 0x0000FF00;
            } else if (colTimer <= 3.f) {
                col = 0x000000FF;
            }
            col |= (u32)(fminf(1.f, sinf(time) + 1.f) * 255.f) << 24;

            // enable lighting for hawks
            sfr_set_lighting(1, sfr_vec_normf(0.6f, 1.f, -1.f), 0.4f);

            // transparent hawk
            sfr_reset();
            sfr_rotate_x(SFR_PI * 1.5f);
            sfr_rotate_y(time * 1.5f);
            sfr_scale(0.125f, 0.125f, 0.125f);
            sfr_translate(-2.5f, -2.f, 8.f);
            sfr_mesh_tex(mesh, col, meshTex);

            // solid untextured hawk
            sfr_reset();
            sfr_rotate_x(SFR_PI * 1.5f);
            sfr_rotate_y(time * 1.5f);
            sfr_scale(0.125f, 0.125f, 0.125f);
            sfr_translate(2.5f, -2.f, 8.f);
            sfr_mesh(mesh, col);

            // disable lighting on cube
            sfr_set_lighting(0, SFR_VEC0, 0.f);

            // textured cube
            sfr_reset();
            sfr_rotate_y(sinf(time) * 0.2f + 0.5f);
            sfr_rotate_x(cosf(time) * 0.125f + 2.5f);
            sfr_scale(10.f, 10.f, 10.f);
            sfr_translate(0.f, -2.f, 20.f);
            sfr_cube_tex(0xFFFFFFFF, cubeTex);
        }

        // reset depth buffer before drawing ui
        memset(sfrDepthBuf, 0x7F, sizeof(f32) * sfrWidth * sfrHeight);

        { // draw ui text
            static f32 fpsTimer = 0.f;
            static i32 fpsCounter = 0;
            fpsTimer += frameTime;
            fpsCounter += 1;

            static u32 trisCounter = 0;

            static char fpsBuf[8] = "999";
            static i32 fpsLen = 3;

            static char trisBuf[24] = "";
            static i32 trisLen = 0;

            if (fpsTimer >= 1.f) {
                fpsLen = 0;
                snprintf(fpsBuf, sizeof(fpsBuf), "%d", fpsCounter);
                while ('\0' != fpsBuf[fpsLen++]) { }

                trisLen = 0;
                snprintf(trisBuf, sizeof(trisBuf), "%d tris", trisCounter / fpsCounter);
                while ('\0' != trisBuf[trisLen++]) { }

                fpsTimer -= 1.f;
                fpsCounter = 0;
                trisCounter = 0;
            }
            
            sfr_set_camera(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
            
            sfr_reset();
            sfr_scale(0.01f, 0.01f, 0.01f);
            sfr_translate(-0.87f, 0.43f, 1.f);
            for (i32 i = 0; i < fpsLen; i += 1) {
                sfr_translate(0.07f, 0.f, 0.f);
                sfr_glyph(font, fpsBuf[i], 0xFF77FF77);    
            }

            sfr_reset();
            sfr_scale(0.01f, 0.01f, 0.01f);
            sfr_translate(-0.6f, 0.43f, 1.f);
            for (i32 i = 0; i < trisLen; i += 1) {
                sfr_translate(0.07f, 0.f, 0.f);
                sfr_glyph(font, trisBuf[i], 0xFFCA9999);
            }

            // count tris from drawing text as well
            trisCounter += sfrRasterCount;
        }

        #ifndef SFR_NO_ALPHA
            // draw transparent parts of the scene
            sfr_present_alpha();
        #endif

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
        #ifndef SFR_NO_ALPHA
            free(sfrAccumBuf);
        #endif
        
        SDL_DestroyTexture(sdlTex);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
    
    return 0;
}