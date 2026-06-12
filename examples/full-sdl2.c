#define SFR_IMPL
#define SFR_USE_CGLTF
#define SFR_USE_STB_IMAGE
#include "../sofren.c"

#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>

#include <string.h>

// rasterizing on a big window is expensize and the easiest
// solution is to just have less pixels (RES_SCALE < 1.0)
#define RES_SCALE 1.f

#ifdef _WIN32
    #include <malloc.h>
    // GCC/MinGW with -O3 optimizations can aggressively upgrade unaligned AVX intrinsics 
    // (e.g. _mm256_maskstore_epi32) into strict aligned vector instructions (e.g. vmovdqa) 
    // if the compiler's loop analysis falsely assumes the pointer is aligned.
    // While Linux natively aligns the heap to 32 bytes for AVX, the Windows x64 ABI 
    // only guarantees 16 byte heap alignment by default which causes an immediate SIGTRAP/SIGSEGV 
    // when the 32 byte instruction fires, so these wrappers force strict 32 byte alignment on Windows.
    static void* sfr_malloc(u64 size) { return _aligned_malloc(size, 32); }
    static void* sfr_realloc(void* ptr, u64 size) { return _aligned_realloc(ptr, size, 32); }
    static void sfr_free(void* ptr) { _aligned_free(ptr); }
#else
    #include <stdlib.h>
    // Linux/macOS natively handle the alignment well enough, so just standard allocators
    static void* sfr_malloc(u64 size) { return malloc(size); }
    static void* sfr_realloc(void* ptr, u64 size) { return realloc(ptr, size); }
    static void sfr_free(void* ptr) { free(ptr); }
#endif

// draw the scene and ui
static void draw_scene(f32 time, f32 frameTime);
static void draw_ui(f32 frameTime);

// resize if needed
static void handle_resize(SDL_Window* window, SDL_Renderer* renderer, SDL_Texture* texture);

// move camera based on inputs
static void handle_inputs(f32 frameTime);

// load scenes and update sceneCount
static void parse_argv(i32 argc, char** argv);

// input flags
static struct {
    u8 current[SDL_NUM_SCANCODES];  // is down
    u8 previous[SDL_NUM_SCANCODES]; // was down last frame
    u8 pressed[SDL_NUM_SCANCODES];  // just pressed this frame
    u8 released[SDL_NUM_SCANCODES]; // just released this frame
} input;
static SDL_GameController* gamepad = NULL;

static SfrScene* scenes[3];
static i32 sceneCount = 0;
static i32 sceneCurrInd = 0;

static SfrMaterial* cubeMat;
static SfrFont* font;

static SfrLight* sunLight;
static SfrLight* pointLight;

i32 main(i32 argc, char** argv) {
    if (argc <= 1) {
        printf("Usage: %s [map.cmp] [model.glb] [model2.gltf]\n\t(between 1 and 3 args)\n",
            argv[0]);
        return 0;
    }
    argc = (argc - 1 > 3) ? 3 : argc - 1;

    // initial window dimensions
    const i32 startWidth = 1280 * RES_SCALE, startHeight = 720 * RES_SCALE;

    // initialize sofren
    sfr_init(startWidth, startHeight, 80.f, sfr_malloc, sfr_free, sfr_realloc);

    // load assets
    cubeMat = sfr_load_material("examples/res/test.bmp", NULL);
    font = sfr_load_font("examples/res/basic-font.srft");
    if (!cubeMat || !font) {
        return 1;
    }

    // load scenes
    parse_argv(argc, argv);

    // add lights
    sunLight = sfr_light_add_directional(-0.64f, -1.12f, -0.26f, 0.4f, 0.6f, 1.f, 0.95f, 0.9f);
    pointLight = sfr_light_add_point(-3.2f, 3.f, -6.4f, 0.f, 2.f, 18.f, 0.9f, 0.3f, 0.1f);
    // if (2 == argc && scene) {
    //     sfr_scene_apply_lights(scene);
    // }
    
    // load shadowmap render target
    SfrRenderTarget* shadowmap = sfr_create_target(1024, 1024, 0);
    // shadowmap->shadowBias = 0.0004f; // tiny bias for high resolution shadowmap
    shadowmap->shadowStrength = 0.4f;

    // initialize SDL
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER);
    printf("Video driver: %s\n", SDL_GetCurrentVideoDriver());

    // find and open the first available gamepad
    for (int i = 0; i < SDL_NumJoysticks(); i += 1) {
        if (SDL_IsGameController(i)) {
            gamepad = SDL_GameControllerOpen(i);
            if (gamepad) {
                printf("Controller connected: %s\n", SDL_GameControllerName(gamepad));
                break;
            }
        }
    }
    
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "best");
    SDL_Window* window = SDL_CreateWindow("Window",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        0, 0, SDL_WINDOW_FULLSCREEN_DESKTOP);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture* sdlTex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, startWidth, startHeight);

    // render target to avoid implicit memcpy in SDL_UpdateTexture
    // SfrRenderTarget* sdlTarget = sfr_create_target(startWidth, startHeight, 1);
    // sfr_set_render_target(sdlTarget);

    // main loop
    for (u8 shouldQuit = 0; !shouldQuit;) {
        // get frame time
        static f32 prevTime = 0.f;
        const f32 time = SDL_GetTicks() / 1000.f;
        const f32 frameTime = time - prevTime;
        prevTime = time;

        // static i32 totalFrames = 0;
        // totalFrames += 1;
        // if (time >= 20.f) {
        //     printf("TOTAL FRAMES: %d\n", totalFrames);
        //     break;
        // }

        // handle sdl events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT: {
                    shouldQuit = 1;
                } break;
                case SDL_KEYDOWN: {
                    if (!event.key.repeat) {
                        input.current[event.key.keysym.scancode] = 1;
                    }
                    if (SDL_SCANCODE_ESCAPE == event.key.keysym.scancode) {
                        shouldQuit = 1;
                    }
                } break;
                case SDL_KEYUP: {
                    input.current[event.key.keysym.scancode] = 0;
                } break;
                default: break;
            }
        }
        for (int i = 0; i < SDL_NUM_SCANCODES; i += 1) {
            input.pressed[i]  =  input.current[i] && !input.previous[i];
            input.released[i] = !input.current[i] &&  input.previous[i];
        }

        // resize if needed
        handle_resize(window, renderer, sdlTex);

        static SfrRenderTarget* savedShadowmapMap = NULL;
        static sfrmat savedLightVP = {0};

        if (input.pressed[SDL_SCANCODE_Y]) {
            savedShadowmapMap = NULL;
        }

        static u8 shadowPass = 0;
        if (input.pressed[SDL_SCANCODE_R]) {
            shadowPass = !shadowPass;
            printf("Shadows: %s\n", shadowPass ? "Realtime" : savedShadowmapMap ? "Static" : "None");
        }
        if (shadowPass) { // rendering pass 1, shadowmap
            const sfrvec sunDir = { sunLight->x, sunLight->y, sunLight->z };
            const sfrvec center = { -5.28f, -1.83f, -1.f };            
            const f32 orthoSize = 12.f;
            const f32 distance = 40.f;

            sfr_shadow_pass_begin(shadowmap, sunDir, center, orthoSize, distance, 200.f);
                draw_scene(time, frameTime);
                sfr_present();
            sfr_shadow_pass_end();

            // NOTE: pressing R then T then R will go to static lightmap,
            //  this is just a test and sfrState should not be accessed normally
            if (input.pressed[SDL_SCANCODE_T]) {
                savedShadowmapMap = sfrState.activeShadowmap;
                savedLightVP = sfrState.matLightVP;
                printf("Saved shadowmap\n");
            }
        }

        void* savedSfrPixelBuf = sfrPixelBuf;

        { // rendering pass 2, main scene
            void* lockedPixels = NULL;
            i32 pitch = 0;
            SDL_LockTexture(sdlTex, NULL, &lockedPixels, &pitch);
            sfrPixelBuf = lockedPixels;

            if (savedShadowmapMap && !shadowPass) {
                sfr_set_shadowmap(savedShadowmapMap, savedLightVP);
            }

            handle_inputs(frameTime); // restores normal FPS camera

            sfr_clear(0xFF041411);
            draw_scene(time, frameTime);
            sfr_present();
        }

        { // rendering pass 3, ui
            sfr_set_shadowmap(NULL, (sfrmat){0}); // ui shouldn't be in shadow
            draw_ui(frameTime);
            sfr_present();
        }

        static i32 renderMode = SFR_RENDERMODE_SHADED;
        if (input.pressed[SDL_SCANCODE_LALT]) {
            renderMode = (renderMode + 1) % (SFR_RENDERMODE_COUNT - 1);
            sfr_set_rendermode(renderMode);
        }

        // update SDL window
        SDL_UnlockTexture(sdlTex);
        sfrPixelBuf = savedSfrPixelBuf;
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdlTex, NULL, NULL);
        SDL_RenderPresent(renderer);

        sfr_memcpy(input.previous, input.current, SDL_NUM_SCANCODES);
    }

    // cleanup and close SDL window
    sfr_release_font(&font);
    sfr_release_material(&cubeMat);
    sfr_release_scene(&scenes[0], 0);
    sfr_release_scene(&scenes[1], 0);
    sfr_release_target(&shadowmap);
    sfr_release();

    if (gamepad) SDL_GameControllerClose(gamepad);
    SDL_DestroyTexture(sdlTex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

static void draw_scene(f32 time, f32 frameTime) {
    { // draw scene
        // enable/disable lighting
        static u8 lighting = 1;
        if (input.pressed[SDL_SCANCODE_Z]) lighting = !lighting;
        sfr_set_lighting(lighting);

        // switch scenes with tab
        if (input.pressed[SDL_SCANCODE_TAB]) {
            sceneCurrInd = (sceneCurrInd + 1) % sceneCount;
        }
        sfr_reset();
        sfr_scene_draw(scenes[sceneCurrInd]);

        // draw simple scene
        static u8 showBasicScene = 0;
        if (input.pressed[SDL_SCANCODE_X]) {
            showBasicScene = !showBasicScene;
        }
        if (showBasicScene) {
            // textured cube
            sfr_reset();
            sfr_scale(10.f, 10.f, 10.f);
            sfr_rotate_y(sinf(time) * 0.2f + 0.5f);
            sfr_rotate_x(cosf(time) * 0.125f + 2.5f);
            sfr_translate(0.f, -2.f, 20.f);
            sfr_cube(0xFFFFFFFF, cubeMat);

            // room walls
            sfr_reset();
            sfr_scale(40.f, 20.f, 25.f);
            sfr_translate(0.f, -1.f, 20.f);
            sfr_cube_inv(0xFFAABBCC, NULL);
        }
    }

    // simple raycasting example, will only interact with the scene
    if (input.current[SDL_SCANCODE_F]) {
        const sfrvec f = sfr_vec_norm(sfr_vec_sub(sfrCamTarget, sfrCamPos));
        const SfrRayHit hit = sfr_scene_raycast(scenes[0], sfrCamPos.x, sfrCamPos.y, sfrCamPos.z, f.x, f.y, f.z);

        if (hit.hit) {
            #ifdef SFR_MULTITHREADED
                sfr_present();
            #endif
            sfr_clear_depth();

            const SfrSceneObject* const obj = &scenes[0]->objects[hit.objectInd];

            // NOTE this is a hack just for the visualization,
            // normally sfrState shouldn't be modified
            sfrMatModel = obj->_model;
            sfrState.normalMatDirty = 1;

            const f32* t = &obj->mesh->tris[hit.triangleInd * 9];

            sfr_triangle(
                t[0], t[1], t[2],
                t[3], t[4], t[5],
                t[6], t[7], t[8],
                0xFFFFFF00
            );

            sfr_reset();
        }
    }
}

static void draw_ui(f32 frameTime) {
    // reset depth buffer before drawing ui
    sfr_clear_depth();
    sfr_set_lighting(0);

    static f32 fpsTimer = 0.f;
    static i32 fpsCounter = 0;
    fpsTimer += frameTime;
    fpsCounter += 1;

    static u32 trisCounter = 0;

    static char fpsBuf[8] = "999";
    static i32 fpsLen = 3;

    static char trisBuf[24] = "";
    static i32 trisLen = 0;

    const f32 interval = 0.334f;
    if (fpsTimer >= interval) {
        fpsLen = 0;
        snprintf(fpsBuf, sizeof(fpsBuf), "%d", (i32)(fpsCounter / fpsTimer + 0.5f));
        while ('\0' != fpsBuf[fpsLen++]) { }

        trisLen = 0;
        snprintf(trisBuf, sizeof(trisBuf), "%d tris", trisCounter / fpsCounter);
        while ('\0' != trisBuf[trisLen++]) { }

        fpsTimer -= interval;
        fpsCounter = 0;
        trisCounter = 0;
    }

    sfr_set_camera(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

    sfr_reset();
    sfr_scale(0.01f, 0.01f, 0.01f);
    sfr_translate(-1.3f, 0.7f, 1.f);
    for (i32 i = 0; i < fpsLen; i += 1) {
        sfr_translate(0.07f, 0.f, 0.f);
        sfr_glyph(font, fpsBuf[i], 0xFF77FF77);
    }

    sfr_reset();
    sfr_scale(0.01f, 0.01f, 0.01f);
    sfr_translate(-1.f, 0.7f, 1.f);
    for (i32 i = 0; i < trisLen; i += 1) {
        sfr_translate(0.07f, 0.f, 0.f);
        sfr_glyph(font, trisBuf[i], 0xFFCA9999);
    }

    // count tris from drawing text as well
    #ifdef SFR_MULTITHREADED
        trisCounter += sfr_atomic_get(&sfrRasterCount);
    #else
        trisCounter += sfrRasterCount;
    #endif
}

static void handle_resize(SDL_Window* window, SDL_Renderer* renderer, SDL_Texture* texture) {
    i32 width, height;
    SDL_GetWindowSize(window, &width, &height);

    width *= RES_SCALE;
    height *= RES_SCALE;

    // if the window hasn't resized don't resize it
    if (width == sfrWidth && height == sfrHeight) {
        return;
    }

    // update sfr internals
    sfr_resize(width, height);

    // update projection matrix
    sfr_set_fov(sfrCamFov);

    // update render texture
    SDL_DestroyTexture(texture);
    texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, width, height);
}

static void handle_inputs(f32 frameTime) {
    static f32 camX = 12.54f, camY = 3.38f, camZ = -14.57f;
    static f32 camYaw = 0.78f, camPitch = 0.16f;
    static f32 camForwardSpeed = 0.f;
    static f32 camStrafeSpeed = 0.f;
    static f32 camUpSpeed = 0.f;

    const f32 moveMult =
        (input.current[SDL_SCANCODE_LSHIFT] ||
        (gamepad && SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_B))) ? 3.f : 0.5f;
    const f32 turnMult = 2.75f;
    const f32 accel = 50.f;
    const f32 decel = 7.f;

    f32 joyMoveX = 0.f, joyMoveY = 0.f;
    f32 joyLookX = 0.f, joyLookY = 0.f;
    f32 joyUp = 0.f, joyDown = 0.f;

    if (gamepad) {
        const i16 lx = SDL_GameControllerGetAxis(gamepad, SDL_CONTROLLER_AXIS_LEFTX);
        const i16 ly = SDL_GameControllerGetAxis(gamepad, SDL_CONTROLLER_AXIS_LEFTY);
        const i16 rx = SDL_GameControllerGetAxis(gamepad, SDL_CONTROLLER_AXIS_RIGHTX);
        const i16 ry = SDL_GameControllerGetAxis(gamepad, SDL_CONTROLLER_AXIS_RIGHTY);

        const i16 deadzone = 5000;
        if (abs(lx) > deadzone) joyMoveX = (f32)lx / 32767.f;
        if (abs(ly) > deadzone) joyMoveY = (f32)ly / 32767.f;
        if (abs(rx) > deadzone) joyLookX = (f32)rx / 32767.f;
        if (abs(ry) > deadzone) joyLookY = (f32)ry / 32767.f;

        joyUp = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
        joyDown = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
    }

    if (input.current[SDL_SCANCODE_W]) camForwardSpeed +=  accel * moveMult * frameTime;
    if (input.current[SDL_SCANCODE_S]) camForwardSpeed += -accel * moveMult * frameTime;
    if (0.f != joyMoveY) camForwardSpeed += -joyMoveY * accel * moveMult * frameTime;

    if (input.current[SDL_SCANCODE_A]) camStrafeSpeed +=  accel * moveMult * frameTime;
    if (input.current[SDL_SCANCODE_D]) camStrafeSpeed += -accel * moveMult * frameTime;
    if (0.f != joyMoveX) camStrafeSpeed += -joyMoveX * accel * moveMult * frameTime;

    if (input.current[SDL_SCANCODE_E] || joyUp) camUpSpeed +=  accel * moveMult * frameTime;
    if (input.current[SDL_SCANCODE_Q] || joyDown) camUpSpeed += -accel * moveMult * frameTime;

    camX -= cosf(camYaw - SFR_PI / 2.f) * camForwardSpeed * frameTime;
    camZ -= sinf(camYaw - SFR_PI / 2.f) * camForwardSpeed * frameTime;
    camX -= cosf(camYaw) * camStrafeSpeed * frameTime;
    camZ -= sinf(camYaw) * camStrafeSpeed * frameTime;
    camY += camUpSpeed * frameTime;

    camForwardSpeed -= camForwardSpeed * decel * frameTime;
    camStrafeSpeed  -= camStrafeSpeed  * decel * frameTime;
    camUpSpeed      -= camUpSpeed * decel * frameTime;

    if (input.current[SDL_SCANCODE_I]) camPitch -= frameTime * turnMult * 0.7f;
    if (input.current[SDL_SCANCODE_K]) camPitch += frameTime * turnMult * 0.7f;
    if (input.current[SDL_SCANCODE_J]) camYaw += frameTime * turnMult;
    if (input.current[SDL_SCANCODE_L]) camYaw -= frameTime * turnMult;
    
    if (0.f != joyLookY) camPitch += joyLookY * frameTime * turnMult * 0.7f;
    if (0.f != joyLookX) camYaw -= joyLookX * frameTime * turnMult;

    if (camPitch < -1.5f) camPitch = -1.5f;
    if (camPitch >  1.5f) camPitch = 1.5f;

    sfr_set_fov(80.f);
    sfr_set_camera(camX, camY, camZ, camYaw, camPitch, 0.f);
    
    // adjust point light's position
    if (input.current[SDL_SCANCODE_X]) {
        pointLight->x = camX;
        pointLight->y = camY;
        pointLight->z = camZ;
    }
    // toggle point light
    if (input.pressed[SDL_SCANCODE_C]) {
        pointLight->intensity = 2.f - pointLight->intensity;
    }

    // debug print camera info
    if (input.pressed[SDL_SCANCODE_P]) {
        printf("cam info | pos: %.2f, %.2f, %.2f, yaw: %.2f, pitch: %.2f\n",
            camX, camY, camZ, camYaw, camPitch);
    }
}

static void parse_argv(i32 argc, char** argv) {
    for (int i = 1; i <= argc; i += 1) {
        const char* filepath = argv[i];
        
        // last occurrence of '.' to get the extension
        const char* ext = strrchr(filepath, '.');

        if (NULL == ext) {
            printf("Warning: No file extension found for '%s', skipping.\n", filepath);
            continue;
        }

        if (strcmp(ext, ".glb") == 0 || strcmp(ext, ".gltf") == 0) {
            SfrModel* sceneModel = sfr_load_gltf(filepath, 0, 256, 256);
            if (sceneModel) {
                scenes[sceneCount++] = sfr_scene_from_model(sceneModel);
                printf("Loaded GLTF scene '%s'\n", filepath);
            } else {
                printf("Failed to load GLTF scene: '%s'\n", filepath);
            }
        } else if (strcmp(ext, ".cmp") == 0) {
            SfrScene* scene = sfr_load_scene(filepath);
            if (scene) {
                scenes[sceneCount++] = scene;
                printf("Loaded CMP scene '%s'\n", filepath);
            } else {
                printf("Failed to load CMP scene: '%s'\n", filepath);
            }
        } else {
            printf("Warning: Unsupported file extension '%s' for file '%s', skipping.\n", ext, filepath);
        }
    }
}
