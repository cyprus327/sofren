/* SOFREN EXAMPLES, full-sdl2.c

demonstrates:
    transparent drawing (alpha in use),
    loading textures and meshes and drawing them,
    ui with an FPS counter / text rendering,
    resizeable window,
    camera controller

*/

#define SFR_IMPL
// #define SFR_NO_ALPHA // ~15 more FPS when defined but no transparency
#include "../sofren.c"

#include <SDL2/SDL.h>

// rasterizing on a big window is expensize,
// easiest solution is to just have less pixels
#define RES_SCALE 0.8f

// draw the scene
static void draw(f32 time, f32 frameTime);

// resize if needed
static void handle_resize(SDL_Window* window, SDL_Renderer* renderer, SDL_Texture* texture);

// move camera based on inputs
static void handle_inputs(f32 frameTime);

// input flags
static struct {
    u8 up, down, left, right, sprint;
    u8 turnUp, turnDown, turnLeft, turnRight;
} inputs = {0};

static Mesh* mesh;
static Texture* meshTex;
static Texture* cubeTex;
static Font* font;

i32 main() {    
    // load mesh, textures, font
    mesh = sfr_load_mesh("examples/res/hawk.obj");
    meshTex = sfr_load_texture("examples/res/hawk.bmp");
    cubeTex = sfr_load_texture("examples/res/test.bmp");
    font = sfr_load_font("examples/res/basic-font.srft");
    if (!mesh || !meshTex || !cubeTex || !font) {
        return 1;
    }

    // initial window dimensions
    const i32 startWidth = 1280 * RES_SCALE, startHeight = 720 * RES_SCALE;
    
    { // initialize sofren
        // in brackets so these variables aren't used anywhere else,
        // use sfrPixelBuf, sfrDepthBuf, sfrAccumBuf
        u32* pixelBuf = malloc(sizeof(u32) * startWidth * startHeight);
        f32* depthBuf = malloc(sizeof(f32) * startWidth * startHeight);
        #ifdef SFR_NO_ALPHA
            sfr_init(pixelBuf, depthBuf, startWidth, startHeight, 50.f);
        #else
            // accumBuf is only needed when alpha blending is enabled
            SfrAccumCol* accumBuf = malloc(sizeof(SfrAccumCol) * startWidth * startHeight);
            sfr_init(accumBuf, pixelBuf, depthBuf, startWidth, startHeight, 50.0f);
        #endif
    }
    
    // initialize SDL
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        sfrWidth / RES_SCALE, sfrHeight / RES_SCALE, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture* sdlTex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, startWidth, startHeight);

    // main loop
    for (u8 shouldQuit = 0; !shouldQuit;) {
        // get frame time
        static f32 prevTime = 0.f;
        const f32 time = SDL_GetTicks() / 1000.f;
        const f32 frameTime = time - prevTime;
        prevTime = time;

        // handle sdl events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (SDL_QUIT == event.type) {
                shouldQuit = 1;
            } else if (SDL_KEYDOWN == event.type || SDL_KEYUP == event.type) {
                switch (event.key.keysym.scancode) {
                    case SDL_SCANCODE_ESCAPE: shouldQuit = 1; break;
                    case SDL_SCANCODE_W: inputs.up = event.key.state; break;
                    case SDL_SCANCODE_S: inputs.down = event.key.state; break;
                    case SDL_SCANCODE_A: inputs.left = event.key.state; break;
                    case SDL_SCANCODE_D: inputs.right = event.key.state; break;
                    case SDL_SCANCODE_LSHIFT: inputs.sprint = event.key.state; break;
                    case SDL_SCANCODE_I: inputs.turnUp = event.key.state; break;
                    case SDL_SCANCODE_K: inputs.turnDown = event.key.state; break;
                    case SDL_SCANCODE_J: inputs.turnLeft = event.key.state; break;
                    case SDL_SCANCODE_L: inputs.turnRight = event.key.state; break;
                    default: break;
                }
            }
        }

        // resize if needed
        handle_resize(window, renderer, sdlTex);

        // move camera from inputs
        handle_inputs(frameTime);

        // clear previous frame and draw next one
        sfr_clear(0xFF000000);
        draw(time, frameTime);

        // update SDL window
        SDL_UpdateTexture(sdlTex, NULL, sfrPixelBuf, sfrWidth * 4);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdlTex, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

    // cleanup and close SDL window
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
    
    return 0;
}

static void draw(f32 time, f32 frameTime) {
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
    
        // lighting settings
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
    
        // textured cube
        sfr_reset();
        sfr_rotate_y(sinf(time) * 0.2f + 0.5f);
        sfr_rotate_x(cosf(time) * 0.125f + 2.5f);
        sfr_scale(10.f, 10.f, 10.f);
        sfr_translate(0.f, -2.f, 20.f);
        sfr_cube_tex(0xFFFFFFFF, cubeTex);
    
        // cube clipping into hawks
        sfr_reset();
        sfr_scale(10.f, 0.5f, 5.f);
        sfr_translate(0.f, -1.f, 8.f);
        sfr_cube(0xFFAABBCC);

        // draw transparent parts of the scene
        #ifndef SFR_NO_ALPHA
            sfr_present_alpha();
        #endif
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

    // update buffers
    sfrPixelBuf = realloc(sfrPixelBuf, sizeof(u32) * width * height);
    sfrDepthBuf = realloc(sfrDepthBuf, sizeof(f32) * width * height);
    #ifndef SFR_NO_ALPHA
        sfrAccumBuf = realloc(sfrAccumBuf, sizeof(SfrAccumCol) * width * height);
    #endif
    
    // update projection matrix
    sfr_set_fov(sfrCamFov);

    // update render texture
    SDL_DestroyTexture(texture);
    texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, width, height);
}

static void handle_inputs(f32 frameTime) {
    const f32 camStandHeight = 1.f;
    static f32 camX = 0.f, camZ = 0.f, camY = 1.f;
    static f32 camYaw = 0.f, camPitch = 0.f;
    static f32 camForwardSpeed = 0.f;
    static f32 camStrafeSpeed = 0.f;

    const f32 moveMult = inputs.sprint ? 3.f : 1.f;
    const f32 turnMult = 2.75f;
    const f32 accel = 50.f;
    const f32 decel = 7.f;

    if (inputs.up)    camForwardSpeed +=  accel * moveMult * frameTime;
    if (inputs.down)  camForwardSpeed += -accel * moveMult * frameTime;
    if (inputs.left)  camStrafeSpeed +=  accel * moveMult * frameTime;
    if (inputs.right) camStrafeSpeed += -accel * moveMult * frameTime;

    camX -= cosf(camYaw - SFR_PI / 2.f) * camForwardSpeed * frameTime;
    camZ -= sinf(camYaw - SFR_PI / 2.f) * camForwardSpeed * frameTime;
    camX -= cosf(camYaw) * camStrafeSpeed * frameTime;
    camZ -= sinf(camYaw) * camStrafeSpeed * frameTime;

    camForwardSpeed -= camForwardSpeed * decel * frameTime;
    camStrafeSpeed  -= camStrafeSpeed  * decel * frameTime;

    if (inputs.turnUp)    camPitch -= frameTime * turnMult * 0.7f;
    if (inputs.turnDown)  camPitch += frameTime * turnMult * 0.7f;
    if (inputs.turnLeft)  camYaw += frameTime * turnMult;
    if (inputs.turnRight) camYaw -= frameTime * turnMult;

    if (camPitch < -SFR_PI / 2.f) camPitch = -1.57f;
    if (camPitch >  SFR_PI / 2.f) camPitch = 1.57f;    

    sfr_set_camera(camX, camY, camZ, camYaw, camPitch, 0.f);
}
