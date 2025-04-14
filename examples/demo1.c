#define SFR_IMPL
#define SFR_NO_STD
#define SFR_NO_MATH
#define SFR_NO_STRING
#define SFR_NO_STDINT
#include "../sofren.c"

#ifndef SFR_NO_STD // cant load obj file without stdio
    #define ROCK_PATH "examples/res/rock.obj"
#endif

#include <SDL2/SDL.h>

#define RES_SCALE ((sfrflt_t)0.7)
#define USE_VSYNC 1

static inline sfrflt_t rand01(void) {
    return rand() / (sfrflt_t)(0x7FFFFFFF);
}

sfrint_t main(void) {
    { // use sfrPixelBuf and sfrDepthBuf later for clarity
        const sfrint_t w = 1280 * RES_SCALE, h = 720 * RES_SCALE;
        sfrcol_t* pixelBuf = (sfrcol_t*)malloc(sizeof(sfrcol_t) * w * h);
        sfrfix_t* depthBuf = (sfrfix_t*)malloc(sizeof(sfrfix_t) * w * h);
        sfr_init(pixelBuf, depthBuf, w,  h, 80.0);
    }

    // using sfrWidth and sfrHeight
    SDL_Init(SDL_INIT_VIDEO);
    const Uint32 flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
    SDL_Window* window = SDL_CreateWindow(
        "Window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, sfrWidth / RES_SCALE, sfrHeight / RES_SCALE, flags);
    SDL_Renderer* renderer = SDL_CreateRenderer(
        window, -1, USE_VSYNC ? SDL_RENDERER_PRESENTVSYNC : 0);
    SDL_Texture* texture = SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, sfrWidth, sfrHeight);

    // load rock obj and set it up
    #ifndef SFR_NO_STD
        sfrmesh_t* mesh = sfr_load_mesh(ROCK_PATH);
        mesh->col = 0xfada80;
        mesh->scale = (sfrvec_t){0.0025, 0.0025, 0.0025};
    #endif

    // player controller variables
    const sfrflt_t playerCrouchHeight = 0.5, playerStandHeight = 1.0;
    sfrflt_t playerX = -1.0, playerZ = -3.0, playerY = 1.0;
    sfrflt_t playerYaw = 1.0, playerPitch = 0.0;
    sfrflt_t playerForwardSpeed = 0.0;
    sfrflt_t playerStrafeSpeed = 0.0;

    // input flags
    sfrint_t up = 0, down = 0, left = 0, right = 0, crouch = 0, sprint = 0;
    sfrint_t turnUp = 0, turnDown = 0, turnLeft = 0, turnRight = 0;

    // world variables
    const sfrint_t worldSize = 30;
    const sfrflt_t boundary = 2.2f;

    // srand is called throughout the program so save seed
    // seed isn't time(NULL) so time.h doesn't have to be included
    const sfrint_t seed = 0;
    srand(seed);

    // main game loop
    sfrint_t shouldQuit = 0;
    while (!shouldQuit) {
        static sfrflt_t prevTime = 0.0;
        const sfrflt_t time = SDL_GetTicks() / 1000.0;
        const sfrflt_t delta = time - prevTime;
        prevTime = time;

        char titleBuf[64];
        sprintf(titleBuf, "Window | FPS: %f\n", 1000.f / delta / 1000.f);
        SDL_SetWindowTitle(window, titleBuf);

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (SDL_QUIT == event.type) {
                shouldQuit = 1;
            } else if (SDL_KEYDOWN == event.type || SDL_KEYUP == event.type) {
                switch (event.key.keysym.scancode) {
                    case SDL_SCANCODE_ESCAPE: shouldQuit = 1; break;
                    case SDL_SCANCODE_W: up = event.key.state; break;
                    case SDL_SCANCODE_S: down = event.key.state; break;
                    case SDL_SCANCODE_A: left = event.key.state; break;
                    case SDL_SCANCODE_D: right = event.key.state; break;
                    case SDL_SCANCODE_LCTRL: crouch = event.key.state; break;
                    case SDL_SCANCODE_LSHIFT: sprint = event.key.state; break;
                    case SDL_SCANCODE_I: turnUp = event.key.state; break;
                    case SDL_SCANCODE_K: turnDown = event.key.state; break;
                    case SDL_SCANCODE_J: turnLeft = event.key.state; break;
                    case SDL_SCANCODE_L: turnRight = event.key.state; break;
                    default: break;
                }
            }
        }

        { // handle resizing
            int width, height;
            SDL_GetWindowSize(window, &width, &height);
            
            width *= RES_SCALE;
            height *= RES_SCALE;

            if (width != sfrWidth || height != sfrHeight) {
                // update sfr internals
                sfrWidth = width;
                sfrHeight = height;
                sfrPixelBuf = (sfrcol_t*)realloc(sfrPixelBuf, sizeof(sfrcol_t) * width * height);
                sfrDepthBuf = (sfrfix_t*)realloc(sfrDepthBuf, sizeof(sfrfix_t) * width * height);
                
                // update projection matrix
                sfr_set_fov(sfrCamFov);

                // update render texture
                SDL_DestroyTexture(texture);
                texture = SDL_CreateTexture(renderer,
                    SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, width, height);
            }
        }

        // clear pixel and depth buffers
        sfr_clear();

        { // update player, TODO fix things that don't use delta
            const sfrflt_t moveMult = sprint ? 3.0 : 1.0;
            const sfrflt_t turnMult = 2.75;

            if (up)    playerForwardSpeed = 2.5 * delta * moveMult;
            if (down)  playerForwardSpeed = -2.5 * delta * moveMult;
            if (left)  playerStrafeSpeed = 2.5 * delta * moveMult;
            if (right) playerStrafeSpeed = -2.5 * delta * moveMult;
            playerY += ((crouch ? playerCrouchHeight : playerStandHeight) - playerY) * 0.1; // <--
            
            playerX -= cosf(playerYaw - SFR_PI / 2.0) * playerForwardSpeed;
            playerZ -= sinf(playerYaw - SFR_PI / 2.0) * playerForwardSpeed;
            playerX -= cosf(playerYaw) * playerStrafeSpeed;
            playerZ -= sinf(playerYaw) * playerStrafeSpeed;
            
            if      (playerX < -worldSize + boundary) playerX = -worldSize + boundary;
            else if (playerX >  worldSize - boundary) playerX =  worldSize - boundary;
            if      (playerZ < -worldSize + boundary) playerZ = -worldSize + boundary;
            else if (playerZ >  worldSize - boundary) playerZ =  worldSize - boundary;

            playerForwardSpeed *= 0.9; // <--
            playerStrafeSpeed *= 0.9; // <--
    
            if (turnUp)    playerPitch -= delta * turnMult * 0.7;
            if (turnDown)  playerPitch += delta * turnMult * 0.7;
            if (turnLeft)  playerYaw += delta * turnMult;
            if (turnRight) playerYaw -= delta * turnMult;

            if (playerPitch < -SFR_PI / 2.0) playerPitch = -1.57;
            if (playerPitch >  SFR_PI / 2.0) playerPitch = 1.57;    
        
            sfr_set_camera(playerX, playerY, playerZ, playerYaw, playerPitch, 0.0);
        }

        { // draw wavy checkerboard floor
            const sfrflt_t amp = 0.3f;   // wave height
            const sfrflt_t freq = 0.6f;  // wave density
            const sfrflt_t speed = 1.5f; // animation speed
            const sfrflt_t size = 0.75f; // square size
            
            sfr_reset();
            sfr_set_lighting(1, sfr_vec_norm((sfrvec_t){1.0, 1.0, 0.0}), 0.1f);
            for (sfrflt_t z = -worldSize; z < worldSize; z += size * 2.f) {
                for (sfrflt_t x = -worldSize; x < worldSize; x += size * 2.f) {
                    const sfrcol_t col = (int)(x + z) % 2 ? 0x2a6f5f : 0x2d2f6f;
                    
                    // triangle's world positions calculated so no transformations needed
                    const sfrflt_t ax = x + size;
                    const sfrflt_t az = z + size;
                    const sfrflt_t ay = amp * sinf((ax + az) * freq + time * speed);
                    const sfrflt_t bx = x - size;
                    const sfrflt_t bz = z - size;
                    const sfrflt_t by = amp * sinf((bx + bz) * freq + time * speed);
                    const sfrflt_t cx = x - size;
                    const sfrflt_t cz = z + size;
                    const sfrflt_t cy = amp * sinf((cx + cz) * freq + time * speed);
                    const sfrflt_t dx = x + size;
                    const sfrflt_t dz = z - size;
                    const sfrflt_t dy = amp * sinf((dx + dz) * freq + time * speed);
                    sfr_triangle(ax, ay, az, bx, by, bz, cx, cy, cz, col);
                    sfr_triangle(ax, ay, az, dx, dy, dz, bx, by, bz, col);
                }
            }
        }

        { // show the border using transformed cubes, with shading disabled
            srand(seed);
            sfr_set_lighting(0, SFR_VEC0, 0.0);
            for (sfrint_t i = -worldSize; i < worldSize; i += 2) {
                for (sfrint_t j = 0; j < 4; j += 1) {
                    sfrflt_t x = i, z = i;
                    switch (j) {
                        case 0: x = -worldSize; break;
                        case 1: x =  worldSize; break;
                        case 2: z = -worldSize; break;
                        case 3: z =  worldSize; break;
                    }

                    sfr_reset();
                    sfr_rotate_y(rand01() * SFR_PI);
                    sfr_rotate_x(rand01() * SFR_PI);
                    sfr_rotate_z(rand01() * SFR_PI);
                    sfr_scale(1.0f + rand01() * 3.0, 1.0 + rand01() * 6.0, 1.0 + rand01() * 3.0);
                    sfr_translate(x, 0.5, z);
                    sfr_cube((sfrcol_t)(
                        ((sfrint_t)(50.0 + rand01() * 170.0) << 16) | 
                        ((sfrint_t)(50.0 + rand01() * 170.0) << 8)  |
                        ((sfrint_t)(50.0 + rand01() * 170.0) << 0)
                    ));
                }
            }
        }

        #ifndef SFR_NO_STD
        { // draw objects
            srand(seed);
            sfr_set_lighting(1, sfr_vec_norm((sfrvec_t){1.0, 1.0, 0.0}), 0.3);
            for (sfrint_t i = 0; i < 10; i += 1) {
                const sfrflt_t x = (rand01() - 0.5) * 2.0 * (worldSize - boundary);
                const sfrflt_t z = (rand01() - 0.5) * 2.0 * (worldSize - boundary);

                mesh->rot.y = i + time * 3.0;
                mesh->pos.x = x;
                mesh->pos.y = 0.3 + sinf(i + time * 2.0) * 0.2;
                mesh->pos.z = z;

                sfr_reset();
                sfr_mesh(mesh);
            }
        }
        #endif

        SDL_Delay(1);
        SDL_RenderClear(renderer);
        SDL_UpdateTexture(texture, NULL, sfrPixelBuf, sizeof(sfrcol_t) * sfrWidth);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

    #ifndef SFR_NO_STD
        sfr_release_mesh(&mesh);
    #endif
    free(sfrPixelBuf);
    free(sfrDepthBuf);

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
