/* SOFREN EXAMPLES, starter-sdl2.c

a slightly larger example that shows more:
    handling a window resizing,
    simple FPS player controller,
    drawing a simple scene,
    drawing OBJ files

*/

#define SFR_IMPL
#include "../sofren.c"

#ifndef SFR_NO_STD // cant load obj file without stdio
    #define OBJ_PATH "examples/res/rock.obj"
#endif

#include <SDL2/SDL.h>

#define RES_SCALE 0.8f
#define USE_VSYNC 1

i32 main(void) {
    { // use sfrPixelBuf and sfrDepthBuf later for clarity
        const i32 w = 1280 * RES_SCALE, h = 720 * RES_SCALE;
        u32* pixelBuf = (u32*)malloc(sizeof(u32) * w * h);
        f32* depthBuf = (f32*)malloc(sizeof(f32) * w * h);
        sfr_init(pixelBuf, depthBuf, w,  h, 80.f);
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
        Mesh* mesh = sfr_load_mesh(OBJ_PATH);
        if (!mesh) {
            return 1;
        }
    #endif

    // player controller variables
    const f32 playerCrouchHeight = 0.5f, playerStandHeight = 1.f;
    f32 playerX = -1.f, playerZ = -3.f, playerY = 1.f;
    f32 playerYaw = 1.f, playerPitch = 0.f;
    f32 playerForwardSpeed = 0.f;
    f32 playerStrafeSpeed = 0.f;

    // input flags
    i32 up = 0, down = 0, left = 0, right = 0, crouch = 0, sprint = 0;
    i32 turnUp = 0, turnDown = 0, turnLeft = 0, turnRight = 0;

    // world variables
    const i32 worldSize = 30;
    const f32 boundary = 2.2f;

    // srand is called throughout the program so save seed
    // seed isn't time(NULL) so time.h doesn't have to be included
    const u32 seed = 2;
    sfr_rand_seed(seed);

    // main game loop
    for (i32 shouldQuit = 0; !shouldQuit;) {
        static f32 prevTime = 0.f;
        const f32 time = SDL_GetTicks() / 1000.f;
        const f32 delta = time - prevTime;
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

        // clear pixel and depth buffers
        sfr_clear();

        { // update player, TODO time independent things (marked with '<--')
            const f32 moveMult = sprint ? 3.f : 1.f;
            const f32 turnMult = 2.75f;

            if (up)    playerForwardSpeed = 2.5f * delta * moveMult;
            if (down)  playerForwardSpeed = -2.5f * delta * moveMult;
            if (left)  playerStrafeSpeed = 2.5f * delta * moveMult;
            if (right) playerStrafeSpeed = -2.5f * delta * moveMult;
            playerY += ((crouch ? playerCrouchHeight : playerStandHeight) - playerY) * 0.1f; // <--
            
            playerX -= cosf(playerYaw - SFR_PI / 2.f) * playerForwardSpeed;
            playerZ -= sinf(playerYaw - SFR_PI / 2.f) * playerForwardSpeed;
            playerX -= cosf(playerYaw) * playerStrafeSpeed;
            playerZ -= sinf(playerYaw) * playerStrafeSpeed;
            
            if      (playerX < -worldSize + boundary) playerX = -worldSize + boundary;
            else if (playerX >  worldSize - boundary) playerX =  worldSize - boundary;
            if      (playerZ < -worldSize + boundary) playerZ = -worldSize + boundary;
            else if (playerZ >  worldSize - boundary) playerZ =  worldSize - boundary;

            playerForwardSpeed *= 0.9f; // <--
            playerStrafeSpeed *= 0.9f; // <--
    
            if (turnUp)    playerPitch -= delta * turnMult * 0.7f;
            if (turnDown)  playerPitch += delta * turnMult * 0.7f;
            if (turnLeft)  playerYaw += delta * turnMult;
            if (turnRight) playerYaw -= delta * turnMult;

            if (playerPitch < -SFR_PI / 2.f) playerPitch = -1.57f;
            if (playerPitch >  SFR_PI / 2.f) playerPitch = 1.57f;    
        
            sfr_set_camera(playerX, playerY, playerZ, playerYaw, playerPitch, 0.f);
        }

        { // draw wavy floor and ceiling
            const f32 amp   = 0.3f; // wave height
            const f32 freq  = 0.6f; // wave density
            const f32 speed = 1.5f; // animation speed
            const f32 size  = 0.8f; // grid size
            
            sfr_set_lighting(1, sfr_vec_norm((Vec){1.f, 1.f, 0.f}), 0.1f);
            
            // draw floor
            sfr_reset();
            for (f32 z = -worldSize; z < worldSize; z += size * 2.f) {
                for (f32 x = -worldSize; x < worldSize; x += size * 2.f) {
                    const i32 col = (int)(x + z) % 2 ? 0x2A6F5F : 0x2D2F6F;
                    
                    // triangle's world positions calculated so no transformations needed
                    const f32 ax = x + size;
                    const f32 az = z + size;
                    const f32 ay = amp * sinf((ax + az) * freq + time * speed);
                    const f32 bx = x - size;
                    const f32 bz = z - size;
                    const f32 by = amp * sinf((bx + bz) * freq + time * speed);
                    const f32 cx = x - size;
                    const f32 cz = z + size;
                    const f32 cy = amp * sinf((cx + cz) * freq + time * speed);
                    const f32 dx = x + size;
                    const f32 dz = z - size;
                    const f32 dy = amp * sinf((dx + dz) * freq + time * speed);
                    sfr_triangle(ax, ay, az, bx, by, bz, cx, cy, cz, col);
                    sfr_triangle(ax, ay, az, dx, dy, dz, bx, by, bz, col);
                }
            }

            sfr_set_lighting(0, SFR_VEC0, 0.f); // no shading on the ceiling

            // draw ceiling
            sfr_reset();
            sfr_rotate_z(SFR_PI);
            sfr_translate(0.f, 6.f, 0.f);
            for (f32 z = -worldSize; z < worldSize; z += size * 2.f) {
                for (f32 x = -worldSize; x < worldSize; x += size * 2.f) {
                    const i32 col = (int)(x + z) % 2 ? 0xFA6F5F : 0xCDFB6F;
                    const f32 ax = x + size;
                    const f32 az = z + size;
                    const f32 ay = amp * sinf((ax + az) * freq + time * speed);
                    const f32 bx = x - size;
                    const f32 bz = z - size;
                    const f32 by = amp * sinf((bx + bz) * freq + time * speed);
                    const f32 cx = x - size;
                    const f32 cz = z + size;
                    const f32 cy = amp * sinf((cx + cz) * freq + time * speed);
                    const f32 dx = x + size;
                    const f32 dz = z - size;
                    const f32 dy = amp * sinf((dx + dz) * freq + time * speed);
                    sfr_triangle(ax, ay, az, bx, by, bz, cx, cy, cz, col);
                    sfr_triangle(ax, ay, az, dx, dy, dz, bx, by, bz, col);
                }
            }
        }

        { // show the border using transformed cubes, with shading disabled
            sfr_rand_seed(seed);
            sfr_set_lighting(0, SFR_VEC0, 0.f);
            for (i32 i = -worldSize; i < worldSize; i += 2) {
                for (i32 j = 0; j < 4; j += 1) {
                    f32 x = i, z = i, y = sfr_rand_flt(15.f, 30.f);
                    switch (j) {
                        case 0: x = -worldSize; break;
                        case 1: x =  worldSize; break;
                        case 2: z = -worldSize; break;
                        case 3: z =  worldSize; break;
                    }

                    u32 cols[12];
                    for (i32 c = 0; c < 12; c += 1) {
                        cols[c] = (u32)(
                            (sfr_rand_int(50, 210) << 16) | 
                            (sfr_rand_int(50, 210) << 8)  |
                            (sfr_rand_int(50, 210) << 0));
                    }

                    sfr_reset();
                    sfr_rotate_y(sfr_rand_flt(0.f, SFR_PI));
                    sfr_rotate_x(sfr_rand_flt(0.f, SFR_PI));
                    sfr_rotate_z(sfr_rand_flt(0.f, SFR_PI));
                    sfr_scale(sfr_rand_flt(8.f, 12.f), y, sfr_rand_flt(8.f, 12.f));
                    sfr_translate(x, y / 2.f, z);
                    sfr_cube_ex(cols);
                }
            }
        }

        #ifndef SFR_NO_STD
        { // draw objects
            sfr_rand_seed(seed);
            sfr_set_lighting(1, sfr_vec_norm((Vec){1.f, 1.f, 0.f}), 0.3);
            const f32 range = worldSize - boundary;
            for (i32 i = 0; i < 10; i += 1) {
                const f32 x = sfr_rand_flt(-range, range);
                const f32 z = sfr_rand_flt(-range, range);

                sfr_reset();
                sfr_rotate_y(i + time * 3.f);
                sfr_scale(0.0025f, 0.0025f, 0.0025f);
                sfr_translate(x, 0.3f + sinf(i + time * 2.f) * 0.2f, z);
                sfr_mesh(mesh, 0xFADA80);
            }
        }
        #endif

        // output pixel buf to SDL window
        SDL_RenderClear(renderer);
        SDL_UpdateTexture(texture, NULL, sfrPixelBuf, sizeof(i32) * sfrWidth);
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
