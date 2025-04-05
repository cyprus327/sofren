#include <time.h> // for time(0)

#include <SDL2/SDL.h>

#define SFR_IMPL
#include "../sofren.c" // you'll probably have to change both if you're copy pasting this example
#define ROCK_PATH "examples/res/rock.obj"

#define USE_VSYNC 1

static inline float rand01(void) {
    return rand() / (float)RAND_MAX;
}

int main(void) {
    { // use sfrPixelBuf and sfrDepthBuf later for clarity
        const int w = 1280, h = 720; // start resolution will be kept with this SDL setup
        sfrcol_t* pixelBuf = (sfrcol_t*)malloc(sizeof(sfrcol_t) * w * h);
        float* depthBuf = (float*)malloc(sizeof(float) * w * h);
        sfr_init(pixelBuf, depthBuf, w,  h, 80.f);
    }

    // using sfrWidth and sfrHeight
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow(
        "Window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, sfrWidth, sfrHeight, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(
        window, -1, USE_VSYNC ? SDL_RENDERER_PRESENTVSYNC : 0);
    SDL_Texture* texture = SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, sfrWidth, sfrHeight);

    // load rock obj and set it up
    sfrmesh_t* mesh = sfr_load_mesh(ROCK_PATH);
    mesh->col = 0xfada80;
    mesh->scale = (sfrvec_t){0.0025f, 0.0025f, 0.0025f};

    // player controller variables
    const float playerCrouchHeight = 0.5f, playerStandHeight = 1.f;
    float playerX = -1.f, playerZ = -3.f, playerY = 1.f;
    float playerYaw = 1.f, playerPitch = 0.f;
    float playerForwardSpeed = 0.f;
    float playerStrafeSpeed = 0.f;

    // input flags
    int up = 0, down = 0, left = 0, right = 0, crouch = 0, sprint = 0;
    int turnUp = 0, turnDown = 0, turnLeft = 0, turnRight = 0;

    // world variables
    const int worldSize = 30;
    const float boundary = 2.5f;

    // srand is called throughout the program so save seed
    const int seed = time(0);
    srand(seed);

    // main game loop
    int shouldQuit = 0;
    while (!shouldQuit) {
        static float prevTime = 0.f;
        const float time = SDL_GetTicks() / 1000.f;
        const float delta = prevTime - time;
        prevTime = time;

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

        // clear pixel and depth buffers
        sfr_clear();

        { // update player, TODO fix things that don't use delta
            const float moveMult = sprint ? 3.f : 1.f;
            const float turnMult = 2.75f;

            if (up)    playerForwardSpeed = -2.5f * delta * moveMult;
            if (down)  playerForwardSpeed = 2.5f * delta * moveMult;
            if (left)  playerStrafeSpeed = -2.5f * delta * moveMult;
            if (right) playerStrafeSpeed = 2.5f * delta * moveMult;
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
    
            if (turnUp)    playerPitch += delta * turnMult;
            if (turnDown)  playerPitch -= delta * turnMult;
            if (turnLeft)  playerYaw -= delta * turnMult;
            if (turnRight) playerYaw += delta * turnMult;

            if (playerPitch < -SFR_PI / 2.f) playerPitch = -1.57f;
            if (playerPitch >  SFR_PI / 2.f) playerPitch = 1.57f;    
        
            sfr_set_camera(playerX, playerY, playerZ, playerYaw, playerPitch, 0.f);
        }

        { // draw wavy checkerboard floor
            const float amp = 0.2f;   // wave height
            const float freq = 0.6f;  // wave density
            const float speed = 1.5f; // animation speed
            
            sfr_reset();
            sfr_set_lighting(1, sfr_vec_norm((sfrvec_t){1.f, 1.f, 0.f}), 0.5f);
            for (int z = -worldSize; z < worldSize; z += 1) {
                for (int x = -worldSize; x < worldSize; x += 1) {
                    const sfrcol_t col = (x + z) % 2 ? 0x2a6f5f : 0x2d2f6f;
                    
                    // triangle's world positions calculated so no transformations needed
                    const float ax = x + 0.5f;
                    const float az = z + 0.5f;
                    const float ay = amp * sinf((ax + az) * freq + time * speed);
                    const float bx = x - 0.5f;
                    const float bz = z - 0.5f;
                    const float by = amp * sinf((bx + bz) * freq + time * speed);
                    const float cx = x - 0.5f;
                    const float cz = z + 0.5f;
                    const float cy = amp * sinf((cx + cz) * freq + time * speed);
                    const float dx = x + 0.5f;
                    const float dz = z - 0.5f;
                    const float dy = amp * sinf((dx + dz) * freq + time * speed);
                    sfr_triangle(ax, ay, az, bx, by, bz, cx, cy, cz, col);
                    sfr_triangle(ax, ay, az, dx, dy, dz, bx, by, bz, col);
                }
            }
        }

        { // show the border using transformed cubes, with shading disabled
            srand(seed);
            sfr_set_lighting(0, SFR_VEC0, 0.f);
            for (int i = -worldSize; i < worldSize; i += 2) {
                for (int j = 0; j < 4; j += 1) {
                    float x = i, z = i;
                    switch (j) {
                        case 0: x = -worldSize; break;
                        case 1: x =  worldSize; break;
                        case 2: z = -worldSize; break;
                        case 3: z =  worldSize; break;
                    }

                    sfr_reset();
                    sfr_rotate_y(rand01() * 3.14159f);
                    sfr_rotate_x(rand01() * 3.14159f);
                    sfr_rotate_z(rand01() * 3.14159f);
                    sfr_scale(1.0f + rand01() * 3.f, 1.0f + rand01() * 6.f, 1.0f + rand01() * 3.f);
                    sfr_translate(x, 0.5, z);
                    sfr_cube((sfrcol_t)(
                        ((int)(50.f + rand01() * 170.f) << 16) | 
                        ((int)(50.f + rand01() * 170.f) << 8)  |
                        ((int)(50.f + rand01() * 170.f) << 0)
                    ));
                }
            }
        }

        { // draw objects
            srand(seed);
            sfr_set_lighting(1, sfr_vec_norm((sfrvec_t){1.f, 1.f, 0.f}), 0.3f);
            for (int i = 0; i < 10; i += 1) {
                const float x = (rand01() - 0.5f) * 2.f * (worldSize - boundary);
                const float z = (rand01() - 0.5f) * 2.f * (worldSize - boundary);

                mesh->rot.y = i + time * 3.f;
                mesh->pos.x = x;
                mesh->pos.y = 0.3f + sinf(i + time * 2.f) * 0.2f;
                mesh->pos.z = z;

                sfr_reset();
                sfr_mesh(mesh);
            }
        }

        SDL_RenderClear(renderer);
        SDL_UpdateTexture(texture, NULL, sfrPixelBuf, sizeof(sfrcol_t) * sfrWidth);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

    sfr_release_mesh(&mesh);
    free(sfrPixelBuf);
    free(sfrDepthBuf);

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
