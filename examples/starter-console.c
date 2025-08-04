/* SOFREN EXAMPLES, starter-console.c

simple example for how to render to a console window efficiently,
the performance of this example is highly determined by the terminal's performance

*/

#define SFR_IMPL
#define SFR_NO_ALPHA
#include "../sofren.c"

#include <stdio.h> // for printing rendered output
#include <math.h>  // for 'sinf', 'cosf'
#include <time.h>  // for 'clock'

// for 'get_console_size'
#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
    #include <sys/ioctl.h>
#endif

// pixel colors to grayscale ascii
static const i8 COLOR_RAMP[] = " .:-=+*#%%@";
#define RAMP_LENGTH (sizeof(COLOR_RAMP) - 1)

// helpers
static inline i8 color_to_ascii(u32 col);
static inline i32 get_console_size(i32* width, i32* height);

i32 main() {
    #ifdef _WIN32
        // enable ansi escape codes on windows
        const HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
        DWORD dwMode = 0;
        GetConsoleMode(hOut, &dwMode);
        dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
        SetConsoleMode(hOut, dwMode);
    #endif

    { // initialize sofren
        i32 w, h;
        if (get_console_size(&w, &h)) {
            fprintf(stderr, "Failed to get console size\n");
            return 1;
        }

        sfr_init(w, h, 80.f, malloc, free);
    
        const sfrvec l = sfr_vec_normf(-0.5f, 0.1f, -1.f);
        sfr_set_light(0, (SfrLight){
            .dirX = l.x, .dirY = l.y, .dirZ = l.z,
            .ambient = 0.4f, .intensity = 1.f,
            .type = SFR_LIGHT_DIRECTIONAL
        });
        sfr_set_lighting(1);
    }

    // character out buffer
    i8* screenBuf = (i8*)malloc(sizeof(i8) * (sfrWidth + 1) * sfrHeight + 1);

    // main loop, idk a simple cross platform no dependency non blocking way to check if a
    // key is down so the only way to close is Ctrl+C or close the whole terminal
    for (i32 shouldClose = 0; !shouldClose;) {
        i32 width, height;
        if (get_console_size(&width, &height)) {
            fprintf(stderr, "Failed to get console size\n");
            return 1;
        }

        static f32 time = 0.f;
        #ifdef _WIN32
            const f32 currTime = clock() / 1000.f;
        #else
            const f32 currTime = clock() / 1000000.f;
        #endif
        const f32 frameTime = currTime - time;
        time = currTime;

        { // handle resizing
            if (width != sfrWidth || height != sfrHeight) {
                sfr_resize(width, height);
                screenBuf = (i8*)realloc(screenBuf, sizeof(i8) * (width + 1) * height + 1);
            }
        }

        // clear pixel and depth buffers
        sfr_clear(0x000000);

        { // render scene
            sfr_rand_seed(5);
            for (i32 i = 0; i < 100; i += 1) {
                const f32 rt = sfr_rand_flt(0.f, 500.f) + time;
                sfr_reset();
                sfr_scale(
                    (1.4f + sinf(rt * 0.6f)) / 2.f,
                    (1.2f + sinf(rt * 0.8f)) / 2.f,
                    (1.8f + sinf(rt * 0.3f)) / 2.f);
                sfr_rotate_x(rt * SFR_PI * 0.2f);
                sfr_rotate_y(rt * SFR_PI * 0.3f);
                sfr_rotate_z(rt * SFR_PI * 0.6f);
                sfr_translate(sfr_rand_flt(-12.f, 12.f), sfr_rand_flt(-3, 3) + sinf(rt), sfr_rand_flt(4.f, 12.f));
                sfr_cube(sfr_rand_int(0, 0xFFFFFF), NULL);
            }
        }

        { // draw scene
            i8* out = screenBuf;
            for (i32 y = 0, i = 0; y < sfrHeight; y += 1) {
                for (i32 x = 0; x < sfrWidth; x += 1, i += 1) {
                    *out = color_to_ascii(sfrPixelBuf[i]);
                    out += 1;
                }
                *out = '\n';
                out += 1;
            }
            *out = '\0';
            fputs("\x1b[H", stdout); // move cursor to top left
            fwrite(screenBuf, 1, (sfrWidth + 1) * sfrHeight + 1, stdout);
            fflush(stdout);
        }
    }

    // release resources
    free(screenBuf);
    sfr_release();

    return 0;
}

static inline i8 color_to_ascii(u32 col) {
    if (0 == col) {
        return ' ';
    }
    
    const i32 r = (col >> 16) & 0xFF;
    const i32 g = (col >> 8)  & 0xFF;
    const i32 b = (col >> 0)  & 0xFF;
    
    // luminance
    const f32 luminance = 0.299f * r + 0.587f * g + 0.114f * b;
    
    // map to ramp index
    f32 t = luminance / 255.f;
    t = powf(t, 0.5f); // gamma correction
    
    return COLOR_RAMP[(i32)(t * (RAMP_LENGTH - 1))];
}

// returns height - 1 so the frames don't jitter
static inline i32 get_console_size(i32* width, i32* height) {
    #ifdef _WIN32
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        if (!GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi)) {
            return -1;
        }

        *width = csbi.srWindow.Right - csbi.srWindow.Left + 1;
        *height = csbi.srWindow.Bottom - csbi.srWindow.Top;
    #else
        struct winsize w;
        if (-1 == ioctl(STDOUT_FILENO, TIOCGWINSZ, &w)) {
            return -1;
        }

        *width = w.ws_col;
        *height = w.ws_row - 1;
    #endif

    return 0;
}
