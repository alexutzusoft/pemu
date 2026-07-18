#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "processor.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(ms) usleep((ms) * 1000)
#endif

void print_usage(const char *prog_name) {
    printf("Pemu - Processor Emulator CLI\n");
    printf("Usage: %s [options]\n\n", prog_name);
    printf("Options:\n");
    printf("  -l, --list           List available modular processors\n");
    printf("  -p <name>            Select processor to emulate\n");
    printf("  -b <file>            Load binary program file into processor\n");
    printf("  -a <addr>            Set start load address (dec or hex starting with 0x, default 0)\n");
    printf("  -s, --step           Enable interactive step-by-step mode\n");
    printf("  -d, --delay <ms>     Set delay (in milliseconds) between execution steps\n");
    printf("  -m, --max-steps <n>  Limit maximum number of execution steps (default: 10000)\n");
    printf("  -r, --realtime       Run emulation at real-time clock speed\n");
    printf("  -f, --frequency <hz> Override the clock speed in Hz (implies --realtime)\n");
    printf("  -h, --help           Show this help message\n");
}

uint8_t* read_file(const char *path, size_t *out_size) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    if (sz < 0) {
        fclose(f);
        return NULL;
    }
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = (uint8_t*)malloc(sz);
    if (!buf) {
        fclose(f);
        return NULL;
    }
    size_t read_bytes = fread(buf, 1, sz, f);
    if (read_bytes != (size_t)sz) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *out_size = (size_t)sz;
    return buf;
}

double get_time_sec(void) {
#ifdef _WIN32
    LARGE_INTEGER freq, counter;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart / freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

#if defined(_WIN32) && !defined(USE_SDL2)
#define FRAMEWORK_WIN32_GDI
#else
#define FRAMEWORK_SDL2
#endif

#ifdef _WIN32
#include <process.h>
#else
#include <pthread.h>
#include <unistd.h>
#define Sleep(ms) usleep((ms) * 1000)
#endif

#ifdef FRAMEWORK_SDL2
#include <SDL2/SDL.h>
#endif

static volatile int g_window_closed = 0;
static void *g_mos6502_cpu = NULL;
uint8_t g_keyboard_state = 0; // global NES controller status byte

typedef struct EmulationArgs {
    const ProcessorInfo *info;
    void *cpu;
    long long max_steps;
    int realtime_mode;
    uint32_t target_frequency;
} EmulationArgs;

static EmulationArgs g_emulation_args;

void mos6502_render_screen(void *context, uint32_t *display_buffer);

#ifdef FRAMEWORK_WIN32_GDI
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_CREATE:
            SetTimer(hwnd, 1, 33, NULL); // 30 FPS repaint timer
            break;
        case WM_TIMER:
            InvalidateRect(hwnd, NULL, FALSE);
            break;
        case WM_KEYDOWN:
        case WM_KEYUP:
            {
                int is_down = (msg == WM_KEYDOWN);
                uint8_t bit = 0;
                int key_handled = 0;
                switch (wParam) {
                    case 'Z': case 'z': bit = 0x01; key_handled = 1; break; // A
                    case 'X': case 'x': bit = 0x02; key_handled = 1; break; // B
                    case VK_SPACE:      bit = 0x04; key_handled = 1; break; // Select
                    case VK_RETURN:     bit = 0x08; key_handled = 1; break; // Start
                    case VK_UP:         bit = 0x10; key_handled = 1; break; // Up
                    case VK_DOWN:       bit = 0x20; key_handled = 1; break; // Down
                    case VK_LEFT:       bit = 0x40; key_handled = 1; break; // Left
                    case VK_RIGHT:      bit = 0x80; key_handled = 1; break; // Right
                }
                if (key_handled) {
                    if (is_down) {
                        g_keyboard_state |= bit;
                    } else {
                        g_keyboard_state &= ~bit;
                    }
                }
            }
            break;
        case WM_PAINT:
            {
                PAINTSTRUCT ps;
                HDC hdc = BeginPaint(hwnd, &ps);
                
                static uint32_t display_buffer[256 * 240];
                mos6502_render_screen(g_mos6502_cpu, display_buffer);
                
                BITMAPINFO bmi;
                memset(&bmi, 0, sizeof(bmi));
                bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
                bmi.bmiHeader.biWidth = 256;
                bmi.bmiHeader.biHeight = -240; // top-down
                bmi.bmiHeader.biPlanes = 1;
                bmi.bmiHeader.biBitCount = 32;
                bmi.bmiHeader.biCompression = BI_RGB;
                
                // Scale 256x240 pixels to 512x480 GDI window
                StretchDIBits(hdc, 0, 0, 512, 480, 0, 0, 256, 240, display_buffer, &bmi, DIB_RGB_COLORS, SRCCOPY);
                
                EndPaint(hwnd, &ps);
            }
            break;
        case WM_DESTROY:
            g_window_closed = 1;
            PostQuitMessage(0);
            break;
        default:
            return DefWindowProc(hwnd, msg, wParam, lParam);
    }
    return 0;
}

void run_win32_gui_loop(void) {
    HINSTANCE hInstance = GetModuleHandle(NULL);
    
    WNDCLASS wc;
    memset(&wc, 0, sizeof(wc));
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = "PemuDisplayClass";
    wc.hbrBackground = (HBRUSH)(COLOR_BACKGROUND + 1);
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    
    RegisterClass(&wc);
    
    RECT rect = {0, 0, 512, 480}; // 256x240 aspect ratio upscaled 2x
    AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX, FALSE);
    
    HWND hwnd = CreateWindow(
        "PemuDisplayClass",
        "Pemu - MOS 6502 Display",
        WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rect.right - rect.left, rect.bottom - rect.top,
        NULL, NULL, hInstance, NULL
    );
    
    if (!hwnd) return;
    
    ShowWindow(hwnd, SW_SHOW);
    UpdateWindow(hwnd);
    
    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
}
#endif

#ifdef FRAMEWORK_SDL2
void run_sdl2_gui_loop(void) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return;
    }
    
    SDL_Window *window = SDL_CreateWindow(
        "Pemu - MOS 6502 Display",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        512, 480,
        SDL_WINDOW_SHOWN
    );
    if (!window) {
        fprintf(stderr, "Window could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_Quit();
        return;
    }
    
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        renderer = SDL_CreateRenderer(window, -1, 0);
    }
    
    SDL_Texture *texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        256, 240
    );
    
    static uint32_t display_buffer[256 * 240];
    SDL_Event e;
    int quit = 0;
    
    while (!quit && !g_window_closed) {
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = 1;
            } else if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                int is_down = (e.type == SDL_KEYDOWN);
                uint8_t bit = 0;
                int key_handled = 0;
                switch (e.key.keysym.sym) {
                    case SDLK_z:      bit = 0x01; key_handled = 1; break; // A
                    case SDLK_x:      bit = 0x02; key_handled = 1; break; // B
                    case SDLK_SPACE:  bit = 0x04; key_handled = 1; break; // Select
                    case SDLK_RETURN: bit = 0x08; key_handled = 1; break; // Start
                    case SDLK_UP:     bit = 0x10; key_handled = 1; break; // Up
                    case SDLK_DOWN:   bit = 0x20; key_handled = 1; break; // Down
                    case SDLK_LEFT:   bit = 0x40; key_handled = 1; break; // Left
                    case SDLK_RIGHT:  bit = 0x80; key_handled = 1; break; // Right
                }
                if (key_handled) {
                    if (is_down) {
                        g_keyboard_state |= bit;
                    } else {
                        g_keyboard_state &= ~bit;
                    }
                }
            }
        }
        
        mos6502_render_screen(g_mos6502_cpu, display_buffer);
        SDL_UpdateTexture(texture, NULL, display_buffer, 256 * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
        
        SDL_Delay(16); // ~60 FPS rate limiting
    }
    
    g_window_closed = 1;
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
#endif

void emulation_thread_func(void *arg) {
    (void)arg;
    const ProcessorInfo *info = g_emulation_args.info;
    void *cpu = g_emulation_args.cpu;
    long long max_steps = g_emulation_args.max_steps;
    uint32_t target_frequency = g_emulation_args.target_frequency;
    
    long long steps_count = 0;
    int halt = 0;
    
    uint32_t speed = target_frequency > 0 ? target_frequency : info->default_speed_hz;
    if (speed == 0) speed = 1000;
    
    uint32_t batch_size = speed / 100;
    if (batch_size == 0) batch_size = 1;
    
    double start_wall = get_time_sec();
    
    while (steps_count < max_steps && !halt && !g_window_closed) {
        for (uint32_t i = 0; i < batch_size; ++i) {
            int step_res = info->step(cpu);
            steps_count++;
            if (step_res == 1) {
                halt = 1;
                break;
            } else if (step_res < 0) {
                halt = 1;
                break;
            }
            if (steps_count >= max_steps || g_window_closed) {
                break;
            }
        }
        if (halt || g_window_closed) break;
        
        double expected_time = (double)steps_count / speed;
        double elapsed_wall = get_time_sec() - start_wall;
        if (expected_time > elapsed_wall) {
            double sleep_time = expected_time - elapsed_wall;
            #ifdef _WIN32
                Sleep((DWORD)(sleep_time * 1000.0));
            #else
                struct timespec ts;
                ts.tv_sec = (time_t)sleep_time;
                ts.tv_nsec = (long)((sleep_time - ts.tv_sec) * 1e9);
                nanosleep(&ts, NULL);
            #endif
        }
    }
}

int main(int argc, char **argv) {
    if (argc == 1) {
        print_usage(argv[0]);
        printf("\n");
        processor_list_all();
        return 0;
    }

    int list_processors = 0;
    const char *proc_name = NULL;
    const char *bin_path = NULL;
    uint32_t load_address = 0;
    int step_mode = 0;
    int delay_ms = 0;
    long long max_steps = 10000;
    int max_steps_specified = 0;
    int realtime_mode = 0;
    uint32_t target_frequency = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--list") == 0) {
            list_processors = 1;
        } else if (strcmp(argv[i], "-p") == 0) {
            if (i + 1 < argc) {
                proc_name = argv[++i];
            } else {
                fprintf(stderr, "Error: -p requires processor name\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-b") == 0) {
            if (i + 1 < argc) {
                bin_path = argv[++i];
            } else {
                fprintf(stderr, "Error: -b requires file path\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-a") == 0) {
            if (i + 1 < argc) {
                char *end;
                const char *addr_str = argv[++i];
                if (addr_str[0] == '0' && (addr_str[1] == 'x' || addr_str[1] == 'X')) {
                    load_address = (uint32_t)strtoul(addr_str + 2, &end, 16);
                } else {
                    load_address = (uint32_t)strtoul(addr_str, &end, 10);
                }
            } else {
                fprintf(stderr, "Error: -a requires address\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--step") == 0) {
            step_mode = 1;
        } else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--delay") == 0) {
            if (i + 1 < argc) {
                delay_ms = atoi(argv[++i]);
            } else {
                fprintf(stderr, "Error: -d requires delay in ms\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--max-steps") == 0) {
            if (i + 1 < argc) {
                max_steps = strtoll(argv[++i], NULL, 10);
                max_steps_specified = 1;
            } else {
                fprintf(stderr, "Error: -m requires max steps count\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--realtime") == 0) {
            realtime_mode = 1;
        } else if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--frequency") == 0) {
            if (i + 1 < argc) {
                target_frequency = (uint32_t)strtoul(argv[++i], NULL, 10);
                realtime_mode = 1;
            } else {
                fprintf(stderr, "Error: -f requires frequency in Hz\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    if (list_processors) {
        processor_list_all();
        return 0;
    }

    if (!proc_name) {
        fprintf(stderr, "Error: Must specify processor name using -p <name>\n");
        processor_list_all();
        return 1;
    }

    const ProcessorInfo *info = processor_find(proc_name);
    if (!info) {
        fprintf(stderr, "Error: Processor '%s' not found.\n", proc_name);
        processor_list_all();
        return 1;
    }

    // Instantiation
    void *cpu = info->create();
    if (!cpu) {
        fprintf(stderr, "Error: Failed to create processor context.\n");
        return 1;
    }

    if (info->init(cpu) != 0) {
        fprintf(stderr, "Error: Failed to initialize processor context.\n");
        info->destroy(cpu);
        return 1;
    }

    // Load binary if provided
    if (bin_path) {
        size_t size = 0;
        uint8_t *data = read_file(bin_path, &size);
        if (!data) {
            fprintf(stderr, "Error: Failed to read binary file '%s'\n", bin_path);
            info->destroy(cpu);
            return 1;
        }
        
        int load_res = info->load(cpu, data, size, load_address);
        free(data);
        
        if (load_res != 0) {
            fprintf(stderr, "Error: Failed to load binary (code %d)\n", load_res);
            info->destroy(cpu);
            return 1;
        }
        printf("Loaded %zu bytes from '%s' at address 0x%X.\n", size, bin_path, load_address);
    }

#if defined(FRAMEWORK_WIN32_GDI) || defined(FRAMEWORK_SDL2)
    if (strcmp(info->name, "mos6502") == 0 && !step_mode) {
        g_mos6502_cpu = cpu;
        g_emulation_args.info = info;
        g_emulation_args.cpu = cpu;
        g_emulation_args.max_steps = realtime_mode && !max_steps_specified ? 2000000000000LL : max_steps;
        g_emulation_args.realtime_mode = realtime_mode;
        g_emulation_args.target_frequency = target_frequency;
        
        printf("Spawning emulation thread for %s...\n", info->name);
        #ifdef _WIN32
            _beginthread((void (__cdecl *)(void *))emulation_thread_func, 0, NULL);
        #else
            pthread_t thread_id;
            pthread_create(&thread_id, NULL, (void* (*)(void*))emulation_thread_func, NULL);
        #endif
        
        #ifdef FRAMEWORK_WIN32_GDI
            printf("Running Win32 GUI window thread on main thread. Close the window to exit.\n");
            run_win32_gui_loop();
        #else
            printf("Running SDL2 GUI window thread on main thread. Close the window to exit.\n");
            run_sdl2_gui_loop();
        #endif
        
        g_window_closed = 1;
        info->destroy(cpu);
        return 0;
    }
#endif

    if (realtime_mode && !max_steps_specified) {
        max_steps = 2000000000LL; // default to a very large number for realtime execution
    }

    printf("Starting emulation of %s...\n", info->name);
    long long steps_count = 0;
    int halt = 0;

    // Show initial state
    printf("\n=== Initial State ===\n");
    info->print_state(cpu);

    if (realtime_mode && !step_mode) {
        uint32_t speed = target_frequency > 0 ? target_frequency : info->default_speed_hz;
        if (speed == 0) speed = 1000;
        printf("Running in real-time mode at %u Hz...\n", speed);
        
        uint32_t batch_size = speed / 100; // 10ms batch
        if (batch_size == 0) batch_size = 1;
        
        double start_wall = get_time_sec();
        
        while (steps_count < max_steps && !halt) {
            for (uint32_t i = 0; i < batch_size; ++i) {
                int step_res = info->step(cpu);
                steps_count++;
                
                if (step_res == 1) {
                    printf("\nExecution halted normally (HALT / Loop detected).\n");
                    halt = 1;
                    break;
                } else if (step_res < 0) {
                    fprintf(stderr, "\nExecution error: %d\n", step_res);
                    halt = 1;
                    break;
                }
                
                if (steps_count >= max_steps) {
                    break;
                }
            }
            
            if (halt) break;
            
            // Rate limiting sleep
            double expected_time = (double)steps_count / speed;
            double elapsed_wall = get_time_sec() - start_wall;
            if (expected_time > elapsed_wall) {
                double sleep_time = expected_time - elapsed_wall;
                #ifdef _WIN32
                    Sleep((DWORD)(sleep_time * 1000.0));
                #else
                    struct timespec ts;
                    ts.tv_sec = (time_t)sleep_time;
                    ts.tv_nsec = (long)((sleep_time - ts.tv_sec) * 1e9);
                    nanosleep(&ts, NULL);
                #endif
            }
        }
    } else {
        while (steps_count < max_steps && !halt) {
            char disasm[64] = {0};
            info->get_disassembly(cpu, disasm, sizeof(disasm));

            printf("\n=== Step %lld ===\n", steps_count + 1);
            printf("Next instruction to execute: %s\n", disasm);

            int step_res = info->step(cpu);
            steps_count++;

            info->print_state(cpu);

            if (step_res == 1) {
                printf("\nExecution halted normally (HALT / Loop detected).\n");
                halt = 1;
                break;
            } else if (step_res < 0) {
                fprintf(stderr, "\nExecution error: %d\n", step_res);
                break;
            }

            if (step_mode) {
                printf("--- Press Enter to Step, 'q' to Quit, 'r' to Run to HLT --- ");
                fflush(stdout);
                char line[64];
                if (fgets(line, sizeof(line), stdin)) {
                    if (line[0] == 'q' || line[0] == 'Q') {
                        printf("Exiting emulation...\n");
                        break;
                    }
                    if (line[0] == 'r' || line[0] == 'R') {
                        step_mode = 0;
                        printf("Running to completion...\n");
                    }
                }
            } else if (delay_ms > 0) {
                #ifdef _WIN32
                    Sleep(delay_ms);
                #else
                    usleep(delay_ms * 1000);
                #endif
            }
        }
    }

    if (steps_count >= max_steps && !halt) {
        printf("\nStopped: Reached maximum steps limit (%lld).\n", max_steps);
    }

    printf("\n=== Final State ===\n");
    info->print_state(cpu);

    info->destroy(cpu);
    return 0;
}
