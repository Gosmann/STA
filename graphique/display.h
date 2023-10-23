#define SCREEN_HEIGHT       400
#define SCREEN_WIDTH        400
#define SCREEN_ORIGIN_X     100
#define SCREEN_ORIGIN_Y     100

void create_window( SDL_Window ** window,  SDL_Renderer ** renderer);

void draw_row(SDL_Renderer * renderer, uint32_t row, SDL_Color color);

void close_window( SDL_Window * window);