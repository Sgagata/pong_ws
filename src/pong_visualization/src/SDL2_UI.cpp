/**
 * \file
 * Description of SDL2_UI.h file.
 */
#include <string>
#include <iostream>
#include "../include/SDL2_UI.h"


SDL2_UI::SDL2_UI(std::string title, int sizeX, int sizeY, int fontSize): 
    sizeX(sizeX), // sizeX in pixels
    sizeY(sizeY) // sizeY in pixels
{
    SDL_Init(SDL_INIT_EVERYTHING);

    // Create a Window in the middle of the screen
    window = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED, sizeX, sizeY, SDL_WINDOW_SHOWN);

    renderer = SDL_CreateRenderer(window, -1,
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    InitializeFontStuff(fontSize);
}


/// Destructor fully de-initializes the SDL2_UI, including closing the main window.
SDL2_UI::~SDL2_UI() {
    DestroyFontStuff();
    if (renderer)
        SDL_DestroyRenderer(renderer);
    if (window)
        SDL_DestroyWindow(window);
    SDL_Quit();
}

/// Initialize Font rendering stuff
void SDL2_UI::InitializeFontStuff(int fontSize) {
    // For simplicty, font type and blockiness are hard-coded.
    // We take a font that is most probably available in your Linux/Ubuntu distribution.
    // If it cannot be found, type in a terminal:
    //    fc-list | grep "\.ttf"
    // and choose one of the listed fonts. Include the full path here.
    const char *fontFilePath = "/usr/share/fonts/truetype/freefont/FreeMono.ttf"; 
    blockiness = 10;

    if(TTF_Init() == -1) {
        printf("[ERROR] TTF_Init() Failed with: %s.\n", TTF_GetError());
        exit(2);
    }

    // Load our font file and set the font size
    theFont = TTF_OpenFont(fontFilePath,fontSize/blockiness);
    // Confirm that it was loaded
    if (theFont == nullptr) {
        printf("Could not load font %s\n",fontFilePath);
        printf("Make sure that the font (%s) is in the given directory.\n\n",fontFilePath);
        printf("If the font  cannot be found, type in a terminal:\n  fc-list | grep "".ttf"" ");
        printf("and choose one of the listed fonts. Include the full path in SDL2_UI.cpp\n");
        exit(1);
    }
}

void SDL2_UI::DestroyFontStuff() {
    TTF_Quit();
}

/// Clears the draw buffer (black).
void SDL2_UI::clear()
{
    setDrawColor(0, 0, 0, 255);
    SDL_RenderClear(renderer);
    setDrawColor(255, 255, 255, 255);
}

/// Presents the draw buffer to the screen.
void SDL2_UI::present()
{
    SDL_RenderPresent(renderer);
}

/// Draw pixel, coordinate (0,0) is the top-left pixel
void SDL2_UI::drawPixel(int x, int y)
{
    SDL_RenderDrawPoint(renderer, x, y);
}

/**
 *  Draw a rectangle with top left coordinate (x1, y1) and bottom right coordinate (x2,y2)
 *  Coordinate (0,0) is the top-left pixel
 *  The most recent color is used for drawin.
*/
void SDL2_UI::drawRectangle(int x1, int y1, int x2, int y2)
{
  SDL_Rect r;
  r.x = x1; r.y = y1; r.w = x2-x1; r.h = y2-y1;
  SDL_RenderFillRect(renderer, &r);
}

void SDL2_UI::setDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    SDL_SetRenderDrawColor(renderer, r, g, b, a);
}

void SDL2_UI::drawText(std::string s, int x_center, int y_center)
{ 
    // This is a very basic and non-optimized implementation. It can only use one font 
    // and font size (specified in InitializeFontStuff).
    // To mimic old-fashioned computers, the text is purposely made blocky (set amout
    // with `blockiness` in InitializeFontStuff).

    // Pixels from our text
    if (s.length()==0) {
        // Nothing to write, so just return
        return;
    }
    auto surface = TTF_RenderText_Solid(theFont,s.c_str(),{255,255,255,255});
    if (surface == nullptr) {
        std::cout<<"Error: no surface created.";
    }
    // Setup the texture
    auto texture = SDL_CreateTextureFromSurface(renderer,surface);
    
    // Create a rectangle to draw on
    SDL_Rect coordsToDraw;
    coordsToDraw.w = surface->w * blockiness;
    coordsToDraw.h = surface->h * blockiness;
    coordsToDraw.x = x_center - coordsToDraw.w/2;
    coordsToDraw.y = y_center - coordsToDraw.h/2;

    // Render our text on a rectangle
    SDL_RenderCopy(renderer,texture,NULL,&coordsToDraw);
    if(texture != nullptr){
        SDL_DestroyTexture(texture);
    }

    // Free the surface (We are done with it after we have uploaded to the texture)
    SDL_FreeSurface(surface);

}

/**
 * @brief Process SDL events (such as mouse clicks on the window and 'close' button). You should call this function regularly (a number of times per second)
 * 
 * @return true if the user has clicked the 'close' button on the top right; false otherwise
 */
bool SDL2_UI::processEvents() {
    SDL_Event event;
    bool quit = false;
    while (SDL_PollEvent(&event)) {
        // quit button
        if (event.type == SDL_QUIT) {
            quit = true;
        }
    }
    return quit;
}