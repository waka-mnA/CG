#include <SDL/SDL.h>
#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include "SDLauxiliary.h"
using namespace std;
using glm::vec3;
// ---------------------------------------------------------
// GLOBAL VARIABLES
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;
SDL_Surface* screen;

// ---------------------------------------------------------
// FUNCTION DECLARATIONS
void Draw();
// ---------------------------------------------------------
// FUNCTION DEFINITIONS
int main( int argc, char* argv[] )
{
screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
while( NoQuitMessageSDL() )
{
Draw();
}
SDL_SaveBMP( screen, "screenshot.bmp" );
return 0;
}
void Draw()
{
if( SDL_MUSTLOCK(screen) )
SDL_LockSurface(screen);
for( int y=0; y<SCREEN_HEIGHT; ++y )
{
for( int x=0; x<SCREEN_WIDTH; ++x )
{
vec3 color(0,0,1);
PutPixelSDL( screen, x, y, color );
}
}
if( SDL_MUSTLOCK(screen) )
SDL_UnlockSurface(screen);
SDL_UpdateRect( screen, 0, 0, 0, 0 );
}
