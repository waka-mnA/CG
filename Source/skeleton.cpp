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
int t;
vector<vec3> stars( 1000 );//1000 stars represented by x, y, z coodinates

// ---------------------------------------------------------
// FUNCTION DECLARATIONS
void Draw();
void Update();
// ---------------------------------------------------------
// FUNCTION DEFINITIONS
int main( int argc, char* argv[] )
{

//Generate randomised 3D star location
	for (int i = 0;i<stars.size();i++){
		stars[i].x = float(rand()) / float(RAND_MAX) *2 -1;//-1~1
		stars[i].y = float(rand()) / float(RAND_MAX) *2 -1;//-1~1
		stars[i].z = float(rand()) / float(RAND_MAX) ;//0~1
	}

	screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
	t = SDL_GetTicks();	// Set start value for timer.

	while( NoQuitMessageSDL() )
	{
		Update();//Update star location in next frame
		Draw();
	}
	SDL_SaveBMP( screen, "screenshot.bmp" );
	return 0;
}
void Draw()
{
		//Fill the screen with black
		SDL_FillRect( screen, 0, 0 );

		//focal length of camera
		float f = SCREEN_HEIGHT/2;

		if( SDL_MUSTLOCK(screen) ) SDL_LockSurface(screen);
		for( size_t s=0; s<stars.size(); ++s )
		{
				vec3 star=stars[s];
				//int u = (SCREEN_HEIGHT*star.x+SCREEN_WIDTH*star.z)/(2*star.z);
				//int v= SCREEN_HEIGHT*(star.y+star.z)/(2*star.z);
				//Convert 3D point (x, y, z) projection onto 2D plane at (u, v)
				int u = f*star.x/star.z + SCREEN_WIDTH/2;
				int v = f*star.y/star.z + SCREEN_HEIGHT/2;
				//fade the brightness
				vec3 color = 0.2f* vec3(1, 1, 1)/(star.z*star.z);
				PutPixelSDL( screen,u, v, color );
		}
		
		if( SDL_MUSTLOCK(screen) ) SDL_UnlockSurface(screen);
		SDL_UpdateRect( screen, 0, 0, 0, 0 );
}
void Update()
{
	float v = 0.001;
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	for (int s = 0;s<stars.size();s++){
		vec3 star = stars[s];
		stars[s].z = star.z - v*dt;

		if( stars[s].z <= 0 )
		stars[s].z += 1;
		if( stars[s].z > 1 )
		stars[s].z -= 1;

	}
	cout << "Render time: " << dt << " ms." << endl;
}


