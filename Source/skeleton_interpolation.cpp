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
//void Interpolate(float a, float b, vector<float>& result);
void Interpolate(vec3 a, vec3 b, vector<vec3>& result);
// ---------------------------------------------------------
// FUNCTION DEFINITIONS
int main( int argc, char* argv[] )
{
	//Test Interpolate float
		/*vector<float> result( 10 ); // Create a vector width 10 floats
		Interpolate( 5, 14, result ); // Fill it with interpolated values
		for( int i=0; i<result.size(); ++i )
		cout << result[i] << endl; // Print the result to the terminal
		*/

	vector<vec3> result( 4 );
	vec3 a(1,4,9.2);
	vec3 b(4,1,9.8);
	Interpolate( a, b, result );
	for( int i=0; i<result.size(); ++i )
	{
		cout << "( "
		<< result[i].x << ", "
		<< result[i].y << ", "
		<< result[i].z << " ) ";
	}
	cout<<endl;

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
		vec3 topLeft(1,0,0);     // red
		vec3 topRight(0,0,1);    // blue
		vec3 bottomRight(0,1,0); // green
		vec3 bottomLeft(1,1,0);  // yellow
		//create column interpolation
		vector<vec3> leftSide( SCREEN_HEIGHT );
		vector<vec3> rightSide( SCREEN_HEIGHT );
		Interpolate( topLeft, bottomLeft, leftSide );
		Interpolate( topRight, bottomRight, rightSide );
		
		if( SDL_MUSTLOCK(screen) ) SDL_LockSurface(screen);
		for( int y=0; y<SCREEN_HEIGHT; ++y )
		{
			//create vector for row interpolation
			vector<vec3> row(SCREEN_WIDTH);
			Interpolate(leftSide[y], rightSide[y], row);
			
			for( int x=0; x<SCREEN_WIDTH; ++x )
			{
				vec3 color=row[x];
				PutPixelSDL( screen, x, y, color );
			}
		}
		if( SDL_MUSTLOCK(screen) ) SDL_UnlockSurface(screen);
		SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

//result = filled with values linearly interpolated between a and b
/*void Interpolate(float a, float b, vector<float>& result)
{
	int size = result.size();
	if (size == 1) {
		result[0] = a;
		return;
	}

	float x = b-a;

	for (int i = 0;i<size;i++){
		result[i]=a+ x/(float)(size-1)*i;
	}
}*/

void Interpolate(vec3 a, vec3 b, vector<vec3>& result)
{
	int size = result.size();
	float re = b.x - a.x;
	float gr = b.y - a.y;
	float bl = b.z - a.z;
	
	if (size == 1) {
		result[0] = a;
		return;
	}

	for (int i = 0;i<size;i++){
		result[i].x=a.x+ re/(float)(size-1)*i;
		result[i].y=a.y+ gr/(float)(size-1)*i;
		result[i].z=a.z+ bl/(float)(size-1)*i;
	}
}
