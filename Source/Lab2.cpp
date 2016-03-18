#include <SDL/SDL.h>
#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <limits>
#include <math.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::ivec2;
using glm::vec2;
// ---------------------------------------------------------

struct Intersection
{
  vec3 position;
  float distance;
  int triangleIndex;
};

// GLOBAL VARIABLES
const int SCREEN_WIDTH = 512;
const int SCREEN_HEIGHT = 512;
SDL_Surface* screen;
vector<Triangle> triangles;
float focalLength=SCREEN_WIDTH;
vec3 cameraPos( 0, 0, -3.001 );

mat3 R = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);//rotation matrix of the camera

mat3 RY = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);//rotation matrix of the camera
mat3 RX = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);//rotation matrix of the camera
float yaw = 0; //store the angle which the camera should be rotated
float xaw = 0; //store the angle which the camera should be rotated
int t;

vec3 lightPos(0, -0.5, -0.7);//-0.5, -0.7);

//power P: energy per time of the emitted light for each color component
//Each component has the unit W = J/s
vec3 lightColor = 14.f * vec3(1, 1, 1);
vec3 indirectLight = 0.5f * vec3(1,1,1);

vec3 currentColor;

// ---------------------------------------------------------
// FUNCTION DECLARATIONS

//bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles, Intersection& closestIntersection );
void Draw();
void VertexShader(const vec3 & v, ivec2& p);
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec3>& vertices );
void ComputePolygonRows( const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels );
void DrawRows( const vector<ivec2>& leftPixels, const vector<ivec2>& rightPixels );
void DrawPolygon( const vector<vec3>& vertices );
void Update();
void UpdateRY();
void UpdateRX();
//vec3 DirectLight(const Intersection& i);
// ---------------------------------------------------------


// FUNCTION DEFINITIONS
int main( int argc, char* argv[] )
{

//To fill this vector with some triangles representing a 3D model you can use the function:
  LoadTestModel(triangles );
  screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );

  t = SDL_GetTicks();

  while( NoQuitMessageSDL() )
  {
    Update();
    Draw();
  }
  SDL_SaveBMP( screen, "screenshot.bmp" );

  return 0;
}

void Draw()
{
  SDL_FillRect( screen, 0, 0 );

  if( SDL_MUSTLOCK(screen) ) SDL_LockSurface(screen);

  for( int i=0; i<triangles.size(); ++i )
  {
    currentColor = triangles[i].color;
    vector<vec3> vertices(3);
    vertices[0] = triangles[i].v0;
    vertices[1] = triangles[i].v1;
    vertices[2] = triangles[i].v2;

    //DrawPolygonEdges(vertices);
    DrawPolygon(vertices);
  }
  if ( SDL_MUSTLOCK(screen) )
    SDL_UnlockSurface(screen);
  SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

//3D position of a vertex v
//compute its 2D image position and store it in the integer vector p
void VertexShader(const vec3 & v, ivec2& p){
  vec3 position = (v - cameraPos)* RY * RX;

  p.x = focalLength* position.x/position.z + SCREEN_WIDTH/2;
  p.y = focalLength* position.y/position.z + SCREEN_HEIGHT/2;
}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result )
{
  int N = result.size();
  vec2 step = vec2(b-a) / float(max(N-1,1));
  vec2 current( a );
  for( int i=0; i<N; ++i )
  {
    result[i] = current;
    current += step;
  }
}

void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color ){
  ivec2 delta = glm::abs( a - b );
  int pixels = glm::max( delta.x, delta.y ) + 1;

  vector<ivec2> line( pixels );
  Interpolate( a, b, line );
  for (int i = 0;i<pixels;i++){
    PutPixelSDL( surface, line[i].x, line[i].y, color );
  }

}

void DrawPolygonEdges( const vector<vec3>& vertices )
{
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices( V );
  for( int i=0; i<V; ++i )
  {
    VertexShader( vertices[i], projectedVertices[i] );
  }
  // Loop over all vertices and draw the edge from it to the next vertex:
  for( int i=0; i<V; ++i )
  {
    int j = (i+1)%V; // The next vertex
    vec3 color( 1, 1, 1 );
    DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
  }
}

void ComputePolygonRows( const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels )
{
  int min = numeric_limits<int>::max();
  int max = 0;
  for (int i = 0;i<vertexPixels.size();i++){
    if (min > vertexPixels[i].y) min = vertexPixels[i].y;
    if (max < vertexPixels[i].y) max = vertexPixels[i].y;
  }
  int ROWS = max - min + 1;

  leftPixels = vector<ivec2>( ROWS );
  rightPixels = vector<ivec2>( ROWS );

  for (int i = 0;i<ROWS;++i){
    leftPixels[i].x = +numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
    leftPixels[i].y = min + i;
    rightPixels[i].y = min + i;
  }

  for (int i = 0;i<vertexPixels.size();i++){
    int j = (i+1)%vertexPixels.size(); // The next vertex
    ivec2 delta = glm::abs( vertexPixels[i] - vertexPixels[j]);
    int size = glm::max( delta.x, delta.y ) + 1;
    vector<ivec2> line(size);
    Interpolate(  vertexPixels[i], vertexPixels[j], line );

    for (int k = 0;k<ROWS;k++){
      for (int n = 0;n<size;n++){
        if (line[n].y == min+k){
          if (leftPixels[k].x > line[n].x) leftPixels[k].x = line[n].x;
          if (rightPixels[k].x < line[n].x) rightPixels[k].x = line[n].x;
        }
      }
    }
  }
}

void DrawRows( const vector<ivec2>& leftPixels, const vector<ivec2>& rightPixels ){
  for (int i = 0;i<leftPixels.size();i++){

    ivec2 delta = glm::abs( leftPixels[i] - rightPixels[i] );
    int pixels = glm::max( delta.x, delta.y ) + 1;
    vector<ivec2> result( pixels );

    Interpolate( leftPixels[i], rightPixels[i], result );
    for (int i = 0;i<pixels;i++){
      PutPixelSDL( screen, result[i].x, result[i].y, currentColor );
    }
  }
}

void DrawPolygon( const vector<vec3>& vertices )
{
  int V = vertices.size();
  vector<ivec2> vertexPixels( V );
  for( int i=0; i<V; ++i ) VertexShader( vertices[i], vertexPixels[i] );
  vector<ivec2> leftPixels;
  vector<ivec2> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  DrawRows( leftPixels, rightPixels );
}
void Update(){
  R = RY * RX;
  vec3 right( R[0][0], R[0][1], R[0][2] );
  vec3 down( R[1][0], R[1][1], R[1][2] );
  vec3 forward( R[2][0], R[2][1], R[2][2] );
  // Compute frame time:
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);//time);
  //time = t2;
  t = t2;
  cout << "Render time: " << dt << " ms." << endl;
  Uint8* keystate = SDL_GetKeyState( 0 );
  //PRESS SHIFT& ARROW to rotate on x-axis
  if ((keystate[SDLK_RSHIFT] || keystate[SDLK_LSHIFT]) &&  keystate[SDLK_UP]){
    xaw -= 1;
    UpdateRX();
  }
  else if( keystate[SDLK_UP]) {
    cameraPos+=forward;//cameraPos.z++;
  }
  if ((keystate[SDLK_RSHIFT] || keystate[SDLK_LSHIFT]) &&  keystate[SDLK_DOWN]){
    xaw += 1;
    UpdateRX();
  }
  else if( keystate[SDLK_DOWN] ) {
    cameraPos-=forward;//cameraPos.z--;
  }
  if( keystate[SDLK_LEFT] )
  {
    yaw += 1;
    UpdateRY();
  }
  if( keystate[SDLK_RIGHT] )
  {
    yaw -= 1;
    UpdateRY();
  }

  if( keystate[SDLK_w] ) { lightPos+=forward; }
  if( keystate[SDLK_s] ) { lightPos-=forward; }
  if( keystate[SDLK_a] ) { lightPos-=right; }
  if( keystate[SDLK_d] ) { lightPos+=right; }
  if( keystate[SDLK_q] ) { lightPos-=down; }
  if( keystate[SDLK_e] ) { lightPos+=down; }

}

void UpdateRY(){
  RY = mat3(
    cos(yaw * M_PI / 180), 0 ,  sin(yaw * M_PI / 180), //R[0][0], R[0][2]
    0 , 1, 0,
    -sin(yaw * M_PI / 180), 0 , cos(yaw * M_PI / 180)
  );
}
void UpdateRX(){
  RX = mat3(
    1,  0 , 0 , //R[0][0], R[0][2]
    0 , cos(xaw * M_PI / 180), -sin(xaw * M_PI / 180),
    0, sin(xaw * M_PI / 180) , cos(xaw * M_PI / 180)
  );
}
