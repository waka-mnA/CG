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

struct Intersection{
  vec3 position;
  float distance;
  int triangleIndex;
};

struct Pixel{
  int x;
  int y;
  float zinv;
  vec3 illumination;
};

struct Vertex{
  vec3 position;
  vec3 normal;
  vec3 reflectance;
};

// GLOBAL VARIABLES
const int SCREEN_WIDTH = 512;
const int SCREEN_HEIGHT = 512;
SDL_Surface* screen;
vector<Triangle> triangles;
float focalLength = SCREEN_WIDTH;
vec3 cameraPos( 0, 0, -3.001 );

mat3 R = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);//rotation matrix of the camera
mat3 RY = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);//rotation matrix of the camera
mat3 RX = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);//rotation matrix of the camera
float yaw = 0; //store the angle which the camera should be rotated
float xaw = 0; //store the angle which the camera should be rotated
int t;

vec3 lightPos(0, -0.5, -0.7);
vec3 lightPower = 14.f * vec3(1, 1, 1);
vec3 indirectLightPowerPerArea = 0.5f * vec3(1, 1, 1);

vec3 currentColor;

//1.7
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

// ---------------------------------------------------------
// FUNCTION DECLARATIONS

//bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles, Intersection& closestIntersection );
void Draw();
void VertexShader(const Vertex & v, Pixel& p);
void PixelShader(const Pixel& p);
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec3>& vertices );
void ComputePolygonRows( const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels );
void DrawRows( const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels );
void DrawPolygon( const vector<Vertex>& vertices );
void Update();
void UpdateRY();
void UpdateRX();
// ---------------------------------------------------------


// FUNCTION DEFINITIONS
int main( int argc, char* argv[] ){

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

void Draw(){
  //Clear depthBuffer
  for( int y=0; y<SCREEN_HEIGHT; ++y ){
    for( int x=0; x<SCREEN_WIDTH; ++x ) depthBuffer[y][x] = 0;
  }

  SDL_FillRect( screen, 0, 0 );

  if( SDL_MUSTLOCK(screen) ) SDL_LockSurface(screen);

    for( int i=0; i<triangles.size(); ++i )
  {
    currentColor = triangles[i].color;
    vector<Vertex> vertices(3);

    vertices[0].position = triangles[i].v0;
    vertices[0].normal = triangles[i].normal;
    vertices[0].reflectance = triangles[i].color;

    vertices[1].position = triangles[i].v1;
    vertices[1].normal = triangles[i].normal;
    vertices[1].reflectance = triangles[i].color;

    vertices[2].position = triangles[i].v2;
    vertices[2].normal = triangles[i].normal;
    vertices[2].reflectance = triangles[i].color;

    DrawPolygon(vertices);
  }
  if ( SDL_MUSTLOCK(screen) )
    SDL_UnlockSurface(screen);
  SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

//3D position of a vertex v
//compute its 2D image position and store it in the integer vector p
//void VertexShader(const vec3 & v, Pixel& p){
void VertexShader(const Vertex & v, Pixel& p){
  vec3 pos = (v.position - cameraPos) * R;

  //D = lightPower * max(n * r, 0)/(4*pie*r^2)
  //R = reflectance * (D + incomingIndirectIllumination)
  vec3 ur = lightPos - v.position;  //vector from the surface to the light source
  float r = length( ur );           //Distance between surface and light source
  ur = normalize( ur );             //unit vector
  float nr = dot(  v.normal, ur );

  vec3 D = lightPower * glm::max( nr, 0.f ) /float( 4 * M_PI * r * r );

  p.illumination = v.reflectance * ( D + indirectLightPowerPerArea );
  p.zinv = 1/( pos.z );
  p.x = int(focalLength* pos.x*p.zinv) + SCREEN_WIDTH/2;
  p.y = int(focalLength* pos.y*p.zinv) + SCREEN_HEIGHT/2;
}

void PixelShader(const Pixel& p){
  int x = p.x;
  int y = p.y;
  if( p.zinv > depthBuffer[y][x] )
  {
    depthBuffer[y][x] = p.zinv;
    PutPixelSDL( screen, x, y, p.illumination );
  }
}

void Interpolate( Pixel a, Pixel b, vector<Pixel>& result )
{
  int N = result.size();
  float d = float(max(N-1, 1));

  float stepX = float(b.x - a.x)/d;
  float stepY = float(b.y - a.y)/d;
  float stepZinv = float(b.zinv - a.zinv)/d;
  vec3 stepI = (b.illumination - a.illumination)/d;

  for( int i=0; i < N; ++i )
  {
    result[i].x = a.x + stepX * i;
    result[i].y = a.y + stepY * i;
    result[i].zinv = a.zinv + stepZinv * i;
    result[i].illumination = a.illumination + stepI * float(i) ;
  }
}

/*void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color ){
  ivec2 delta = glm::abs( a - b );
  int pixels = glm::max( delta.x, delta.y ) + 1;

  vector<ivec2> line( pixels );
  Interpolate( a, b, line );
  for (int i = 0;i<pixels;i++){
    PutPixelSDL( surface, line[i].x, line[i].y, color );
  }

}

void DrawPolygonEdges( const vector<vec3>& vertices ){
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices( V );
  for( int i=0; i<V; ++i )
  {
    //VertexShader( vertices[i], projectedVertices[i] );
  }
  // Loop over all vertices and draw the edge from it to the next vertex:
  for( int i=0; i<V; ++i )
  {
    int j = (i+1)%V; // The next vertex
    vec3 color( 1, 1, 1 );
    //DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
  }
}*/

void ComputePolygonRows( const vector<Pixel>& vertexPixels,
                          vector<Pixel>& leftPixels,
                          vector<Pixel>& rightPixels ){
  int min = numeric_limits<int>::max();
  int max = 0;
  for (int i = 0;i<vertexPixels.size();i++){
    if (min > vertexPixels[i].y) min = vertexPixels[i].y;
    if (max < vertexPixels[i].y) max = vertexPixels[i].y;
  }
  int ROWS = max - min + 1;

  leftPixels = vector<Pixel>( ROWS );
  rightPixels = vector<Pixel>( ROWS );

  for (int i = 0;i<ROWS;++i){
    leftPixels[i].x = +numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
    leftPixels[i].y = min + i;
    rightPixels[i].y = min + i;
  }
  //for each vertex, find the line between next vertex and itself
  for (int i = 0;i<vertexPixels.size();i++){
    int j = (i+1)%vertexPixels.size(); // The next vertex
    int deltaX = glm::abs( vertexPixels[i].x - vertexPixels[j].x);
    int deltaY = glm::abs( vertexPixels[i].y - vertexPixels[j].y);
    int size = glm::max( deltaX, deltaY ) + 1;
    vector<Pixel> line(size);
    Interpolate(  vertexPixels[i], vertexPixels[j], line );

    for (int k = 0;k<ROWS;k++){
      for (int n = 0;n<size;n++){
        if (line[n].y == min+k){
          if (leftPixels[k].x > line[n].x) {
            leftPixels[k].x = line[n].x;
            leftPixels[k].zinv = line[n].zinv;
            leftPixels[k].illumination = line[n].illumination;
          }
          if (rightPixels[k].x < line[n].x) {
            rightPixels[k].x = line[n].x;
            rightPixels[k].zinv = line[n].zinv;
            rightPixels[k].illumination = line[n].illumination;
          }
        }
      }
    }
  }
}

void DrawRows( const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels ){
  for (int i = 0;i<leftPixels.size();i++){
    int deltaX = glm::abs( leftPixels[i].x - rightPixels[i].x );
    int deltaY = glm::abs( leftPixels[i].y - rightPixels[i].y );
    int pixels = glm::max( deltaX, deltaY ) + 1;
    vector<Pixel> result( pixels );
    Interpolate( leftPixels[i], rightPixels[i], result );

    for (int j = 0;j<pixels;j++){
      PixelShader(result[j]);
    }
  }
}

//void DrawPolygon( const vector<vec3>& vertices )
void DrawPolygon( const vector<Vertex>& vertices )
{
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i ) {
    VertexShader( vertices[i], vertexPixels[i] );
  }
  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
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
  float dt = float(t2-t);
  t = t2;
  cout << "Render time: " << dt << " ms." << endl;
  Uint8* keystate = SDL_GetKeyState( 0 );

  //PRESS SHIFT& ARROW to rotate on x-axis
  if ((keystate[SDLK_RSHIFT] || keystate[SDLK_LSHIFT]) &&  keystate[SDLK_UP]){
    xaw -= 1;
    UpdateRX();
  } else if( keystate[SDLK_UP]) {
    cameraPos+=forward;//cameraPos.z++;
  }
  if ((keystate[SDLK_RSHIFT] || keystate[SDLK_LSHIFT]) &&  keystate[SDLK_DOWN]){
    xaw += 1;
    UpdateRX();
  } else if( keystate[SDLK_DOWN] ) {
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

    R = RY * RX;
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
