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
// ---------------------------------------------------------

struct Intersection
{
  vec3 position;
  float distance;
  int triangleIndex;
};

// GLOBAL VARIABLES
const int SCREEN_WIDTH = 100;//512;
const int SCREEN_HEIGHT = 100;//512;
SDL_Surface* screen;
vector<Triangle> triangle;
float focalLength=SCREEN_WIDTH;
vec3 cameraPos(0, 0, -3);//0, 0, -3

mat3 R = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);//rotation matrix of the camera
float yaw = 0; //store the angle which the camera should be rotated
int t;

vec3 lightPos(0, -0.5, -0.7);//-0.5, -0.7);

//power P: energy per time of the emitted light for each color component
//Each component has the unit W = J/s
vec3 lightColor = 14.f * vec3(1, 1, 1);
vec3 indirectLight = 0.5f * vec3(1,1,1);


// ---------------------------------------------------------
// FUNCTION DECLARATIONS

bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles, Intersection& closestIntersection );
void Draw();
void Update();
void UpdateRY();
vec3 DirectLight(const Intersection& i);
// ---------------------------------------------------------


// FUNCTION DEFINITIONS
int main( int argc, char* argv[] )
{
//To fill this vector with some triangles representing a 3D model you can use the function:
  LoadTestModel(triangle );
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
  vec3 right( R[0][0], R[0][1], R[0][2] );
  vec3 down( R[1][0], R[1][1], R[1][2] );
  vec3 forward( R[2][0], R[2][1], R[2][2] );

  if( SDL_MUSTLOCK(screen) ) SDL_LockSurface(screen);
  for( int y=0; y<SCREEN_HEIGHT; ++y )
  {
    for( int x=0; x<SCREEN_WIDTH; ++x )
    {
      //normalised vector from origin to image-plane
      vec3 d(x - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength);
      //d = R * d;
      d.x = right.x * d.x+right.y * d.y+right.z * d.z;
      d.y = down.x * d.x+down.y * d.y+down.z * d.z;
      d.z = forward.x * d.x+forward.y * d.y+forward.z * d.z;

      //d = normalize(d);
      Intersection is;
      bool a = ClosestIntersection(cameraPos, d, triangle, is);

      if (a) {
        vec3 D = DirectLight(is);
        vec3 p = triangle[is.triangleIndex].color;
        vec3 R = p * (D +indirectLight);
        PutPixelSDL( screen, x, y, R);
      }else{
        PutPixelSDL( screen, x, y, vec3(0, 0, 0));
      }

    }

  }
  if( SDL_MUSTLOCK(screen) ) SDL_UnlockSurface(screen);
  SDL_UpdateRect( screen, 0, 0, 0, 0 );
}


bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles,  Intersection& closestIntersection ){
  closestIntersection.distance= 0;
  int size = triangles.size();
  int boolFlag = 0;
  vec3 v0, v1, v2, e1, e2, b, r;
  mat3 A, A1, A2, A3;
  float t, u, v;
  float distance = 0;

  for (int i = 0;i<size;i++){
  //Intersection coputation
    v0 = triangles[i].v0;
    v1 = triangles[i].v1;
    v2 = triangles[i].v2;
    //Compute two vectors in plane
    e1 = v1 - v0;
    e2 = v2 - v0;

    b = start - v0;
    A=mat3( -dir, e1, e2 );
    A1=mat3(b, e1, e2);
    A2=mat3(-dir, b, e2);
    A3=mat3(-dir, e1, b);
    t = determinant(A1)/determinant(A);
    //Inside-Outside test
    // If the point lies on the line, it would be considered as it's inside
    if (t <0 || t > std::numeric_limits<float>::max()) continue;
    u = determinant(A2)/determinant(A);
    if (u <0|| u > std::numeric_limits<float>::max()) continue;
    v = determinant(A3)/determinant(A);
    if (v <0|| v > std::numeric_limits<float>::max()) continue;
    if (u+v >1) continue;

    //All points on plane that triangle lies in
    r = start + t * dir;
    distance = sqrt((start.x - r.x)*(start.x - r.x)+(start.y - r.y)*(start.y - r.y)+(start.z - r.z)*(start.z-r.z));
    if ((distance < closestIntersection.distance) || (closestIntersection.distance == 0))
    {
        closestIntersection.position=r;
        closestIntersection.distance = distance;
        closestIntersection.triangleIndex = i;
        boolFlag = 1;
    }
  }
  if (boolFlag ==1)return true;
  return false;
  //float m = std::numeric_limits<float>::max();
}

void Update(){
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

  // Move camera forward
  if( keystate[SDLK_UP]) { cameraPos+=forward;//cameraPos.z++;
  }
  // Move camera backward
  if( keystate[SDLK_DOWN] ) { cameraPos-=forward;//cameraPos.z--;
  }
  if( keystate[SDLK_LEFT] )
  {
    // Move camera to the left
    //cameraPos.x--;
    yaw += 1;
    UpdateRY();
  }
  if( keystate[SDLK_RIGHT] )
  {
    // Move camera to the right
    //cameraPos.x++;
    yaw -= 1;
    UpdateRY();
  }
  if( keystate[SDLK_w] ) { lightPos+=forward; }
  if( keystate[SDLK_s] ) { lightPos-=forward; }
  if( keystate[SDLK_a] ) { lightPos-=right; }
  if( keystate[SDLK_d] ) { lightPos+=right; }
  if( keystate[SDLK_q] ) { lightPos-=down; }
  if( keystate[SDLK_e] ) { lightPos+=down; }

  cout<<lightPos.x <<" "<<lightPos.y<<" "<<lightPos.z<<endl;
}

void UpdateRY(){
  R = mat3(
    cos(yaw * M_PI / 180), 0 ,  sin(yaw * M_PI / 180), //R[0][0], R[0][2]
    0 , 1, 0,
    -sin(yaw * M_PI / 180), 0 , cos(yaw * M_PI / 180)
  );
}

//Takes an intersection, which gives the position
//where we want to know the direct illumination
//as well as the index to the triangle that should get illuminated
//Return the resulting direct illumination described by D = (P max(r, n, 0))/4 pi r^2
vec3 DirectLight(const Intersection& i) {

//Simulate shadows if another surface intersects
// the ray from the light source to the surface

  //When illuminating a surface point
  //  we cast another ray from it to the light source
  //THen we check the distance to the closest intersecting surface
  //If closer than the light source, the surface is in shadow
  // and does not receive any direct illumination from the light source

  vec3 D;

  Triangle tri = triangle[i.triangleIndex];

  vec3 n = tri.normal;//surface point normal
  vec3 ur = lightPos - i.position;//direction vector from surface to light source
  float r = length(ur); //Distance between surface and light source

  ur = normalize(ur); //get unit vector

  Intersection is;

  bool a = ClosestIntersection(i.position, ur, triangle, is);
  if (a && (is.distance < r)&& (is.distance > 0.001)){
    D = vec3(0, 0, 0);
  }
  else{
    float A = 4 * M_PI * r * r;
    vec3 B = lightColor/A;
    float nr = dot(n, ur);
    D = B * max(nr, 0.f);
  }

  return D;
}
