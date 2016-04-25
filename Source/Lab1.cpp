#include <SDL/SDL.h>
#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include "BoundingBox.h"
#include "KDTree.h"
#include <limits>
#include <math.h>
#include <thread>
#include <future>


using namespace std;
using glm::vec3;
using glm::mat3;


struct Intersection
{
  vec3 position;
  float distance;
  int triangleIndex;
};

// ---------------------------------------------------------
// GLOBAL VARIABLES
const int SCREEN_WIDTH = 512;
const int SCREEN_HEIGHT = 512;

SDL_Surface* screen;
vector<Triangle> triangle;

float focalLength=SCREEN_WIDTH;
vec3 cameraPos(0, 0, -3);
mat3 R = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);//rotation matrix of the camera
float yaw = 0; //store the angle which the camera should be rotated
int t;

vec3 lightPos(0, -0.5, -0.7);
vec3 lightColor = 14.f * vec3(1, 1, 1);
vec3 indirectLight = 0.5f * vec3(1,1,1);

float ns = 4;//Anti-aliasing variable

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

  vector<std::thread> threads;
  int num = 50;   //number of operations in one thread

  if( SDL_MUSTLOCK(screen) ) SDL_LockSurface(screen);

  for (int y2 = 0;y2<SCREEN_HEIGHT;y2=y2+num){
    threads.push_back(std::thread([](int y2, int num){
      for (int y = y2;y<y2+num;y++){
        //Return if y exceeds screen boundary
        if (y >= SCREEN_HEIGHT) return;
        for (int x2 = 0;x2<SCREEN_WIDTH;x2++){
          vec3 color(0, 0, 0);
          //Anti-aliasing
          for (int i = -(ns)/2; i< (ns)/2; i++){
            for (int j = -(ns)/2; j< (ns)/2; j++){
              float AAx = x2+i/ns;
    					float AAy = y+j/ns;
              vec3 d = vec3(AAx - SCREEN_WIDTH/2, AAy - SCREEN_HEIGHT/2, focalLength)*R;
              Intersection is;
              if (ClosestIntersection(cameraPos, d, triangle, is)) {
                vec3 D = DirectLight(is);
                vec3 p = triangle[is.triangleIndex].color;
                color += p * (D +indirectLight);
              }
            }
          }
          PutPixelSDL( screen, x2, y, color/(ns*ns));
        }
      }
    }, y2, num));
  }

  for (std::thread &th:threads){
    th.join();
  }
  if( SDL_MUSTLOCK(screen) ) SDL_UnlockSurface(screen);
  SDL_UpdateRect( screen, 0, 0, 0, 0 );
}


bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles,  Intersection& closestIntersection ){
  closestIntersection.distance= 0;
  int boolFlag = 0;
  vec3 v0, v1, v2, e1, e2, b, r, p, q;
  float t, u, v;
  float distance = 0;

  for (int i = 0;i<triangles.size();i++){
    v0 = triangles[i].v0;
    //Compute two vectors in plane
    e1 = triangles[i].v1 - v0;
    e2 = triangles[i].v2 - v0;
    b = start - v0;

    //determinant(mat3(A, B, C))= -dot(cross(A, C), B)  = -dot(cross(C, B), A)
    //speed up calculation by precomputing cross products
    p = cross(dir, e2);
    q = cross(b, e1);

    //Compute barycentric coordinate
    //http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
    //Inside-Outside test
    // If the point lies on the line, it would be considered as it's inside
    float a = 1/(dot(p, e1)) ;
    t = a* dot(q, e2);
    if (t < 0 || t > std::numeric_limits<float>::max()) continue;
    u = a* dot(p, b);
    if (u < 0|| u > std::numeric_limits<float>::max()) continue;
    v = a* dot(q, dir);
    if (v < 0|| v > std::numeric_limits<float>::max()) continue;
    if (u+v >1) continue;

    //All points on plane that triangle lies in
    r = start + t * dir;
    distance = sqrt((start.x - r.x)*(start.x - r.x)+(start.y - r.y)*(start.y - r.y)+(start.z - r.z)*(start.z-r.z));
    if (distance < closestIntersection.distance ||closestIntersection.distance== 0)
    {
        closestIntersection.position=r;
        closestIntersection.distance = distance;
        closestIntersection.triangleIndex = i;
        boolFlag = 1;
    }
  }
  if (boolFlag ==1)return true;
  return false;
}

void Update(){
  vec3 right( R[0][0], R[0][1], R[0][2] );
  vec3 down( R[1][0], R[1][1], R[1][2] );
  vec3 forward( R[2][0], R[2][1], R[2][2] );
  // Compute frame time:
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  cout << "Render time: " << dt << " ms." << endl;
  Uint8* keystate = SDL_GetKeyState( 0 );

  // Move camera forward
  if( keystate[SDLK_UP]) cameraPos+=forward;
  // Move camera backward
  if( keystate[SDLK_DOWN] ) cameraPos-=forward;
  //Rotate camera
  if( keystate[SDLK_LEFT] )
  {
    yaw -= 1;
    UpdateRY();
  }
  if( keystate[SDLK_RIGHT] )
  {
    yaw += 1;
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
  R = mat3(
    cos(yaw * M_PI / 180), 0 ,  sin(yaw * M_PI / 180), //R[0][0], R[0][2]
    0 , 1, 0,
    -sin(yaw * M_PI / 180), 0 , cos(yaw * M_PI / 180)
  );
}

//Takes an intersection, where we want to know the direct illumination
//Return the resulting direct illumination described by D = (P max(r, n, 0))/4 pi r^2
vec3 DirectLight(const Intersection& i) {
  vec3 D;
  Intersection is;
  Triangle tri = triangle[i.triangleIndex];
  vec3 n = tri.normal;  //surface point normal
  vec3 ur = lightPos - i.position;//direction vector from surface to light source
  float r = length(ur); //Distance between surface and light source
  ur = normalize(ur);   //get unit vector
  bool a = ClosestIntersection(i.position, ur, triangle, is);
  if (a && (is.distance < r)&& (is.distance > 0.001)){
    D = vec3(0, 0, 0);
  }
  else{
    vec3 B = lightColor/(float)(4 * M_PI * r * r);
    float nr = dot(n, ur);
    D = B * max(nr, 0.f);
  }
  return D;
}
