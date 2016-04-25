#include "BoundingBox.h"
#include "TestModel.h"
#include <SDL/SDL.h>
#include <glm/glm.hpp>
#include <vector>
#include "SDLauxiliary.h"
#include <limits>
#include <math.h>

BoundingBox::BoundingBox getBoundingBox(Triangle triangle){
  BoundingBox b;
  int maxX = 0;
  int minX = std::numeric_limits<float>::max();
  int maxY= 0;
  int minY = std::numeric_limits<float>::max();
  int maxZ= 0;
  int minZ = std::numeric_limits<float>::max();

    if (maxX<triangle.v0.x) maxX = triangle.v0.y;
    if (maxX<triangle.v1.x) maxX = triangle.v1.y;
    if (maxX<triangle.v2.x) maxX = triangle.v2.y;
    if (minX>triangle.v0.x) minX = triangle.v0.y;
    if (minX>triangle.v1.x) minX = triangle.v1.y;
    if (minX>triangle.v2.x) minX = triangle.v2.y;

    if (maxY<triangle.v0.y) maxX = triangle.v0.y;
    if (maxY<triangle.v1.y) maxX = triangle.v1.y;
    if (maxY<triangle.v2.y) maxX = triangle.v2.y;
    if (minY>triangle.v0.y) minX = triangle.v0.y;
    if (minY>triangle.v1.y) minX = triangle.v1.y;
    if (minY>triangle.v2.y) minX = triangle.v2.y;

    if (maxZ<triangle.v0.z) maxZ = triangle.v0.z;
    if (maxZ<triangle.v1.z) maxZ = triangle.v1.z;
    if (maxZ<triangle.v2.z) maxZ = triangle.v2.z;
    if (minZ>triangle.v0.z) minZ = triangle.v0.z;
    if (minZ>triangle.v1.z) minZ = triangle.v1.z;
    if (minZ>triangle.v2.z) minZ = triangle.v2.z;


  b.start = vec3(minX, minY, minZ);
  b.end = vec3(maxX, maxY, maxZ);
  return b;
}

BoundingBox::void addTriangle(Triangle tri)
{
  int maxX = this.end.x;
  int minX = this.start.x;
  int maxY= this.end.y;
  int minY = this.start.y;
  int maxZ= this.end.z;
  int minZ = this.start.z;

  if (maxX<tri.v0.x) maxX = tri.v0.y;
  if (maxX<tri.v1.x) maxX = tri.v1.y;
  if (maxX<tri.v2.x) maxX = tri.v2.y;
  if (minX>tri.v0.x) minX = tri.v0.y;
  if (minX>tri.v1.x) minX = tri.v1.y;
  if (minX>tri.v2.x) minX = tri.v2.y;

  if (maxY<tri.v0.y) maxX = tri.v0.y;
  if (maxY<tri.v1.y) maxX = tri.v1.y;
  if (maxY<tri.v2.y) maxX = tri.v2.y;
  if (minY>tri.v0.y) minX = tri.v0.y;
  if (minY>tri.v1.y) minX = tri.v1.y;
  if (minY>tri.v2.y) minX = tri.v2.y;

  if (maxZ<tri.v0.z) maxZ = tri.v0.z;
  if (maxZ<tri.v1.z) maxZ = tri.v1.z;
  if (maxZ<tri.v2.z) maxZ = tri.v2.z;
  if (minZ>tri.v0.z) minZ = tri.v0.z;
  if (minZ>tri.v1.z) minZ = tri.v1.z;
  if (minZ>tri.v2.z) minZ = tri.v2.z;
  this.start = vec3(minX,minY, minZ );
  this.end = vec3(maxX,maxY, maxZ );
}

BoundingBox::int logestAxis(){
  int x = this.end.x - this.start.x;
  int y = this.end.y - this.start.y;
  int z = this.end.z - this.start.z;
  int flag = 0;
  if (x < y) {flag = 1;x = y}
  if (x < z) flag = 2;

  return flag;
}
