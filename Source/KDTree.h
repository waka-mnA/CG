#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <glm/glm.hpp>
#include <vector>
#include "BoundingBox.h"

class KDTree
{
public:
  KDTree* left;
  KDTree* right;
  vector<Triangle*> triangles;
  BoundingBox box;

  KDTree();
  KDTree* build(vector<Triangle*> tri, int depth) ;
  vec3 get_midPt(Triangle t);

};

#endif
