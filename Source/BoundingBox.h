#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

// Defines a simple test model: The Cornel Box

#include <glm/glm.hpp>
#include <vector>

class BoundingBox{
  public:
    glm::vec3 start;
    glm::vec3 end;

  BoundingBox getBoundingBox(Triangle tri);
  void addTriangle(Triangle tri);
  int logestAxis();

};
#endif
