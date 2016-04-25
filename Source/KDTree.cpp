#include <SDL/SDL.h>
#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include "BoundingBox.h"
#include <limits>
#include <math.h>

using namespace std;
using glm::vec3;
using glm::mat3;

KDTree::KDTree* build(vector<Triangle*> tri, int depth) {
    KDtree* node = new KDTree();
    node.triangles = tri;
    node.left = NULL:
    node.right = NULL:
    node.box = new BoundingBox():

    if (tri.size() == 0) return node;
    if (tri.size() == 1) {
      node.box = getBounxingBox(tri[0]);
      node -> left = new KDTree():
      node -> right = new KDTree():
      node -> left -> triangle = vector<Triangle*>();
      node -> right -> triangle = vector<Triangle*>();
      return node;
    }

    node.box = getBoundingBox(tri[0]);
    for (int i = 1;i<tri.size();i++){
      node.box.addTriangle(tri[i]);
    }

    vec3 midpt(0, 0, 0);
    for (int i = 0;i<tri.size();i++){
      midpt  += get_midPt(tri[i]) /tri.size();
    }
    vector<Triangle*> left_tri;
    vector<Triangle*> right_tri;

    int axis = node.box.logestAxis();

    for (int i = 0;i<tri.size();i++){
      switch(axis){
        case 0:
        if (midpt.x > get_midPt(tri[i]).x) right_tri.push_back(tri[i]);
        else left_tri.push_back(tri[i]);
          break;
        case 1:
        if (midpt.y > get_midPt(tri[i]).y) right_tri.push_back(tri[i]);
        else left_tri.push_back(tri[i]);
          break;
        case 2:
        if (midpt.z > get_midPt(tri[i]).z) right_tri.push_back(tri[i]);
        else left_tri.push_back(tri[i]);
          break;
      }
    }

    if (left_tri.size()==0 && right_tri.size() >0) left_tri = right_tri;
    if (right_tri.size()==0 && left_tri.size() >0) right_tri = left_tri;

    int m = 0;
    for (int i = 0;i<left_tri.size();i++){
      for (int j = 0;j<right_tri.size();j++){
        if (left_tri[i] == right_tri[j]) m++;
      }
    }

    if ((float) m/left_tri.size() < 0.5 && float) m/right_tri.size() < 0.5){
      node.left = build(left_tri, depth+1);
      node.right = build(right_tri, depth+1);
    }
    else{
      node.left = new KDTree():
      node.right = new KDTree():
      node.left.triangle = vector<Triangle*>();
      node.right.triangle = vector<Triangle*>();
    }
    return node;
  }

KDTree::vec3 get_midPt(Triangle t){
    return (t.v0 + t.v1+ t.v2)/3;
  }
