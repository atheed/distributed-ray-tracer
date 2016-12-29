/***********************************************************
    Some boilerplate code written by Jack Wang, University
    of Toronto.

    classes defining primitives in the scene

***********************************************************/

#include "util.h"

bool quadraticEqnSolver(double A, double B, double C, double *root_1, double *root2);

// All primitives should provide a intersection function.  
// To create more primitives, inherit from SceneObject.
// Namely, you can create, Sphere, Cylinder, etc... classes
// here.
class SceneObject {
public:
  // Returns true if an intersection occured, false otherwise.
  virtual bool intersect( Ray3D&, const Matrix4x4&, const Matrix4x4& ) = 0;
};

// Example primitive you can create, this is a unit square on 
// the xy-plane.
class UnitSquare : public SceneObject {
public:
  bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
      const Matrix4x4& modelToWorld );
};

class UnitSphere : public SceneObject {
public:
  bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
      const Matrix4x4& modelToWorld );
};

class UnitCylinder : public SceneObject {
public:
  bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
      const Matrix4x4& modelToWorld ); 
};