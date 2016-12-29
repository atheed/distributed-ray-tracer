/***********************************************************
    Some boilerplate code written by Jack Wang, University
    of Toronto.
    
    implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"

void PointLight::shade( Ray3D& ray ) {
  // TODO: implement this function to fill in values for ray.col 
  // using phong shading.  Make sure your vectors are normalized, and
  // clamp colour values to 1.0.
  //
  // It is assumed at this point that the intersection information in ray 
  // is available.  So be sure that traverseScene() is called on the ray 
  // before this function. 

  Material* material = ray.intersection.mat;      // materials at intersection point

  /* --------------------
   * shading mode -- COMMENT OUT APPROPRIATELY FOR DIFFERENT SHADING BEHAVIOURS
   * --------------------
   */
  //int shading_mode = 1;		// scene signature
  //int shading_mode = 2;		// no specular (only ambient + diffuse)
  int shading_mode = 3;		// with specular (ambient + diffuse + specular)

  Vector3D v = -ray.dir;						// reflected ray sent out from intersection pt
  v.normalize();

  Vector3D n = ray.intersection.normal;		// normal at intersection pt
  n.normalize();

  Vector3D l = _pos - ray.intersection.point;	// vector in the direction from intsctPt to the light source
  l.normalize();

  Vector3D r = -l - 2 * ((-l).dot(n)) * n;	// the perfect mirror direction given n and l
  r.normalize();

  Colour amb = _col_ambient * material->ambient;		// i_a * k_a
  Colour diff = fmax(l.dot(n), 0.0) * _col_diffuse * material->diffuse;		// max(l DOT n, 0) * i_d * k_d
  Colour spec = pow(fmax(r.dot(v), 0.0), material->specular_exp) * _col_specular * material->specular;		// (max(r DOT n, 0))^alpha * i_s * k_s

  switch(shading_mode) {
    case 1:				// scene signature
      ray.col = material->diffuse;
      break;
    case 2:				// ambient + diffuse
      ray.col = amb + diff;
      break;
    case 3:				// ambient + diffuse + specular
      ray.col = amb + diff + spec;
      break;
    default:
      break;
  }

  ray.col.clamp();
}
