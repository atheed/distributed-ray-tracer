/***********************************************************
    Some boilerplate code written by Jack Wang, University
    of Toronto.

    implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {
  // TODO: implement intersection code for UnitSquare, which is
  // defined on the xy-plane, with vertices (0.5, 0.5, 0), 
  // (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
  // (0, 0, 1).
  //
  // Your goal here is to fill ray.intersection with correct values
  // should an intersection occur.  This includes intersection.point, 
  // intersection.normal, intersection.none, intersection.t_value.   
  //
  // HINT: Remember to first transform the ray into object space  
  // to simplify the intersection test.

  // transform ray into object space
  Point3D q = worldToModel * ray.origin;
  Vector3D r = worldToModel * ray.dir;

  // we know that we must solve the implicit equation (below) for the plane, for t.
  // the x and y values for the normal n(0, 0, 1) are 0, leaving only the z 
  //		n_z DOT (q_z + r_z * t) = 0 
  // 	==> (n_z * q_z) + (n_z * r_z * t) = 0
  //	==> q_z + r_z * t) = 0     # since n_z = 1
  //	==> t = (0 - q_z) / r_z
  double t_val = (0 - q[2]) / r[2];

  if (t_val <= 0) {
    // if the intersection point is in the opposite direction of the ray (i.e.
    // behind), not relevant so return false
    return false;
  
  } else {
    // in this case, intersection point *is* relevant. the intersection point is the 
    // point on the ray *at this t_val*
    Point3D intersectPt = q + t_val * r;

    // check if x in [-0.5,0.5] AND y in [-0.5,0.5]
    if (fabs(intersectPt[0]) < 0.5 && fabs(intersectPt[1]) < 0.5) {
      // if we have not yet seen an intersection on this ray, or if this
      // new intersection point is closer than the previous intersection point,
      // then we must update ray.intersection
      if (ray.intersection.none || t_val < ray.intersection.t_value) {
        // set the intersection point, *IN WORLD COORDINATES*
        ray.intersection.point = modelToWorld * intersectPt;

        // transform the normal(0,0,1) to world-coordinates
        ray.intersection.normal = transNorm(worldToModel, Vector3D(0.0, 0.0, 1.0)); 
        ray.intersection.normal.normalize();

        ray.intersection.t_value = t_val;
        ray.intersection.none = false;     // we have reached an intersection
        return true;
      }
    }
  }

  return false;
}

bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {
  // TODO: implement intersection code for UnitSphere, which is centred 
  // on the origin.  
  //
  // Your goal here is to fill ray.intersection with correct values
  // should an intersection occur.  This includes intersection.point, 
  // intersection.normal, intersection.none, intersection.t_value.   
  //
  // HINT: Remember to first transform the ray into object space  
  // to simplify the intersection test.

  // transform ray into object space
  Point3D q = worldToModel * ray.origin;
  Vector3D r = worldToModel * ray.dir;

  // center of the sphere
  Point3D c = Point3D(0.0, 0.0, 0.0);

  // We know we need to solve the quadratic equation below for t:
  // (q + r * t - c) * (q + r * t - c) - 1 = 0
  // Using the equation for a closed-form solution of the above quadratic, 
  // we know D = B^2 - 4AC, where A, B, and C are as below
  double A = r.dot(r);
  double B = 2 * (q - c).dot(r);
  double C = (q - c).dot(q - c) - 1;

  double root_1;
  double root_2;

  // solve the quadratic equation with coefficients as above
  bool isSolutionFound = quadraticEqnSolver(A, B, C, &root_1, &root_2);

  double t_val = fmin(root_1, root_2);		// pick lowest t, because we want the closest intersection

  // if no solutions, or if the intersection is behind, it is irrelevant
  if (!isSolutionFound || t_val < 0)
    return false;

  if (ray.intersection.none || t_val < ray.intersection.t_value) {
    // in this case, intersection point *is* relevant. the intersection point is the 
    // point on the ray *at this t_val*
    Point3D intersectPt = q + t_val * r;

    // set the intersection point, *IN WORLD COORDINATES*
    ray.intersection.point = modelToWorld * intersectPt;
    
    // normal at the point of intersection, and transform it to world-coordinates
    ray.intersection.normal = intersectPt - c;
    ray.intersection.normal = transNorm(worldToModel, ray.intersection.normal);
    ray.intersection.normal.normalize();

    ray.intersection.t_value = t_val;
    ray.intersection.none = false;     // we have reached an intersection
    return true;
  }

  return false;
}

bool UnitCylinder::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {
  ////////////////////////////////////////////////////////////////////////////
  // --ADVANCED RAY TRACING--
  // Handling a non-trivial compound object: cylinder (implementation of ray
  // intersection function)
  ////////////////////////////////////////////////////////////////////////////

  // transform ray into object space
  Point3D q = worldToModel * ray.origin;
  Vector3D r = worldToModel * ray.dir;

  // centre of the cylinder
  Point3D c = Point3D(0.0, 0.0, 0.0);

  /******************************************************************************************
  * Checking if there is an intersection with the roof or floor of the cylinder (i.e. caps)
  *******************************************************************************************/
  double t_val_top_cap = (0.5 - q[2]) / r[2];			// t_val corresponding to ray-intersection with the top cap
  double t_val_bottom_cap = (-0.5 - q[2]) / r[2];		// t_val corresponding to ray-intersection with the bottom caps

  if (t_val_top_cap <= 0) {
    return false;
  }
  if (t_val_bottom_cap <= 0) {
    return false;
  }

  // pick lower of the two intersection points with caps
  double t_val_caps = fmin(t_val_top_cap, t_val_bottom_cap);

  // points upwards if the intersection is with the top cap (smaller t_val_top_cap), or downwards if 
  // the intersection is with the bottom cap (smaller t_val_bottom_cap)
  Vector3D norm = t_val_top_cap < t_val_bottom_cap ? Point3D(0, 0, 1) - c : Point3D(0, 0, -1) - c;

  // compute the closest intersection point if the ray intersects either cap
  Point3D intersectPt = q + t_val_caps * r;

  Vector3D v = Vector3D(intersectPt[0], intersectPt[1], 0);
  bool doesIntersectCap= (v.dot(v) <= 1);

  // check if the ray intersects with either cap
  if (doesIntersectCap) {
    if (ray.intersection.none || t_val_caps < ray.intersection.t_value) {

      // set the intersection point, *IN WORLD COORDINATES*
      ray.intersection.point = modelToWorld * intersectPt;

      // set the normal, and transform it to world-coordinates
      ray.intersection.normal = transNorm(worldToModel, norm);	// set normal pointing in right direction
      ray.intersection.normal.normalize();

      ray.intersection.t_value = t_val_caps;
      ray.intersection.none = false;     // we have reached an intersection
      return true;
    }
  }

  /******************************************************************************************
  * If no intersection with the caps was found, then we must check if there is an intersection
  * with any wall of the cylinder
  *******************************************************************************************/

  // We need to solve the following quadratic equation of the cylinder:
  // x^2 + z^2 = 1, with y in the range [-0.5, 0.5]
  // Thus, we define the coefficients of the quadratic as below
  double A = r[0]*r[0] + r[1]*r[1];
  double B = q[0]*r[0] + q[1]*r[1];
  double C = q[0]*q[0] + q[1]*q[1] - 1;

  // get discriminant of the quadratic equation (with coefficients as above)
  double discr = B*B - A*C;
  double root_1 = -B/A + sqrt(discr) / A;
  double root_2 = -B/A - sqrt(discr) / A;

  double t_val_wall = fmin(root_1, root_2);		// pick lower t, because we want the closest intersection

  // if there are no intersections, or if the intersection is behind us, it is not relevant
  if (discr < 0 || t_val_wall < 0)
    return false;

  // if not, intersection point *is* relevant. the intersection point is the 
  // point on the ray at t_val_wall
  intersectPt = q + t_val_wall * r;

  // we must ensure that the intersection occurs within the bounds of the cylinder
  if (fabs(intersectPt[2]) < 0.5) {
    if (ray.intersection.none || t_val_wall < ray.intersection.t_value) {

      // set the intersection point, *IN WORLD COORDINATES*
      ray.intersection.point = modelToWorld * intersectPt;

      // set the normal, and transform it to world-coordinates
      ray.intersection.normal = modelToWorld * (Point3D(intersectPt[0], intersectPt[1], 0) - c);
      ray.intersection.normal = transNorm(worldToModel, ray.intersection.normal);
      ray.intersection.normal.normalize();

      ray.intersection.t_value = t_val_wall;
      ray.intersection.none = false;     // we have reached an intersection
      return true;
    }
  }

  return false;
}



/********************************************************************
* HELPER FUNCTIONS
*********************************************************************/

bool quadraticEqnSolver(double A, double B, double C, double *root_1, double *root_2) {
  // Helper function that solves the quadratic equation of the form 
  // A*(t_val^2) + B*(t_val) + C = 0
  // return true if a relevant solution was found; false otherwise

  double D = B*B - 4*A*C;				// discriminant = B^2 - AC

  if (D < 0) {				// no intersections
    *root_1 = -1;
    *root_2 = -1;
    return false;
  } else if (D == 0) {		// one intersection
    *root_1 = -B / (2*A);
    *root_2 = -B / (2*A);
  } else {					// two intersections
    // two solutions to the equation 
    *root_1 = (-B + sqrt(D)) / (2*A);
    *root_2 = (-B - sqrt(D)) / (2*A);
  }

  return true;
}