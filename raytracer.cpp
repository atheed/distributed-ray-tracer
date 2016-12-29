/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

SceneDagNode* sphere;
SceneDagNode* cylinder;

/************************************************************
* FLAGS. SET AS NEEDED. 
************************************************************/
// 0 represents we will NOT execute motion blur, 1 represents we WILL execute motion blur
// Associated setting: if we execute motion blurring, we will execute 6 blurring iterations
int EXECUTE_MOTION_BLUR = 0;

// 0 represents we will NOT execute depth of field, 1 represents we WILL execute motion blur
// Associated settings: below
int EXECUTE_DEPTH_OF_FIELD = 0;

/************************************************************
* SETTINGS FOR THE DIFFERENT MODES
*************************************************************/
// Mode: executing depth of field
int FOCUS_PLANE_POINT_Z = -5;			// z-coord of the focus plane
int DOF_RAYS_CAST = 40;					// number of rays to cast for depth of field, to randomly perturb and average over
int APERTURE_SIZE = 2;					// size of the aperture
/************************************************************/

Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}
void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
    traverseScene(node,ray,_modelToWorld,_worldToModel);
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray, const Matrix4x4& modelToWorld, const Matrix4x4& worldToModel ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	Matrix4x4 myModelToWorld = modelToWorld*node->trans;
	Matrix4x4 myWorldToModel = node->invtrans*worldToModel;
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, myWorldToModel, myModelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray, myModelToWorld,myWorldToModel);
		childPtr = childPtr->next;
	}

}

void Raytracer::computeShading( Ray3D& ray ) {
	LightListNode* curLight = _lightSource;
	for (;;) {
		if (curLight == NULL) break;

		// Each lightSource provides its own shading function.

		// Implement shadows here if needed.

		////////////////////////////////////////////////////////////////////////////
		// --ADVANCED RAY TRACING--
		// Shadows (1 of the 2 mandatory features), INCLUDING soft shadows (1 of the choose-your-options features)
		////////////////////////////////////////////////////////////////////////////
		Colour rayCol;
		// vector that will point in the direction from the intersection point to the light source,
		// with an amount of perturbation, to simulate an area light
		Vector3D l;

		// perturb over the interval -1 < x < 1
		for (float i = -1.0; i < 1.0; i += 0.05) {
			l = curLight->light->get_position() - ray.intersection.point;
			l[0] += i;
			l[1] += i;
			l[2] += i;
			double t_val = l.length();
			l.normalize();

			// build the ray for the shadow at this perturbation (i.e. one part of the many rays that
			// make up the simulated area light) with a slight offset from the intersection
			Ray3D r = Ray3D(ray.intersection.point + 0.005 * l, l);
			traverseScene(_root, r);	// shoot the ray
			curLight->light->shade(ray);

			// is in shadow if 
			bool isNotInShadow = (r.intersection.none || t_val < r.intersection.t_value);

			// if the object is not in shadow, we want the ray.col colour, so we ensure that we 
			// add enough at each iteration to ultimately cumulatively have 1*ray.col
			if (isNotInShadow) {
				rayCol = rayCol + 0.025 * ray.col;
			}
			// if the object is in shadow, we want rayCol to remain Colour(0,0,0) -- that is, dark
		}
		ray.col = rayCol;
		curLight = curLight->next;
	}
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray ) {
	Colour col(0.0, 0.0, 0.0); 
	traverseScene(_root, ray); 
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {
		computeShading(ray); 

		// You'll want to call shadeRay recursively (with a different ray, 
		// of course) here to implement reflection/refraction effects.

		////////////////////////////////////////////////////////////////////////////
		// --ADVANCED RAY TRACING--
		// Reflection (1 of the 2 mandatory features)
		////////////////////////////////////////////////////////////////////////////

		// find the reflected ray from the object, and compute the colour of the ray
		Vector3D v = -ray.dir;						// reflected ray sent out from intersection pt
		v.normalize();

		Vector3D n = ray.intersection.normal;		// normal at intersection pt
		n.normalize();

		Vector3D reflectedVector = -v - 2 * ((-v).dot(n)) * n;	// the mirrored/reflected direction given n and v
		reflectedVector.normalize();

		Ray3D reflectedRay = Ray3D(ray.intersection.point + 0.005 * reflectedVector, reflectedVector);

		shadeRay(reflectedRay);

		// if the we are inside 0 < t_value < 5.0 (this value was chosen by trial-and-error; any small enough
		// value is reasonable here), only then should we compute the reflections. i.e., we dont want to reflect if 
		// objects are too far apart
		if (reflectedRay.intersection.t_value > 0 && reflectedRay.intersection.t_value < 5.0) {
			double reflectionBackoff = fabs(1.0 / reflectedRay.intersection.t_value);
			if (reflectionBackoff > 0.75) {
				reflectionBackoff = 0.75;		// disallow the backoff coefficient from being too high
			}
			col = ray.col + reflectionBackoff*reflectedRay.col;
		} else {
			// if we are outside of 0 < t_value < 5.0, then we are too far away to have realistic reflection,
			// so we instead do regular shading
			col = ray.col;
		}
		col.clamp();
	}

	return col; 
}

double randomise(double range_min, double range_max) {
	// function that random number between range_min and range_max

	// CREDIT: This function was adapted from an online example, found at:
	// http://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
	// Example written by: user rep_movsd, Apr 24, 2012

	double f = (double)rand() / RAND_MAX;
    return range_min + f * (range_max - range_min);
}

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	// total number of anti-aliasing rays (one for each "quadrant" of the pixels)
	float num_antialias_rays = 4.0;

	// number of offsets (or subdivisions) in each dimension (x and y)
	float num_offsets = 2.0;

	// number of iterations of motion blur (i.e. the number of "streaks" of the object)
	float num_blurs = 1.0;

	if (EXECUTE_MOTION_BLUR)
		num_blurs = 6.0;
	
	////////////////////////////////////////////////////////////////////////////
	// --ADVANCED RAY TRACING--
	// Motion blur
	// This outer loop (with blur_iters) is only to execute motion blur. the number of iterations is the 
	// number of 'movements' / 'streaks' of the object. if blur_iters = 1, then there is no 
	// motion blurring. Set the flag appropriately at the top of THIS FILE
	////////////////////////////////////////////////////////////////////////////
	for (int blur_iters = 0; blur_iters < num_blurs; blur_iters++) {
		// Construct a ray for each pixel.
		for (int i = 0; i < _scrHeight; i++) {
			for (int j = 0; j < _scrWidth; j++) {

				////////////////////////////////////////////////////////////////
				// --ADVANCED RAY TRACING--
				// Anti-aliasing
				////////////////////////////////////////////////////////////////

				// row-coords and col-coords for anti-aliasing rays per pixel. effectively,
				// the "extra" anti-aliasing rays will be shot from (0.0, 0.0), (0.0, 0.5),
				// (0.5, 0.0), and (0.5, 0.5) position per pixel. essentially, we are subdividing
				// the pixel into 4 (2 x 2).
				float row_subdivide[2] = {0.0, 0.5};
				float col_subdivide[2] = {0.0, 0.5};

				for (int rowAntiAlias = 0; rowAntiAlias < num_offsets; rowAntiAlias++) {
					for (int colAntiAlias = 0; colAntiAlias < num_offsets; colAntiAlias++) {
						// Sets up ray origin and direction in view space, 
						// image plane is at z = -1.
						Point3D origin(0, 0, 0);
						Point3D imagePlane;

						// add the offsets (for anti-alisasing) in order to, effectively, sample at the center
						// of each of the four subdivisions of a pixel
						imagePlane[0] = (-double(width)/2 + j + col_subdivide[colAntiAlias])/factor;
						imagePlane[1] = (-double(height)/2 + i + row_subdivide[rowAntiAlias])/factor;
						imagePlane[2] = -1;

						////////////////////////////////////////////////////////////////
						// --ADVANCED RAY TRACING--
						// Depth of field
						////////////////////////////////////////////////////////////////
						if (EXECUTE_DEPTH_OF_FIELD) {
							// ray from the origin to the image plane, in order to find the point that the 
							// ray intersects with the focus plane
							Vector3D ray_direction = imagePlane - origin;
							ray_direction.normalize();

							// t_value at the point that the ray intersects the focus plane (computed using the z-coordinate
							// of the focus plane)
							double t_val = FOCUS_PLANE_POINT_Z / ray_direction[2];

							// define the intersection point based on the t-value found above
							Point3D intsctPtFocus = Point3D(t_val * ray_direction[0], t_val * ray_direction[1], t_val * ray_direction[2]);

							Colour DOFcolour;

							// perturb the rays (to cast towards the focus plane) randomly in order to capture the variability in colour,
							// and average those colours over all DOF_RAYS_CAST rays
							for (int dof_iter = 0; dof_iter < DOF_RAYS_CAST; dof_iter++) {
								double angle = randomise(0, 2 * M_PI);
								double radius = randomise(0, APERTURE_SIZE);

								// origin of the ray to be drawn, perturbed using the above values. we perturb using our understanding
								// of circles, as we know circle = (r cos theta, r sin theta). we know our r is limited by aperture size, 
								// and theta can define a circle 
								Point3D current_ray_origin = Point3D(radius * cos(angle), radius * sin(angle), 0);

								Ray3D ray;
								ray.origin = viewToWorld * current_ray_origin;
								ray.dir = viewToWorld * (intsctPtFocus - current_ray_origin);   // direction is from the origin to the focus point
								
								// run shade() for this ray, and update the accumulated colour value
								DOFcolour = DOFcolour + shadeRay(ray);
							}

							// average the depth-of-field colour obtained from the DOF_RAYS_CAST so that we still sum to the right value
							DOFcolour = (double) 1.0 / DOF_RAYS_CAST * DOFcolour;

							// sum the colour values obtained from each of the 4 anti-aliasing rays (and weight each sum by
							// 1/4, to get an "average" in the correct range)
							_rbuffer[i*width+j] += int(DOFcolour[0]*255/num_antialias_rays);
							_gbuffer[i*width+j] += int(DOFcolour[1]*255/num_antialias_rays);
							_bbuffer[i*width+j] += int(DOFcolour[2]*255/num_antialias_rays);

						} else {
							// TODO: Convert ray to world space and call
							// shadeRay(ray) to generate pixel colour. 
							Ray3D ray;
							ray.origin = viewToWorld * origin;
							ray.dir = viewToWorld * (imagePlane - origin);   // direction is from the origin to the image-plane

							Colour col = shadeRay(ray);
							
							if (!EXECUTE_MOTION_BLUR) {
								// CASE: No motion blur
								// sum the colour values obtained from each of the 4 anti-aliasing rays (and weight each sum by
								// 1/4, to get an "average" in the correct range)
								_rbuffer[i*width+j] += int(col[0]*255/num_antialias_rays);
								_gbuffer[i*width+j] += int(col[1]*255/num_antialias_rays);
								_bbuffer[i*width+j] += int(col[2]*255/num_antialias_rays);
							} else {
								// CASE: We are executing motion blur
								// in addition to taking into account the 4 anti-aliasing rays, we must also ensure to render the 
								// right colour for each movement of the object. We know that the summation of 1/(2^n) = 1, so dividing
								// by decreasing powers of 2 until n=1 will still give us the same total colour. we want less colour in the
								// initial movements of the object (since it has "been longer" since the object was in that position), and 
								// more colour in later movements. thus, we divide by decreasing powers of 2 (i.e. increasing total, thus 
								// increasing colour)
								_rbuffer[i*width+j] += int(col[0]*255)/num_antialias_rays/pow(2.0, num_blurs - blur_iters);
								_gbuffer[i*width+j] += int(col[1]*255)/num_antialias_rays/pow(2.0, num_blurs - blur_iters);
								_bbuffer[i*width+j] += int(col[2]*255)/num_antialias_rays/pow(2.0, num_blurs - blur_iters);	
							}
						}
					}
				}
			}
		}
		if (EXECUTE_MOTION_BLUR)
			this->translate(cylinder, Vector3D(0.3, 0.5, 0));
	}
	if (EXECUTE_MOTION_BLUR)
		this->translate(cylinder, Vector3D(-0.3 * num_blurs, -0.5 * num_blurs, 0));

	flushPixelBuffer(fileName);
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320; 
	int height = 240; 

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	Point3D eye(0, 0, 1);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	double fov = 60;

	// Defines a material for shading.
	Material gold( Colour(0.24725, 0.1995, 0.0745), Colour(0.75164, 0.60648, 0.22648), 
			Colour(0.628281, 0.555802, 0.366065), 
			51.2 );
	Material jade( Colour(0, 0, 0), Colour(0.54, 0.89, 0.63), 
			Colour(0.316228, 0.316228, 0.316228), 
			12.8 );
	Material chrome( Colour(0.25, 0.25, 0.25), Colour(0.4, 0.4, 0.4), 
			Colour(0.774597, 0.774597, 0.774597), 
			76.8 );
	Material emerald( Colour(0.0215, 0.1745, 0.0215), Colour(0.07568, 0.61424, 0.07568), 
			Colour(0.633, 0.72781, 0.633), 
			76.8 );
	Material ruby( Colour(0.1745, 0.01175, 0.01175), Colour(0.61424, 0.04136, 0.04136), 
			Colour(0.727811, 0.626959, 0.626959), 
			76.8 );

	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(0, 0, 5), 
				Colour(0.9, 0.9, 0.9) ) );

	// Add a unit square into the scene with material mat.
	sphere = raytracer.addObject( new UnitSphere(), &gold );
	SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );
	cylinder = raytracer.addObject( new UnitCylinder(), &ruby );
	
	// Apply some transformations to the unit square.
	double factor1[3] = { 1.0, 2.0, 1.0 };
	double factor2[3] = { 6.0, 6.0, 6.0 };
	double factor3[3] = { 1.0, 1.0, 1.5 };
	double factor4[3] = { 1.0, 1.0, 1.0 };
	raytracer.translate(sphere, Vector3D(0, 0, -5));	
	raytracer.rotate(sphere, 'x', -45); 
	raytracer.rotate(sphere, 'z', 45); 
	raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

	raytracer.translate(plane, Vector3D(0, 0, -7));	
	raytracer.rotate(plane, 'z', 45); 
	raytracer.scale(plane, Point3D(0, 0, 0), factor2);

	raytracer.translate(cylinder, Vector3D(-4, 0, -5));	
	raytracer.scale(cylinder, Point3D(0, 0, 0), factor3);

	if (EXECUTE_DEPTH_OF_FIELD) {
		SceneDagNode* sphere2 = raytracer.addObject( new UnitSphere(), &emerald );
		raytracer.translate(sphere2, Vector3D(2, 0, -1));	
		raytracer.rotate(sphere2, 'x', -45); 
		raytracer.rotate(sphere2, 'z', 45); 
		raytracer.scale(sphere2, Point3D(0, 0, 0), factor4);
	}

	// Render the scene, feel free to make the image smaller for
	// testing purposes.	
	raytracer.render(width, height, eye, view, up, fov, "view1.bmp");
	
	// Render it from a different point of view.
	Point3D eye2(4, 2, 1);
	Vector3D view2(-4, -2, -6);
	raytracer.render(width, height, eye2, view2, up, fov, "view2.bmp");
	
	return 0;
}

