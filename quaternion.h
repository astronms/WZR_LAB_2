
#ifndef _QUATMATHVECT__H
#define _QUATMATHVECT__H

#define M_PI 3.14159265358979323846  
#define M_RAD 180/M_PI

#include <math.h>

#ifndef _VEKTOR3D__H 
#include "vector3D.h"
#endif


struct quaternion
{
	float x, y, z, w;

	quaternion(float x1, float y1, float z1, float w1);
	quaternion();

	Vector3 AsixAngle();                   // quaternion substitution for angular - axial representation
	Vector3 rotate_vector(Vector3 w);      // vector rotation using rotation reprezented by quaternion

	quaternion operator*(quaternion q);   // quaternion product (turnover assembly)
	quaternion operator~ ();
	quaternion operator+=(quaternion q);  // addition vec+vec
	quaternion operator+ (quaternion q);  // addition vec+vec
	quaternion operator- (quaternion q);  // subtraction vec-vec
	quaternion n();                       // quaternion normalization 
	float l();                            // length of quaternion 
	quaternion operator* (float value);   // multiplication by scalar
	quaternion operator/ (float value);   // dividing by scalar


};

quaternion AsixToQuat(Vector3 v, float angle);

#endif
