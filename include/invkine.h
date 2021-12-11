#ifndef __INVKINE_H
#define __INVKINE_H

#include <cmath>

// #define __SIMULATE__

#ifndef __SIMULATE__
#include <Arduino.h>
#endif

#include <matrix_uC.h>

#define fmatrix TMatrix<float>

#define NUM_LINKS (6)

// const float D2R = (180.0/PI);
const float D2R = PI/180.0;

const float HORN_LEN = 12.5;   // Length of servo horns (mm)
const float LINK_LEN = 115;  // Length of links (mm)
const float PLAT_H = 109.7075;     // Height of platform (mm)

const float R_BASE = 60;      // Dist from platform frame to pivot (mm)

const float R_PLAT = 48;        // Dist from base frame to pivot (mm)

const fvector T(3);

const fvector Beta(6);
const fvector plat_angles(6);
const fvector base_angles(6);

const fmatrix p(4, 6);
const fmatrix b(4, 6);
const fmatrix q(4, 6);


void invkine_setup();

fvector invKine(float desired_x, float desired_y);
fmatrix rotX(float angle);
fmatrix rotY(float angle);
fmatrix createHT(fmatrix R, fvector T);
fvector getCol(fvector v, int i);

fmatrix skew(fvector v);
fvector cross(fvector A, fvector B);
fvector criss(fvector A, fvector B);
fvector R2quat(fmatrix R);

#endif  // __INVKINE_H