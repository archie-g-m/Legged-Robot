#ifndef __INVKINE_H
#define __INVKINE_H

#include <matrix_uC.h>

#include <cmath>

#define fmatrix TMatrix<float>

#define NUM_LINKS (6)

const float HORN_LEN = 15;  // Length of servo horns (mm)
const float LINK_LEN = 115;  // Length of links (mm)
const float PLAT_H = 97;    // Height of platform (mm)
// const float W_MAJOR_BASE = ; // Major dist bewteen platform mount points (mm)
const float W_MINOR_BASE = 59;  // Minor dist bewteen platform mount points (mm)
const float R_BASE = 52.5;        // Dist from platform frame to pivot (mm)
// const float W_MAJOR_PLAT = ; // Major dist bewteen platform mount points (mm)
const float W_MINOR_PLAT = 13;  // Minor dist bewteen platform mount points (mm)
const float R_PLAT = 45;        // Dist from base frame to pivot (mm)

const fvector T(3);
const fvector Beta(6);
const fmatrix p(3, 6);
const fmatrix b(3, 6);

const fvector plat_angles(6);
const fvector base_angles(6);

void invkine_setup();

fvector invKine(float desired_x, float desired_y);
fmatrix rotX(float angle);
fmatrix rotY(float angle);
fvector getCol(fvector v, int i);

fmatrix skew(fvector v);
fvector cross(fvector A, fvector B);
fvector criss(fvector A, fvector B);
fvector R2quat(fmatrix R);

#endif  // __INVKINE_H