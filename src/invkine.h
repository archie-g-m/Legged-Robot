#ifndef __INVKINE_H
#define __INVKINE_H

#include <matrix_uC.h>

#define NUM_LINKS (6)

const float HORN_LEN = 1.5;  // Length of servo horns (mm)
const float LINK_LEN = 115;  // Length of links (mm)
const float PLAT_H = 120;    // Height of platform (mm)
const float pk = 45;         // Dist from platform frame to pivot (mm)
const float bk = 60;         // Dist from base frame to pivot

const fvector T(3);
const fvector Beta(6);

void invkine_setup();

fvector invKine(float desired_x, float desired_y);

#endif // __INVKINE_H