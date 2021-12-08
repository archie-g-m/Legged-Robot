#include "invkine.h"

void invkine_setup() {
    // Base to plaform translation
    T[0] = 0;
    T[1] = 0;
    T[2] = PLAT_H;
    // Servo orientation
    Beta[0] = 240;
    Beta[1] = 60;
    Beta[2] = 0;
    Beta[3] = 180;
    Beta[4] = 120;
    Beta[5] = 300;
}

fvector invKine(float desired_x, float desired_y) {
    fmatrix R(3,3);
    
    // for (int i = 0; i < k; i++) {
    //     packet[i] = Serial1.read();
    // }
    return 0;
}
// % Height of platform % 112.5 !!!T = [0 0 t];
// beta_k = zeros(1, 6);
// % Orientation of servo p_k = zeros(1, 6);
// % Rod connector distance(platform, Fp), all 45mm b_k = zeros(1, 6);
// % Rod connector distance(base, Fb), 60 mm

//                                         % Define variables R = zeros(3, 3);
// % Orientation of platform(Fp) wrt.base(Fb)

//         alpha_k = zeros(1, 6); % Servo angles (from horizontal)

// for i = 1:k
//     l_k = T + R * p_k(i) - b_k(i);
// e_k = 2 * abs(h) * l_k(3);
// f_k = 2 * abs(h) * (cos(beta_k(i)) * l_k(1) + sin(beta_k(i)) * l_k(2));
// g_k = (l_k(1).^ 2) + (l_k(2).^ 2) + (l_k(3).^ 2) - (d.^ 2) + (h.^ 2);
// alpha_k(i) = asin(g_k / sqrt((e_k.^ 2) + f_k.^ 2)) - atan2(f_k, e_k);
// end