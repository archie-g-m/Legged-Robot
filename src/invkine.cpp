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

    float phi_plat = asin(W_MINOR_PLAT / (2 * R_PLAT));
    float phi_base = asin(W_MINOR_BASE / (2 * R_BASE));

    for (int i = 0; i < 3; i++) {
        float mid_angle =
            PI / 2 + 2 * PI / 3 * ((i + 2) % 3);  // TODO: PLATFORM ONLY

        // pk[0:2][0,2,4]
        pk[0][(2 * i)] = R_PLAT * sin(mid_angle - phi_plat);
        pk[1][(2 * i)] = R_PLAT * cos(mid_angle - phi_plat);
        pk[2][(2 * i)] = 0;

        // pk[0:2][1,3,5]
        pk[0][(2 * i) + 1] = R_PLAT * sin(mid_angle + phi_plat);
        pk[1][(2 * i) + 1] = R_PLAT * sin(mid_angle + phi_plat);
        pk[2][(2 * i) + 1] = 0;

        // pk[0:2][0,2,4]
        bk[0][(2 * i)] = R_BASE * sin(mid_angle - phi_base);
        bk[1][(2 * i)] = R_BASE * cos(mid_angle - phi_base);
        bk[2][(2 * i)] = 0;

        // pk[0:2][1,3,4]
        bk[0][(2 * i) + 1] = R_BASE * sin(mid_angle + phi_base);
        bk[1][(2 * i) + 1] = R_BASE * sin(mid_angle + phi_base);
        bk[2][(2 * i) + 1] = 0;
    }

    // const fmatrix pk(3, 6);
    // const fmatrix bk(3, 6);
}

fvector getCol(fmatrix v, int i) {
    fvector x(3);
    x[0] = v[0][i];
    x[1] = v[1][i];
    x[2] = v[2][i];
    return x;
}

fvector invKine(float desired_x, float desired_y) {
    fmatrix R(3, 3);
    fvector alpha(6);
    R = rotX(desired_x) * rotY(desired_y);

    for (int i = 0; i < NUM_LINKS; i++) {
        fvector p(3);
        fvector b(3);
        fvector l(3);

        p = getCol(pk, i);
        b = getCol(bk, i);

        float beta = Beta[i] * PI / 180;

        l = T + R * p - b;
        float e = 2 * PLAT_H * l[2];
        float f = 2 * PLAT_H * (cos(beta) * l[0] + sin(beta) * l[1]);
        float g = pow(l[0], 2) + pow(l[1], 2) + pow(l[2], 2) -
                  pow(LINK_LEN, 2) + pow(PLAT_H, 2);
        float a = asin(g / sqrt(pow(e, 2) + pow(f, 2))) - atan2(f, e);
        a = a * 180 / PI;
        alpha[i] = a;
    }
    return alpha;
}

fmatrix rotX(float angle) {
    fmatrix R(3, 3);

    angle = angle * PI / 180;

    R[0][1] = 1;
    R[0][2] = 0;
    R[0][3] = 0;

    R[1][1] = 0;
    R[1][2] = cos(angle);
    R[1][3] = sin(angle);

    R[2][1] = 0;
    R[2][2] = sin(angle);
    R[2][3] = cos(angle);

    return R;
}

fmatrix rotY(float angle) {
    fmatrix R(3, 3);

    float theta = angle * PI / 180;

    R[0][1] = cos(theta);
    R[0][2] = 0;
    R[0][3] = -sin(theta);

    R[1][1] = 0;
    R[1][2] = 1;
    R[1][3] = 0;

    R[2][1] = sin(theta);
    R[2][2] = 0;
    R[2][3] = -cos(theta);

    return R;
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