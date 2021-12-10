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

    plat_angles[0] = (11*PI/6) - phi_plat; 
    plat_angles[1] = (11*PI/6) + phi_plat;  
    plat_angles[2] = (3*PI/6)  - phi_plat; 
    plat_angles[3] = (3*PI/6)  + phi_plat; 
    plat_angles[4] = (7*PI/6)  - phi_plat; 
    plat_angles[5] = (7*PI/6)  + phi_plat; 

    base_angles[0] = (11*PI/6) - phi_base;
    base_angles[1] = (11*PI/6) + phi_base;
    base_angles[2] = (3*PI/6)  - phi_base;
    base_angles[3] = (3*PI/6)  + phi_base;
    base_angles[4] = (7*PI/6)  - phi_base;
    base_angles[5] = (7*PI/6)  + phi_base;

    for (int i = 0; i < 6; i++) {

        // p[0:2][0,2,4]
        p[0][i] = R_PLAT * cos(plat_angles[i]);
        p[1][i] = R_PLAT * sin(plat_angles[i]);
        p[2][i] = 0;

        // b[0:2][0,2,4]
        b[0][i] = R_BASE * cos(base_angles[i]);
        b[1][i] = R_BASE * sin(base_angles[i]);
        b[2][i] = 0;
    }

    // const fmatrix p(3, 6);
    // const fmatrix b(3, 6);
}

fvector getCol(fmatrix v, int i) {
    fvector x(3);
    x[0] = v[0][i];
    x[1] = v[1][i];
    x[2] = v[2][i];
    return x;
}

fvector invKine(float desired_x, float desired_y) {
    fmatrix R_pb(3, 3);
    fmatrix R_bp(3, 3);
    fvector R(3);
    fvector invR(3);
    fvector alpha(6);
    fmatrix lengths(4,6);
    R_pb = rotX(desired_x) * rotY(desired_y);
    R_bp = R_pb.FindInverse();
    R = R2quat(R_pb);
    invR = R2quat(R_bp);

    //Code to print R matrix
    // Serial.printf("[");
    // for (int i = 0; i < 3; i++) {
    //     Serial.printf("[");
    //     for (int j = 0; j < 3; j++) {
    //         Serial.printf("\t");
    //         Serial.print(R_pb[i][j]);
    //     }
    //     Serial.printf("]\n");
    // }
    // Serial.printf("]\n");

    for (int i = 0; i < NUM_LINKS; i++) {
        fvector pk(3);
        fvector bk(3);
        fvector qk(3);
        fvector l(3);

        pk = getCol(p, i);
        bk = getCol(b, i);

        float betak = Beta[i] * PI / 180;
        // TODO FIX THIS?, Link lengths end up being incerse of servo attachment locations at 0 rotation
        qk = T + (criss(cross(R,pk),invR));
        l =  qk - bk;
        lengths[0][i] = l[0];
        lengths[1][i] = l[1];
        lengths[2][i] = l[2];
        lengths[3][i] = l.CalcNorm(2);
        

        float e = 2 * HORN_LEN * l[2];
        float f = 2 * HORN_LEN * ((cos(betak) * l[0]) + (sin(betak) * l[1]));
        float g = pow(l.CalcNorm(2),2) - (pow(LINK_LEN, 2) - pow(HORN_LEN, 2));
        float a = asin(g / (sqrt(pow(e, 2) + pow(f, 2)))) - atan2(f, e);
        a = a * 180 / PI;
        alpha[i] = a;
    }

    //------------ PRINT STATEMENTS ------------//

    // //Code to print calculated link lengths
    // Serial.printf("Link Lengths:\n");
    // for (int i = 0; i < 4; i++) {
    //     if(i==0){Serial.printf("X: ");}
    //     if(i==1){Serial.printf("Y: ");}
    //     if(i==2){Serial.printf("Z: ");}
    //     if(i==3){Serial.printf("N: ");}
    //     for(int j = 0; j < 6; j++){
    //         Serial.printf("\t");
    //         Serial.print(lengths[i][j]);
    //     }
    //     Serial.println();
    // }
    // Serial.println("");

    // Serial.printf("Delta Servo to Plate:\n");
    // for (int i = 0; i < 3; i++) {
    //     if(i==0){Serial.printf("X: ");}
    //     if(i==1){Serial.printf("Y: ");}
    //     if(i==2){Serial.printf("Z: ");}
    //     for(int j = 0; j < 6; j++){
    //         Serial.printf("\t");
    //         Serial.print(p[i][j]-b[i][j]);
    //     }
    //     Serial.println();
    // }
    // Serial.println("");

    // //Code to print calculated servo locaiton angles
    // Serial.printf("Platform Angles:");
    //     for (int i = 0; i < NUM_LINKS; i++) {
    //         Serial.printf("\t%i: ", i);
    //         Serial.print(base_angles[i]*180/PI);
    //     }
    //     Serial.println("");

    // //Code to print calculated servo locations
    // Serial.printf("Servo Locations:\n");
    // for (int i = 0; i < 3; i++) {
    //     if(i==0){Serial.printf("X: ");}
    //     if(i==1){Serial.printf("Y: ");}
    //     if(i==2){Serial.printf("Z: ");}
    //     for(int j = 0; j < 6; j++){
    //         Serial.printf("\t");
    //         Serial.print(b[i][j]);
    //     }
    //     Serial.println();
    // }
    // Serial.println("");

    // Serial.printf("Plate Attachment Locations:\n");
    // for (int i = 0; i < 3; i++) {
    //     if(i==0){Serial.printf("X: ");}
    //     if(i==1){Serial.printf("Y: ");}
    //     if(i==2){Serial.printf("Z: ");}
    //     for(int j = 0; j < 6; j++){
    //         Serial.printf("\t");
    //         Serial.print(p[i][j]);
    //     }
    //     Serial.println();
    // }
    // Serial.println("");
    
    // delay(1000);
    
    return alpha;
}

fmatrix rotX(float angle) {
    fmatrix R(3, 3);

    float theta = angle * PI / 180;

    R[0][0] = 1;
    R[0][1] = 0;
    R[0][2] = 0;

    R[1][0] = 0;
    R[1][1] = cos(theta);
    R[1][2] = sin(theta);

    R[2][0] = 0;
    R[2][1] = -sin(theta);
    R[2][2] = cos(theta);

    return R;
}

fmatrix rotY(float angle) {
    fmatrix R(3, 3);

    float theta = angle * PI / 180;

    R[0][0] = cos(theta);
    R[0][1] = 0;
    R[0][2] = -sin(theta);

    R[1][0] = 0;
    R[1][1] = 1;
    R[1][2] = 0;

    R[2][0] = sin(theta);
    R[2][1] = 0;
    R[2][2] = cos(theta);

    return R;
}

fmatrix skew(fvector v) {
    fmatrix mat(3, 3);

    mat[0][1] = -v[2];
    mat[0][2] = v[1];
    mat[1][2] = -v[0];

    mat[1][0] = v[2];
    mat[2][0] = -v[1];
    mat[2][1] = v[0];

    return mat;
}

fvector cross(fvector A, fvector B) { return skew(A) * B; }
fvector criss(fvector A, fvector B) { return cross(A,B); }


fvector R2quat(fmatrix R) {
    fvector q(3);
    float w = 4 * sqrt(1.0 + R[0][0] + R[1][1] + R[2][2]) / 2.0;

    q[0] = (R[2][1] - R[1][2]) / w;
    q[1] = (R[0][2] - R[2][0]) / w;
    q[2] = (R[1][0] - R[0][1]) / w;

    return q;
}

// int main(int argc, char **argv){

//     fvector servo_d(6);

//     servo_d = invKine(0,0);


//     printf("Servo Angles:");
//     for (int i = 0; i < NUM_LINKS; i++) {
//         printf("\t%i: ", i);
//         float thisA = servo_d[i];
//         printf("%f", thisA);
//     }
//     printf("\n");

//     return 0;
// }
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