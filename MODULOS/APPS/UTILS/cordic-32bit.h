#include "OS_types.h"
//Cordic in 32 bit signed fixed point math
//Function is valid for arguments in range -pi/2 -- pi/2
//for values pi/2--pi: value = half_pi-(theta-half_pi) and similarly for values -pi---pi/2
//
// 1.0 = 1073741824
// 1/k = 0.6072529350088812561694
// pi = 3.1415926536897932384626

#pragma warn_implicitconv off



//Constants
#define cordic_1K   0x26DD3B6A
#define half_pi     0x6487ED51
#define MUL         (long double)1073741824
#define CORDIC_NTAB 32


// Minhas contribuições
// pi/4
#define pi4     (INT32S)843314856
// pi/8
#define pi8     (INT32S)421657428
// pi/16
#define pi16    (INT32S)210828714

#define ang44   (INT32S)724574526


// Calculo de seno e cosseno
// angulo theta em radianos
// n = número de iterações

void cordic(INT32S* theta, INT32S *s, INT32S *c, INT32S n);
void cordic1(INT32S *theta, INT32S *s, INT32S *c, INT32S n);
// tangente em ponto fixo
void tan_fp(INT32S s, INT32S c, INT32S *tan);

void cordit2(INT32S* x0, INT32S* y0, INT32S* z0, INT32S vecmode);

