#include "math.h"

/************************************************************************************************************/
/* definições */

/* definição do valor usado para o raio da terra (em metros)*/
#define RAIO_TERRA   6378135  // ou 63781000

/* definição - retorno de sucesso da execução de função  */
//#define OK    (unsigned char)0

/* definição - retorno de erro da execução de função  */
//#define ERRO  (unsigned char)1

/* Constants */
#define M_PI      3.14159265358979323846   /* pi         */
#define M_1PI     0.31830988618379067154   /* 1 / pi     */
#define M_2PI     6.28318530717958647692   /* 2 * pi     */
#define M_PI2     1.57079632679489661923   /* pi / 2     */
#define M_PI4     0.78539816339744830961   /* pi / 4     */
#define M_S1_2    0.70710678118654752440   /* sqrt (0.5) */
#define M_LOGE    0.43429448190325182765   /* log10 (e)  */

#define M_EPS     1.053671213e-8
#define M_EXP_MAX 709.7877

/* endereço casa - conforme planilha excel */
/* usado como ponto de referência */
//x 228578,13717772037
//y 6712926,388417843

#define LAT1 -29.695733
#define LON1 -53.809131

#define X1   (signed long)11702   //228187,28673424874
#define Y1   (signed long)7410    //6711626,483192012

#define LAT2 -29.71124
#define LON2 -53.716621

#define X2    (signed long)20697    //237182,5142412018
#define Y2    (signed long)5905     //6710121,35411519

#define LAT0  -29.759907
#define LON0  -53.931885

#define X0    0
#define Y0    0


#define LAT3  -29.695434
#define LON3  -53.808256

#define X3   (signed long)11786   //228271.1813058264
#define Y3   (signed long)7445    //6711661.692281629

#define LAT4  -29.759907
#define LON4  -53.682632

#define LAT5  -29.637336
#define LON5  -53.931885

#define LAT6  -29.637336
#define LON6  -53.682632


/************************************************************************************************************/

unsigned int    CalculaDistancia( double nLat1, double nLon1, double nLat2, double nLon2 );
unsigned short  SquareRoot(unsigned long A);