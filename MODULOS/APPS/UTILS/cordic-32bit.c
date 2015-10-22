#include "cordic-32bit.h"

const INT32S cordic_ctab [] = {0x3243F6A8, 0x1DAC6705, 0x0FADBAFC, 0x07F56EA6, 0x03FEAB76, 0x01FFD55B, 0x00FFFAAA, 0x007FFF55, 0x003FFFEA, 0x001FFFFD, 0x000FFFFF, 0x0007FFFF, 0x0003FFFF, 0x0001FFFF, 0x0000FFFF, 0x00007FFF, 0x00003FFF, 0x00001FFF, 0x00000FFF, 0x000007FF, 0x000003FF, 0x000001FF, 0x000000FF, 0x0000007F, 0x0000003F, 0x0000001F, 0x0000000F, 0x00000008, 0x00000004, 0x00000002, 0x00000001, 0x00000000, };


void cordic(INT32S* theta, INT32S *s, INT32S *c, INT32S n)
{
  volatile INT32S k, d, tx, ty, tz;
  INT32S x=cordic_1K,y=0,z=*theta;
  n = (n>CORDIC_NTAB) ? CORDIC_NTAB : n;
  for (k=0; k<n; ++k)
  {
    d = z>>31;
    //get sign. for other architectures, you might want to use the more portable version
    //d = z>=0 ? 0 : -1;
    tx = x - (((y>>k) ^ d) - d);
    ty = y + (((x>>k) ^ d) - d);
    tz = z - ((cordic_ctab[k] ^ d) - d);
    x = tx; y = ty; z = tz;
  }  
 *c = x; *s = y;
}


void cordic1(INT32S *theta, INT32S *s, INT32S *c, INT32S n)
{
  INT32S k,tx, ty;
  INT32S x=*c,y=*s,z=*theta;
  
  n = (n>CORDIC_NTAB) ? CORDIC_NTAB : n;
  for (k=0; k<n; ++k)
  {
   
    if(y>=0){       
      tx = x + (y>>k);
      ty = y - (x>>k);
      z = z - cordic_ctab[k]; 
    }else{
      tx = x - (y>>k);
      ty = y + (x>>k);
      z = z + cordic_ctab[k]; 
    }
    x = tx; y = ty;
    
  }  
 *c = x; *s = y;*theta=-z;
}


// tangente em ponto fixo
void tan_fp(INT32S s, INT32S c, INT32S *tan)
{
  INT32S tangente;  
  tangente = s/c;
  
  *tan = tangente << 15;
}


//tang_45 = 1 << 30;      // Tangente de 45 graus em Q30
// t = tangente p/ cancular o angula
// theta = 0      onde será passado a resposta
//cordit1(&tang_45, &t, &theta, 0);

void cordit2(INT32S* x0, INT32S* y0, INT32S* z0, INT32S vecmode)
{
    /* this is the circular method. 
     * one slight change from the other methods is the y < vecmode 
     * test. this is to implement arcsin, otherwise it can be
     * y < 0 and you can compute arcsin from arctan using
     * trig identities, so its not essential.
     */
    INT32S t;
    INT32S x, y, z;
    int i;
    t = 1 << 30;
    x = *x0; y = *y0; z = *z0;
    for (i = 0; i < 32; ++i) {
        INT32S x1;
        if (vecmode >= 0 && y < vecmode || vecmode<0  && z >= 0) {
            x1 = x - (y>>15)*(t>>15);
            y = y + (x>>15)*(t>>15);
            z = z - cordic_ctab[i];
        }
        else {
            x1 = x + (y>>15)*(t>>15);
            y = y - (x>>15)*(t>>15);
            z = z + cordic_ctab[i];
        }
        x = x1;
        t /= 2;
    }
    *x0 = x;
    *y0 = y;
    *z0 = z;
}




