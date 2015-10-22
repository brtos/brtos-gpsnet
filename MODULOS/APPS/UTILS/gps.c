#include "gps.h"

/************************************************************************************************************/
// função para cálculo da distância entre dois pontos - dados long/lat de cada ponto em graus
// retorna valor da distância calculada
//
// param entrada: 
// double nLat1: variável que guarda valor de latitude 1
// double nLon1: variável que guarda valor de longitude 1
// double nLat2: variável que guarda valor de latitude 2
// double nLon2: variável que guarda valor de longitude 2
// obs.: valores devem ser passados em graus dd.dddddd
// 
// param saída:
// unsigned long int: distância calculada
/*************************************************************************************************************/

unsigned int CalculaDistancia( double nLat1, double nLon1, double nLat2, double nLon2 )
{
 
  // Converte para radianos
  nLon1 =  ((nLon1)*M_PI)/180;
  nLon2 =  ((nLon2)*M_PI)/180;

  // calcula cos(lon2 - lon1)
  nLon2 =  (nLon2 - nLon1);
  nLon2 = cos(nLon2);

  // Converte para radianos
  nLat1 =  ((nLat1)*M_PI)/180;
  nLat2 =  ((nLat2)*M_PI)/180;  

  // calcula cos(lat1)*cos(lat2)
  nLon1 = cos(nLat1)*cos(nLat2);
  
  // calcula cos(lat1)*cos(lat2)* cos(lon2 - lon1)
  nLon1 = nLon1*nLon2;  
  
  // calcula sen(lat2)*sen(lat1)
  nLon2 = sin(nLat1)*sin(nLat2);
  
  // calcula sen(lat2)*sen(lat1)+ cos(lat1)*cos(lat2)*cos(lon2 - lon1)
  nLat1 = nLon1 + nLon2; 
  
  // saturação para evitar erro em acos
  if(nLat1 > 0.999999999999) nLat1 = 0.999999999999;
  if(nLat1 < -0.999999999999) nLat1 = -0.999999999999;
    
  // calcula arcos (sen(lat2)*sen(lat1)+ cos(lat1)*cos(lat2)*cos(lon2 - lon1)) 
  // nLon1 = acosf(nLon1);
  nLat1 = acos(nLat1);

  // multiplica pelo raio da terra para ter distância em metros
  nLon1 = RAIO_TERRA*nLat1; 

  /* retorna a distância calculada */ 
  /* força inteiro sem sinal para preservar memória */
  if(nLon1 > 0x7FFFFFFF) nLon1 = 0xFFFFFFFF; /* saturação da distância - máx.: 65535 m */ 
  return (unsigned int) nLon1; 
}


//--------------------------------------------
//   Guess the initial root square seed
//
//   Input parameter: unsigned 32-bits 
//   Output parameter: unsigned 16-bits
//--------------------------------------------
/*Root Square Initial Seed*/
unsigned long ASM_FF1(unsigned long A)
{
  asm
  {                            
    ff1.l d0                        // find the fist one
  }                               
  A= (32-A)>>1;                     // first one index diveded by 2
  A = (unsigned long)1<<A;          // multiplies by two
  return(A);                        // Return parameter
}

//--------------------------------------------
//   Square root
//
//   Input parameter: unsigned 32-bits 
//   Output parameter: unsigned 16-bits
//--------------------------------------------
/*Square root*/

#define MAX_INT 5      // maximum number of interactions for square root calculation

unsigned short  SquareRoot(unsigned long A)
{
  unsigned short j;                                     // local variable 
  unsigned long Acc, Acc_temp;
  
  Acc = ASM_FF1(A);                                     // initial seed
  for(j=0;j<MAX_INT;j++)                                // execute maximum of MAX_INT interactions
  {                    
    Acc_temp = (A/Acc + Acc)/2;                         // Calculate interaction
    if((abs(Acc - Acc_temp) < 0x0001)||(Acc_temp == 0)) // Test is precision is good enough     
      break;                                            // break if a good precision was reached
    Acc = Acc_temp;                                     // Save previous interaction
  }
  return((unsigned short)Acc_temp);                     // Return parameter 
}
