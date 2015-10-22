/**********************************************************************************
@file   gpsr.c
@brief  GPSR routing functions
@authors: Gustavo Weber Denardin
          Carlos Henrique Barriquello

Copyright (c) <2009-2013> <Universidade Federal de Santa Maria>

  * Software License Agreement
  *
  * The Software is owned by the authors, and is protected under 
  * applicable copyright laws. All rights are reserved.
  *
  * The above copyright notice shall be included in
  * all copies or substantial portions of the Software.
  *  
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  * THE SOFTWARE. 
*********************************************************************************/

#include "gpsr.h"  

static INT32U abs(INT32S x) {
  
  if (x<0) return -x;
  else return x;
}

/* Euclidean implementation */ 
double diferenca_angulo(LATITUDE node_lat, LONGITUDE node_long, LATITUDE neighbor_lat, LONGITUDE neighbor_long, INT32S *ant_node_lat, INT32S *ant_node_long, INT32S *dst_lat, INT32S *dst_long)
{
	LATITUDE      no_anterior_lat;
	LONGITUDE     no_anterior_long;
	LATITUDE      possivel_vizinho_lat;
	LONGITUDE     possivel_vizinho_long;	
	
	double        ang_possivel_vizinho;
	double        ang_no_anterior;
  INT8U         quad_p, quad_a;

	possivel_vizinho_lat.x = neighbor_lat.x - node_lat.x;
	possivel_vizinho_long.y = neighbor_long.y - node_long.y;

	if (((*ant_node_lat) != 0) && ((*ant_node_long) != 0))
	{ 
		no_anterior_lat.x = *ant_node_lat - node_lat.x;
		no_anterior_long.y = *ant_node_long - node_long.y;
	}else
	{
		no_anterior_lat.x = *dst_lat - node_lat.x;
		no_anterior_long.y = *dst_long - node_long.y;
	}


	// Quadrante do possivel vizinho
	if (possivel_vizinho_lat.x >= 0)
	{
		if (possivel_vizinho_long.y >= 0)
		{
			quad_p = 1;
		}else
		{
			quad_p = 4;
		}
	}else
	{
		if (possivel_vizinho_long.y >= 0)
		{
			quad_p = 2;
		}else
		{
			quad_p = 3;
		}
	}

	// Quadrante do vizinho anterior
	if (no_anterior_lat.x >= 0)
	{
		if (no_anterior_long.y >= 0)
		{
			quad_a = 1;
		}else
		{
			quad_a = 4;
		}
	}else
	{
		if (no_anterior_long.y >= 0)
		{
			quad_a = 2;
		}else
		{
			quad_a = 3;
		}
	}

	if (possivel_vizinho_lat.x != 0)
	{
		ang_possivel_vizinho = (double)abs(possivel_vizinho_long.y)/(double)abs(possivel_vizinho_lat.x);
		ang_possivel_vizinho = atan(ang_possivel_vizinho);
	}else
	{
		ang_possivel_vizinho = (double)((double)3.141592/2);
	}

	if (no_anterior_lat.x != 0)
	{
		ang_no_anterior = (double)abs(no_anterior_long.y)/(double)abs(no_anterior_lat.x);
		ang_no_anterior = atan(ang_no_anterior);
	}else
	{
		ang_no_anterior = (double)((double)1.570796);
	}

	switch(quad_p)
	{	
		case 1:
			break;
		case 2:
			ang_possivel_vizinho = (double)3.141592 - ang_possivel_vizinho;
			break;
		case 3:
			ang_possivel_vizinho = (double)3.141592 + ang_possivel_vizinho;
			break;
		case 4:
			ang_possivel_vizinho = (double)(2*(double)3.141592) - ang_possivel_vizinho;
			break;
	}	

	switch(quad_a)
	{	
		case 1:
			break;
		case 2:
			ang_no_anterior = (double)3.141592 - ang_no_anterior;
			break;
		case 3:
			ang_no_anterior = (double)3.141592 + ang_no_anterior;
			break;
		case 4:
			ang_no_anterior = (double)(2*(double)3.141592) - ang_no_anterior;
			break;
	}

	ang_possivel_vizinho = (double)(2*(double)3.141592) - ang_no_anterior + ang_possivel_vizinho;
	if (ang_possivel_vizinho > (double)(2*(double)3.141592))
	{
		ang_possivel_vizinho = (double)(2*(double)3.141592) - ang_possivel_vizinho;
	}
	
	ang_possivel_vizinho = fabs(ang_possivel_vizinho);
	if (!((ang_possivel_vizinho > 0.0001) ||(ang_possivel_vizinho < -0.0001)))
	{
		ang_possivel_vizinho = (double)(2*(double)3.141592);
	}
	
	ang_no_anterior = 0;
	
	return (ang_possivel_vizinho);
}

/* Taxicab implementation */
INT32U ang_taxicab(INT32S x, INT32S y);

INT32U diferenca_ang_taxicab(LATITUDE node_lat, LONGITUDE node_long, LATITUDE neighbor_lat, LONGITUDE neighbor_long, INT32S *ant_node_lat, INT32S *ant_node_long, INT32S *dst_lat, INT32S *dst_long)
{

	LATITUDE      no_anterior_lat;
	LONGITUDE     no_anterior_long;
	LATITUDE      possivel_vizinho_lat;
	LONGITUDE     possivel_vizinho_long;	
	
	INT32U        ang_possivel_vizinho;
	INT32U        ang_no_anterior; 	


	possivel_vizinho_lat.x = neighbor_lat.x - node_lat.x;
	possivel_vizinho_long.y = neighbor_long.y - node_long.y;

	if (((*ant_node_lat) != 0) && ((*ant_node_long) != 0))
	{ 
		no_anterior_lat.x = *ant_node_lat - node_lat.x;
		no_anterior_long.y = *ant_node_long - node_long.y;
	}else
	{
		no_anterior_lat.x = *dst_lat - node_lat.x;
		no_anterior_long.y = *dst_long - node_long.y;
	}

  // Angulo do possivel vizinho
  ang_possivel_vizinho = ang_taxicab(possivel_vizinho_lat.x,possivel_vizinho_long.y);  
 
	// Angulo do vizinho anterior
	ang_no_anterior = ang_taxicab(no_anterior_lat.x,no_anterior_long.y);  

  if (ang_possivel_vizinho >= ang_no_anterior){       
       ang_possivel_vizinho = ang_possivel_vizinho - ang_no_anterior;
  }else{
       ang_possivel_vizinho = (8<<20) + ang_possivel_vizinho - ang_no_anterior;
  }
  
  return  ang_possivel_vizinho;

}

INT32U ang_taxicab(INT32S x, INT32S y)
{  

  INT32U ang_t;

	if (x >= 0)
	{
		if (y >= 0)
		{
			ang_t =  (INT32U)((INT64S)((INT64S)(2*y) << (INT32U)20) / (INT64S)(x + y));
		}else
		{
			ang_t = (INT32U)((INT64S)((INT64S)(8*x - 6*y ) << (INT32U)20) / (INT64S)(x - y));
		}
	}else
	{
		if (y >= 0)
		{
			 ang_t = (INT32U)((INT64S)((INT64S)(2*y - 4*x) << (INT32U)20) / (INT64S)(y - x));
		}else
		{
			 ang_t = (INT32U)((INT64S)((INT64S)(6*y + 4*x) << (INT32U)20) / (INT64S)(x + y));
		}
	}
	
	return ang_t;
	
}


