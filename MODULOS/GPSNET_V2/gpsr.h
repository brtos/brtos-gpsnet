/**********************************************************************************
@file   gpsr.h
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


#ifndef GPSR_H
#define GPSR_H

#include <math.h>
#include "network.h"
#include "BRTOS.h"


double diferenca_angulo(LATITUDE node_lat, LONGITUDE node_long, LATITUDE neighbor_lat, LONGITUDE neighbor_long, INT32S *ant_node_lat, INT32S *ant_node_long, INT32S *dst_lat, INT32S *dst_long);
INT32U diferenca_ang_taxicab(LATITUDE node_lat, LONGITUDE node_long, LATITUDE neighbor_lat, LONGITUDE neighbor_long, INT32S *ant_node_lat, INT32S *ant_node_long, INT32S *dst_lat, INT32S *dst_long);

#endif