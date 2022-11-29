#ifndef _H_3D_H_
#define _H_3D_H_

#include "hls_stream.h"
#include "ap_int.h"

#ifndef __SYNTHESIS__
#include <stdio.h>
#endif

#define NX 512
#define NY 512
#define NZ 8

#ifndef __SYNTHESIS__
#define ITR 2
#endif

#ifdef __SYNTHESIS__
#define ITR 20000
#endif

#ifndef __SYNTHESIS__
#define TY 64
#endif

#ifdef __SYNTHESIS__
#define TY 64
#endif

#define STAGES 1
#define ITY (TY+2*STAGES)



void kernel_3D_hw(float* powerIn, float* tempIn, float* tempOut, float Cap, float Rx, float Ry, float Rz, float dt, float amb_temp);
void load(float* src, float dst[ITY][NX][NZ], int row);
void store(float* dst, float src[TY][NX][NZ], int row);
void compute(float powerIn[ITY][NX][NZ], float tempIn[ITY][NX][NZ],float tempOut[ITY][NX][NZ], float cc, float cwe, float cns, float ctb, float stepDivCap, float amb_temp); 
#endif
