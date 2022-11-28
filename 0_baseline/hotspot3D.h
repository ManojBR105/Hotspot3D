#ifndef _H_3D_H_
#define _H_3D_H_

#define NX 512
#define NY 512
#define NZ 8

#ifndef __SYNTHESIS__
#define ITR 2
#endif

#ifdef __SYNTHESIS__
#define ITR 10000
#endif

#define TY 64

#ifndef __SYNTHESIS__
#include <stdio.h>
#endif


void kernel_3D_hw(float* powerIn, float* tempIn, float* tempOut, float Cap, float Rx, float Ry, float Rz, float dt, float amb_temp);
void compute(float* powerIn, float* tempIn, float* tempOut, float cc, float cwe, float cns, float ctb, float stepDivCap, float amb_temp);
// void load()

#endif
