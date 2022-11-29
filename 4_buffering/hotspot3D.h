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

#define WIDTH_FACTOR 16 //(512/32)
#define WD_t ap_uint<WIDTH_FACTOR*32>
#define STAGES 1
#define ITY (TY+2*STAGES)
#define PF (WIDTH_FACTOR/NZ)
#define TX (NX/PF)
#define FF_LEN 3
#define FIFO_LEN (NX/PF)-(FF_LEN-1)



void kernel_3D_hw(WD_t* powerIn, WD_t* tempIn, WD_t* tempOut, float Cap, float Rx, float Ry, float Rz, float dt, float amb_temp);
void load0(WD_t* src, WD_t dst[ITY][TX], int row);
void load1(WD_t* src, WD_t dst[ITY][TX], int row);
void store(WD_t* dst, WD_t src[TY][TX], int row);
void workload(WD_t powerIn[ITY][TX], WD_t tempIn[ITY][TX], WD_t tempOut[TY][TX], float cc, float cwe, float cns, float ctb, float stepDivCap, float amb_temp);
void compute(WD_t powerIn, WD_t &tempOut, WD_t &powerOut, float ff0[PF][NZ], float ff1[FF_LEN][PF][NZ], float ff2[PF][NZ], float cc, float cwe, float cns, float ctb, float amb_temp, float stepDivCap, int x);
void forward_data(WD_t tempIn, float ff0[PF][NZ], float ff1[FF_LEN][PF][NZ], float ff2[PF][NZ], hls::stream<WD_t,FIFO_LEN> &fifo0, hls::stream<WD_t,FIFO_LEN> &fifo1);

#endif
