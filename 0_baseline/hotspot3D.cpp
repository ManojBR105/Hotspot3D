#include "hotspot3D.h" 

void compute(float* powerIn, float* tempIn, float* tempOut, float cc, float cwe, float cns, float ctb, float stepDivCap, float amb_temp) {
#pragma HLS inline off
for(int y = 0; y < NY; y++){
  for(int x = 0; x < NX; x++){
      for(int z = 0; z < NZ; z++){
          int c = y * NX * NZ + x * NZ + z;

          int w = (x == 0) ? c      : c - NZ;
          int e = (x == NX - 1) ? c : c + NZ;
          int n = (y == 0) ? c      : c - NX * NZ;
          int s = (y == NY - 1) ? c : c + NX * NZ;
          int b = (z == 0) ? c      : c - 1;
          int t = (z == NZ - 1) ? c : c + 1;

          tempOut[c] = tempIn[c]*cc + tempIn[n]*cns + tempIn[s]*cns + tempIn[e]*cwe + tempIn[w]*cwe + tempIn[t]*ctb + tempIn[b]*ctb + stepDivCap * powerIn[c] + ctb*amb_temp;

          #ifndef __SYNTHESIS__
          if(y==0 && x==0){
            printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", tempIn[c], tempIn[w], tempIn[e], tempIn[n], tempIn[s], tempIn[b], tempIn[t], stepDivCap * powerIn[c], tempOut[c]);
          }
          #endif
      }
    }
  }
}

void kernel_3D_hw(float* powerIn, float* tempIn, float* tempOut, float Cap, float Rx, float Ry, float Rz, float dt, float amb_temp)
{
  #pragma HLS interface m_axi port=powerIn bundle=gmem0 offset=slave
  #pragma HLS interface m_axi port=tempIn bundle=gmem1 offset=slave
  #pragma HLS interface m_axi port=tempOut bundle=gmem2 offset=slave
  #pragma HLS interface s_axilite port=powerIn bundle=control
  #pragma HLS interface s_axilite port=tempIn bundle=control
  #pragma HLS interface s_axilite port=tempOut bundle=control
  #pragma HLS interface s_axilite port=Cap bundle=control
  #pragma HLS interface s_axilite port=Rx bundle=control
  #pragma HLS interface s_axilite port=Ry bundle=control
  #pragma HLS interface s_axilite port=Rz bundle=control
  #pragma HLS interface s_axilite port=dt bundle=control
  #pragma HLS interface s_axilite port=return bundle=control

  float stepDivCap = dt / Cap;
  float ce = stepDivCap/ Rx;
  float cn = stepDivCap/ Ry;
  float ct = stepDivCap/ Rz;
  float cc = 1.0 - (2.0*ce + 2.0*cn + 3.0*ct);

  // main: for (int y = 0; y<NY; y+=TY){
  // load()
  for(int i = 0; i < ITR/2; i++) {
    compute(powerIn, tempIn, tempOut, cc, ce, cn, ct, stepDivCap, amb_temp);
    compute(powerIn, tempOut, tempIn, cc, ce, cn, ct, stepDivCap, amb_temp);
  }
  // }
}
  // }
