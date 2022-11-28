#include "hotspot3D.h" 

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

  float local_tin [ITY][NX][NZ];
  float local_pin [ITY][NX][NZ];
  float local_tout [TY][NX][NZ];

  main_i:for(int i = 0; i < ITR/2; i++) {
    main_j0:for (int j = 0; j<NY; j+=TY) {
      load(tempIn, local_tin, j);
      load(powerIn, local_pin, j);
      compute(local_pin, local_tin, local_tout, cc, ce, cn, ct, stepDivCap, amb_temp);
      store(tempOut, local_tout, j);
    }
    main_j1:for (int j = 0; j<NY; j+=TY) {
      load(tempOut, local_tin, j);
      load(powerIn, local_pin, j);
      compute(local_pin, local_tin, local_tout, cc, ce, cn, ct, stepDivCap, amb_temp);
      store(tempIn, local_tout, j);
    }
  }
}



void load(float* src, float dst[ITY][NX][NZ], int row) {
  #pragma HLS inline off
  for(int y = 0; y < ITY; y++) {
    for(int x = 0; x < NX; x++) {
      for(int z = 0; z < NZ; z++) {
        int ty = (y+row-1);
        ty = (ty < 0) ? 0: (ty > NY-1) ? NY-1 : ty;
        dst[y][x][z] = src[ty*NX*NZ + x*NZ + z];
      }
    }
  }
}

void store(float* dst, float src[TY][NX][NZ], int row){
  #pragma HLS inline off
  for(int y = 0; y < TY; y++) {
    for(int x = 0; x < NX; x++) {
      for(int z = 0; z < NZ; z++) {
        dst[(row+y)*NX*NZ + x*NZ + z] = src[y][x][z];
      }
    }
  }
}

void compute(float powerIn[ITY][NX][NZ], float tempIn[ITY][NX][NZ], float tempOut[TY][NX][NZ], float cc, float cwe, float cns, float ctb, float stepDivCap, float amb_temp) {
  #pragma HLS inline off
  for(int yy = 0; yy < TY; yy++) {
    for(int x = 0; x < NX; x++) {
      for(int z = 0; z < NZ; z++) {
          int b = (z == 0) ? z      : z - 1;
          int t = (z == NZ - 1) ? z : z + 1;
          int w = (x == 0) ? x      : x - 1;
          int e = (x == NX - 1) ? x : x + 1;
          #ifndef __SYNTHESIS__
          if(yy==0 && x==0)
            printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", tempIn[yy+1][x][z], tempIn[yy+1][w][z], tempIn[yy+1][e][z], tempIn[yy][x][z], tempIn[yy+2][x][z], tempIn[yy+1][x][b], tempIn[yy+1][x][t], stepDivCap *powerIn[yy+1][x][z]);
          #endif
          tempOut[yy][x][z] = tempIn[yy+2][x][z]*cns + tempIn[yy+1][x][z]*cc +  tempIn[yy][x][z]*cns + tempIn[yy+1][w][z]*cwe + tempIn[yy+1][e][z]*cwe + tempIn[yy+1][x][t]*ctb + tempIn[yy+1][x][b]*ctb + stepDivCap * powerIn[yy+1][x][z] + ctb*amb_temp;
      }
    }
  }
}