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

  #pragma HLS array_partition variable=local_tin complete dim=3
  #pragma HLS array_partition variable=local_tin cyclic factor=8 dim=2 //factor = PF/2
  #pragma HLS array_partition variable=local_pin complete dim=3
  #pragma HLS array_partition variable=local_pin cyclic factor=8 dim=2 //factor = PF/2
  #pragma HLS array_partition variable=local_tout complete dim=3
  #pragma HLS array_partition variable=local_tout cyclic factor=16 dim=2 //factor = PF
  #pragma HLS bind_storage variable = local_tin type = RAM_2P impl = uram
  #pragma HLS bind_storage variable = local_pin type = RAM_2P impl = uram
  #pragma HLS bind_storage variable = local_tout type = RAM_T2P impl = bram

  main_i:for(int i = 0; i < ITR/2; i++) {
    main_j0:for (int j = 0; j<NY; j+=TY) {
      load(tempIn, local_tin, j);
      load(powerIn, local_pin, j);
      workload(local_pin, local_tin, local_tout, cc, ce, cn, ct, stepDivCap, amb_temp);
      store(tempOut, local_tout, j);
    }
    main_j1:for (int j = 0; j<NY; j+=TY) {
      load(tempOut, local_tin, j);
      load(powerIn, local_pin, j);
      workload(local_pin, local_tin, local_tout, cc, ce, cn, ct, stepDivCap, amb_temp);
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

void workload(float powerIn[ITY][NX][NZ], float tempIn[ITY][NX][NZ], float tempOut[TY][NX][NZ], float cc, float cwe, float cns, float ctb, float stepDivCap, float amb_temp) {
  #pragma HLS inline off
  WD_t tempIn_stage0, powerIn_stage0, tempOut_stage0, powerOut_stage0;
  hls::stream<WD_t,FIFO_LEN> fifo0_0, fifo1_0;
  float ff0_0[PF][NZ], ff1_0[FF_LEN][PF][NZ], ff2_0[PF][NZ];
  
  #pragma HLS array_partition variable=ff0_0 complete dim=0
  #pragma HLS array_partition variable=ff1_0 complete dim=0
  #pragma HLS array_partition variable=ff2_0 complete dim=0

  #pragma HLS bind_storage variable = fifo0_0 type = FIFO impl = bram
  #pragma HLS bind_storage variable = fifo1_0 type = FIFO impl = bram

  fifo_fill:for(int l=0; l<FIFO_LEN; l++){
    if(!fifo0_0.full())
      fifo0_0.write((WD_t)0);
    if(!fifo1_0.full())
      fifo1_0.write((WD_t)0);
  }

  for(int yy = 0; yy < ITY; yy++) {
    for(int x = 0; x < NX/PF; x++) {
      tempIn_stage0  = read(tempIn, x, yy);//read till NY rows
      powerIn_stage0 = yy>0 ? read(powerIn, x, yy-1) : (WD_t)0;//wait for 1 row and read NY rows
      forward_data(tempIn_stage0, ff0_0, ff1_0, ff2_0, fifo0_0, fifo1_0);
      compute(powerIn_stage0, tempOut_stage0, powerOut_stage0, ff0_0, ff1_0, ff2_0, cc, cwe, cns, ctb, amb_temp, stepDivCap, x*PF);
      if(yy>1 && yy<ITY) {
        write(tempOut, tempOut_stage0, x, yy-2);
      }
    }
  }

  fifo_flush:for(int l=0; l<FIFO_LEN; l++){
    if(!fifo0_0.empty())
      fifo0_0.read();
    if(!fifo1_0.empty())
      fifo1_0.read();
  }
}

WD_t read(float local_buffer[ITY][NX][NZ], int x, int y) {
  #pragma HLS inline
  WD_t data;
  read_xx:for(int xx=0; xx<PF; xx++){
    #pragma HLS unroll
    read_z:for(int z=0; z<NZ; z++){
      #pragma HLS unroll
      int idx = xx*NZ+z;
      data.range((idx+1)*32-1, idx*32) = *((int*)(&local_buffer[y][x+xx][z]));
    }
  }
  return data;
}

void write(float local_buffer[TY][NX][NZ], WD_t data, int x, int y) {
  #pragma HLS inline
  write_xx:for(int xx=0; xx<PF; xx++){
    #pragma HLS unroll
    write_z:for(int z=0; z<NZ; z++){
      #pragma HLS unroll
      int idx = xx*NZ+z;
      int val = data.range((idx+1)*32-1, idx*32);
      local_buffer[y][x+xx][z] = *((float*)(&val));
    }
  }
}

void compute(WD_t powerIn, WD_t &tempOut, WD_t &powerOut, float ff0[PF][NZ], float ff1[FF_LEN][PF][NZ], float ff2[PF][NZ], float cc, float cwe, float cns, float ctb, float amb_temp, float stepDivCap, int x) {
  #pragma HLS inline
  comp_xx:for(int xx=0; xx<PF; xx++) {
    #pragma HLS unroll 
    comp_z:for(int z = 0; z < NZ; z++) {
      #pragma HLS unroll 
      int b = (z == 0) ?          z : z  - 1;
      int t = (z == NZ - 1) ?     z : z  + 1;
      float center = ff1[1][xx][z];
      float west   = (x+xx == 0) ?    center : (xx == 0) ?   ff1[2][PF-1][z] : ff1[1][xx-1][z];
      float east   = (x+xx == NX-1) ? center : (xx == PF - 1) ? ff1[0][0][z] : ff1[1][xx+1][z];
      float north  = ff2[xx][z];
      float south  = ff0[xx][z];
      float bottom = ff1[1][xx][b];
      float top    = ff1[1][xx][t];

      int p = powerIn.range((xx*NZ+z+1)*32-1, 32*(xx*NZ+z));
      float power  = *(float*)(&p);

      float result = ((cc*center + power*stepDivCap) + cwe*(east+west)) + (cns*(north+south) +  ctb*((top+bottom)+amb_temp));
      tempOut.range(32*(xx*NZ+z+1)-1, 32*(xx*NZ+z)) = *(int*)(&result);
    }
  }
  powerOut = powerIn;
}


void forward_data(WD_t tempIn, float ff0[PF][NZ], float ff1[FF_LEN][PF][NZ], float ff2[PF][NZ], hls::stream<WD_t,FIFO_LEN> &fifo0, hls::stream<WD_t,FIFO_LEN> &fifo1) {
  #pragma HLS inline 
  WD_t data_out1;
  data_out1 = fifo1.read();

  fwd_data_xx4:for(int xx=0; xx<PF; xx++){
    #pragma HLS unroll 
    fwd_data_z4:for(int z=0; z<NZ; z++) {
      #pragma HLS unroll
        int d = data_out1.range((xx*NZ+z+1)*32-1, (xx*NZ+z)*32);
        ff2[xx][z] = *((float*)(&d));
    }
  }

  WD_t data_in1;
  fwd_data_xx3:for(int xx=0; xx<PF; xx++)
    #pragma HLS unroll 
    fwd_data_z3:for(int z=0; z<NZ; z++) 
      #pragma HLS unroll    
      data_in1.range((xx*NZ+z+1)*32-1, (xx*NZ+z)*32) = *((int*)(&ff1[FF_LEN-1][xx][z]));

  fifo1.write(data_in1);

  fwd_data_l1:for(int l = FF_LEN-1; l>0; l--)
    #pragma HLS unroll
      fwd_data_xx2:for(int xx=0; xx<PF; xx++)
        #pragma HLS unroll 
        fwd_data_z2:for(int z=0; z<NZ; z++) 
          #pragma HLS unroll 
          ff1[l][xx][z] = ff1[l - 1][xx][z];
      
  WD_t data_out0;
  data_out1 = fifo0.read();

  fwd_data_xx1:for(int xx=0; xx<PF; xx++){
    #pragma HLS unroll 
    fwd_data_z1:for(int z=0; z<NZ; z++) {
      #pragma HLS unroll
        int d = data_out1.range((xx*NZ+z+1)*32-1, (xx*NZ+z)*32);
        ff1[0][xx][z] = *((float*)(&d));
    }
  }

  WD_t data_in0;
  fwd_data_xx0:for(int xx=0; xx<PF; xx++)
    #pragma HLS unroll 
    fwd_data_z0:for(int z=0; z<NZ; z++) 
      #pragma HLS unroll    
      data_in0.range((xx*NZ+z+1)*32-1, (xx*NZ+z)*32) = *((int*)(&ff0[xx][z]));

  fifo0.write(data_in0);

  fwd_data_xx:for(int xx=0; xx<PF; xx++) {
    #pragma HLS unroll 
    fwd_data_z:for(int z = 0; z < NZ; z++) {
      #pragma HLS unroll 
      int t = tempIn.range((xx*NZ+z+1)*32-1, (xx*NZ+z)*32);
      ff0[xx][z] = *((float*)(&t));
    }
  }
}