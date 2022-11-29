#include "hotspot3D.h" 
extern "C" {
  void kernel_3D_hw(WD_t powerIn[NY*TX], WD_t tempIn[NY*TX], WD_t tempOut[NY*TX], float Cap, float Rx, float Ry, float Rz, float dt, float amb_temp)
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

    for(int i=0; i<ITR/STAGES/2; i++) {
      hotspot3D(powerIn, tempIn, tempOut, cc, ce, cn, ct, stepDivCap, amb_temp);
      hotspot3D(powerIn, tempOut, tempIn, cc, ce, cn, ct, stepDivCap, amb_temp);
    }
  }
}

void hotspot3D(WD_t powerIn[NY*TX], WD_t tempIn[NY*TX], WD_t tempOut[NY*TX], float cc, float ce, float cn, float ct, float stepDivCap, float amb_temp)
{
  #pragma HLS inline off
  WD_t tempIn_stage0, powerIn_stage0, tempOut_stage0, powerOut_stage0;
  hls::stream<WD_t,FIFO_LEN+2> fifo0_0, fifo1_0;
  float ff0_0[PF][NZ], ff1_0[FF_LEN][PF][NZ], ff2_0[PF][NZ];

  hls::stream<WD_t,3> tempInter0;
  hls::stream<WD_t,NX/PF+2> powerInter0;

  WD_t tempIn_stage1, powerIn_stage1, tempOut_stage1, powerOut_stage1;
  hls::stream<WD_t,FIFO_LEN+2> fifo0_1, fifo1_1;
  float ff0_1[PF][NZ], ff1_1[FF_LEN][PF][NZ], ff2_1[PF][NZ];

  hls::stream<WD_t,3> tempInter1;
  hls::stream<WD_t,NX/PF+2> powerInter1;

  WD_t tempIn_stage2, powerIn_stage2, tempOut_stage2, powerOut_stage2;
  hls::stream<WD_t,FIFO_LEN+2> fifo0_2, fifo1_2;
  float ff0_2[PF][NZ], ff1_2[FF_LEN][PF][NZ], ff2_2[PF][NZ];

  hls::stream<WD_t,3> tempInter2;
  hls::stream<WD_t,NX/PF+2> powerInter2;

  WD_t tempIn_stage3, powerIn_stage3, tempOut_stage3, powerOut_stage3;
  hls::stream<WD_t,FIFO_LEN+2> fifo0_3, fifo1_3;
  float ff0_3[PF][NZ], ff1_3[FF_LEN][PF][NZ], ff2_3[PF][NZ];

  hls::stream<WD_t,3> tempInter3;
  hls::stream<WD_t,NX/PF+2> powerInter3;

  WD_t tempIn_stage4, powerIn_stage4, tempOut_stage4, powerOut_stage4;
  hls::stream<WD_t,FIFO_LEN+2> fifo0_4, fifo1_4;
  float ff0_4[PF][NZ], ff1_4[FF_LEN][PF][NZ], ff2_4[PF][NZ];
  
  //stage 0
  #pragma HLS array_partition variable=ff0_0 complete dim=0
  #pragma HLS array_partition variable=ff1_0 complete dim=0
  #pragma HLS array_partition variable=ff2_0 complete dim=0

  #pragma HLS bind_storage variable = fifo0_0 type = FIFO impl = bram
  #pragma HLS bind_storage variable = fifo1_0 type = FIFO impl = bram

  #pragma HLS bind_storage variable = tempInter0 type = FIFO 
  #pragma HLS bind_storage variable = powerInter0 type = FIFO impl = bram

  //stage 1
  #pragma HLS array_partition variable=ff0_1 complete dim=0
  #pragma HLS array_partition variable=ff1_1 complete dim=0
  #pragma HLS array_partition variable=ff2_1 complete dim=0

  #pragma HLS bind_storage variable = fifo0_1 type = FIFO impl = bram
  #pragma HLS bind_storage variable = fifo1_1 type = FIFO impl = bram

  #pragma HLS bind_storage variable = tempInter1 type = FIFO 
  #pragma HLS bind_storage variable = powerInter1 type = FIFO impl = bram

  //stage 2
  #pragma HLS array_partition variable=ff0_2 complete dim=0
  #pragma HLS array_partition variable=ff1_2 complete dim=0
  #pragma HLS array_partition variable=ff2_2 complete dim=0

  #pragma HLS bind_storage variable = fifo0_2 type = FIFO impl = bram
  #pragma HLS bind_storage variable = fifo1_2 type = FIFO impl = bram

  #pragma HLS bind_storage variable = tempInter2 type = FIFO 
  #pragma HLS bind_storage variable = powerInter2 type = FIFO impl = bram

  //stage 3
  #pragma HLS array_partition variable=ff0_3 complete dim=0
  #pragma HLS array_partition variable=ff1_3 complete dim=0
  #pragma HLS array_partition variable=ff2_3 complete dim=0

  #pragma HLS bind_storage variable = fifo0_3 type = FIFO impl = bram
  #pragma HLS bind_storage variable = fifo1_3 type = FIFO impl = bram

  #pragma HLS bind_storage variable = tempInter3 type = FIFO 
  #pragma HLS bind_storage variable = powerInter3 type = FIFO impl = bram

  //stage 4
  #pragma HLS array_partition variable=ff0_4 complete dim=0
  #pragma HLS array_partition variable=ff1_4 complete dim=0
  #pragma HLS array_partition variable=ff2_4 complete dim=0

  #pragma HLS bind_storage variable = fifo0_4 type = FIFO impl = bram
  #pragma HLS bind_storage variable = fifo1_4 type = FIFO impl = bram

  
  fifo_fill:for(int l=0; l<FIFO_LEN; l++){
    if(!fifo0_0.full())
      fifo0_0.write((WD_t)0);
    if(!fifo1_0.full())
      fifo1_0.write((WD_t)0);

    if(!fifo0_1.full())
      fifo0_1.write((WD_t)0);
    if(!fifo1_1.full())
      fifo1_1.write((WD_t)0);

    if(!fifo0_2.full())
      fifo0_2.write((WD_t)0);
    if(!fifo1_2.full())
      fifo1_2.write((WD_t)0);

    if(!fifo0_3.full())
      fifo0_3.write((WD_t)0);
    if(!fifo1_3.full())
      fifo1_3.write((WD_t)0);

    if(!fifo0_4.full())
      fifo0_4.write((WD_t)0);
    if(!fifo1_4.full())
      fifo1_4.write((WD_t)0);
  }


  main_y0:for(int y=0; y<NY+STAGES; y++){
    main_x0:for(int x=0; x<TX; x++){
      #pragma HLS pipeline II=1
      tempIn_stage0  = (y < NY) ?               tempIn[y*TX+x] : WD_t(0);//read till NY rows
      powerIn_stage0 = (y > 0 && y < NY+1) ?   powerIn[(y-1)*TX+x] : WD_t(0);//wait for 1 row and read NY rows
      forward_data(tempIn_stage0, ff0_0, ff1_0, ff2_0, fifo0_0, fifo1_0);
      compute(powerIn_stage0, tempOut_stage0, powerOut_stage0, ff0_0, ff1_0, ff2_0, cc, ce, cn, ct, amb_temp, stepDivCap, x*PF, y-1);
      if(y>0 && y<NY+1) {
        tempInter0.write(tempOut_stage0);
        powerInter0.write(powerOut_stage0);
      }

      tempIn_stage1  = (y > 0 && y < NY+1) ?  tempInter0.read() : WD_t(0);//wait for 1 row and read NY rows
      powerIn_stage1 = (y > 1 && y < NY+2) ? powerInter0.read() : WD_t(0);//wait for 2 rows and read NY rows
      forward_data(tempIn_stage1, ff0_1, ff1_1, ff2_1, fifo0_1, fifo1_1);
      compute(powerIn_stage1, tempOut_stage1, powerOut_stage1, ff0_1, ff1_1, ff2_1, cc, ce, cn, ct, amb_temp, stepDivCap,x*PF, y-2);
      if(y > 1 && y < NY+2) {
        tempInter1.write(tempOut_stage1);
        powerInter1.write(powerOut_stage1);
      }

      tempIn_stage2  = (y > 1 && y < NY+2) ?  tempInter1.read() : WD_t(0);//wait for 2 rows and read NY rows
      powerIn_stage2 = (y > 2 && y < NY+3) ? powerInter1.read() : WD_t(0);//wait for 3 rows and read NY rows
      forward_data(tempIn_stage2, ff0_2, ff1_2, ff2_2, fifo0_2, fifo1_2);
      compute(powerIn_stage2, tempOut_stage2, powerOut_stage2, ff0_2, ff1_2, ff2_2, cc, ce, cn, ct, amb_temp, stepDivCap, x*PF, y-3);
      if(y > 2 && y < NY+3) {
        tempInter2.write(tempOut_stage2);
        powerInter2.write(powerOut_stage2);
      }

      tempIn_stage3  = (y > 2 && y < NY+3) ?  tempInter2.read() : WD_t(0);//wait for 3 rows and read NY rows
      powerIn_stage3 = (y > 3 && y < NY+4) ? powerInter2.read() : WD_t(0);//wait for 4 rows and read NY rows
      forward_data(tempIn_stage3, ff0_3, ff1_3, ff2_3, fifo0_3, fifo1_3);
      compute(powerIn_stage3, tempOut_stage3, powerOut_stage3, ff0_3, ff1_3, ff2_3, cc, ce, cn, ct, amb_temp, stepDivCap,x*PF, y-4);
      if(y > 3 && y < NY+4) {
        tempInter3.write(tempOut_stage3);
        powerInter3.write(powerOut_stage3);
      }

      tempIn_stage4  = (y > 3 && y < NY+4) ?  tempInter3.read() : WD_t(0);//wait for 4 rows and read NY rows
      powerIn_stage4 = (y > 4 && y < NY+5) ? powerInter3.read() : WD_t(0);//wait for 5 rows and read NY rows
      forward_data(tempIn_stage4, ff0_4, ff1_4, ff2_4, fifo0_4, fifo1_4);
      compute(powerIn_stage4, tempOut_stage4, powerOut_stage4, ff0_4, ff1_4, ff2_4, cc, ce, cn, ct, amb_temp, stepDivCap,x*PF, y-5);
      if(y > 4) {
        tempOut[(y-5)*TX+x] = tempOut_stage4;
      }
    }
  }
  

  fifo_flush:for(int l=0; l<FIFO_LEN; l++){
    if(!fifo0_0.empty())
      fifo0_0.read();
    if(!fifo1_0.empty())
      fifo1_0.read();

    if(!fifo0_1.empty())
      fifo0_1.read();
    if(!fifo1_1.empty())
      fifo1_1.read();

    if(!fifo0_2.empty())
      fifo0_2.read();
    if(!fifo1_2.empty())
      fifo1_2.read();

    if(!fifo0_3.empty())
      fifo0_3.read();
    if(!fifo1_3.empty())
      fifo1_3.read();
    
    if(!fifo0_4.empty())
      fifo0_4.read();
    if(!fifo1_4.empty())
      fifo1_4.read();
  }
}

void compute(WD_t powerIn, WD_t &tempOut, WD_t &powerOut, float ff0[PF][NZ], float ff1[FF_LEN][PF][NZ], float ff2[PF][NZ], float cc, float cwe, float cns, float ctb, float amb_temp, float stepDivCap, int x, int y) {
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
      float north  = (y == 0) ?    center : ff2[xx][z];
      float south  = (y == NY-1) ? center : ff0[xx][z];
      float bottom = ff1[1][xx][b];
      float top    = ff1[1][xx][t];

      int p = powerIn.range((xx*NZ+z+1)*32-1, 32*(xx*NZ+z));
      float power  = *(float*)(&p);

      float result = ((cc*center + power*stepDivCap) + cwe*(east+west)) + (cns*(north+south) +  ctb*((top+bottom)+amb_temp));
      // #ifndef __SYNTHESIS__
      // if(x+xx==0 && z==0){
      //     // fwd_data_l1:for(int l = FF_LEN-1; l>=0; l--)
      //     //   fwd_data_xx2:for(int xx=0; xx<PF; xx++)
      //     //     fwd_data_z2:for(int z=0; z<NZ; z++) 
      //     //       printf("ff1[%d][%d][%d]=%f\n",l,xx,z,ff1[l][xx][z]);
        
      //   printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", center, west, east, north, south, bottom, top, power*stepDivCap,result);
      // }
      // #endif
      tempOut.range(32*(xx*NZ+z+1)-1, 32*(xx*NZ+z)) = *(int*)(&result);
    }
  }
  powerOut = powerIn;
}


void forward_data(WD_t tempIn, float ff0[PF][NZ], float ff1[FF_LEN][PF][NZ], float ff2[PF][NZ], hls::stream<WD_t,FIFO_LEN+2> &fifo0, hls::stream<WD_t,FIFO_LEN+2> &fifo1) {
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
