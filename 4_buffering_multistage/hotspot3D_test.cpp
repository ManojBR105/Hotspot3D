#include <iostream>
#include <stdio.h>
#include <time.h>
#include <assert.h>
#include <stdlib.h> 
#include <math.h> 
#include <sys/time.h>
#include <string.h>
#include "hotspot3D.h"

#define STR_SIZE (256)
#define MAX_PD	(3.0e6)
/* required precision in degrees	*/
#define PRECISION	0.001
#define SPEC_HEAT_SI 1.75e6
#define K_SI 100
/* capacitance fitting factor	*/
#define FACTOR_CHIP	0.5

std::string powerfile = "/net/home/mba151/data/hotspot3D/power_512x8";
std::string tempfile = "/net/home/mba151/data/hotspot3D/temp_512x8";
std::string outfile = "/net/home/mba151/hotspot3D_HLS_yxz/4_buffering_multistage/output_512x8.out";

const float t_chip = 0.0005;
const float chip_height = 0.016; 
const float chip_width = 0.016; 
const float amb_temp = 80.0;

void fatal(std::string str)
{
    char *s = new char[str.length() + 1];
    strcpy(s, str.c_str());
    fprintf(stderr, "Error: %s\n", s);
}

void readinput(float *vect, int grid_rows, int grid_cols, int layers, char *file) {
    int i,j,k;
    FILE *fp;
    char str[STR_SIZE];
    float val;

    if( (fp  = fopen(file, "r" )) ==0 )
      fatal( (const char*)"The file was not opened" );


    for (i=0; i <grid_rows*grid_cols*layers; i++) {
            if (fgets(str, STR_SIZE, fp) == NULL) fatal((const char*)"Error reading file\n");
            if (feof(fp))
              fatal((const char*)"not enough lines in file");
            if ((sscanf(str, "%f", &val) != 1))
              fatal((const char*)"invalid file format");
            vect[i] = val;
          }

    fclose(fp);	

}


void writeoutput(float *vect, int grid_rows, int grid_cols, int layers, char *file) {

    int i,j,k, index=0;
    FILE *fp;
    char str[STR_SIZE];

    if( (fp = fopen(file, "w" )) == 0 )
      printf( "The file was not opened\n" );

    for (i=0; i < grid_cols*grid_rows*layers; i++)
          {
            sprintf(str, "%d\t%g\n", index, vect[i]);
            fputs(str,fp);
            index++;
          }

    fclose(fp);	
}

void computeTempCPU(float *pIn, float* tIn, float *tOut, 
        int nx, int ny, int nz, float Cap, 
        float Rx, float Ry, float Rz, 
        float dt, int numiter) 
{   float ce, cw, cn, cs, ct, cb, cc;
    float stepDivCap = dt / Cap;
    ce = cw =stepDivCap/ Rx;
    cn = cs =stepDivCap/ Ry;
    ct = cb =stepDivCap/ Rz;

    cc = 1.0 - (2.0*ce + 2.0*cn + 3.0*ct);

    int c,w,e,n,s,b,t;
    int x,y,z;
    int i = 0;
    do{
        for(y = 0; y < ny; y++)
            for(x = 0; x < nx; x++)
                for(z = 0; z < nz; z++)
                {
                    c = y * nx * nz + x * nz + z;

                    w = (x == 0) ? c      : c - nz;
                    e = (x == nx - 1) ? c : c + nz;
                    n = (y == 0) ? c      : c - nx * nz;
                    s = (y == ny - 1) ? c : c + nx * nz;
                    b = (z == 0) ? c      : c - 1;
                    t = (z == nz - 1) ? c : c + 1;


                    tOut[c] = tIn[c]*cc + tIn[n]*cn + tIn[s]*cs + tIn[e]*ce + tIn[w]*cw + tIn[t]*ct + tIn[b]*cb + (dt/Cap) * pIn[c] + ct*amb_temp;
                }
        float *temp = tIn;
        tIn = tOut;
        tOut = temp; 
        i++;
    }
    while(i < numiter);

}

void accuracy(float *arr1, float *arr2, int len)
{
    float rmse = 0.0; 
    int i;
    int count = 0;
    for(i = 0; i < len; i++)
    {
        rmse += (arr1[i]-arr2[i]) * (arr1[i]-arr2[i]);
        if(abs((arr1[i]-arr2[i]))>0.001){
          printf("y=%d\tx=%d\tz=%d\n", i/(NX*NZ), (i/NZ)%NX, i%NZ);
          count++;
        }
    }
    printf("count=%d\n", count);
    rmse =sqrt(rmse/len);
    printf("Error : %e\n", rmse);
}

void usage(int argc, char **argv)
{
    fprintf(stderr, "Usage: %s <rows/cols> <layers> <iterations> <powerFile> <tempFile> <outputFile>\n", argv[0]);
    fprintf(stderr, "\t<rows/cols>  - number of rows/cols in the grid (positive integer)\n");
    fprintf(stderr, "\t<layers>  - number of layers in the grid (positive integer)\n");

    fprintf(stderr, "\t<iteration> - number of iterations\n");
    fprintf(stderr, "\t<powerFile>  - name of the file containing the initial power values of each cell\n");
    fprintf(stderr, "\t<tempFile>  - name of the file containing the initial temperature values of each cell\n");
    fprintf(stderr, "\t<outputFile - output file\n");
    exit(1);
}



int main(int argc, char** argv)
{
    int iterations = ITR;

    char *pfile = new char[powerfile.length() + 1];
    strcpy(pfile, powerfile.c_str());
    char *tfile = new char[tempfile.length() + 1];
    strcpy(tfile, tempfile.c_str());
    char *ofile = new char[outfile.length() + 1];
    strcpy(ofile, outfile.c_str());

    std::cout<<pfile<<std::endl;
    std::cout<<tfile<<std::endl;
    std::cout<<ofile<<std::endl;
    //testFile = argv[7];
    int numCols = NX;
    int numRows = NY;
    int layers = NZ;
    /* calculating parameters*/

    float dx = chip_height/numRows;
    float dy = chip_width/numCols;
    float dz = t_chip/layers;

    float Cap = FACTOR_CHIP * SPEC_HEAT_SI * t_chip * dx * dy;
    float Rx = dy / (2.0 * K_SI * t_chip * dx);
    float Ry = dx / (2.0 * K_SI * t_chip * dy);
    float Rz = dz / (K_SI * dx * dy);

    // cout << Rx << " " << Ry << " " << Rz << endl;
    float max_slope = MAX_PD / (FACTOR_CHIP * t_chip * SPEC_HEAT_SI);
    float dt = PRECISION / max_slope;


    float *powerIn, *temp_sw, *temp_hw;// *pCopy;
    //    float *d_powerIn, *d_tempIn, *d_tempOut;
    int size = numCols * numRows * layers;

    powerIn = (float*)malloc(size * sizeof(float));
    temp_sw = (float*)malloc(size * sizeof(float));
    temp_hw = (float*)malloc(size * sizeof(float));
    // tempIn = (float*)calloc(size,sizeof(float));
    // tempOut = (float*)calloc(size, sizeof(float));
    //pCopy = (float*)calloc(size,sizeof(float));
    float* answer_sw = (float*)malloc(size * sizeof(float));
    float* answer_hw = (float*)malloc(size * sizeof(float));

    readinput(powerIn, numRows, numCols, layers, pfile);
    readinput(temp_sw, numRows, numCols, layers, tfile);

    memcpy(temp_hw, temp_sw, size * sizeof(float));

    struct timeval start, stop;
    float time1, time2;

    gettimeofday(&start,NULL);
    kernel_3D_hw((WD_t*)powerIn, (WD_t*)temp_hw, (WD_t*)answer_hw, Cap, Rx, Ry, Rz, dt, amb_temp);
    gettimeofday(&stop,NULL);
    time1 = (stop.tv_usec-start.tv_usec)*1.0e-6 + stop.tv_sec - start.tv_sec;
    gettimeofday(&start,NULL);
    computeTempCPU(powerIn, temp_sw, answer_sw, numCols, numRows, layers, Cap, Rx, Ry, Rz, dt, iterations);
    gettimeofday(&stop,NULL);
    time2 = (stop.tv_usec-start.tv_usec)*1.0e-6 + stop.tv_sec - start.tv_sec;
    

    printf("FPGA Time: %.3f (s)\n",time1);
    printf("CPU Time: %.3f (s)\n",time2);
    accuracy(temp_sw,temp_hw,numRows*numCols*layers);
    writeoutput(temp_hw, numRows, numCols, layers, ofile);
    
    // free(tempIn);
    // free(tempOut); 
    free(powerIn);
    free(answer_hw);
    free(answer_sw);
    free(temp_hw);
    free(temp_sw);
    return 0;
}	


