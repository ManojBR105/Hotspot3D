/**
* Copyright (C) 2019-2021 Xilinx, Inc
*
* Licensed under the Apache License, Version 2.0 (the "License"). You may
* not use this file except in compliance with the License. A copy of the
* License is located at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governing permissions and limitations
* under the License.
*/

/*******************************************************************************

Description:

    Vitis matrix multiply example which showcases how the host code works.

*******************************************************************************/
// OpenCL utility layer include
#include "xcl2.hpp"
#include <vector>
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

// std::string powerfile = "/net/home/dst5/ENSC894/project/hotspot3D/original/rodinia_3.1/data/hotspot3D/power_512x8";
// std::string tempfile = "/net/home/dst5/ENSC894/project/hotspot3D/original/rodinia_3.1/data/hotspot3D/temp_512x8";
// std::string outfile = "/net/home/dst5/ENSC894/project/optimizations/gitTemp/ENSC894_Final_Proj/3_coelescing/output_512x8.out";
// std::string outfile1 = "/net/home/dst5/ENSC894/project/optimizations/gitTemp/ENSC894_Final_Proj/3_coelescing/output1_512x8.out";

const float t_chip = 0.0005;
const float chip_height = 0.016; 
const float chip_width = 0.016; 
const float amb_temp = 80.0;

// HBM Pseudo-channel(PC) requirements
#define MAX_HBM_PC_COUNT 32

void fatal(std::string str)
{
    char *s = new char[str.length() + 1];
    strcpy(s, str.c_str());
    fprintf(stderr, "Error: %s\n", s);
}

/* input array reads from file. */
void readinput(std::vector<float, aligned_allocator<float> >& vect, int grid_rows, int grid_cols, int layers, char *file) {
    int i;
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

/* output array write to file*/
void writeoutput(std::vector<float, aligned_allocator<float> >& vect, int grid_rows, int grid_cols, int layers, char *file) {

    int i, index=0;
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


/* Main computational kernel in SW */
void computeTempCPU(std::vector<float, aligned_allocator<float> >& pIn, 
                    std::vector<float, aligned_allocator<float> >& tIn, 
                    std::vector<float, aligned_allocator<float> >& tOut, 
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


                    tIn[c] = tOut[c]*cc + tOut[n]*cn + tOut[s]*cs + tOut[e]*ce + tOut[w]*cw + tOut[t]*ct + tOut[b]*cb + (dt/Cap) * pIn[c] + ct*amb_temp;
                }
        // float *temp = tIn;
        // tIn = tOut;
        // tOut = temp; 
        i++;
    }
    while(i < numiter/2);

}

/* Calculate Mean Square error and determine pass*/
bool accuracy(std::vector<float, aligned_allocator<float> >& arr1, std::vector<float, aligned_allocator<float> >& arr2, int len)
{
    float rmse = 0.0; 
    int i;
    int count = 0;
    for(i = 0; i < len; i++)
    {
        rmse += (arr1[i]-arr2[i]) * (arr1[i]-arr2[i]);
        if(abs((arr1[i]-arr2[i]))>0.001){
          printf("x=%d\ty=%d\tz=%d\n", i/(NX*NY), (i/NX)%NY, i%NX);
          count++;
        }
    }
    printf("count=%d\n", count);
    rmse =sqrt(rmse/len);
    if (rmse<0.0001){
      printf("No Error\n");
      return true;
    }
    else{
      printf("Error : %e\n", rmse);
      return false;
    }

}

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cout << "Usage: " << argv[0] << " <XCLBIN File> <power input> <temperature input> <hw result output> <sw result output>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string binaryFile = argv[1];
    std::string powerfile = argv[2];
    std::string tempfile = argv[3];
    std::string outfile = argv[4];
    std::string outfile1 = argv[5];

    int iterations = ITR;

    char *pfile = new char[powerfile.length() + 1];
    strcpy(pfile, powerfile.c_str());
    char *tfile = new char[tempfile.length() + 1];
    strcpy(tfile, tempfile.c_str());
    char *ofile = new char[outfile.length() + 1];
    strcpy(ofile, outfile.c_str());
    char *ofile1 = new char[outfile1.length() + 1];
    strcpy(ofile1, outfile1.c_str());
    std::cout<<pfile<<std::endl;
    std::cout<<tfile<<std::endl;
    std::cout<<ofile<<std::endl;

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

    // // Allocate Memory in Host Memory
    // if (DATA_SIZE > MAX_SIZE) {
    //     std::cout << "Size is bigger than internal buffer size,"
    //               << " please use a size smaller than " << MAX_SIZE << "!" << std::endl;
    //     return EXIT_FAILURE;
    // }

    // size_t matrix_size_bytes = sizeof(int) * DATA_SIZE * DATA_SIZE;
    cl_int err;
    cl::CommandQueue q;
    cl::Context context;
    cl::Kernel krnl_hotspot3D;

    int size = numCols * numRows * layers;

    std::vector<float, aligned_allocator<float> > powerIn(sizeof(float) * size);
    std::vector<float, aligned_allocator<float> > temp_sw(sizeof(float) * size);
    std::vector<float, aligned_allocator<float> > temp_hw(sizeof(float) * size);
    std::vector<float, aligned_allocator<float> > answer_sw(sizeof(float) * size);
    std::vector<float, aligned_allocator<float> > answer_hw(sizeof(float) * size);


    // Create the test data and Software Result
    /* Initialize array(s). */
    readinput(powerIn, numRows, numCols, layers, pfile);
    readinput(temp_sw, numRows, numCols, layers, tfile);
    readinput(temp_hw, numRows, numCols, layers, tfile);

    // OPENCL HOST CODE AREA START
    auto devices = xcl::get_xil_devices();
    // read_binary_file() is a utility API which will load the binaryFile
    // and will return the pointer to file buffer.
    auto fileBuf = xcl::read_binary_file(binaryFile);
    cl::Program::Binaries bins{{fileBuf.data(), fileBuf.size()}};
    bool valid_device = false;
    for (unsigned int i = 0; i < devices.size(); i++) {
        auto device = devices[i];
        // Creating Context and Command Queue for selected Device
        OCL_CHECK(err, context = cl::Context(device, nullptr, nullptr, nullptr, &err));
        OCL_CHECK(err, q = cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE, &err));

        std::cout << "Trying to program device[" << i << "]: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;
        cl::Program program(context, {device}, bins, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::cout << "Failed to program device[" << i << "] with xclbin file!\n";
        } else {
            std::cout << "Device[" << i << "]: program successful!\n";
            OCL_CHECK(err, krnl_hotspot3D = cl::Kernel(program, "kernel_3D_hw", &err));
            valid_device = true;
            break; // we break because we found a valid device
        }
    }
    if (!valid_device) {
        std::cout << "Failed to program any device found, exit!\n";
        exit(EXIT_FAILURE);
    }

// For Allocating Buffer to specific Global Memory PC, user has to use
// cl_mem_ext_ptr_t
// and provide the PCs
	
    // cl_mem_ext_ptr_t inBufExt1;
    // cl_mem_ext_ptr_t inBufExt2;
    // cl_mem_ext_ptr_t inoutBufExt;

    cl_mem_ext_ptr_t inPowerIn;
    cl_mem_ext_ptr_t inoutTempIn_hw;
    cl_mem_ext_ptr_t outTempOut_hw;

    inPowerIn.obj = powerIn.data();
    inPowerIn.param = 0;
    inPowerIn.flags = 0|XCL_MEM_TOPOLOGY;

    inoutTempIn_hw.obj = temp_hw.data();
    inoutTempIn_hw.param = 0;
    inoutTempIn_hw.flags = 4|XCL_MEM_TOPOLOGY;

    outTempOut_hw.obj = answer_hw.data();
    outTempOut_hw.param = 0;
    outTempOut_hw.flags = 8|XCL_MEM_TOPOLOGY;
    
    OCL_CHECK(err, cl::Buffer buffer_inPowerIn(context, CL_MEM_USE_HOST_PTR | CL_MEM_EXT_PTR_XILINX | CL_MEM_READ_ONLY, 
				             	sizeof(float) * size,
						&inPowerIn, &err));
    OCL_CHECK(err, cl::Buffer buffer_inoutTempIn_hw(context, CL_MEM_USE_HOST_PTR | CL_MEM_EXT_PTR_XILINX | CL_MEM_READ_WRITE,
				                sizeof(float) * size,
						&inoutTempIn_hw, &err));
    OCL_CHECK(err, cl::Buffer buffer_outTempOut_hw(context, CL_MEM_USE_HOST_PTR | CL_MEM_EXT_PTR_XILINX | CL_MEM_WRITE_ONLY, 
					        sizeof(float) * size,
						&outTempOut_hw, &err));

    /*
    // [Default] Allocate Buffer in Global Memory
    OCL_CHECK(err, cl::Buffer buffer_in1(context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY, sizeof(float) * N * N,
                                         A.data(), &err));
    OCL_CHECK(err, cl::Buffer buffer_in2(context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY, sizeof(float) * N * N,
                                         B.data(), &err));
    OCL_CHECK(err, cl::Buffer buffer_inout(context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE, sizeof(float) * N * N,
                                            C_HW.data(), &err));
	*/

    OCL_CHECK(err, err = krnl_hotspot3D.setArg(0, buffer_inPowerIn));
    OCL_CHECK(err, err = krnl_hotspot3D.setArg(1, buffer_inoutTempIn_hw));
    OCL_CHECK(err, err = krnl_hotspot3D.setArg(2, buffer_outTempOut_hw));
    OCL_CHECK(err, err = krnl_hotspot3D.setArg(3, Cap));
    OCL_CHECK(err, err = krnl_hotspot3D.setArg(4, Rx));
    OCL_CHECK(err, err = krnl_hotspot3D.setArg(5, Ry));
    OCL_CHECK(err, err = krnl_hotspot3D.setArg(6, Rz));
    OCL_CHECK(err, err = krnl_hotspot3D.setArg(7, dt));
    OCL_CHECK(err, err = krnl_hotspot3D.setArg(8, amb_temp));

    // Copy input data to device global memory
    std::cout << "Copy the data to device memory\n";
    OCL_CHECK(err, err = q.enqueueMigrateMemObjects({buffer_inPowerIn, buffer_inoutTempIn_hw}, 0 /* 0 means from host*/));
    std::cout << "Copying data to device memory ended succesfully!\n";

    struct timeval start, stop;
    float time1, time2;

    // Launch the Kernel
    std::cout << "Running HW kernel\n";
    gettimeofday(&start,NULL);
    OCL_CHECK(err, err = q.enqueueTask(krnl_hotspot3D));
    q.finish();
    gettimeofday(&stop,NULL);
    time1 = (stop.tv_usec-start.tv_usec)*1.0e-6 + stop.tv_sec - start.tv_sec;
    std::cout << "HW kernel run ended successful!\n";

    // Copy Result from Device Global Memory to Host Local Memory
    std::cout << "Copy the data to host memory\n";
    OCL_CHECK(err, err = q.enqueueMigrateMemObjects({buffer_inoutTempIn_hw,buffer_outTempOut_hw}, CL_MIGRATE_MEM_OBJECT_HOST));
    q.finish();
    std::cout << "Copying data to host memory ended succesfully!\n";

    // OPENCL HOST CODE AREA END

    // Compute Software Results
    // Create the test data
    std::cout << "Running SW kernel\n";
    gettimeofday(&start,NULL);
    computeTempCPU(powerIn, temp_sw, answer_sw, numCols, numRows, layers, Cap, Rx, Ry, Rz, dt, iterations);
    gettimeofday(&stop,NULL);
    time2 = (stop.tv_usec-start.tv_usec)*1.0e-6 + stop.tv_sec - start.tv_sec;
    std::cout << "SW kernel run ended successful!\n";

    //print timing values
    printf("FPGA Time: %.3f (s)\n",time1);
    printf("CPU Time: %.3f (s)\n",time2);

    // Compare the results of the Device to the simulation
    bool match;
    /* Print root mean squared error. */
    match = accuracy(temp_sw,temp_hw,numRows*numCols*layers);

    std::cout << "TEST " << (match ? "PASSED" : "FAILED") << std::endl;

    writeoutput(temp_hw, numRows, numCols, layers, ofile);
    
    writeoutput(temp_sw, numRows, numCols, layers, ofile1);
    
    return (match ?EXIT_SUCCESS:EXIT_FAILURE);
}
