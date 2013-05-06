#include <stdio.h>
#include "cudaptr.cuh"

struct PtrContext {
  CudaFloatPtr pA, pB, pC;
  __host__ void init() {
    pA.init();
    pB.init();
    pC.init();
  }
  __host__ inline void map(float* h_A, float* h_B, float* h_C, int numElements){
    cudaError_t err;
    err = pA.map(h_A, numElements, numElements);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device vector A (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = pB.map(h_B, numElements, numElements);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device vector B (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = pC.map(h_C, numElements, numElements);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device vector C (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
  }

  __host__ void copyToDevice(){
    printf ("copyToDevice\n");
    cudaError_t err;
    err = pA.copyToDevice();
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy device vector A (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = pB.copyToDevice();
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy device vector B (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
  }
  __host__ void copyFromDevice(){
    cudaError_t err;
    err = pC.copyFromDevice();
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy device vector D (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
  }
};

/**
 * CUDA Kernel Device code
 *
 * Computes the vector addition of A and B into C. The 3 vectors have the same
 * number of elements numElements.
 */
__global__ void
vectorAdd(PtrContext* pc)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x;

    if (i < pc->pA.size())
    {
      pc->pC.dptr()[i] = pc->pA.dptr()[i] + pc->pB.dptr()[i];
    }
}

/**
 * Host main routine
 */
int
main(void)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;

    // Print the vector length to be used, and compute its size
    int numElements = 50000;
    size_t size = numElements * sizeof(float);
    printf("[Vector addition of %d elements]\n", numElements);

    // Allocate the host input vector A
    float *h_A = (float *)malloc(size);

    // Allocate the host input vector B
    float *h_B = (float *)malloc(size);

    // Allocate the host output vector C
    float *h_C = (float *)malloc(size);


    // Verify that allocations succeeded
    if (h_A == NULL || h_B == NULL || h_C == NULL)
    {
        fprintf(stderr, "Failed to allocate host vectors!\n");
        exit(EXIT_FAILURE);
    }

    // Initialize the host input vectors
    for (int i = 0; i < numElements; ++i)
    {
        h_A[i] = rand()/(float)RAND_MAX;
        h_B[i] = rand()/(float)RAND_MAX;
    }

    
    PtrContext pc, *d_pc;
    pc.init();
    pc.map(h_A, h_B, h_C, numElements);
    cudaMalloc((void**)&d_pc, sizeof(PtrContext));
    cudaMemcpy(d_pc,&pc, sizeof(PtrContext), cudaMemcpyHostToDevice);
    pc.copyToDevice();

    //Launch the Vector Add CUDA Kernel
    int threadsPerBlock = 256;
    int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
    printf("CUDA kernel launch with %d blocks of %d threads\n", blocksPerGrid, threadsPerBlock);
    vectorAdd<<<blocksPerGrid, threadsPerBlock>>>(d_pc);
    err = cudaGetLastError();

    pc.copyFromDevice();

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to launch vectorAdd kernel (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }


    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector C from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Verify that the result vector is correct
    for (int i = 0; i < numElements; ++i)
    {
        if (fabs(h_A[i] + h_B[i] - h_C[i]) > 1e-5)
        {
            fprintf(stderr, "Result verification failed at element %d!\n", i);
            exit(EXIT_FAILURE);
        }
    }


    pc.map(0,0,0,0);
 
    // // Free device global memory
    // err = cudaFree(d_A);

    // if (err != cudaSuccess)
    // {
    //     fprintf(stderr, "Failed to free device vector A (error code %s)!\n", cudaGetErrorString(err));
    //     exit(EXIT_FAILURE);
    // }
    // err = cudaFree(d_B);

    // if (err != cudaSuccess)
    // {
    //     fprintf(stderr, "Failed to free device vector B (error code %s)!\n", cudaGetErrorString(err));
    //     exit(EXIT_FAILURE);
    // }
    // err = cudaFree(d_C);

    // if (err != cudaSuccess)
    // {
    //     fprintf(stderr, "Failed to free device vector C (error code %s)!\n", cudaGetErrorString(err));
    //     exit(EXIT_FAILURE);
    // }

    // Free host memory
    free(h_A);
    free(h_B);
    free(h_C);

    // Reset the device and exit
    err = cudaDeviceReset();

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to deinitialize the device! error=%s\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    printf("Done\n");
    return 0;
}

