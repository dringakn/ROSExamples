/**
 * @file example_gpu_node.cpp
 * @author Dr. -Ing. Ahmad Kamal Nasir (dringakn@gmail.com)
 * @brief 
 * 
 * How to install CUDA :
 * sudo apt update
 * sudo apt install nvidia-cuda-toolkit
 * nvcc --version
 * 
 * How to install CUDA toolkit from CUDA repository:
 * sudo wget -O /etc/apt/preferences.d/cuda-repository-pin-600 https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
 * sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
 * sudo add-apt-repository "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
 * sudo apt update
 * sudo apt install cuda
 * echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
 * nvcc --version
 * 
 * find_package(CUDA REQUIRED)
 * set(CUDA_NVCC_FLAGS "-arch=compute_75" CACHE STRING "nvcc flags" FORCE)
 * set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
 * set(LIB_TYPE STATIC)
 * cuda_add_library(SampLib ${LIB_TYPE} src/template.cu)
 * add_dependencies(roscuda_template_node SampleLib)
 * TARGET_LINK_LIBRARIES(example_gpu_node ${catkin_LIBRARIES} SampleLib)
 * 
 * @version 0.1
 * @date 2023-02-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <cuda.h>
#include <cuda_runtime.h>

__global__ void matrixMulKernel(float *a, float *b, float *c, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    float sum = 0.0;
    for (int k = 0; k < n; k++) {
        sum += a[i*n+k] * b[k*n+j];
    }
    c[i*n+j] = sum;
}

void matrixMultiplication(float *a, float *b, float *c, int n)
{
    int size = n * n * sizeof(float);
    float *d_a, *d_b, *d_c;
    cudaMalloc((void **)&d_a, size);
    cudaMalloc((void **)&d_b, size);
    cudaMalloc((void **)&d_c, size);
    cudaMemcpy(d_a, a, size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, size, cudaMemcpyHostToDevice);
    dim3 dimBlock(16, 16);
    dim3 dimGrid((n + dimBlock.x - 1) / dimBlock.x, (n + dimBlock.y - 1) / dimBlock.y);
    matrixMulKernel<<<dimGrid, dimBlock>>>(d_a, d_b, d_c, n);
    cudaMemcpy(c, d_c, size, cudaMemcpyDeviceToHost);
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_c);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpu_matrix_multiplication_node");
    ros::NodeHandle nh;
    int n = 1000;
    float *a = (float *)malloc(n * n * sizeof(float));
    float *b = (float *)malloc(n * n * sizeof(float));
    float *c = (float *)malloc(n * n * sizeof(float));
    for (int i = 0; i < n * n; i++) {
        a[i] = 1.0;
        b[i] = 1.0;
        c[i] = 0.0;
    }
    matrixMultiplication(a, b, c, n);
    for (int i = 0; i < n * n; i++) {
        if (c[i] != n) {
            ROS_ERROR("Error in matrix multiplication");
            break;
        }
    }
    ROS_INFO("Matrix multiplication successful");
    free(a);
    free(b);
    free(c);
    return 0;
}
