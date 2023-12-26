#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "dataStructs.h"
#include <cfloat>

__device__ int squaredDistance(point_t a, point_t b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

__global__ void findNearestNodeKernel(point_t coordinate, node_t *list_of_nodes, int num_of_nodes, int *min_d2, int *min_index) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < num_of_nodes) {
        int d2 = squaredDistance(coordinate, list_of_nodes[i].point);
        atomicMin(min_d2, d2);
        if (*min_d2 == d2) {
            *min_index = i;
        }
    }
}

bool findNearestNodeToCoordinateCUDA(point_t coordinate, node_t *list_of_nodes, int num_of_nodes, node_t **nearest_node,
                                     int map_dim_x, int map_dim_y, int dist_to_grow) {
    node_t *d_list_of_nodes;
    int *d_min_d2, *d_min_index;
    int min_d2 = pow(map_dim_x + map_dim_y, 2);
    int min_index = -1;

    // Allocate memory on the device
    cudaMalloc((void **)&d_list_of_nodes, num_of_nodes * sizeof(node_t));
    cudaMalloc((void **)&d_min_d2, sizeof(int));
    cudaMalloc((void **)&d_min_index, sizeof(int));

    // Copy data from host to device
    cudaMemcpy(d_list_of_nodes, list_of_nodes, num_of_nodes * sizeof(node_t), cudaMemcpyHostToDevice);
    cudaMemcpy(d_min_d2, &min_d2, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_min_index, &min_index, sizeof(int), cudaMemcpyHostToDevice);

    // Kernel launch code - assuming 1024 threads per block
    int threadsPerBlock = 1024;
    int blocksPerGrid = (num_of_nodes + threadsPerBlock - 1) / threadsPerBlock;
    findNearestNodeKernel<<<blocksPerGrid, threadsPerBlock>>>(coordinate, d_list_of_nodes, num_of_nodes, d_min_d2, d_min_index);

    // Copy result back to host
    cudaMemcpy(&min_d2, d_min_d2, sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&min_index, d_min_index, sizeof(int), cudaMemcpyDeviceToHost);

    // Clean up
    cudaFree(d_list_of_nodes);
    cudaFree(d_min_d2);
    cudaFree(d_min_index);

    *nearest_node = &list_of_nodes[min_index];

    return min_d2 >= dist_to_grow;
}