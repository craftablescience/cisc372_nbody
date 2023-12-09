#include <cuda_runtime.h>
#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "vector.h"

__global__ void computePairwiseAcceleration(vector3* hPos, vector3* values, vector3** accels, double* mass) {
	// Row
	int i = threadIdx.x;
	if (i >= NUMENTITIES) {
		return;
	}

	accels[i] = &values[i * NUMENTITIES];

	for (int j = 0; j < NUMENTITIES; j++) {
		if (i == j) {
			FILL_VECTOR(accels[i][j], 0, 0, 0);
		} else {
			vector3 dist;
			for (int k = 0; k < 3; k++) {
				dist[k] = hPos[i][k] - hPos[j][k];
			}
			double mag_sq = dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2];
			double mag = sqrt(mag_sq);
			double accel = -GRAV_CONSTANT * mass[j] / mag_sq;
			FILL_VECTOR(accels[i][j], accel * dist[0] / mag, accel * dist[1] / mag, accel * dist[2] / mag);
		}
	}
}

__global__ void computeSum(vector3* hVel, vector3* hPos, vector3** accels) {
	// Row
	int i = threadIdx.x;
	if (i >= NUMENTITIES) {
		return;
	}

	vector3 sum{0, 0, 0};
	for (int j = 0; j < NUMENTITIES; j++) {
		for (int k = 0; k < 3; k++) {
			sum[k] += accels[i][j][k];
		}
	}
	for (int k = 0; k < 3; k++) {
		hVel[i][k] += sum[k] * INTERVAL;
		hPos[i][k] += hVel[i][k] * INTERVAL;
	}
}

extern vector3* hVel;
extern vector3* hPos;
extern double*  mass;

//compute: Updates the positions and locations of the objects in the system based on gravity
//Parameters: None
//Returns: None
//Side Effect: Modifies the hVel and hPos arrays with the new positions and accelerations
//             after 1 INTERVAL
void compute() {
	vector3*  dhVel;
	vector3*  dhPos;
	vector3*  values;
	vector3** accels;
	double*   dMass;

	cudaMalloc(&dhVel,  sizeof(vector3) * NUMENTITIES);
	cudaMalloc(&dhPos,  sizeof(vector3) * NUMENTITIES);
	cudaMalloc(&values, sizeof(vector3) * NUMENTITIES * NUMENTITIES);
	cudaMalloc(&accels, sizeof(vector3) * NUMENTITIES);
	cudaMalloc(&dMass,  sizeof(double)  * NUMENTITIES);

	cudaMemcpy(dhVel, hVel, sizeof(vector3) * NUMENTITIES, cudaMemcpyHostToDevice);
	cudaMemcpy(dhPos, hPos, sizeof(vector3) * NUMENTITIES, cudaMemcpyHostToDevice);
	cudaMemcpy(dMass, mass, sizeof(double)  * NUMENTITIES, cudaMemcpyHostToDevice);

	computePairwiseAcceleration<<<1,108>>>(dhPos, values, accels, dMass);
	cudaDeviceSynchronize();

	computeSum<<<1,108>>>(dhVel, dhPos, accels);
	cudaDeviceSynchronize();

	cudaMemcpy(hPos, dhPos, sizeof(vector3) * NUMENTITIES, cudaMemcpyDeviceToHost);
	cudaMemcpy(hVel, dhVel, sizeof(vector3) * NUMENTITIES, cudaMemcpyDeviceToHost);

	cudaFree(dMass);
	cudaFree(accels);
	cudaFree(dhPos);
	cudaFree(dhVel);
}
