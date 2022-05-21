#ifndef ONBOARD_UTILS_CUDA_UTIL_H_
#define ONBOARD_UTILS_CUDA_UTIL_H_

#include <cuda_runtime.h>

namespace qcraft::cuda {

// Synchronize the given CUDA stream with CPU blocking instead of spinning.
inline void SyncStreamWithCpuBlocking(cudaStream_t stream) {
  cudaEvent_t event;
  cudaEventCreateWithFlags(&event, cudaEventBlockingSync);
  cudaEventRecord(event, stream);
  cudaEventSynchronize(event);
}

}  // namespace qcraft::cuda

#endif  // ONBOARD_UTILS_CUDA_UTIL_H_
