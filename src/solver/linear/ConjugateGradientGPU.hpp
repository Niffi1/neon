
#pragma once

#ifdef ENABLE_CUDA

#include "LinearSolver.hpp"

#include <cuda/cublas_v2.h>
#include <cuda/cusparse.h>

namespace neon
{
/**
 * ConjugateGradientGPU is a GPU based solver using the conjugate
 * gradient solver from the CUDA examples.
 */
class ConjugateGradientGPU : public LinearSolver
{
public:
    explicit ConjugateGradientGPU();

    explicit ConjugateGradientGPU(double const residual_tolerance);

    explicit ConjugateGradientGPU(int const maxIter);

    explicit ConjugateGradientGPU(double const residual_tolerance, int const maxIter);

    ~ConjugateGradientGPU();

    void solve(SparseMatrix const& A, Vector& x, Vector const& b) override final;

protected:
    void allocate_device_memory(SparseMatrix const& A, Vector& x, Vector const& b);

    void find_compute_device();

protected:
    // Device side pointers
    int *d_col, *d_row;
    double *d_val, *d_x;

    double *d_r, *d_p, *d_Ap, *d_y, *d_z;

    cublasHandle_t cublasHandle = 0;
    cusparseHandle_t cusparseHandle = 0;
    cusparseMatDescr_t descr = 0;
};
}
#endif
