
#include "Pastix.hpp"

#include "SimulationControl.hpp"

#include <chrono>
#include <termcolor/termcolor.hpp>

#include <Eigen/PaStiXSupport>

namespace neon
{
void PaStiX::solve(SparseMatrix const& A, Vector& x, Vector const& b)
{
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::PastixLLT<Eigen::SparseMatrix<double>, Eigen::Upper> pastix;

    // Verbosity
    pastix.iparm(3) = 0;

    // Number of threads
    pastix.iparm(34) = SimulationControl::threads;

    // Number of Cuda devices
    // pastix.iparm(64) = 1;

    pastix.compute(A);

    x = pastix.solve(b);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << std::string(6, ' ') << "PaStiX LLT direct solver took "
              << elapsed_seconds.count() << "s\n";
}
}
