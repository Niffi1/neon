
#pragma once

#include "femStaticMatrix.hpp"

#include "solver/time/GeneralisedTrapezoidal.hpp"

namespace neon::diffusion
{
class femDynamicMatrix : public femStaticMatrix
{
public:
    explicit femDynamicMatrix(femMesh& fem_mesh, Json::Value const& simulation_data);

    ~femDynamicMatrix();

    void solve() override final;

protected:
    /** Assembles the consistent (full) mass matrix */
    void assemble_mass();

protected:
    SparseMatrix M; //!< Consistent capacity matrix

    GeneralisedTrapezoidal time_solver;
};
}
