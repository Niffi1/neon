
#pragma once

#include "LinearSolver.hpp"

namespace neon
{
/**
 * MUMPS is a base class for the multifrontal direct solver.  This solver
 * is widely used in the parallel solution of linear systems.
 *
 * TODO Put link to the solver website and documentation
 */
class MUMPS : public DirectLinearSolver
{
public:
    enum Ordering { AMD, AMF = 2, Scotch, Pord, Metis, QAMD, Automatic };

    // Jobs in MUMPS use the following:
    // 4   Job = 1 && Job = 2
    // 5   Job = 2 && Job = 3
    // 6   Job = 1 && Job = 2 && Job = 3
    enum Job {
        Terminate = -2,
        Initialization = -1,
        Analysis = 1,
        Factorisation = 2,
        BackSubstitution = 3
    };

    enum Residual { None, Expensive, Cheap };

    enum MatrixProperty { Unsymmetric, SPD, GeneralSymmetric };

protected:
    /**
     * Expand the sparse matrix into coordinate format only using the upper
     * diagonal values if the only_upper flag is set, otherwise expand
     * the entire matrix into compressed coordinate (COO) format
     */
    void allocate_coordinate_format_storage(SparseMatrix const& A, bool const only_upper);

protected:
    std::vector<int> irn, jcn; //!< Row and column index storage (uncompressed)
    std::vector<double> a;     //!< Sparse matrix coefficients
};

/**
 * MUMPSLLT is the LL^T factorisation (Cholesky) for a symmetric positive
 * definite matrix.  This solver can only be applied on a linear system and
 * takes the lower triangular part of the sparse matrix
 */
class MUMPSLLT : public MUMPS
{
public:
    void solve(SparseMatrix const& A, Vector& x, Vector const& b) override final;
};

/**
 * MUMPSLLT is the LL^T factorisation (Cholesky) for a general unsymmetric matrix.
 * This solver can only be applied on a linear system and takes the entire
 * uncompressed matrix
 */
class MUMPSLU : public MUMPS
{
public:
    void solve(SparseMatrix const& A, Vector& x, Vector const& b) override final;
};
}
