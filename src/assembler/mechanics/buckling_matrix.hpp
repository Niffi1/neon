
#pragma once

#include "numeric/sparse_matrix.hpp"

#include "assembler/sparsity_pattern.hpp"
#include "assembler/homogeneous_dirichlet.hpp"
#include "solver/eigen/eigenvalue_solver.hpp"

#include <chrono>
#include <iostream>

namespace neon::mechanics
{
/// fem_buckling_matrix assembles and solves the eigenvalue buckling problem
/// for linear constitutive models only.
template <typename fem_mesh_type>
class fem_buckling_matrix
{
public:
    using mesh_type = fem_mesh_type;

public:
    fem_buckling_matrix(mesh_type& mesh) : mesh(mesh), solver(5) {}

    /// Compute the eigenvalue for the buckling load and the corresponding
    /// buckling mode.
    void solve();

protected:
    void assemble_stiffness();

protected:
    /// fem mesh
    mesh_type& mesh;
    /// Stiffness matrix
    sparse_matrix K;
    /// Eigenvalue solver
    eigenvalue_solver solver;
};

template <typename fem_mesh_type>
void fem_buckling_matrix<fem_mesh_type>::solve()
{
    // Solve the eigenvalue problem for the first eigenvalue only
    assemble_stiffness();

    // fem::apply_dirichlet_conditions(K, mesh);

    // solver.solve(K);

    // file_io.write(0, 0.0, d);
}

template <typename fem_mesh_type>
void fem_buckling_matrix<fem_mesh_type>::assemble_stiffness()
{
    fem::compute_sparsity_pattern(K, mesh);

    auto const start = std::chrono::high_resolution_clock::now();

    K.coeffs() = 0.0;

    for (auto const& submesh : mesh.meshes())
    {
        for (std::int64_t element = 0; element < submesh.elements(); ++element)
        {
            auto const [dofs, ke] = submesh.tangent_stiffness(element);

            for (std::int64_t a{0}; a < dofs.size(); a++)
            {
                for (std::int64_t b{0}; b < dofs.size(); b++)
                {
                    K.coefficient_update(dofs(a), dofs(b), ke(a, b));
                }
            }
        }
    }

    auto const end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> const elapsed_seconds = end - start;

    std::cout << std::string(6, ' ') << "Stiffness assembly took " << elapsed_seconds.count()
              << "s\n";
}
}
