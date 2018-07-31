
#define EIGEN_DONT_PARALLELIZE

#include "fem_submesh.hpp"

#include "exceptions.hpp"

#include "constitutive/constitutive_model_factory.hpp"
#include "interpolations/interpolation_factory.hpp"
#include "material/material_property.hpp"
#include "mesh/material_coordinates.hpp"
#include "numeric/gradient_operator.hpp"
#include "numeric/mechanics"
#include "mesh/dof_allocator.hpp"
#include "traits/mechanics.hpp"

#include <termcolor/termcolor.hpp>

#include <tbb/parallel_for.h>

#include <cfenv>
#include <chrono>

namespace neon::mechanical::solid
{
fem_submesh::fem_submesh(json const& material_data,
                         json const& mesh_data,
                         std::shared_ptr<material_coordinates>& coordinates,
                         basic_submesh const& submesh)
    : basic_submesh(submesh),
      coordinates(coordinates),
      sf(make_volume_interpolation(topology(), mesh_data)),
      view(sf->quadrature().points()),
      variables(std::make_shared<internal_variables_t>(elements() * sf->quadrature().points())),
      cm(make_constitutive_model(variables, material_data, mesh_data))
{
    // Allocate storage for the displacement gradient
    variables->add(variable::second::displacement_gradient,
                   variable::second::deformation_gradient,
                   variable::second::cauchy_stress);

    variables->add(variable::scalar::DetF);

    // Get the old data to the undeformed configuration
    auto& deformation_gradients = variables->get(variable::second::deformation_gradient);

    std::fill(begin(deformation_gradients), end(deformation_gradients), matrix3::Identity());

    variables->commit();

    dof_allocator(node_indices, dof_indices, traits::dof_order);
}

void fem_submesh::save_internal_variables(bool const have_converged)
{
    if (have_converged)
    {
        variables->commit();
    }
    else
    {
        variables->revert();
    }
}

std::pair<index_view, matrix> fem_submesh::tangent_stiffness(std::int32_t const element) const
{
    auto const x = coordinates->current_configuration(local_node_view(element));

    matrix ke = material_tangent_stiffness(x, element);

    if (!cm->is_finite_deformation()) return {local_dof_view(element), ke};

    ke.noalias() += geometric_tangent_stiffness(x, element);

    return {local_dof_view(element), ke};
}

std::pair<index_view, vector> fem_submesh::internal_force(std::int32_t const element) const
{
    auto const& x = coordinates->current_configuration(local_node_view(element));

    return {local_dof_view(element), internal_nodal_force(x, element)};
}

matrix fem_submesh::geometric_tangent_stiffness(matrix3x const& x, std::int32_t const element) const
{
    auto const& cauchy_stresses = variables->get(variable::second::cauchy_stress);

    auto n = nodes_per_element();

    // clang-format off
    matrix const kgeo = sf->quadrature().integrate(matrix::Zero(n, n).eval(),
                                                   [&](auto const& femval, auto const& l) -> matrix {
                                                        auto const & [ N, rhea ] = femval;

                                                        matrix3 const Jacobian = local_deformation_gradient(rhea, x);

                                                        matrix3 const& cauchy_stress = cauchy_stresses[view(element, l)];

                                                        // Compute the symmetric gradient operator
                                                        auto const L = (rhea * Jacobian.inverse()).transpose();

                                                        return L.transpose() * cauchy_stress * L * Jacobian.determinant();
                                                    });
    // clang-format on
    return identity_expansion(kgeo, dofs_per_node());
}

matrix fem_submesh::material_tangent_stiffness(matrix3x const& x, std::int32_t const element) const
{
    auto const local_dofs = nodes_per_element() * dofs_per_node();

    auto const& tangent_operators = variables->get(variable::fourth::tangent_operator);

    matrix const kmat = matrix::Zero(local_dofs, local_dofs);
    matrix B = matrix::Zero(6, local_dofs);

    return sf->quadrature().integrate(kmat, [&](auto const& femval, auto const& l) -> matrix {
        auto const& [N, dN] = femval;

        auto const& D = tangent_operators[view(element, l)];

        matrix3 const Jacobian = local_deformation_gradient(dN, x);

        // Compute the symmetric gradient operator
        fem::sym_gradient<3>(B, (dN * Jacobian.inverse()).transpose());

        return B.transpose() * D * B * Jacobian.determinant();
    });
}

vector fem_submesh::internal_nodal_force(matrix3x const& x, std::int32_t const element) const
{
    auto const& cauchy_stresses = variables->get(variable::second::cauchy_stress);

    auto const m = nodes_per_element();
    auto const n = dofs_per_node();

    matrix fint_matrix = sf->quadrature()
                             .integrate(matrix::Zero(m, n).eval(),
                                        [&](auto const& femval, auto const l) -> matrix {
                                            auto const& [N, dN] = femval;

                                            matrix3 const Jacobian = local_deformation_gradient(dN,
                                                                                                x);

                                            matrix3 const&
                                                cauchy_stress = cauchy_stresses[view(element, l)];

                                            // Compute the symmetric gradient operator
                                            auto const Bt = dN * Jacobian.inverse();

                                            return Bt * cauchy_stress * Jacobian.determinant();
                                        });
    return Eigen::Map<vector>(fint_matrix.data(), fint_matrix.size());
}

std::pair<index_view, matrix> fem_submesh::consistent_mass(std::int32_t const element) const
{
    auto const& X = coordinates->initial_configuration(local_node_view(element));

    auto const density_0 = cm->intrinsic_material().initial_density();

    auto m = sf->quadrature().integrate(matrix::Zero(nodes_per_element(), nodes_per_element()).eval(),
                                        [&](auto const& femval, auto const& l) -> matrix {
                                            auto const& [N, dN] = femval;

                                            matrix3 const Jacobian = local_deformation_gradient(dN,
                                                                                                X);

                                            return N * density_0 * N.transpose()
                                                   * Jacobian.determinant();
                                        });
    return {local_dof_view(element), identity_expansion(m, dofs_per_node())};
}

std::pair<index_view, vector> fem_submesh::diagonal_mass(std::int32_t const element) const
{
    auto const& [dofs, consistent_m] = this->consistent_mass(element);

    vector diagonal_m(consistent_m.rows());
    for (auto i = 0; i < consistent_m.rows(); ++i)
    {
        diagonal_m(i) = consistent_m.row(i).sum();
    }
    return {local_dof_view(element), diagonal_m};
}

void fem_submesh::update_internal_variables(double const time_step_size)
{
    std::feclearexcept(FE_ALL_EXCEPT);

    update_deformation_measures();

    update_Jacobian_determinants();

    cm->update_internal_variables(time_step_size);

    if (std::fetestexcept(FE_INVALID))
    {
        throw computational_error("Floating point error reported\n");
    }
}

void fem_submesh::update_deformation_measures()
{
    auto& H_list = variables->get(variable::second::displacement_gradient);
    auto& F_list = variables->get(variable::second::deformation_gradient);

    tbb::parallel_for(std::int64_t{0}, elements(), [&](auto const element) {
        // Gather the material coordinates
        auto const X = coordinates->initial_configuration(local_node_view(element));
        auto const x = coordinates->current_configuration(local_node_view(element));

        sf->quadrature().for_each([&](auto const& femval, auto const l) {
            auto const& [N, rhea] = femval;

            // Local deformation gradient for the initial configuration
            matrix3 const F_0 = local_deformation_gradient(rhea, X);
            matrix3 const F = local_deformation_gradient(rhea, x);

            // Gradient operator in index notation
            matrixxd<3> const B_0t = rhea * F_0.inverse();

            // Displacement gradient
            matrix3 const H = (x - X) * B_0t;

            H_list[view(element, l)] = H;
            F_list[view(element, l)] = F * F_0.inverse();
        });
    });
}

void fem_submesh::update_Jacobian_determinants()
{
    auto const& deformation_gradients = variables->get(variable::second::deformation_gradient);

    auto& F_determinants = variables->get(variable::scalar::DetF);

    std::transform(begin(deformation_gradients),
                   end(deformation_gradients),
                   begin(F_determinants),
                   [](matrix3 const& F) { return F.determinant(); });

    auto const found = std::find_if(begin(F_determinants), end(F_determinants), [](auto const i) {
        return std::signbit(i);
    });

    if (found != end(F_determinants))
    {
        auto const count = std::count_if(begin(F_determinants),
                                         end(F_determinants),
                                         [](auto const i) { return std::signbit(i); });

        auto const i = std::distance(begin(F_determinants), found);

        auto const [element, quadrature_point] = std::div(i, sf->quadrature().points());

        throw computational_error("Positive Jacobian assumption violated at element "
                                  + std::to_string(element) + " and local quadrature point "
                                  + std::to_string(quadrature_point) + " (" + std::to_string(*found)
                                  + "), another " + std::to_string(count - 1) + " violations found");
    }
}

std::pair<vector, vector> fem_submesh::nodal_averaged_variable(variable::second const tensor_name) const
{
    vector count = vector::Zero(coordinates->size() * 9);
    vector value = count;

    auto const& tensor_list = variables->get(tensor_name);

    auto const& E = sf->local_quadrature_extrapolation();

    // vector format of values
    vector component = vector::Zero(sf->quadrature().points());

    for (std::int64_t e{0}; e < elements(); ++e)
    {
        // Assemble these into the global value vector
        auto const& node_list = local_node_view(e);

        for (auto ci = 0; ci < 3; ++ci)
        {
            for (auto cj = 0; cj < 3; ++cj)
            {
                for (std::size_t l{0}; l < sf->quadrature().points(); ++l)
                {
                    auto const& tensor = tensor_list[view(e, l)];
                    component(l) = tensor(ci, cj);
                }

                // Local extrapolation to the nodes
                vector const nodal_component = E * component;

                for (auto n = 0; n < nodal_component.rows(); n++)
                {
                    value(node_list[n] * 9 + ci * 3 + cj) += nodal_component(n);
                    count(node_list[n] * 9 + ci * 3 + cj) += 1.0;
                }
            }
        }
    }
    return {value, count};
}

std::pair<vector, vector> fem_submesh::nodal_averaged_variable(variable::scalar const scalar_name) const
{
    vector count = vector::Zero(coordinates->size());
    vector value = count;

    auto const& scalar_list = variables->get(scalar_name);

    auto const& E = sf->local_quadrature_extrapolation();

    // vector format of values
    vector component = vector::Zero(sf->quadrature().points());

    for (std::int64_t e{0}; e < elements(); ++e)
    {
        // Assemble these into the global value vector
        auto const& node_list = local_node_view(e);

        for (std::size_t l{0}; l < sf->quadrature().points(); ++l)
        {
            component(l) = scalar_list[view(e, l)];
        }

        // Local extrapolation to the nodes
        vector const nodal_component = E * component;

        for (auto n = 0; n < nodal_component.rows(); n++)
        {
            value(node_list[n]) += nodal_component(n);
            count(node_list[n]) += 1.0;
        }
    }
    return {value, count};
}
}
