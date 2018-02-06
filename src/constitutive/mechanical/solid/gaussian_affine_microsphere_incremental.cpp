
#include "gaussian_affine_microsphere_incremental.hpp"

#include "constitutive/InternalVariables.hpp"
#include "constitutive/mechanical/volumetric_free_energy.hpp"

#include "numeric/dense_matrix.hpp"

#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <omp.h>

namespace neon::mechanical::solid
{
void gaussian_affine_microsphere_incremental::update_internal_variables(double const time_step_size)
{
    auto& tangent_operators = variables->fetch(InternalVariables::rank4::tangent_operator);

    auto const& deformation_gradients = variables->fetch(
        InternalVariables::Tensor::DeformationGradient);
    auto& cauchy_stresses = variables->fetch(InternalVariables::Tensor::Cauchy);
    auto& macro_stresses = variables->fetch(InternalVariables::Tensor::Kirchhoff);

    auto const& det_deformation_gradients = variables->fetch(InternalVariables::Scalar::DetF);

    auto const K = material.bulk_modulus();
    auto const G = material.shear_modulus();
    auto const N = material.segments_per_chain();

// Compute the macrostresses on the unit sphere
#pragma omp parallel for
    for (auto l = 0; l < deformation_gradients.size(); ++l)
    {
        auto const& F = deformation_gradients[l];
        macro_stresses[l] = compute_macro_stress(unimodular(F), G, N);
    }

    // Project the stresses to obtain the Cauchy stress
    cauchy_stresses = ranges::view::zip(macro_stresses, det_deformation_gradients)
                      | ranges::view::transform([&](auto const& tpl) -> matrix3 {
                            auto const& [macro_stress, J] = tpl;

                            auto const pressure = J * volumetric_free_energy_dJ(J, K);

                            return compute_kirchhoff_stress(pressure, macro_stress) / J;
                        });

#pragma omp parallel for
    for (auto l = 0; l < deformation_gradients.size(); ++l)
    {
        auto const& F = deformation_gradients[l];
        auto const& macro_stress = macro_stresses[l];
        auto const& J = det_deformation_gradients[l];

        tangent_operators[l] = compute_material_tangent(J,
                                                        K,
                                                        compute_macro_moduli(unimodular(F), G, N),
                                                        macro_stress);
    }
}

matrix3 gaussian_affine_microsphere_incremental::compute_kirchhoff_stress(
    double const pressure, matrix3 const& macro_stress) const
{
    using namespace voigt;

    return pressure * matrix3::Identity() + kinetic::from(P * kinetic::to(macro_stress));
}

matrix6 gaussian_affine_microsphere_incremental::compute_material_tangent(
    double const J, double const K, matrix6 const& macro_C, matrix3 const& macro_stress) const
{
    auto const pressure = J * volumetric_free_energy_dJ(J, K);
    auto const kappa = std::pow(J, 2) * volumetric_free_energy_second_d2J(J, K);

    // clang-format off
    matrix6 const D = macro_C
                    + 2.0 / 3.0 * macro_stress.trace() * voigt::kinematic::identity()
                    - 2.0 / 3.0 * (outer_product(macro_stress, matrix3::Identity()) +
                                   outer_product(matrix3::Identity(), macro_stress));
    // clang-format on
    return (kappa + pressure) * IoI - 2.0 * pressure * I + P * D * P;
}

matrix3 gaussian_affine_microsphere_incremental::compute_macro_stress(matrix3 const& F_unimodular,
                                                                      double const shear_modulus,
                                                                      double const N) const
{
    return 3.0 * shear_modulus
           * unit_sphere.integrate(matrix3::Zero().eval(),
                                   [&](auto const& coordinates, auto const& l) -> matrix3 {
                                       auto const& [r, r_outer_r] = coordinates;

                                       vector3 const t = deformed_tangent(F_unimodular, r);

                                       return outer_product(t, t);
                                   });
}

matrix6 gaussian_affine_microsphere_incremental::compute_macro_moduli(matrix3 const& F_unimodular,
                                                                      double const bulk_modulus,
                                                                      double const N) const
{
    // clang-format off
    return -3.0 * bulk_modulus * unit_sphere.integrate(matrix6::Zero().eval(),
                                                      [&](auto const& coordinates, auto const& l) -> matrix6 {

                                                          auto const & [ r, r_outer_r ] = coordinates;

                                                          vector3 const t = deformed_tangent(F_unimodular, r);

                                                          auto const micro_stretch = compute_microstretch(t);

                                                          return std::pow(micro_stretch, -2) * outer_product(t, t, t, t);
                                                      });
    // clang-format on
}
}
