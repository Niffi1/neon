
#include "AffineMicrosphere.hpp"

#include "InternalVariables.hpp"
#include "numeric/DenseTypes.hpp"

#include <json/json.h>
#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>

#include <exception>
#include <omp.h>

namespace neon
{
AffineMicrosphere::AffineMicrosphere(InternalVariables& variables, Json::Value const& material_data)
    : Hyperelastic(variables), material(material_data)
{
    if (material_data["SegmentsPerChain"].empty())
        throw std::runtime_error("SegmentsPerChain not specified in material data\n");

    if (!material_data["ChainDecayRate"].empty())
    {
        chain_decay_rate = material_data["ChainDecayRate"].asDouble();
    }

    auto const μ0 = material.shear_modulus();

    number_of_chains = μ0 / (boltzmann_constant * temperature);

    segments_per_chain = material_data["SegmentsPerChain"].asInt();

    variables.add(InternalVariables::Matrix::TruesdellModuli, 6);

    // Deviatoric stress
    variables.add(InternalVariables::Tensor::Kirchhoff);
}

void AffineMicrosphere::update_internal_variables(double const Δt)
{
    using namespace ranges;

    // Decay the number of chains available
    number_of_chains *= 1.0 / (1.0 + chain_decay_rate * Δt);

    μ = number_of_chains * boltzmann_constant * temperature;

    // Get references into the hash table
    auto& F_list = variables(InternalVariables::Tensor::DeformationGradient);
    auto& σ_list = variables(InternalVariables::Tensor::Cauchy);
    auto& τ_list = variables(InternalVariables::Tensor::Kirchhoff);

    auto const& detF_list = variables(InternalVariables::Scalar::DetF);

    auto const N = segments_per_chain;

#pragma omp parallel for
    for (auto quadrature_point = 0; quadrature_point < F_list.size(); ++quadrature_point)
    {
        auto const& F = F_list[quadrature_point];
        auto const& J = detF_list[quadrature_point];

        // Unimodular decomposition of F
        Matrix3 const unimodular_F = std::pow(J, -1.0 / 3.0) * F;

        τ_list[quadrature_point] =
            μ * unit_sphere.integrate(Matrix3::Zero(),
                                      [&](auto const& coordinates, auto const& l) -> Matrix3 {
                                          auto const & [ r, r_outer_r ] = coordinates;

                                          // Deformed tangents
                                          auto const t = unimodular_F * r;

                                          // Microstretches
                                          auto const λ = t.norm();

                                          return (3.0 * N - std::pow(λ, 2)) / (N - std::pow(λ, 2)) *
                                                 t * t.transpose();
                                      });
    }

    // Perform the projection of the stresses
    σ_list = view::zip(τ_list, detF_list) | view::transform([this](auto const& τdetF) -> Matrix3 {

                 auto const & [ τ_dev, J ] = τdetF;

                 auto const pressure = J * volumetric_free_energy_derivative(J, μ);

                 return deviatoric_projection(pressure, τ_dev) / J;
             });

    // Compute tangent moduli

    auto& D_list = variables(InternalVariables::Matrix::TruesdellModuli);

    Matrix const IoI = I_outer_I();
    Matrix const I = fourth_order_identity();

#pragma omp parallel for
    for (auto quadrature_point = 0; quadrature_point < F_list.size(); ++quadrature_point)
    {
        auto const& F = F_list[quadrature_point];
        auto const& τ_dev = τ_list[quadrature_point];
        auto const& J = detF_list[quadrature_point];

        auto const pressure = J * volumetric_free_energy_derivative(J, μ);
        auto const κ = std::pow(J, 2) * volumetric_free_energy_second_derivative(J, μ);

        // Unimodular decomposition of F
        Matrix3 const unimodular_F = std::pow(J, -1.0 / 3.0) * F;

        auto D = unit_sphere.integrate(Matrix::Zero(6, 6),
                                       [&](auto const& coordinates, auto const& l) -> Matrix {
                                           auto const & [ r, r_outer_r ] = coordinates;

                                           // Deformed tangents
                                           auto const t = unimodular_F * r;

                                           // Microstretches
                                           auto const λ = t.norm();

                                           auto const a =
                                               std::pow(λ, -2) *
                                               ((std::pow(λ, 4) + 3.0 * std::pow(N, 2)) /
                                                    std::pow(N - std::pow(λ, 2), 2) -
                                                (3.0 * N - std::pow(λ, 2)) / (N - std::pow(λ, 2)));

                                           return a * voigt(t * t.transpose()) *
                                                  voigt(t * t.transpose()).transpose();
                                       });

        D_list[quadrature_point] =
            deviatoric_projection(D, τ_dev) + (κ + pressure) * IoI - 2.0 * pressure * I;
    }
}

Matrix3 AffineMicrosphere::deviatoric_projection(double const pressure, Matrix3 const& τ_dev) const
{
    Matrix3 P_double_dot_τ;
    P_double_dot_τ << 2 * τ_dev(0, 0) / 3.0 - τ_dev(1, 1) / 3.0 - τ_dev(2, 2) / 3.0, //
        τ_dev(0, 1) / 2.0 + τ_dev(1, 0) / 2.0,                                       //
        τ_dev(0, 2) / 2.0 + τ_dev(2, 0) / 2.0,                                       //
                                                                                     //
        τ_dev(0, 1) / 2.0 + τ_dev(1, 0) / 2.0,                                       //
        -τ_dev(0, 0) / 3.0 + 2 * τ_dev(1, 1) / 3.0 - τ_dev(2, 2) / 3.0,              //
        τ_dev(1, 2) / 2.0 + τ_dev(2, 1) / 2.0,                                       //
                                                                                     //
        τ_dev(0, 2) / 2.0 + τ_dev(2, 0) / 2.0,                                       //
        τ_dev(1, 2) / 2.0 + τ_dev(2, 1) / 2.0,                                       //
        -τ_dev(0, 0) / 3.0 - τ_dev(1, 1) / 3.0 + 2 * τ_dev(2, 2) / 3.0;
    return pressure * Matrix3::Identity() + P_double_dot_τ;
}

Matrix AffineMicrosphere::deviatoric_projection(Matrix const& C_dev, Matrix3 const& τ_dev) const
{
    Matrix C(6, 6);
    C << -2 * (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 -
             2 * (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 -
             2 * (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
             (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 -
             2 * (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
             (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
             4 * (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 +
             (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 +
             (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0, //
        4 * (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 -
            2 * (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 -
            2 * (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 +
            (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0, //
        -2 * (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            4 * (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 -
            2 * (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 +
            (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 -
            2 * (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0, //
        (C_dev(0, 3) - 2 * τ_dev(1, 2) / 3.0) / 3.0 + (C_dev(0, 3) - 2 * τ_dev(2, 1) / 3.0) / 3.0 -
            (C_dev(1, 3) - 2 * τ_dev(1, 2) / 3.0) / 6.0 -
            (C_dev(1, 3) - 2 * τ_dev(2, 1) / 3.0) / 6.0 -
            (C_dev(2, 3) - 2 * τ_dev(1, 2) / 3.0) / 6.0 -
            (C_dev(2, 3) - 2 * τ_dev(2, 1) / 3.0) / 6.0, //
        (C_dev(0, 4) - 2 * τ_dev(0, 2) / 3.0) / 3.0 + (C_dev(0, 4) - 2 * τ_dev(2, 0) / 3.0) / 3.0 -
            (C_dev(1, 4) - 2 * τ_dev(0, 2) / 3.0) / 6.0 -
            (C_dev(1, 4) - 2 * τ_dev(2, 0) / 3.0) / 6.0 -
            (C_dev(2, 4) - 2 * τ_dev(0, 2) / 3.0) / 6.0 - (C_dev(2, 4) - 2 * τ_dev(2, 0) / 3.0) / 6.0,
        (C_dev(0, 5) - 2 * τ_dev(0, 1) / 3.0) / 3.0 + (C_dev(0, 5) - 2 * τ_dev(1, 0) / 3.0) / 3.0 -
            (C_dev(1, 5) - 2 * τ_dev(0, 1) / 3.0) / 6.0 -
            (C_dev(1, 5) - 2 * τ_dev(1, 0) / 3.0) / 6.0 -
            (C_dev(2, 5) - 2 * τ_dev(0, 1) / 3.0) / 6.0 -
            (C_dev(2, 5) - 2 * τ_dev(1, 0) / 3.0) / 6.0, //
        //
        (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            4 * (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 -
            2 * (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 -
            2 * (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 +
            (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0, //
        -2 * (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 -
            2 * (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 +
            4 * (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 +
            (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0, //
        (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 -
            2 * (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            4 * (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 -
            2 * (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 -
            2 * (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0,
        -(C_dev(0, 3) - 2 * τ_dev(1, 2) / 3.0) / 6.0 - (C_dev(0, 3) - 2 * τ_dev(2, 1) / 3.0) / 6.0 +
            (C_dev(1, 3) - 2 * τ_dev(1, 2) / 3.0) / 3.0 +
            (C_dev(1, 3) - 2 * τ_dev(2, 1) / 3.0) / 3.0 -
            (C_dev(2, 3) - 2 * τ_dev(1, 2) / 3.0) / 6.0 -
            (C_dev(2, 3) - 2 * τ_dev(2, 1) / 3.0) / 6.0, //
        -(C_dev(0, 4) - 2 * τ_dev(0, 2) / 3.0) / 6.0 - (C_dev(0, 4) - 2 * τ_dev(2, 0) / 3.0) / 6.0 +
            (C_dev(1, 4) - 2 * τ_dev(0, 2) / 3.0) / 3.0 +
            (C_dev(1, 4) - 2 * τ_dev(2, 0) / 3.0) / 3.0 -
            (C_dev(2, 4) - 2 * τ_dev(0, 2) / 3.0) / 6.0 -
            (C_dev(2, 4) - 2 * τ_dev(2, 0) / 3.0) / 6.0, //
        -(C_dev(0, 5) - 2 * τ_dev(0, 1) / 3.0) / 6.0 - (C_dev(0, 5) - 2 * τ_dev(1, 0) / 3.0) / 6.0 +
            (C_dev(1, 5) - 2 * τ_dev(0, 1) / 3.0) / 3.0 +
            (C_dev(1, 5) - 2 * τ_dev(1, 0) / 3.0) / 3.0 -
            (C_dev(2, 5) - 2 * τ_dev(0, 1) / 3.0) / 6.0 -
            (C_dev(2, 5) - 2 * τ_dev(1, 0) / 3.0) / 6.0, //
        //
        (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            4 * (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 +
            (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 -
            2 * (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0, //
        -2 * (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 +
            (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            4 * (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 -
            2 * (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 -
            2 * (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0, //
        (C_dev(0, 1) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 -
            2 * (C_dev(0, 2) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(1, 0) - 2 * (τ_dev(0, 0) + τ_dev(1, 1)) / 3.0) / 9.0 -
            2 * (C_dev(1, 2) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(2, 0) - 2 * (τ_dev(0, 0) + τ_dev(2, 2)) / 3.0) / 9.0 -
            2 * (C_dev(2, 1) - 2 * (τ_dev(1, 1) + τ_dev(2, 2)) / 3.0) / 9.0 +
            (C_dev(0, 0) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(0, 0) / 3.0) / 9.0 +
            (C_dev(1, 1) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(1, 1) / 3.0) / 9.0 +
            4 * (C_dev(2, 2) + 2 * τ_dev.trace() / 3.0 - 4 * τ_dev(2, 2) / 3.0) / 9.0,
        -(C_dev(0, 3) - 2 * τ_dev(1, 2) / 3.0) / 6.0 - (C_dev(0, 3) - 2 * τ_dev(2, 1) / 3.0) / 6.0 -
            (C_dev(1, 3) - 2 * τ_dev(1, 2) / 3.0) / 6.0 -
            (C_dev(1, 3) - 2 * τ_dev(2, 1) / 3.0) / 6.0 +
            (C_dev(2, 3) - 2 * τ_dev(1, 2) / 3.0) / 3.0 +
            (C_dev(2, 3) - 2 * τ_dev(2, 1) / 3.0) / 3.0, //
        -(C_dev(0, 4) - 2 * τ_dev(0, 2) / 3.0) / 6.0 - (C_dev(0, 4) - 2 * τ_dev(2, 0) / 3.0) / 6.0 -
            (C_dev(1, 4) - 2 * τ_dev(0, 2) / 3.0) / 6.0 -
            (C_dev(1, 4) - 2 * τ_dev(2, 0) / 3.0) / 6.0 +
            (C_dev(2, 4) - 2 * τ_dev(0, 2) / 3.0) / 3.0 + (C_dev(2, 4) - 2 * τ_dev(2, 0) / 3.0) / 3.0,
        -(C_dev(0, 5) - 2 * τ_dev(0, 1) / 3.0) / 6.0 - (C_dev(0, 5) - 2 * τ_dev(1, 0) / 3.0) / 6.0 -
            (C_dev(1, 5) - 2 * τ_dev(0, 1) / 3.0) / 6.0 -
            (C_dev(1, 5) - 2 * τ_dev(1, 0) / 3.0) / 6.0 +
            (C_dev(2, 5) - 2 * τ_dev(0, 1) / 3.0) / 3.0 +
            (C_dev(2, 5) - 2 * τ_dev(1, 0) / 3.0) / 3.0, //
        //
        (C_dev(3, 0) - 2 * τ_dev(1, 2) / 3.0) / 3.0 + (C_dev(3, 0) - 2 * τ_dev(2, 1) / 3.0) / 3.0 -
            (C_dev(3, 1) - 2 * τ_dev(1, 2) / 3.0) / 6.0 -
            (C_dev(3, 1) - 2 * τ_dev(2, 1) / 3.0) / 6.0 -
            (C_dev(3, 2) - 2 * τ_dev(1, 2) / 3.0) / 6.0 - (C_dev(3, 2) - 2 * τ_dev(2, 1) / 3.0) / 6.0,
        -(C_dev(3, 0) - 2 * τ_dev(1, 2) / 3.0) / 6.0 - (C_dev(3, 0) - 2 * τ_dev(2, 1) / 3.0) / 6.0 +
            (C_dev(3, 1) - 2 * τ_dev(1, 2) / 3.0) / 3.0 +
            (C_dev(3, 1) - 2 * τ_dev(2, 1) / 3.0) / 3.0 -
            (C_dev(3, 2) - 2 * τ_dev(1, 2) / 3.0) / 6.0 - (C_dev(3, 2) - 2 * τ_dev(2, 1) / 3.0) / 6.0,
        -(C_dev(3, 0) - 2 * τ_dev(1, 2) / 3.0) / 6.0 - (C_dev(3, 0) - 2 * τ_dev(2, 1) / 3.0) / 6.0 -
            (C_dev(3, 1) - 2 * τ_dev(1, 2) / 3.0) / 6.0 -
            (C_dev(3, 1) - 2 * τ_dev(2, 1) / 3.0) / 6.0 +
            (C_dev(3, 2) - 2 * τ_dev(1, 2) / 3.0) / 3.0 +
            (C_dev(3, 2) - 2 * τ_dev(2, 1) / 3.0) / 3.0,             //
        C_dev(3, 3) + τ_dev.trace() / 3.0, C_dev(3, 4), C_dev(3, 5), //
        //
        (C_dev(4, 0) - 2 * τ_dev(0, 2) / 3.0) / 3.0 + (C_dev(4, 0) - 2 * τ_dev(2, 0) / 3.0) / 3.0 -
            (C_dev(4, 1) - 2 * τ_dev(0, 2) / 3.0) / 6.0 -
            (C_dev(4, 1) - 2 * τ_dev(2, 0) / 3.0) / 6.0 -
            (C_dev(4, 2) - 2 * τ_dev(0, 2) / 3.0) / 6.0 -
            (C_dev(4, 2) - 2 * τ_dev(2, 0) / 3.0) / 6.0, //
        -(C_dev(4, 0) - 2 * τ_dev(0, 2) / 3.0) / 6.0 - (C_dev(4, 0) - 2 * τ_dev(2, 0) / 3.0) / 6.0 +
            (C_dev(4, 1) - 2 * τ_dev(0, 2) / 3.0) / 3.0 +
            (C_dev(4, 1) - 2 * τ_dev(2, 0) / 3.0) / 3.0 -
            (C_dev(4, 2) - 2 * τ_dev(0, 2) / 3.0) / 6.0 -
            (C_dev(4, 2) - 2 * τ_dev(2, 0) / 3.0) / 6.0, //
        -(C_dev(4, 0) - 2 * τ_dev(0, 2) / 3.0) / 6.0 - (C_dev(4, 0) - 2 * τ_dev(2, 0) / 3.0) / 6.0 -
            (C_dev(4, 1) - 2 * τ_dev(0, 2) / 3.0) / 6.0 -
            (C_dev(4, 1) - 2 * τ_dev(2, 0) / 3.0) / 6.0 +
            (C_dev(4, 2) - 2 * τ_dev(0, 2) / 3.0) / 3.0 +
            (C_dev(4, 2) - 2 * τ_dev(2, 0) / 3.0) / 3.0,             //
        C_dev(4, 3), C_dev(4, 4) + τ_dev.trace() / 3.0, C_dev(4, 5), //
        //
        (C_dev(5, 0) - 2 * τ_dev(0, 1) / 3.0) / 3.0 + (C_dev(5, 0) - 2 * τ_dev(1, 0) / 3.0) / 3.0 -
            (C_dev(5, 1) - 2 * τ_dev(0, 1) / 3.0) / 6.0 -
            (C_dev(5, 1) - 2 * τ_dev(1, 0) / 3.0) / 6.0 -
            (C_dev(5, 2) - 2 * τ_dev(0, 1) / 3.0) / 6.0 -
            (C_dev(5, 2) - 2 * τ_dev(1, 0) / 3.0) / 6.0, //
        -(C_dev(5, 0) - 2 * τ_dev(0, 1) / 3.0) / 6.0 - (C_dev(5, 0) - 2 * τ_dev(1, 0) / 3.0) / 6.0 +
            (C_dev(5, 1) - 2 * τ_dev(0, 1) / 3.0) / 3.0 +
            (C_dev(5, 1) - 2 * τ_dev(1, 0) / 3.0) / 3.0 -
            (C_dev(5, 2) - 2 * τ_dev(0, 1) / 3.0) / 6.0 -
            (C_dev(5, 2) - 2 * τ_dev(1, 0) / 3.0) / 6.0, //
        -(C_dev(5, 0) - 2 * τ_dev(0, 1) / 3.0) / 6.0 - (C_dev(5, 0) - 2 * τ_dev(1, 0) / 3.0) / 6.0 -
            (C_dev(5, 1) - 2 * τ_dev(0, 1) / 3.0) / 6.0 -
            (C_dev(5, 1) - 2 * τ_dev(1, 0) / 3.0) / 6.0 +
            (C_dev(5, 2) - 2 * τ_dev(0, 1) / 3.0) / 3.0 +
            (C_dev(5, 2) - 2 * τ_dev(1, 0) / 3.0) / 3.0, //
        C_dev(5, 3), C_dev(5, 4), C_dev(5, 5) + τ_dev.trace() / 3.0;

    return C;
}
}
