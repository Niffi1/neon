
#pragma once

#include "Hyperelastic.hpp"

#include "material/MicromechanicalElastomer.hpp"
#include "numeric/Tensor.hpp"
#include "quadrature/UnitSphereQuadrature.hpp"

#include <json/forwards.h>

namespace neon
{
class AffineMicrosphere : public Hyperelastic
{
public:
    /**
     * @param variables Reference to internal state variable store
     * @param data Json object with material data
     */
    explicit AffineMicrosphere(InternalVariables& variables,
                               Json::Value const& material_data);

    virtual void update_internal_variables(double const Δt) override;

    Material const& intrinsic_material() const override final { return material; };

    virtual bool is_finite_deformation() const override final { return true; };

protected:
    /**
     * \f{align*}{
         U' &= \frac{\partial U}{\partial J} = \frac{K}{2}\left(J - \frac{1}{J}\right)
       \f}
     * where
     * \f{align*}{
         U &= \frac{K}{4}(J^2 - 1) - \frac{K}{2}\ln{J}
       \f}
     */
    double volumetric_free_energy_derivative(double const J,
                                             double const bulk_modulus) const;

    /**
     * \f{align*}{
         U'' &= \frac{\partial^2 U}{\partial J^2} = \frac{K}{2} \left(1 +
     \frac{1}{J^2}\right) \f}
     * \sa volumetric_free_energy_derivative
     */
    double volumetric_free_energy_second_derivative(double const J,
                                                    double const bulk_modulus) const;

    /**
     * Compute the Padé approximation of the inverse Langevin stretch model
     * \f{align*}{
         n \psi_f^{'}(\lambda) &= \frac{3N - \lambda^2}{N - \lambda^2}
       }
     */
    double pade_first(double const λ, double const N) const;

    /**
     * Compute the Padé approximation of the inverse Langevin stretch model
     * \f{align*}{
         n \psi_f^{''}(\lambda) &= \frac{\lambda^4 + 3N^2}{(N - \lambda^2)^2}
       }
     */
    double pade_second(double const λ, double const N) const;

    /**
     *\f{align*}{
     * \boldsymbol{\tau} &= p \boldsymbol{g}^{-1} + \mathbb{P} : \bar{\boldsymbol{\tau}}
     * \f}
     */
    Matrix3 deviatoric_projection(double const pressure, Matrix3 const& τ_dev) const;

    /**
     *\f{align*}{
        \boldsymbol{C} &= \mathbb{P} : \left[\bar{\mathbb{C}} +
     \frac{2}{3}(\boldsymbol{\tau} : \boldsymbol{g}) \mathbb{I} - \frac{2}{3}
     (\bar{\boldsymbol{\tau}} \otimes \boldsymbol{g}^{-1} + \boldsymbol{g}^{-1} \otimes
     \bar{\boldsymbol{\tau}}) \right] : \mathbb{P} \f}
     */
    CMatrix deviatoric_projection(CMatrix const& C_dev, Matrix3 const& τ_dev) const;

    /**
     * @param N number of segments per chain
     * @return Kirchhoff stress tensor
     */
    Matrix3 compute_kirchhoff_stress(Matrix3 const& unimodular_F, double const N) const;

    CMatrix compute_material_matrix(Matrix3 const& unimodular_F, double const N) const;

    template <typename MatrixTp, typename Functor>
    MatrixTp weighting(MatrixTp accumulator, Functor f) const;

protected:
    MicromechanicalElastomer material;

    UnitSphereQuadrature unit_sphere;

    Matrix const IoI = I_outer_I();
    Matrix const I = fourth_order_identity();
};

inline double AffineMicrosphere::volumetric_free_energy_derivative(
    double const J, double const bulk_modulus) const
{
    return bulk_modulus / 2.0 * (J - 1.0 / J);
}

inline double AffineMicrosphere::volumetric_free_energy_second_derivative(
    double const J, double const bulk_modulus) const
{
    return bulk_modulus / 2.0 * (1.0 + 1.0 / std::pow(J, 2));
}

inline double AffineMicrosphere::pade_first(double const λ, double const N) const
{
    return (3.0 * N - std::pow(λ, 2)) / (N - std::pow(λ, 2));
}

inline double AffineMicrosphere::pade_second(double const λ, double const N) const
{
    return (std::pow(λ, 4) + 3 * std::pow(N, 2)) / std::pow(N - std::pow(λ, 2), 2);
}

inline CMatrix AffineMicrosphere::deviatoric_projection(CMatrix const& C_dev,
                                                        Matrix3 const& τ_dev) const
{
    return (CMatrix(6, 6) << 1.0 / 9.0
                                 * (4 * C_dev(0, 0) - 4 * C_dev(0, 1) - 4 * C_dev(0, 2)
                                    + C_dev(1, 1) + 2 * C_dev(1, 2) + C_dev(2, 2)
                                    + 4 * τ_dev.trace()), //
            1.0 / 9.0
                * (-2 * C_dev(0, 0) + 5 * C_dev(0, 1) - C_dev(0, 2) - 2 * C_dev(1, 1)
                   - C_dev(1, 2) + C_dev(2, 2) - 2.0 * τ_dev.trace()), //
            1.0 / 9.0
                * (-2 * C_dev(0, 0) - C_dev(0, 1) + 4 * C_dev(0, 2) + C_dev(1, 1)
                   - C_dev(1, 2) + C_dev(2, 0) - 2 * C_dev(2, 2) - 2.0 * τ_dev.trace()), //
            2.0 / 3.0 * (C_dev(0, 3) - C_dev(1, 3) - C_dev(2, 3)), //
            2.0 / 3.0 * (C_dev(0, 4) - C_dev(1, 4) - C_dev(2, 4)), //
            2.0 / 3.0 * (C_dev(0, 5) - C_dev(1, 5) - C_dev(2, 5)), //

            1.0 / 9.0
                * (-2 * C_dev(0, 0) + C_dev(0, 1) + C_dev(0, 2) + 4 * C_dev(1, 0)
                   - 2 * C_dev(1, 1) - 2 * C_dev(1, 2) - 2 * C_dev(2, 0) + C_dev(2, 1)
                   + C_dev(2, 2) - 2.0 * τ_dev.trace()), //
            1.0 / 9.0
                * (C_dev(0, 0) - 2 * C_dev(0, 1) + C_dev(0, 2) - 2 * C_dev(1, 0)
                   + 4 * C_dev(1, 1) - 2 * C_dev(1, 2) + C_dev(2, 0) - 2 * C_dev(2, 1)
                   + C_dev(2, 2) + 4 * τ_dev.trace()), //
            1.0 / 9.0
                * (C_dev(0, 0) + C_dev(0, 1) - 2 * C_dev(0, 2) - 2 * C_dev(1, 0)
                   - 2 * C_dev(1, 1) + 4 * C_dev(1, 2) + C_dev(2, 0) + C_dev(2, 1)
                   - 2 * C_dev(2, 2) - 2.0 * τ_dev.trace()),            //
            1.0 / 3.0 * (-C_dev(0, 3) + 2 * C_dev(1, 3) - C_dev(2, 3)), //
            1.0 / 3.0 * (-C_dev(0, 4) + 2 * C_dev(1, 4) - C_dev(2, 4)), //
            1.0 / 3.0 * (-C_dev(0, 5) + 2 * C_dev(1, 5) - C_dev(2, 5)), //

            1.0 / 9.0
                * (-2 * C_dev(0, 0) + C_dev(0, 1) + C_dev(0, 2) - 2 * C_dev(1, 0)
                   + C_dev(1, 1) + C_dev(1, 2) + 4 * C_dev(2, 0) - 2 * C_dev(2, 1)
                   - 2 * C_dev(2, 2) - 2.0 * τ_dev.trace()), //
            1.0 / 9.0
                * (C_dev(0, 0) - 2 * C_dev(0, 1) + C_dev(0, 2) + C_dev(1, 0)
                   - 2 * C_dev(1, 1) + C_dev(1, 2) - 2 * C_dev(2, 0) + 4 * C_dev(2, 1)
                   - 2 * C_dev(2, 2) - 2.0 * τ_dev.trace()), //
            1.0 / 9.0
                * (C_dev(0, 0) + C_dev(0, 1) - 2 * C_dev(0, 2) + C_dev(1, 0) + C_dev(1, 1)
                   - 2 * C_dev(1, 2) - 2 * C_dev(2, 0) - 2 * C_dev(2, 1) + 4 * C_dev(2, 2)
                   + 4 * τ_dev.trace()),                                //
            1.0 / 3.0 * (-C_dev(0, 3) - C_dev(1, 3) + 2 * C_dev(2, 3)), //
            1.0 / 3.0 * (-C_dev(0, 4) - C_dev(1, 4) + 2 * C_dev(2, 4)), //
            1.0 / 3.0 * (-C_dev(0, 5) - C_dev(1, 5) + 2 * C_dev(2, 5)), //

            1.0 / 3.0 * (2 * C_dev(3, 0) - C_dev(3, 1) - C_dev(3, 2)),  //
            1.0 / 3.0 * (-C_dev(3, 0) + 2 * C_dev(3, 1) - C_dev(3, 2)), //
            1.0 / 3.0 * (-C_dev(3, 0) - C_dev(3, 1) + 2 * C_dev(3, 2)), //
            C_dev(3, 3) + τ_dev.trace() / 3.0,                          //
            C_dev(3, 4),                                                //
            C_dev(3, 5),                                                //

            1.0 / 3.0 * (2 * C_dev(4, 0) - C_dev(4, 1) - C_dev(4, 2)),  //
            1.0 / 3.0 * (-C_dev(4, 0) + 2 * C_dev(4, 1) - C_dev(4, 2)), //
            1.0 / 3.0 * (-C_dev(4, 0) - C_dev(4, 1) + 2 * C_dev(4, 2)), //
            C_dev(4, 3),                                                //
            C_dev(4, 4) + τ_dev.trace() / 3.0,                          //
            C_dev(4, 5),                                                //

            1.0 / 3.0 * (2 * C_dev(5, 0) - C_dev(5, 1) - C_dev(5, 2)),  //
            1.0 / 3.0 * (-C_dev(5, 0) + 2 * C_dev(5, 1) - C_dev(5, 2)), //
            1.0 / 3.0 * (-C_dev(5, 0) - C_dev(5, 1) + 2 * C_dev(5, 2)), //
            C_dev(5, 3),                                                //
            C_dev(5, 4),                                                //
            C_dev(5, 5) + τ_dev.trace() / 3.0)
        .finished();
}

inline Matrix3 AffineMicrosphere::compute_kirchhoff_stress(Matrix3 const& unimodular_F,
                                                           double const N) const
{
    return unit_sphere.integrate(Matrix3::Zero().eval(),
                                 [&](auto const& coordinates, auto const& l) -> Matrix3 {
                                     auto const & [ r, r_outer_r ] = coordinates;

                                     // Deformed tangents
                                     Vector3 const t = unimodular_F * r;

                                     // Microstretches
                                     auto const λ = t.norm();

                                     return pade_first(λ, N) * t * t.transpose();
                                 });
}

inline CMatrix AffineMicrosphere::compute_material_matrix(Matrix3 const& unimodular_F,
                                                          double const N) const
{
    return unit_sphere.integrate(CMatrix::Zero(6, 6).eval(),
                                 [&](auto const& coordinates, auto const& l) -> CMatrix {
                                     auto const & [ r, r_outer_r ] = coordinates;

                                     // Deformed tangents
                                     auto const t = unimodular_F * r;

                                     // Microstretches
                                     auto const λ = t.norm();

                                     auto const a = std::pow(λ, -2)
                                                    * (pade_second(λ, N) - pade_first(λ, N));

                                     return a * voigt(t * t.transpose())
                                            * voigt(t * t.transpose()).transpose();
                                 });
}

template <typename MatrixTp, typename Functor>
inline MatrixTp AffineMicrosphere::weighting(MatrixTp accumulator, Functor f) const
{
    for (auto const & [ N, β ] : material.segment_probability())
    {
        accumulator.noalias() += f(N) * β;
    }
    return accumulator;
}
}
