
#pragma once

#include "constitutive/ConstitutiveModel.hpp"

#include "numeric/Tensor.hpp"

#include "material/IsotropicElasticPlastic.hpp"

namespace neon::solid
{
using InternalVariables = neon::InternalVariables<3>;

class IsotropicLinearElasticity : public ConstitutiveModel
{
public:
    explicit IsotropicLinearElasticity(InternalVariables& variables,
                                       Json::Value const& material_data);

    virtual ~IsotropicLinearElasticity();

    virtual void update_internal_variables(double const time_step_size) override;

    [[nodiscard]] virtual Material const& intrinsic_material() const override { return material; }

    [[nodiscard]] virtual bool is_finite_deformation() const override { return false; }

protected:
    [[nodiscard]] Matrix3 compute_cauchy_stress(Matrix3 const& elastic_strain) const;

    [[nodiscard]] Matrix6 elastic_moduli() const;

private:
    LinearElastic material;

protected:
    Matrix6 const C_e = elastic_moduli();
};

/**
 * J2Plasticity is responsible for computing the small strain J2 plasticity
 * stress and tangent operator matrix for each internal variable at the quadrature
 * points.
 */
class J2Plasticity : public IsotropicLinearElasticity
{
public:
    explicit J2Plasticity(InternalVariables& variables, Json::Value const& material_data);

    virtual ~J2Plasticity();

    virtual void update_internal_variables(double const time_step_size) override;

    virtual Material const& intrinsic_material() const override { return material; }

    virtual bool is_finite_deformation() const override { return false; }

protected:
    [[nodiscard]] Matrix6 deviatoric_projection() const;

    [[nodiscard]] Matrix6 algorithmic_tangent(double const plastic_increment,
                                              double const accumulated_plastic_strain,
                                              double const von_mises,
                                              Matrix3 const& normal) const;

    /**
     * Performs the radial return algorithm with nonlinear hardening for
     * projecting the stress onto the yield surface.  This provides the plastic
     * increment required for updating the internal variables
     */
    [[nodiscard]] double perform_radial_return(double const von_mises,
                                               double const accumulated_plastic_strain) const;

    /**
     * Evaluates the yield function and returns greater than zero if
     * the yield function has been violated
     */
    [[nodiscard]] double evaluate_yield_function(double const von_mises,
                                                 double const accumulated_plastic_strain,
                                                 double const plastic_increment = 0.0) const;

protected:
    IsotropicElasticPlastic material;

    Matrix6 const I_dev = voigt::kinematic::deviatoric();
};
}
