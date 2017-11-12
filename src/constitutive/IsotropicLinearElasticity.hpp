
#pragma once

#include "constitutive/ConstitutiveModel.hpp"

#include "numeric/DenseTypes.hpp"

#include "material/IsotropicElasticPlastic.hpp"

namespace neon
{
namespace mech::solid
{
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
}

// namespace mech::plane
// {
// using InternalVariables = neon::InternalVariables<3>;
//
// class IsotropicLinearElasticity : public ConstitutiveModel
// {
// public:
//     explicit IsotropicLinearElasticity(InternalVariables& variables,
//                                        Json::Value const& material_data);
//
//     virtual ~IsotropicLinearElasticity();
//
//     virtual void update_internal_variables(double const time_step_size) override;
//
//     [[nodiscard]] virtual Material const& intrinsic_material() const override { return material;
//     }
//
//     [[nodiscard]] virtual bool is_finite_deformation() const override { return false; }
//
// protected:
//     [[nodiscard]] Matrix2 compute_cauchy_stress(Matrix3 const& elastic_strain) const;
//
//     [[nodiscard]] Matrix3 elastic_moduli() const;
//
// private:
//     LinearElastic material;
//
// protected:
//     Matrix3 const C_e = elastic_moduli();
// };
// }
}
