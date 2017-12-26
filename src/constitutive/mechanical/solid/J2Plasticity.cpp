
#include "J2Plasticity.hpp"

#include "Exceptions.hpp"

#include "constitutive/InternalVariables.hpp"
#include "numeric/mechanics"

#include <range/v3/view/transform.hpp>

namespace neon::mechanical::solid
{
J2Plasticity::J2Plasticity(std::shared_ptr<InternalVariables>& variables,
                           Json::Value const& material_data)
    : IsotropicLinearElasticity(variables, material_data), material(material_data)
{
    variables->add(InternalVariables::Tensor::LinearisedPlasticStrain);
    variables->add(InternalVariables::Scalar::EffectivePlasticStrain);

    variables->commit();
}

J2Plasticity::~J2Plasticity() = default;

void J2Plasticity::update_internal_variables(double const time_step_size)
{
    auto const shear_modulus = material.shear_modulus();

    // Extract the internal variables
    auto [plastic_strains,
          strains,
          cauchy_stresses] = variables->fetch(InternalVariables::Tensor::LinearisedPlasticStrain,
                                              InternalVariables::Tensor::LinearisedStrain,
                                              InternalVariables::Tensor::Cauchy);

    // Retrieve the accumulated internal variables
    auto [accumulated_plastic_strains,
          von_mises_stresses] = variables->fetch(InternalVariables::Scalar::EffectivePlasticStrain,
                                                 InternalVariables::Scalar::VonMisesStress);

    auto& tangent_operators = variables->fetch(InternalVariables::rank4::tangent_operator);

    // Compute the linear strain gradient from the displacement gradient
    strains = variables->fetch(InternalVariables::Tensor::DisplacementGradient)
              | ranges::view::transform([](auto const& H) { return 0.5 * (H + H.transpose()); });

    // Perform the update algorithm for each quadrature point
    // #pragma omp parallel for
    for (auto l = 0; l < strains.size(); l++)
    {
        auto const& strain = strains[l];
        auto& plastic_strain = plastic_strains[l];
        auto& cauchy_stress = cauchy_stresses[l];
        auto& accumulated_plastic_strain = accumulated_plastic_strains[l];
        auto& von_mises = von_mises_stresses[l];

        // Elastic stress predictor
        cauchy_stress = compute_cauchy_stress(material.shear_modulus(),
                                              material.lambda(),
                                              strain - plastic_strain);

        // Trial von Mises stress
        von_mises = von_mises_stress(cauchy_stress);

        // If this quadrature point is elastic, then set the tangent to the
        // elastic modulus and continue to the next quadrature point
        if (evaluate_yield_function(von_mises, accumulated_plastic_strain) <= 0.0)
        {
            tangent_operators[l] = C_e;
            continue;
        }

        auto const von_mises_trial = von_mises;

        // Compute the normal direction to the yield surface which remains
        // constant throughout the radial return method
        matrix3 const normal = deviatoric(cauchy_stress) / deviatoric(cauchy_stress).norm();

        auto const plastic_increment = perform_radial_return(von_mises, accumulated_plastic_strain);

        plastic_strain += plastic_increment * std::sqrt(3.0 / 2.0) * normal;

        cauchy_stress -= 2.0 * shear_modulus * plastic_increment * std::sqrt(3.0 / 2.0) * normal;

        von_mises = von_mises_stress(cauchy_stress);

        accumulated_plastic_strain += plastic_increment;

        tangent_operators[l] = algorithmic_tangent(plastic_increment,
                                                   accumulated_plastic_strain,
                                                   von_mises_trial,
                                                   normal);
    }
}

matrix6 J2Plasticity::algorithmic_tangent(double const plastic_increment,
                                          double const accumulated_plastic_strain,
                                          double const von_mises,
                                          matrix3 const& normal) const
{
    auto const G = material.shear_modulus();
    auto const H = material.hardening_modulus(accumulated_plastic_strain);

    return C_e - plastic_increment * 6.0 * std::pow(G, 2) / von_mises * I_dev
           + 6.0 * std::pow(G, 2) * (plastic_increment / von_mises - 1.0 / (3.0 * G + H))
                 * outer_product(normal, normal);
}

double J2Plasticity::perform_radial_return(double const von_mises,
                                           double const accumulated_plastic_strain) const
{
    auto const shear_modulus = material.shear_modulus();

    auto plastic_increment{0.0};

    auto f = evaluate_yield_function(von_mises, accumulated_plastic_strain);

    // Perform the non-linear hardening solve
    int iterations{0};
    auto constexpr max_iterations{50};
    while (f > 1.0e-6 && iterations < max_iterations)
    {
        auto const H = material.hardening_modulus(accumulated_plastic_strain + plastic_increment);

        auto const plastic_increment_delta = f / (3.0 * shear_modulus + H);

        plastic_increment += plastic_increment_delta;

        f = evaluate_yield_function(von_mises, accumulated_plastic_strain, plastic_increment);

        iterations++;
    }
    if (iterations == max_iterations)
    {
        std::cout << "\n";
        std::cout << std::string(8, ' ') << "Plastic increment : " << plastic_increment << "\n";

        std::cout << std::string(8, ' ')
                  << "Accumulated plastic strain : " << accumulated_plastic_strain << "\n";

        std::cout << std::string(8, ' ') << "Hardening modulus : "
                  << material.hardening_modulus(accumulated_plastic_strain + plastic_increment)
                  << "\n";

        std::cout << std::string(8, ' ') << "Shear modulus : " << shear_modulus << "\n";

        std::cout << std::string(8, ' ') << "Yield function after mapping : " << f << "\n";

        std::cout << std::string(8, ' ')
                  << "Current yield stress : " << material.yield_stress(accumulated_plastic_strain)
                  << "\n";

        throw computational_error("Non-convergence in radial return method.");
    }
    return plastic_increment;
}

double J2Plasticity::evaluate_yield_function(double const von_mises,
                                             double const accumulated_plastic_strain,
                                             double const plastic_increment) const
{
    return (von_mises - 3.0 * material.shear_modulus() * plastic_increment)
           - material.yield_stress(accumulated_plastic_strain + plastic_increment);
}
}