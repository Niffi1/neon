
#include "isotropic_diffusion.hpp"
#include "constitutive/internal_variables.hpp"

namespace neon::diffusion
{
isotropic_diffusion::isotropic_diffusion(std::shared_ptr<internal_variables_t>& variables,
                                         json const& material_data)
    : constitutive_model(variables), material(material_data)
{
    variables->add(variable::second::conductivity);

    for (auto& k : variables->get(variable::second::conductivity))
    {
        k = material.conductivity() * matrix3::Identity();
    }
}

void isotropic_diffusion::update_internal_variables(double const) {}
}
