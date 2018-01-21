
#include "LinearDiffusion.hpp"

#include "io/json.hpp"

#include "Exceptions.hpp"

namespace neon
{
LinearDiffusion::LinearDiffusion(json const& material_data) : Material(material_data)
{
    if (material_data.isMember("Conductivity"))
    {
        conductivity = material_data["Conductivity"].asDouble();
    }
    if (material_data.isMember("SpecificHeat"))
    {
        specific_heat = material_data["SpecificHeat"].asDouble();
    }
    else
    {
        throw MaterialPropertyException("\"Diffusivity\" needs to be specified as a material "
                                        "property");
    }
}
}
