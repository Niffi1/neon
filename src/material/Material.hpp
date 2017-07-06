
#include <json/forwards.h>
#include <string>

#pragma once

namespace neon
{
class Material
{
public:
    Material(Json::Value const& intrinsic_material_data);

    virtual ~Material() = default;

    std::string const& name() const { return material_name; }

    /** @return the density if specified in the material data */
    double initial_density() const;

protected:
    std::string material_name;

    bool is_density_specified = false;
    double density_0 = 0.0;
};
}
