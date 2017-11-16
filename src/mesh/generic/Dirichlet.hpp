
#pragma once

#include "mesh/generic/Boundary.hpp"

#include "numeric/IndexTypes.hpp"

namespace neon
{
class Dirichlet : public Boundary
{
public:
    explicit Dirichlet(List dofs, Json::Value const& times, Json::Value const& loads);

    [[nodiscard]] auto const& dof_view() const { return dofs; }

    /** Get the value depending on the loading factor */
    [[nodiscard]] auto value_view(double const load_factor = 1.0) const
    {
        return interpolate_prescribed_load(load_factor);
    }

protected:
    List dofs;
};
}
