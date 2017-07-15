
#pragma once

#include "Hyperelastic.hpp"

#include <material/LinearElastic.hpp>

#include <json/forwards.h>

namespace neon
{
class NeoHooke : public Hyperelastic
{
public:
    /**
     * @param variables Reference to internal state variable store
     * @param data Json object with material data
     */
    explicit NeoHooke(InternalVariables& variables, Json::Value const& material_data);

    ~NeoHooke() = default;

    void update_internal_variables(double const Δt) override final;

    Material const& intrinsic_material() const override final { return material; };

private:
    LinearElastic material; //!< Elastic model where C1 = mu/2 and C2 = bulk-modulus / 2
};
}
