
#include "NewmarkBeta.hpp"

#include <json/value.h>

namespace neon
{
NewmarkBeta::NewmarkBeta(Json::Value const& time_solver_data) : time_control(time_solver_data)
{
    if (time_solver_data["IntegrationOptions"].empty())
    {
        // Choose sensible defaults (if they exist)
    }
    else
    {
        if (time_solver_data["IntegrationOptions"]["ViscousDamping"].empty())
            throw std::runtime_error("IterationOptions - ViscousDamping was not set\n");

        if (time_solver_data["IntegrationOptions"]["BetaParameter"].empty())
            throw std::runtime_error("IterationOptions - BetaParameter was not set\n");

        artifical_viscosity = time_solver_data["IntegrationOptions"]["ViscousDamping"].asDouble();

        beta_parameter = time_solver_data["IntegrationOptions"]["BetaParameter"].asDouble();
    }

    if (are_parameters_unstable())
        throw std::runtime_error("Chosen Newmark-Beta parameters are not stable\n");
}

bool NewmarkBeta::time_loop()
{
    time_control.increment();
    return !time_control.is_finished();
}

bool NewmarkBeta::are_parameters_unstable() const
{
    // Unconditional stability condition
    if (beta_parameter >= artifical_viscosity / 2.0 >= 0.25) return false;

    return true;
}
}
