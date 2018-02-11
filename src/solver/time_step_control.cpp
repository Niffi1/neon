
#include "time_step_control.hpp"

#include "io/json.hpp"

#include <stdexcept>

namespace neon
{
time_step_control::time_step_control(json const& time_data)
{
    if (time_data["Start"].is_null())
    {
        throw std::runtime_error("Start time not specified in input (\"Start\")\n");
    }

    if (time_data["End"].is_null())
    {
        throw std::runtime_error("End time not specified in input (\"End\")\n");
    }

    if (time_data["StepSize"].is_null())
    {
        throw std::runtime_error("\"StepSize\" not specified in input\n");
    }
    double total_time = time_data["End"].get<double>() - time_data["Start"].get<double>();

    time_step_size = time_data["StepSize"];

    time_steps = static_cast<std::int64_t>(total_time / time_step_size);
}
}
