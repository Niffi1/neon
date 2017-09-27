
#pragma once

#include <json/forwards.h>

#include "numeric/DenseTypes.hpp"
#include "numeric/SparseTypes.hpp"

namespace neon
{
class GeneralisedTrapezoidal
{
public:
    GeneralisedTrapezoidal(Json::Value const& time_solver_data);

    /** Perform the time integration until returns false */
    bool loop();

    double current_time_step_size() const { return time_step_size; }

protected:
    double method; //!< 0.0 if forward Euler, 0.5 if Crank-Nicolson and 1.0 if backward Euler

    double start_time = 0.0, final_time = 1.0, time_step_size = 1.0;
};
}
