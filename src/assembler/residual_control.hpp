
#pragma once

#include "io/json_forward.hpp"

namespace neon
{
///
class residual_control
{
public:
    residual_control(json const& residual_data);

    void set_initial_residual(double first_residual_norm) noexcept;

    void update(double const displacement_norm,
                double const increment_displacment_norm,
                double const residual_norm,
                double const external_force_norm,
                double const internal_force_norm) noexcept;

    bool is_converged() const noexcept;

    void print() const noexcept;

private:
    bool m_use_relative_norm{true};

    double m_residual_norm;
    double m_displacement_norm;
    double m_norm_initial_residual{1.0};

    double m_residual_tolerance{1.0e-3};
    double m_displacement_tolerance{1.0e-3};
};
}
