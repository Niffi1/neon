
#include "residual_control.hpp"

#include "numeric/float_compare.hpp"
#include "io/json.hpp"

#include <iostream>
#include <termcolor/termcolor.hpp>

namespace neon
{
residual_control::residual_control(json const& residual_data)
{
    if (residual_data.find("displacement_tolerance") == end(residual_data))
    {
        throw std::domain_error("displacement_tolerance not specified in residual_data");
    }
    if (residual_data.find("residual_tolerance") == end(residual_data))
    {
        throw std::domain_error("residual_tolerance not specified in residual_data");
    }
    if (residual_data.find("absolute_tolerance") != end(residual_data))
    {
        m_use_relative_norm = false;
    }

    m_residual_tolerance = residual_data["residual_tolerance"];
    m_displacement_tolerance = residual_data["displacement_tolerance"];
}

void residual_control::set_initial_residual(double first_residual_norm) noexcept
{
    m_norm_initial_residual = first_residual_norm;
}

void residual_control::update(double const displacement_norm,
                              double const increment_displacment_norm,
                              double const residual_norm,
                              double const external_force_norm,
                              double const internal_force_norm) noexcept
{
    if (m_use_relative_norm)
    {
        m_displacement_norm = increment_displacment_norm / displacement_norm;

        double const max_residual = std::max(external_force_norm, internal_force_norm);

        m_residual_norm = is_approx(max_residual, 0.0)
                              ? 1.0
                              : residual_norm / std::max(m_norm_initial_residual, max_residual);
    }
    else
    {
        m_displacement_norm = increment_displacment_norm;
        m_residual_norm = residual_norm;
    }
}

bool residual_control::is_converged() const noexcept
{
    return m_displacement_norm <= m_displacement_tolerance && m_residual_norm <= m_residual_tolerance;
}

void residual_control::print() const noexcept
{
    std::cout << std::string(6, ' ') << termcolor::bold;
    if (m_displacement_norm <= m_displacement_tolerance)
    {
        std::cout << termcolor::green;
    }
    else
    {
        std::cout << termcolor::yellow;
    }
    std::cout << "Incremental displacement norm " << m_displacement_norm << "\n"
              << termcolor::reset << std::string(6, ' ');

    if (m_residual_norm <= m_residual_tolerance)
    {
        std::cout << termcolor::green;
    }
    else
    {
        std::cout << termcolor::yellow;
    }
    std::cout << termcolor::bold << "Residual force norm " << m_residual_norm << termcolor::reset
              << "\n";
}

}
