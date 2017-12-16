
#include "Line.hpp"

#include <Eigen/Geometry>

namespace neon
{
Line2::Line2(LineQuadrature::Rule rule) : LineInterpolation(std::make_unique<LineQuadrature>(rule))
{
    this->precompute_shape_functions();
}

void Line2::precompute_shape_functions()
{
    using NodalCoordinate = std::tuple<int, double>;

    // Initialize nodal coordinates array as Xi, Eta, Zeta
    std::array<NodalCoordinate, 2> constexpr local_coordinates{{{0, -1.0}, {1, 1.0}}};

    matrix N_matrix(numerical_quadrature->points(), nodes());
    matrix local_quadrature_coordinates = matrix::Ones(numerical_quadrature->points(), 2);

    numerical_quadrature->evaluate([&](auto const& coordinates) {
        auto const& [l, xi] = coordinates;

        vector N(2);
        matrix rhea(2, 1);

        N(0) = 1.0 / 2.0 * (1.0 - xi);
        N(1) = 1.0 / 2.0 * (1.0 + xi);

        rhea(0, 0) = -1.0 / 2.0;
        rhea(1, 0) = 1.0 / 2.0;

        local_quadrature_coordinates(l, 0) = xi;

        N_matrix.row(l) = N;

        return std::make_tuple(N, rhea);
    });

    // Compute extrapolation algorithm matrices
    matrix local_nodal_coordinates = matrix::Ones(nodes(), 3);

    for (auto const& [a, xi_a] : local_coordinates)
    {
        local_nodal_coordinates(a, 0) = xi_a;
    }
    compute_extrapolation_matrix(N_matrix, local_nodal_coordinates, local_quadrature_coordinates);
}

double Line2::compute_measure(matrix const& nodal_coordinates) const
{
    return (nodal_coordinates.col(0) - nodal_coordinates.col(1)).norm();
}

Line3::Line3(LineQuadrature::Rule rule) : LineInterpolation(std::make_unique<LineQuadrature>(rule))
{
    this->precompute_shape_functions();
}

void Line3::precompute_shape_functions()
{
    using NodalCoordinate = std::tuple<int, double>;

    // Initialize nodal coordinates array as Xi, Eta, Zeta
    std::array<NodalCoordinate, 2> constexpr local_coordinates{{{0, -1.0}, {1, 1.0}}};

    matrix N_matrix(numerical_quadrature->points(), nodes());
    matrix local_quadrature_coordinates = matrix::Ones(numerical_quadrature->points(), 2);

    numerical_quadrature->evaluate([&](auto const& coordinates) {
        auto const& [l, xi] = coordinates;

        vector N(3);
        matrix rhea(3, 1);

        N(0) = 1.0 / 2.0 * xi * (xi - 1.0);
        N(1) = 1.0 - std::pow(xi, 2);
        N(2) = 1.0 / 2.0 * xi * (xi + 1.0);

        rhea(0, 0) = 1.0 / 2.0 * (2.0 * xi - 1.0);
        rhea(1, 0) = -2.0 * xi;
        rhea(2, 0) = 1.0 / 2.0 * (2.0 * xi + 1.0);

        local_quadrature_coordinates(l, 0) = xi;

        N_matrix.row(l) = N;

        return std::make_tuple(N, rhea);
    });

    // Compute extrapolation algorithm matrices
    matrix local_nodal_coordinates = matrix::Ones(nodes(), 3);

    for (auto const& [a, xi_a] : local_coordinates)
    {
        local_nodal_coordinates(a, 0) = xi_a;
    }
    compute_extrapolation_matrix(N_matrix, local_nodal_coordinates, local_quadrature_coordinates);
}

double Line3::compute_measure(Matrix const& nodal_coordinates) const
{
    return (nodal_coordinates.col(0) - nodal_coordinates.col(2)).norm();
}
}
