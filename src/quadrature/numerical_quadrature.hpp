
#pragma once

#include <tuple>
#include <vector>

#include "numeric/dense_matrix.hpp"

namespace neon
{
/// numerical_quadrature is a variadic class that defines a generic numerical
/// quadrature class that encapsulates the coordinates, weightings and a method
/// to perform the integration for a function that accepts a quadrature point
/// index.
template <typename... Xi>
class numerical_quadrature
{
public:
    using coordinate_type = std::tuple<int, Xi...>;

    /// Fix the size of the shape function derivative to the size of the quadrature points
    using fem_value_type = std::tuple<vector, matrixxd<sizeof...(Xi)>>;

public:
    /// Perform the numerical integration of a lambda function.
    /// \param integral - Initial value for the numerical integration
    /// \param f - A lambda function that accepts an femValue and quadrature point
    /// \return The numerically integrated matrix
    template <typename MatrixType, typename Callable>
    [[nodiscard]] MatrixType integrate(MatrixType operand, Callable&& f) const noexcept
    {
        for (std::size_t l{0}; l < points(); ++l)
        {
            operand.noalias() += f(femvals[l], l) * m_weights[l];
        }
        return operand;
    }

    /// Perform the numerical integration of a lambda function.
    /// \param integral Value for the numerical integration (accumulated into)
    /// \param f A lambda function that accepts an femValue and quadrature point
    /// \return The numerically integrated matrix
    template <typename Callable>
    void integrate_inplace(matrix& integral, Callable&& f) const noexcept
    {
        for (std::size_t l{0}; l < points(); ++l)
        {
            integral.noalias() += f(femvals[l], l) * m_weights[l];
        }
    }

    /// Perform the numerical integration of a lambda function.
    /// \param integral Value for the numerical integration (accumulated into)
    /// \param f A lambda function that accepts an femValue and quadrature point
    /// \return The numerically integrated matrix
    template <typename MatrixType, typename Callable>
    void integrate_inplace(Eigen::Map<MatrixType> integral, Callable&& f) const noexcept
    {
        for (std::size_t index{0}; index < points(); ++index)
        {
            integral.noalias() += f(femvals[index], index) * m_weights[index];
        }
    }

    /// Perform the numerical integration of a lambda function.
    /// \param integral - Initial value for the numerical integration
    /// \param f - A lambda function that accepts an femValue and quadrature point
    /// \return The numerically integrated scalar
    template <typename Callable>
    [[nodiscard]] double integrate(double integral, Callable&& f) const noexcept
    {
        for (std::size_t l{0}; l < points(); ++l)
        {
            integral += f(femvals[l], l) * m_weights[l];
        }
        return integral;
    }

    /// Evaluate the function \p function at each integration point
    /// \tparam A callable type
    template <typename Callable>
    void for_each(Callable&& function) const
    {
        for (std::size_t l{0}; l < points(); ++l)
        {
            function(femvals[l], l);
        }
    }

    /// Evaluate a shape function and derivatives for the quadrature points
    /// Throws std::out_of_memory
    /// \param f - A lambda function that accepts a quadrature coordinate tuple
    template <typename Callable>
    void evaluate(Callable&& f)
    {
        femvals.clear();
        femvals.reserve(points());
        for (auto const& coordinate : m_coordinates)
        {
            femvals.emplace_back(f(coordinate));
        }
    }

    /// \return number of quadrature points
    [[nodiscard]] auto points() const noexcept { return m_weights.size(); }

    /// \return quadrature weights for this scheme
    [[nodiscard]] auto const& weights() const noexcept { return m_weights; }

    /// \return list of tuples representing the index and the coordinates
    [[nodiscard]] auto const& coordinates() const noexcept { return m_coordinates; }

protected:
    /// Quadrature weightings
    std::vector<double> m_weights;
    /// Quadrature coordinates
    std::vector<coordinate_type> m_coordinates;

    /// Shape functions and their derivatives evaluated at the quadrature points
    std::vector<fem_value_type> femvals;
};

extern template class numerical_quadrature<double>;
extern template class numerical_quadrature<double, double>;
extern template class numerical_quadrature<double, double, double>;

using surface_quadrature = numerical_quadrature<double, double>;
using volume_quadrature = numerical_quadrature<double, double, double>;
}
