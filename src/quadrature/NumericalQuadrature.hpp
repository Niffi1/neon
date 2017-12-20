
#pragma once

#include <tuple>
#include <vector>

#include "numeric/DenseMatrix.hpp"

namespace neon
{
/**
 * NumericalQuadrature is a variadic class that defines a generic numerical
 * quadrature class that encapsulates the coordinates, weightings and a method
 * to perform the integration for a functor that accepts a quadrature point
 * index.
 */
template <typename... Xi>
class NumericalQuadrature
{
public:
    using coordinate_type = std::tuple<int, Xi...>;

    /** Fix the size of the shape function derivative to the size of the quadrature points */
    using fem_value_type = std::tuple<vector, matrixxd<std::tuple_size<coordinate_type>::value - 1>>;

public:
    /**
     * Perform the numerical integration of a lambda function.
     * @param integral - Initial value for the numerical integration
     * @param f - A lambda function that accepts an femValue and quadrature point
     * @return The numerically integrated matrix
     */
    template <typename MatrixTp, typename Functor>
    MatrixTp integrate(MatrixTp integral, Functor&& f) const
    {
        for (int l = 0; l < points(); ++l)
        {
            integral.noalias() += f(femvals[l], l) * w[l];
        }
        return integral;
    }

    /**
     * Perform the numerical integration of a lambda function.
     * @param integral - Initial value for the numerical integration
     * @param f - A lambda function that accepts an femValue and quadrature point
     * @return The numerically integrated scalar
     */
    template <typename Functor>
    double integrate(double integral, Functor&& f) const
    {
        for (int l = 0; l < points(); ++l)
        {
            integral += f(femvals[l], l) * w[l];
        }
        return integral;
    }

    template <typename Functor>
    void for_each(Functor&& eval_func) const
    {
        for (auto l = 0; l < points(); ++l) eval_func(femvals[l], l);
    }

    /**
     * Evaluate a shape function and derivatives for the populated quadrature
     * points
     * @param f - A lambda function that accepts a quadrature coordinate tuple
     */
    template <typename Functor>
    void evaluate(Functor&& f)
    {
        femvals.clear();
        femvals.reserve(points());
        for (auto const& coordinate : clist)
        {
            femvals.emplace_back(f(coordinate));
        }
    }

    /** @return The number of quadrature points */
    auto points() const { return w.size(); }

    /** @return The quadrature weights for this scheme */
    auto const& weights() const { return w; }

    /** @return A list of tuples representing the index and the coordinates */
    auto const& coordinates() const { return clist; }

protected:
    std::vector<double> w;              //!< Quadrature weightings
    std::vector<coordinate_type> clist; //!< Quadrature coordinates

    std::vector<fem_value_type> femvals; //!< Shape functions and their derivatives
                                         //!< evaluated at the quadrature points
};

using SurfaceQuadrature = NumericalQuadrature<double, double>;
using VolumeQuadrature = NumericalQuadrature<double, double, double>;
}
