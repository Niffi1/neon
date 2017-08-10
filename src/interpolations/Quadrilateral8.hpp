
#pragma once

#include "SurfaceInterpolation.hpp"
#include "quadrature/QuadrilateralQuadrature.hpp"

namespace neon
{
/** A finite element with 4 nodal points and an isoparametric formulation */
class Quadrilateral8 : public SurfaceInterpolation
{
public:
    Quadrilateral8(QuadrilateralQuadrature::Rule rule);

    virtual int nodes() const override final { return 8; }

    double compute_measure(Matrix const& nodal_coordinates);

protected:
    /**
     * Initialize the shape functions of the quadrilateral 4
     * node element to be:
     * \f{align*}{
     * N_1(\xi, \eta) &= \frac{1}{4}(1-\xi)(1-\eta) \\
     * N_2(\xi, \eta) &= \frac{1}{4}(1+\xi)(1-\eta) \\
     * N_3(\xi, \eta) &= \frac{1}{4}(1+\xi)(1+\eta) \\
     * N_4(\xi, \eta) &= \frac{1}{4}(1-\xi)(1+\eta)
     * \f}
     * TODO The derivatives of the surface element
     */
    void precompute_shape_functions();
};
}
