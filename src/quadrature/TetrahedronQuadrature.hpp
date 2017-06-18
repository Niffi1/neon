
#pragma once

#include "NumericalQuadrature.hpp"

namespace neon
{
class TetrahedronQuadrature : public VolumeQuadrature
{
public:
    /** Available quadrature rules for this element type */
    enum class Rule { OnePoint, FourPoint, FivePoint };

    TetrahedronQuadrature(Rule rule, int interpolationOrder = 1);
};
}
