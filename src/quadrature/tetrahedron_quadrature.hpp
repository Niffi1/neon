
#pragma once

#include "numerical_quadrature.hpp"

namespace neon
{
class tetrahedron_quadrature : public volume_quadrature
{
public:
    /// Available quadrature rules for this element type
    enum class point { one, four, five };

    explicit tetrahedron_quadrature(point const p);
};
}
