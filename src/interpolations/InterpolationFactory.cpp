
#include "InterpolationFactory.hpp"

#include "interpolations/Hexahedron8.hpp"
#include "interpolations/Tetrahedron10.hpp"

#include "PreprocessorExceptions.hpp"

#include <json/value.h>

namespace neon
{
namespace solid
{
/** Factory method for the three dimensional shape functions */
std::unique_ptr<VolumeInterpolation> make_shape_function(ElementTopology const topology,
                                                         Json::Value const& simulation_data)
{
    if (!simulation_data.isMember("ElementOptions"))
    {
        throw EmptyFieldException("Part: ElementOptions");
    }

    auto is_reduced =
        simulation_data["ElementOptions"]["Quadrature"].empty()
            ? false
            : simulation_data["ElementOptions"]["Quadrature"].asString() == "Reduced";

    switch (topology)
    {
        case ElementTopology::Hexahedron8:
        {
            return std::make_unique<Hexahedron8>(
                is_reduced ? HexahedronQuadrature::Rule::OnePoint
                           : HexahedronQuadrature::Rule::EightPoint);
        }
        case ElementTopology::Tetrahedron4:
        case ElementTopology::Tetrahedron10:
        {
            return std::make_unique<Tetrahedron10>(
                is_reduced ? TetrahedronQuadrature::Rule::FourPoint
                           : TetrahedronQuadrature::Rule::FivePoint);
        }
        case ElementTopology::Prism6:
        case ElementTopology::Pyramid5:
        case ElementTopology::Tetrahedron20:
        case ElementTopology::Tetrahedron35:
        case ElementTopology::Tetrahedron56:
        case ElementTopology::Hexahedron64:
        case ElementTopology::Hexahedron125:
        case ElementTopology::Hexahedron27:
        case ElementTopology::Prism18:
        case ElementTopology::Pyramid14:
        case ElementTopology::Hexahedron20:
        case ElementTopology::Prism15:
        case ElementTopology::Pyramid13:
        default:
            throw std::runtime_error("Element shape not implemented for continuum "
                                     "simulations\n");
            break;
    }
    return nullptr;
}
}
}
