
#pragma once

#include "shape_function.hpp"

#include "mesh/element_topology.hpp"

#include <json/forwards.h>

#include <memory>

namespace neon
{
/** Factory method for the three dimensional shape functions */
std::unique_ptr<volume_interpolation> make_volume_interpolation(element_topology const topology,
                                                                Json::Value const& simulation_data);

/** Factory method for the two dimensional shape functions */
std::unique_ptr<surface_interpolation> make_surface_interpolation(element_topology const topology,
                                                                  Json::Value const& simulation_data);

/** Factory method for the two dimensional shape functions */
std::unique_ptr<line_interpolation> make_line_interpolation(element_topology const topology,
                                                            Json::Value const& simulation_data);
}
