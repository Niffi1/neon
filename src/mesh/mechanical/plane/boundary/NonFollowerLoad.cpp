
#include "NonFollowerLoad.hpp"

#include "geometry/Projection.hpp"

#include <utility>

#include <json/value.h>

#include <Eigen/Geometry>

namespace neon::mechanical::plane
{
nonfollower_load_boundary::nonfollower_load_boundary(
    std::shared_ptr<MaterialCoordinates>& material_coordinates,
    std::vector<Submesh> const& submeshes,
    Json::Value const& simulation_data,
    Json::Value const& boundary,
    std::unordered_map<std::string, int> const& dof_table)
{
    for (auto& [is_dof_active, var] : nonfollower_load)
    {
        is_dof_active = false;
    }

    if (auto const& type = boundary["Type"].asString(); type == "Traction")
    {
        for (auto const& name : boundary["Values"].getMemberNames())
        {
            if (dof_table.find(name) == dof_table.end())
            {
                throw std::runtime_error("x, y or z are acceptable coordinates\n");
            }

            auto const dof_offset = dof_table.find(name)->second;

            auto& [is_dof_active, boundary_meshes] = nonfollower_load[dof_offset];

            is_dof_active = true;

            for (auto const& mesh : submeshes)
            {
                boundary_meshes.emplace_back(std::in_place_type_t<traction>{},
                                             make_line_interpolation(mesh.topology(), simulation_data),
                                             mesh.connectivities(),
                                             filter_dof_list(2, dof_offset, mesh.connectivities()),
                                             material_coordinates,
                                             boundary["Time"],
                                             boundary["Values"][name]);
            }
        }
    }
    else if (type == "BodyForce")
    {
        for (auto const& name : boundary["Values"].getMemberNames())
        {
            if (dof_table.find(name) == dof_table.end())
            {
                throw std::runtime_error("x or y are acceptable coordinates\n");
            }
            auto const dof_offset = dof_table.find(name)->second;

            auto& [is_dof_active, boundary_meshes] = nonfollower_load[dof_offset];

            is_dof_active = true;

            for (auto const& mesh : submeshes)
            {
                boundary_meshes.emplace_back(std::in_place_type_t<body_force>{},
                                             make_surface_interpolation(mesh.topology(),
                                                                        simulation_data),
                                             mesh.connectivities(),
                                             filter_dof_list(2, dof_offset, mesh.connectivities()),
                                             material_coordinates,
                                             boundary["Time"],
                                             boundary["Values"][name]);
            }
        }
    }
    else
    {
        throw std::runtime_error("Need to specify a boundary type \"Traction\", \"Pressure\" or "
                                 "\"BodyForce\"");
    }
}
}
