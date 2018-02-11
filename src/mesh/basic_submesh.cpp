
#include "basic_submesh.hpp"

#include "mesh/node_ordering_adapter.hpp"

#include "Exceptions.hpp"

#include "io/json.hpp"

#include <range/v3/action/join.hpp>
#include <range/v3/action/sort.hpp>
#include <range/v3/action/unique.hpp>

namespace neon
{
basic_submesh::basic_submesh(json const& mesh)
{
    // Error checking for empty fields
    if (!mesh.count("Name"))
    {
        throw std::runtime_error("The element group in the mesh file is missing the "
                                 "\"Name\" field");
    }
    if (!mesh.count("Type"))
    {
        throw std::runtime_error("The element group in the mesh file is missing the "
                                 "\"Type\" field");
    }
    if (!mesh.count("NodalConnectivity"))
    {
        throw std::runtime_error("The element group in the mesh file is missing the "
                                 "\"NodalConnectivity\" field");
    }
    if (mesh["NodalConnectivity"].size() == 0 || mesh["NodalConnectivity"][0].size() == 0)
    {
        throw std::runtime_error("The element group in the mesh file is empty");
    }

    m_topology = gmsh_type_to_enum(mesh["Type"].get<int>());

    nodal_connectivity.reserve(mesh["NodalConnectivity"].size());

    for (auto const& mesh_connectivity : mesh["NodalConnectivity"])
    {
        nodal_connectivity.push_back(local_indices());
        nodal_connectivity.back().reserve(mesh_connectivity.size());

        for (auto const& node : mesh_connectivity)
        {
            nodal_connectivity.back().push_back(node.get<int64_t>());
        }
    }
    convert_from_gmsh(nodal_connectivity, m_topology);
}

local_indices basic_submesh::unique_connectivities() const
{
    using namespace ranges;
    return std::ref(nodal_connectivity) | action::join | action::sort | action::unique;
}
}
