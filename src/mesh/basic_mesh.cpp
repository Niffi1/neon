
#include "basic_mesh.hpp"
#include "exceptions.hpp"
#include "io/json.hpp"

namespace neon
{
basic_mesh::basic_mesh(json const& mesh_file) : nodal_coordinates(mesh_file)
{
    if (mesh_file.find("Elements") == mesh_file.end())
    {
        throw std::domain_error("The mesh file is missing the \"Elements\" field");
    }

    for (auto const& mesh : mesh_file["Elements"])
    {
        meshes_map[mesh["Name"].get<std::string>()].push_back(mesh);
    }
}

std::vector<basic_submesh> const& basic_mesh::meshes(std::string const& name) const
{
    auto const found = meshes_map.find(name);

    if (found == meshes_map.end())
    {
        throw std::domain_error("Mesh name " + name + " does not exist in the mesh store");
    }
    return found->second;
}
}
