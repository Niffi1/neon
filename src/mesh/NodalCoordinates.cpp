
#include "NodalCoordinates.hpp"

#include "Exceptions.hpp"

#include "io/json.hpp"

namespace neon
{
NodalCoordinates::NodalCoordinates(matrix3x coordinates) : X(coordinates) {}

NodalCoordinates::NodalCoordinates(json const& mesh_file)
{
    if (mesh_file["Nodes"].empty())
        throw std::runtime_error("The mesh file is missing the \"Nodes\" field");

    auto const& coordinates = mesh_file["Nodes"][0]["Coordinates"];

    auto const nodes = coordinates.size();

    X.resize(3, nodes);

    for (auto node = 0; node < nodes; ++node)
    {
        for (auto i = 0; i < 3; ++i)
        {
            X(i, node) = coordinates[node][i];
        }
    }
}

matrix3x const& NodalCoordinates::coordinates() const { return X; }

matrix3x NodalCoordinates::coordinates(local_indices const& local_node_list) const
{
    return X(Eigen::placeholders::all, local_node_list);
}
}
