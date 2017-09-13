
#include "LinearDiffusionModule.hpp"

#include "io/FileIO.hpp"

namespace neon
{
LinearDiffusionModule::LinearDiffusionModule(BasicMesh const& mesh,
                                             Json::Value const& material,
                                             Json::Value const& simulation)
    : fem_mesh(mesh, material, simulation["Mesh"][0]),
      fem_matrix(fem_mesh,
                 simulation["LinearSolver"],
                 diffusion::FileIO(simulation["Name"].asString(), simulation["Visualisation"], fem_mesh))
{
}
}
