
#pragma once

#include "mesh/generic/neumann.hpp"

#include "NewtonConvection.hpp"

namespace neon
{
class Submesh;

namespace diffusion
{
/**
 * heat_flux is a group of the same elements on the same boundary. These
 * elements are responsible for computing their elemental right hand side contributions
 * with the corresponding shape function.  These are required to be stored in a parent
 * container with the other groups from the collective boundary \sa SurfaceBoundary
 */
using heat_flux = boundary::surface_load<SurfaceInterpolation>;

// using heat_generation = VolumeLoad<VolumeInterpolation>;

/* boundary_mesh contains the boundary conditions and meshes which contribute to
 * the external load vector.  This can include flux boundary conditions and Newton
 * convection type boundaries.  Each element group has an entry in the vector
 */
class boundary_mesh
{
public:
    explicit boundary_mesh(std::shared_ptr<MaterialCoordinates>& material_coordinates,
                           std::vector<Submesh> const& submeshes,
                           Json::Value const& boundary,
                           Json::Value const& mesh_data);

    /** @return the boundaries which contribute only to the load vector */
    [[nodiscard]] auto const& load_interface() const { return load_boundaries; }

    /** @return the boundaries which contribute to the stiffness and the load vector */
    [[nodiscard]] auto const& stiffness_load_interface() const { return stiffness_load_boundaries; }

protected:
    std::vector<heat_flux> load_boundaries;

    std::vector<newton_cooling> stiffness_load_boundaries;
};
}
}
