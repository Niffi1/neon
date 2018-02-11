
#pragma once

#include "abstract_module.hpp"

#include "assembler/mechanical/solid/fem_static_matrix.hpp"
#include "mesh/mechanical/solid/fem_mesh.hpp"

namespace neon
{
/// This namespace groups together all of the classes and functions associated
/// with solid mechanics finite elements.  The sub-namespaces implement mechanics
/// such as solid, plate, beam, axisymmetric etc
namespace mechanical
{
/// This namespace groups together all of the classes and functions associated
/// with three-dimensional solid mechanics finite elements.  These include
/// constitutive models, matrix systems, meshes, element stiffness matrices etc.
namespace solid
{
}
}

/**
 * solid_mechanics_module is responsible for handling the setup and simulation of the class
 * of three dimensional solid mechanics problems.
 */
class solid_mechanics_module : public abstract_module
{
public:
    solid_mechanics_module(basic_mesh const& mesh, json const& material, json const& simulation);

    virtual ~solid_mechanics_module() = default;

    solid_mechanics_module(solid_mechanics_module const&) = delete;

    solid_mechanics_module(solid_mechanics_module&&) = default;

    virtual void perform_simulation() override final { fem_matrix.solve(); }

protected:
    mechanical::solid::fem_mesh fem_mesh;           //!< Mesh with the solid routines
    mechanical::solid::fem_static_matrix fem_matrix; //!< Nonlinear solver routines
};
}
