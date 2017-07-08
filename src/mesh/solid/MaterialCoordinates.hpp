
#pragma once

#include "mesh/NodalCoordinates.hpp"

#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>

#include "vtkSmartPointer.h"
class vtkPoints;
class vtkDoubleArray;

namespace neon::solid
{
class MaterialCoordinates : public NodalCoordinates
{
public:
    /** Construct this class using a set of initial coordinates */
    MaterialCoordinates(Vector const& initial_coordinates);

    /** @return element reference configuration based on the local node numbers*/
    Matrix initial_configuration(List const& local_nodes) const;

    /** @return element current configuration based on the local node numbers*/
    Matrix current_configuration(List const& local_nodes) const;

    /** @param u - displacement vector from initial configuration (x,y,z...) */
    void update_current_configuration(Vector const& u) { x = X + u; };

    Vector displacement() const { return x - X; }

    Vector displacement(List const& local_dofs) const;

    /** @return a vtk object of the initial coordinates */
    vtkSmartPointer<vtkPoints> vtk_coordinates() const;

    /** @return a vtk array of nodal displacements */
    vtkSmartPointer<vtkDoubleArray> vtk_displacement() const;

protected:
    Matrix get_configuration(List const& local_nodes, Vector const& configuration) const;

protected:
    Vector x; //!< Current configuration
};

inline Matrix MaterialCoordinates::initial_configuration(List const& local_node_list) const
{
    return this->get_configuration(local_node_list, X);
}

inline Matrix MaterialCoordinates::current_configuration(List const& local_node_list) const
{
    return this->get_configuration(local_node_list, x);
}
}
