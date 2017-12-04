
#include "MaterialCoordinates.hpp"

#include <range/v3/algorithm/for_each.hpp>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/zip.hpp>

#include "vtkDoubleArray.h"
#include "vtkPoints.h"

namespace neon
{
MaterialCoordinates::MaterialCoordinates(Vector const& initial_coordinates)
    : NodalCoordinates(initial_coordinates), x(initial_coordinates)
{
}

Vector MaterialCoordinates::displacement(List const& local_dofs) const
{
    using namespace ranges;

    Vector localdisp(local_dofs.size());

    for_each(view::zip(view::ints(0), local_dofs), [&](auto const& zip_pair) {
        auto const& [i, local_dof] = zip_pair;
        localdisp(i) = x(local_dof) - X(local_dof);
    });
    return localdisp;
}

/** @return a vtk object of the initial coordinates */
vtkSmartPointer<vtkPoints> MaterialCoordinates::vtk_coordinates() const
{
    auto points = vtkSmartPointer<vtkPoints>::New();

    points->Allocate(X.size() / 3);

    for (auto i = 0; i < X.size(); i += 3)
    {
        points->InsertNextPoint(X(i), X(i + 1), X(i + 2));
    }

    return points;
}

/** @return a vtk array of nodal displacements */
vtkSmartPointer<vtkDoubleArray> MaterialCoordinates::vtk_displacement() const
{
    auto displacements = vtkSmartPointer<vtkDoubleArray>::New();
    displacements->SetNumberOfComponents(3);
    displacements->SetName("Displacements");

    for (auto i = 0; i < X.size(); i += 3)
    {
        displacements->InsertNextTuple3(x(i) - X(i), x(i + 1) - X(i + 1), x(i + 2) - X(i + 2));
    }
    return displacements;
}

Matrix3x MaterialCoordinates::get_configuration(List const& local_nodes,
                                                Vector const& configuration) const
{
    Matrix3x element_displacement(3, local_nodes.size());

    for (auto lnode = 0; lnode < local_nodes.size(); lnode++)
    {
        element_displacement.col(lnode) = configuration.segment<3>(3 * local_nodes[lnode]);
    }
    return element_displacement;
}
}
