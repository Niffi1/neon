
#include "MaterialCoordinates.hpp"

#include <range/v3/algorithm/for_each.hpp>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/zip.hpp>

#include "vtkDoubleArray.h"
#include "vtkPoints.h"

namespace neon
{
template <int stride>
using vector_view = Eigen::Map<vector, 0, Eigen::InnerStride<stride>>;

MaterialCoordinates::MaterialCoordinates(matrix3x const& initial_coordinates)
    : NodalCoordinates(initial_coordinates), x(initial_coordinates)
{
}

matrix3x MaterialCoordinates::displacement() const { return x - X; }

void MaterialCoordinates::update_current_xy_configuration(vector const& u)
{
    x.row(0) = X.row(0) + u(Eigen::seq(0, u.size() - 1, 2)).transpose();
    x.row(1) = X.row(1) + u(Eigen::seq(1, u.size() - 1, 2)).transpose();
}

void MaterialCoordinates::update_current_configuration(vector const& u)
{
    x.row(0) = X.row(0) + u(Eigen::seq(0, u.size() - 1, 3)).transpose();
    x.row(1) = X.row(1) + u(Eigen::seq(1, u.size() - 1, 3)).transpose();
    x.row(2) = X.row(2) + u(Eigen::seq(2, u.size() - 1, 3)).transpose();
}

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

vtkSmartPointer<vtkDoubleArray> MaterialCoordinates::vtk_displacement() const
{
    static_assert(!X.IsRowMajor, "This assumes the storage is column major");

    auto displacements = vtkSmartPointer<vtkDoubleArray>::New();
    displacements->Allocate(X.cols());
    displacements->SetNumberOfComponents(X.rows());
    displacements->SetName("Displacements");

    for (auto i = 0; i < X.cols(); i++)
    {
        displacements->InsertNextTuple3(x(0, i) - X(0, i), x(1, i) - X(1, i), x(2, i) - X(2, i));
    }
    return displacements;
}
}
