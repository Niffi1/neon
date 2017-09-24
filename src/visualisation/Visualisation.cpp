
#include "Visualisation.hpp"

#include "mesh/solid/femMesh.hpp"

#include <json/value.h>

#include <exception>

#include "vtkCellData.h"
#include "vtkCellTypes.h"
#include "vtkDataObject.h"
#include "vtkDoubleArray.h"
#include "vtkIdList.h"
#include "vtkIdTypeArray.h"
#include "vtkInformation.h"
#include "vtkInformationQuadratureSchemeDefinitionVectorKey.h"
#include "vtkPointData.h"
#include "vtkQuadraturePointsGenerator.h"
#include "vtkQuadratureSchemeDefinition.h"
#include "vtkUnstructuredGrid.h"
#include "vtkUnstructuredGridReader.h"
#include "vtkXMLUnstructuredGridReader.h"
#include "vtkXMLUnstructuredGridWriter.h"

namespace neon
{
Visualisation::Visualisation(std::string file_name,
                             solid::femMesh const& fem_mesh,
                             Json::Value const& visualisation_data)
    : file_name(file_name),
      fem_mesh(fem_mesh),
      unstructured_mesh(vtkSmartPointer<vtkUnstructuredGrid>::New()) //,
//   unstructured_mesh_writer(vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New())
{
    pvd_file.open(file_name + ".pvd");

    if (!pvd_file.is_open())
    {
        throw std::runtime_error("Not able to write to disk for visualisation\n");
    }

    pvd_file << "<?xml version=\"1.0\"?>\n";
    pvd_file << "<VTKFile type=\"Collection\" version=\"0.1\">\n";
    pvd_file << std::string(2, ' ') << "<Collection>\n";

    unstructured_mesh->Allocate();

    allocate_field_maps();

    if (visualisation_data.isMember("Fields"))
    {
        for (auto const& field : visualisation_data["Fields"])
        {
            auto const[field_name, inserted] = requested_fields.insert(field.asString());

            if (!inserted)
            {
                throw std::runtime_error(*field_name
                                         + " is duplicated in the visualisation "
                                           "field\n");
            }

            // Check if this result exists in the mappings
            if (string_to_tensor.find(*field_name) == string_to_tensor.end()
                && string_to_scalar.find(*field_name) == string_to_scalar.end()
                && *field_name != "Displacement")
            {
                throw std::runtime_error("Field name " + *field_name + " is not a valid variable\n");
            }
        }
    }

    if (visualisation_data.isMember("WriteEvery"))
    {
        write_every = visualisation_data["WriteEvery"].asInt();
    }

    allocate_static_mesh();

    // Write out initial mesh to file
    write(0, 0.0);
}

Visualisation::~Visualisation()
{
    // Close off the last of the file for the timestepping
    pvd_file << std::string(2, ' ') << "</Collection>\n";
    pvd_file << "</VTKFile>\n";
    pvd_file.close();
}

void Visualisation::write(int const time_step, double const total_time)
{
    if (time_step % write_every != 0) return;

    std::cout << "\n"
              << std::string(4, ' ') << "Writing solution to file for step " << time_step << "\n";

    auto const vtk_filename = file_name + "_" + std::to_string(time_step) + ".vtu";

    // Only write out the differences in the simulation, the nodal connectivity
    // remains static (for now) and only the displacement and internal variable
    // fields will change
    for (auto const& field : requested_fields)
    {
        if (auto found = string_to_tensor.find(field); found != string_to_tensor.end())
        {
            write_tensor_field(found->first, found->second);
        }
        else if (auto found = string_to_scalar.find(field); found != string_to_scalar.end())
        {
            write_scalar_field(found->first, found->second);
        }
        else if (field == "Displacement")
        {
            unstructured_mesh->GetPointData()->AddArray(fem_mesh.coordinates().vtk_displacement());
        }
        else
        {
            throw std::runtime_error("Field \"" + field
                                     + "\" was not found in mesh internal variables\n");
        }
    }
    auto unstructured_mesh_writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();

    unstructured_mesh_writer->SetFileName(vtk_filename.c_str());
    unstructured_mesh_writer->SetInputData(unstructured_mesh);

    if (!use_binary_format) unstructured_mesh_writer->SetDataModeToAscii();

    unstructured_mesh_writer->Write();

    pvd_file << std::string(4, ' ') << "<DataSet timestep = \"" << std::to_string(total_time)
             << "\" file = \"" << file_name << "_" << std::to_string(time_step) << ".vtu\" />"
             << std::endl;
}

void Visualisation::allocate_field_maps()
{
    string_to_tensor = {{"CauchyStress", InternalVariables::Tensor::Cauchy},
                        {"LinearisedStrain", InternalVariables::Tensor::LinearisedStrain},
                        {"LinearisedPlasticStrain",
                         InternalVariables::Tensor::LinearisedPlasticStrain},
                        {"DeformationGradient", InternalVariables::Tensor::DeformationGradient},
                        {"DisplacementGradient", InternalVariables::Tensor::DisplacementGradient}};

    string_to_scalar = {{"AccumulatedPlasticStrain",
                         InternalVariables::Scalar::EffectivePlasticStrain},
                        {"VonMisesStress", InternalVariables::Scalar::VonMisesStress}};
}

void Visualisation::allocate_static_mesh()
{
    // Create an unstructured grid object
    unstructured_mesh->SetPoints(fem_mesh.coordinates().vtk_coordinates());

    for (auto const& submesh : fem_mesh.meshes())
    {
        auto const vtk_ordered_connectivity = adapter.convert_to_vtk(submesh.connectivities(),
                                                                     submesh.topology());
        for (auto const& node_list : vtk_ordered_connectivity)
        {
            auto vtk_node_list = vtkSmartPointer<vtkIdList>::New();

            for (auto const& node : node_list)
            {
                vtk_node_list->InsertNextId(static_cast<long>(node));
            }
            unstructured_mesh->InsertNextCell(adapter.to_vtk(submesh.topology()), vtk_node_list);
        }
    }
    unstructured_mesh->GetPointData()->AddArray(fem_mesh.coordinates().vtk_displacement());
}

void Visualisation::write_tensor_field(std::string const& pretty_name,
                                       InternalVariables::Tensor const& tensor_enum)
{
    Vector nodal_averaged_value = Vector::Zero(fem_mesh.coordinates().size() * 9);
    Vector running_count = Vector::Zero(fem_mesh.coordinates().size() * 9);

    // Add internal variables
    for (auto const& submesh : fem_mesh.meshes())
    {
        if (!submesh.internal_variables().has(tensor_enum))
        {
            throw std::runtime_error("Internal variable " + pretty_name + " does not exist");
        }

        auto const[value, count] = submesh.nodal_averaged_variable(tensor_enum);
        nodal_averaged_value += value;
        running_count += count;
    }

    // Average nodal values
    nodal_averaged_value = nodal_averaged_value.cwiseQuotient(running_count);

    // Put this into a vtkDoubleArray
    auto tensor_value = vtkSmartPointer<vtkDoubleArray>::New();

    tensor_value->SetName(pretty_name.c_str());

    tensor_value->SetNumberOfComponents(9);
    tensor_value->SetNumberOfTuples(fem_mesh.coordinates().size());

    for (auto i = 0; i < fem_mesh.coordinates().size() * 9; i += 9)
    {
        tensor_value->InsertTuple9(i / 9,
                                   nodal_averaged_value(i + 0),
                                   nodal_averaged_value(i + 1),
                                   nodal_averaged_value(i + 2),
                                   nodal_averaged_value(i + 3),
                                   nodal_averaged_value(i + 4),
                                   nodal_averaged_value(i + 5),
                                   nodal_averaged_value(i + 6),
                                   nodal_averaged_value(i + 7),
                                   nodal_averaged_value(i + 8));
    }
    unstructured_mesh->GetPointData()->AddArray(tensor_value);
}

void Visualisation::write_scalar_field(std::string const& pretty_name,
                                       InternalVariables::Scalar const& scalar_enum)
{
    Vector nodal_averaged_value = Vector::Zero(fem_mesh.coordinates().size());
    Vector running_count = Vector::Zero(fem_mesh.coordinates().size());

    // Add internal variables
    for (auto const& submesh : fem_mesh.meshes())
    {
        auto const[value, count] = submesh.nodal_averaged_variable(scalar_enum);
        nodal_averaged_value += value;
        running_count += count;
    }

    // Average nodal values
    nodal_averaged_value = nodal_averaged_value.cwiseQuotient(running_count);

    // Put this into a vtkDoubleArray
    auto scalar_value = vtkSmartPointer<vtkDoubleArray>::New();

    scalar_value->SetName(pretty_name.c_str());

    scalar_value->SetNumberOfComponents(1);
    scalar_value->SetNumberOfTuples(fem_mesh.coordinates().size());

    for (auto i = 0; i < fem_mesh.coordinates().size(); ++i)
    {
        scalar_value->InsertTuple1(i, nodal_averaged_value(i));
    }
    unstructured_mesh->GetPointData()->AddArray(scalar_value);
}

void Visualisation::write_primary_field() {}
}
