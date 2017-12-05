
#include "SimulationControl.hpp"

#include "Exceptions.hpp"
#include "mesh/mechanical/solid/femMesh.hpp"

#include "modules/ModuleFactory.hpp"

#include "modules/AbstractModule.hpp"

#include <iomanip>
#include <thread>

#include <boost/filesystem.hpp>
#include <json/reader.h>
#include <termcolor/termcolor.hpp>

#include <range/v3/algorithm/find.hpp>
#include <range/v3/algorithm/find_if.hpp>

namespace neon
{
int SimulationControl::threads = std::thread::hardware_concurrency();

SimulationControl::SimulationControl(std::string const& input_file_name)
    : input_file_name(input_file_name)
{
    if (input_file_name == "" || input_file_name.empty()) throw NoInputException();

    boost::filesystem::path input_path(input_file_name);

    // Strip the extension from the filename
    std::string extension = boost::filesystem::extension(input_path);
    std::string base_name = boost::filesystem::basename(input_path);

    // Attempt to open the json input file
    if (extension != ".json") throw InvalidExtensionException(extension);

    this->parse();
}

SimulationControl::~SimulationControl() = default;

void SimulationControl::parse()
{
    auto start = std::chrono::high_resolution_clock::now();

    this->print_banner();

    std::cout << std::string(2, ' ') << termcolor::bold << "Preprocessing mesh and simulation data\n"
              << termcolor::reset;

    std::ifstream file(input_file_name);

    Json::CharReaderBuilder reader;

    if (JSONCPP_STRING input_errors; !Json::parseFromStream(reader, file, &root, &input_errors))
    {
        throw std::runtime_error(input_errors);
    }

    this->check_input_fields(root);

    if (root.isMember("Cores")) threads = root["Cores"].asInt();

    auto const material_names = this->parse_material_names(root["Material"]);
    auto const part_names = this->parse_part_names(root["Part"], material_names);

    // Add in the parts and populate the mesh stores
    for (auto const& part : root["Part"])
    {
        auto const material = *ranges::find_if(root["Material"], [&part](auto const& material) {
            return material["Name"].asString() == part["Material"].asString();
        });

        std::ifstream mesh_input_stream(part["Name"].asString() + ".mesh");

        Json::Value mesh_file;

        if (JSONCPP_STRING mesh_errors;
            !Json::parseFromStream(reader, mesh_input_stream, &mesh_file, &mesh_errors))
        {
            throw std::runtime_error(mesh_errors);
        }

        mesh_store.try_emplace(part["Name"].asString(), mesh_file, material);

        std::cout << std::string(4, ' ') << "Inserted " << part["Name"].asString()
                  << " into the mesh store\n";
    }

    std::vector<std::string> const required_fields{"Name",
                                                   "Time",
                                                   "Solution",
                                                   "Visualisation",
                                                   "LinearSolver"};

    // Build a list of all the load steps for a given mesh
    for (auto const& simulation : root["SimulationCases"])
    {
        // Ensure the required fields exist
        for (auto const& required_field : required_fields)
        {
            if (!simulation.isMember(required_field))
            {
                throw std::runtime_error("A simulation case needs a \"" + required_field
                                         + "\" field\n");
            }
        }
        // Multibody simulations not (yet) supported
        assert(simulation["Mesh"].size() == 1);

        // Make sure the simulation mesh exists in the mesh store
        if (mesh_store.find(simulation["Mesh"][0]["Name"].asString()) == mesh_store.end())
        {
            throw std::runtime_error("Mesh name \"" + simulation["Mesh"][0]["Name"].asString()
                                     + "\" was not found in the mesh store");
        }
    }
    build_simulation_tree();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    std::cout << termcolor::bold << termcolor::green << std::string(2, ' ')
              << "Preprocessing complete in " << elapsed_seconds.count() << "s\n"
              << termcolor::reset << std::endl;
}

void SimulationControl::start()
{
    // Allocate the modules storage, which automatically checks for correct input
    // and throws the appropriate exception when an error is detected
    for (auto const& [name, simulations] : multistep_simulations)
    {
        for (auto const& simulation : simulations)
        {
            modules.emplace_back(make_module(simulation, mesh_store));
        }
    }
    for (auto const& module : modules) module->perform_simulation();
}

void SimulationControl::build_simulation_tree()
{
    // For each simulation step, the total number of subsequent relationships
    // need to be determined, such that an analysis can be performed in order
    // Build a list of all the load steps for a given mesh
    for (auto const& simulation : root["SimulationCases"])
    {
        if (!simulation.isMember("Inherits"))
        {
            multistep_simulations[simulation["Name"].asString()].emplace_front(simulation);
            find_children(simulation["Name"].asString(), simulation["Name"].asString());
        }
    }

    for (auto const& [name, queue] : multistep_simulations)
    {
        std::cout << std::string(4, ' ') << "Simulation \"" << name << "\" is continued by:\n";
        for (auto const& item : queue)
        {
            std::cout << std::string(4, ' ') << "\"" << item["Name"].asString() << "\"" << std::endl;
        }
    }
}

void SimulationControl::find_children(std::string const& parent_name,
                                      std::string const& next_parent_name)
{
    for (auto const& simulation : root["SimulationCases"])
    {
        if (!simulation.isMember("Inherits")) continue;

        if (simulation["Inherits"].asString() == next_parent_name)
        {
            multistep_simulations[parent_name].push_back(simulation);
            // Recursive step
            find_children(parent_name, simulation["Name"].asString());
        }
    }
}

void SimulationControl::print_banner() const
{
    std::string const welcome_message("neon - a non-linear finite element code");

    std::cout << termcolor::bold;
    std::cout << std::setw(welcome_message.length() + 9) << std::setfill('=') << "\n";
    std::cout << std::string(4, ' ') << welcome_message << "\n";
    std::cout << std::setw(welcome_message.length() + 9) << std::setfill('=') << "\n";
    std::cout << termcolor::reset << std::endl << std::setfill(' ');
}

void SimulationControl::check_input_fields(Json::Value const& root) const
{
    // Check the important fields exist before anything else is done
    if (!root.isMember("Part")) throw std::runtime_error("Part is not in input file");
    if (!root.isMember("Name")) throw std::runtime_error("Name is not in input file");
    if (!root.isMember("Material")) throw std::runtime_error("Material is not in input file");
    if (!root.isMember("SimulationCases"))
        throw std::runtime_error("SimulationCases is not in input file");
}

std::unordered_set<std::string> SimulationControl::parse_material_names(Json::Value const& materials) const
{
    std::unordered_set<std::string> material_names;

    for (auto const& material : root["Material"])
    {
        if (material["Name"].empty()) throw std::runtime_error("Material: Name");

        auto const [it, inserted] = material_names.emplace(material["Name"].asString());

        if (!inserted) throw DuplicateNameException("Material");
    }
    return material_names;
}

std::unordered_set<std::string> SimulationControl::parse_part_names(
    Json::Value const& parts, std::unordered_set<std::string> const& material_names) const
{
    std::unordered_set<std::string> part_names;

    // Load in all the part names and error check
    for (auto const& part : root["Part"])
    {
        if (part["Name"].empty()) throw std::runtime_error("Part: Name");

        if (ranges::find(material_names, part["Material"].asString()) == material_names.end())
        {
            throw std::runtime_error("The part material was not found in the provided "
                                     "materials\n");
        }

        auto const [it, inserted] = part_names.emplace(part["Name"].asString());

        if (!inserted) throw DuplicateNameException("Part");
    }
    return part_names;
}
}
