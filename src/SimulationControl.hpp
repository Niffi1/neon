
#pragma once

#include "mesh/BasicMesh.hpp"

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include "io/json_forward.hpp"

namespace neon
{
class AbstractModule;

class SimulationControl
{
public:
    explicit SimulationControl(std::string const& input_file_name);

    ~SimulationControl();

    void start();

    static int threads; //!< Number of hardware threads to use
protected:
    void parse();

    void build_simulation_tree();

    /** Find all of the children that depend on a given simulation case */
    void find_children(std::string const& parent_name, std::string const& next_parent_name);

    /** Print the welcome banner to the terminal */
    void print_banner() const;

    void check_input_fields() const;

    /** Extract the material names from the input file */
    std::unordered_set<std::string> parse_material_names(json const& materials) const;

    /** Extract the part names from the input file */
    std::unordered_set<std::string> parse_part_names(
        json const& parts, std::unordered_set<std::string> const& material_names) const;

protected:
    std::string input_file_name;

    // Store the name, mesh connectivity and material
    std::map<std::string, std::pair<BasicMesh, json>> mesh_store;

    std::map<std::string, std::list<json>> multistep_simulations;

    std::vector<std::unique_ptr<AbstractModule>> modules;

    json root; // The file input
};
}
