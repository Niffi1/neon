
#include "Exceptions.hpp"

namespace neon
{
const char* MaterialPropertyException::what() const noexcept
{
    std::cout << "\n!! Error: " << property << " has not been provided in " << input_file;
    return nullptr;
}

const char* PartNameException::what() const noexcept
{
    std::cout << "\n!! Error: Part name \"" << partName << "\" not found in gmsh file.";
    std::cout << "Please check that part name in " << input_file
              << ".neon file matches highest dimension physical name in .msh file.\n";
    return nullptr;
}

const char* NoInputException::what() const noexcept
{
    std::cout << "\n!! Error: No input file found.  An input file needs to be provided: "
              << "\"<filename>.neon\"\n";
    return nullptr;
}

const char* InvalidExtensionException::what() const noexcept
{
    std::cout << "\n!! Error: Extension \"" << extension << "\" is not supported.\n"
              << "Supported extension is \".neon\"";
    return nullptr;
}

const char* DuplicateNameException::what() const noexcept
{
    std::cout << "\n!! Error: Duplicate " << duplParameter << " names not allowed!  Please check "
              << duplParameter << " names in " << input_file;
    return nullptr;
}
}
