
#include "mesh/NodeOrderingAdapter.hpp"

#include "Exceptions.hpp"

namespace neon
{
void NodeOrderingAdapter::convert_from_gmsh(std::vector<List>& nodal_connectivity,
                                            ElementTopology element_topology)
{
    // Reorder based on the differences between the local node numbering
    // provided from Section 9.3 Node ordering
    // http://gmsh.info/doc/texinfo/gmsh.html#Node-ordering
    switch (element_topology)
    {
        case ElementTopology::Tetrahedron10:
        {
            for (auto& nodal_list : nodal_connectivity)
            {
                std::swap(nodal_list.at(0), nodal_list.at(3));
                std::swap(nodal_list.at(4), nodal_list.at(9));
            }
            break;
        }
        case ElementTopology::Hexahedron20:
        {
            /*
              Gmsh ordering (0 based indexing) taken from gmsh.info
               3----13----2
               |\         |\
               | 15       | 14
               9  \       11 \
               |   7----19+---6
               |   |      |   |
               0---+-8----1   |
                \  17      \  18
                10 |        12|
                  \|         \|
                   4----16----5

              Hughes ordering (0 based indexing)

               3----10----2
               |\         |\
               | 19       | 18
              11  \       9  \
               |   7----14+---6
               |   |      |   |
               0---+-8----1   |
                \  15      \  13
                16 |        17|
                  \|         \|
                   4----12----5
            */
            for (auto& nodal_list : nodal_connectivity)
            {
                std::swap(nodal_list.at(11), nodal_list.at(9));
                std::swap(nodal_list.at(13), nodal_list.at(10));

                std::swap(nodal_list.at(12), nodal_list.at(17));
                std::swap(nodal_list.at(16), nodal_list.at(12));
                std::swap(nodal_list.at(16), nodal_list.at(13));

                std::swap(nodal_list.at(13), nodal_list.at(15));
                std::swap(nodal_list.at(13), nodal_list.at(19));

                std::swap(nodal_list.at(13), nodal_list.at(18));
                std::swap(nodal_list.at(14), nodal_list.at(18));
            }
            break;
        }
        default:
            break;
    }
}

std::vector<List> NodeOrderingAdapter::convert_to_vtk(std::vector<List> nodal_connectivity,
                                                      ElementTopology element_topology) const
{
    switch (element_topology)
    {
        case ElementTopology::Tetrahedron10:
        {
            for (auto& nodal_list : nodal_connectivity)
            {
                std::swap(nodal_list.at(6), nodal_list.at(8));
                std::swap(nodal_list.at(8), nodal_list.at(9));
            }
            break;
        }
        case ElementTopology::Hexahedron20:
        {
            // The ordering of the twenty points defining the cell is point ids (0-7,8-19) where
            // point ids 0-7 are the eight corner vertices of the cube; followed by twelve midedge
            // nodes (8-19). Note that these midedge nodes correspond lie on the edges defined by
            // 8 > (0,1), 9 > (1,2), 10 > (2,3), 11 > (3,0),
            // 12 > (4,5), 13 > (5,6), 14 > (6,7), 15 > (7,4),
            // 16 > (0,4), 17 > (1,5), 18 > (2,6), 19 > (3,7).

            // NOTE This corresponds nicely to the Hughes ordering
            break;
        }
        case ElementTopology::Hexahedron27:
        {
            /* top
             *  7--14--6
             *  |      |
             * 15  25  13
             *  |      |
             *  4--12--5
             *
             *  middle
             * 19--23--18
             *  |      |
             * 20  26  21
             *  |      |
             * 16--22--17
             *
             * bottom
             *  3--10--2
             *  |      |
             * 11  24  9
             *  |      |
             *  0-- 8--1
             */
        }
        default:
            break;
    }
    return nodal_connectivity;
}

ElementTopology NodeOrderingAdapter::gmsh_type_to_enum(int element_code) const
{
    auto const found = gmsh_converter.find(element_code);
    if (found == gmsh_converter.end())
    {
        throw std::runtime_error("Element code " + std::to_string(element_code)
                                 + " not implemented for gmsh element type");
    }
    return found->second;
}

int NodeOrderingAdapter::to_vtk(ElementTopology element_topology) const
{
    auto const found = vtk_converter.find(element_topology);
    if (found == vtk_converter.end())
    {
        throw std::runtime_error("Element code " + std::to_string(static_cast<int>(element_topology))
                                 + " not implemented for vtk element type");
    }
    return found->second;
}
}
