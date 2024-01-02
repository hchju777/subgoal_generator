#include "subgoal_generator/bvc_manager.h"

namespace SubgoalGenerator::BufferedVoronoiDiagram
{
    Manager::Manager()
    {
        std::cout << "Buffered Voronoi Diagram Manager has been initilized." << std::endl;
    }

    Manager::~Manager()
    {
        std::cout << "Buffered Voronoi Diagram Manager has been terminated." << std::endl;
    }

    void Manager::clearResultDir(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;
        std::filesystem::path directory_path(directory_path_string);

        if (std::filesystem::exists(directory_path))
            std::filesystem::remove_all(directory_path_string);

        std::filesystem::create_directories(directory_path_string);
    }

    bool Manager::exportVoronoiDiagram(
        const VoronoiDiagram &_voronoi_diagram,
        std::string _dirName, std::string _fileName)
    {
        std::string directory_path_string = "../" + _dirName;
        std::filesystem::path directory_path(directory_path_string);

        if (not(std::filesystem::exists(directory_path)))
        {
            std::cerr << "BufferedVoronoiDiagram::Manager::exportVoronoiDiagram: "
                      << "There is no such directory named " << _dirName << std::endl;
            return false;
        }

        YAML::Node diagram;
        for (const auto &voronoi_cell : _voronoi_diagram)
        {
            YAML::Node cell;

            std::vector<double> point = {voronoi_cell.first.x(), voronoi_cell.first.y()};
            cell["point"] = point;
            cell["point"].SetStyle(YAML::EmitterStyle::Flow);

            for (auto viter = voronoi_cell.second.vertices_begin(); viter != voronoi_cell.second.vertices_end(); ++viter)
            {
                YAML::Node edge;

                std::vector<double> from = {viter->x(), viter->y()};
                edge["from"] = from;
                edge["from"].SetStyle(YAML::EmitterStyle::Flow);

                std::vector<double> to;
                if (viter + 1 != voronoi_cell.second.vertices_end())
                    to = {(viter + 1)->x(), (viter + 1)->y()};
                else
                    to = {voronoi_cell.second.vertices_begin()->x(), voronoi_cell.second.vertices_begin()->y()};
                edge["to"] = to;
                edge["to"].SetStyle(YAML::EmitterStyle::Flow);

                cell["polygon"].push_back(edge);
            }

            diagram.push_back(cell);
        }

        std::string file_path_string = directory_path_string + "/" + _fileName + ".yaml";
        std::ofstream result(file_path_string);

        result << diagram;

        return true;
    }

    void Manager::printPolygon(const CGAL::Polygon_2<Kernel> &_polygon)
    {
        for (auto viter = _polygon.vertices_begin(); viter != _polygon.vertices_end(); ++viter)
        {
            std::cout << "\t" << viter->x() << " " << viter->y() << std::endl;
        }
    }
} // namespace SubgoalGenerator::BufferedVoronoiDiagram