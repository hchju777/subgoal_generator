#include "subgoal_generator/bvc_manager.h"

namespace SubgoalGenerator::BufferedVoronoiDiagram
{
    void Manager::printPolygon(const CGAL::Polygon_2<Kernel> &_polygon)
    {
        for (auto viter = _polygon.vertices_begin(); viter != _polygon.vertices_end(); ++viter)
        {
            std::cout << "\t" << viter->x() << " " << viter->y() << std::endl;
        }
    }

    void Manager::clearResultDir(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;

        if (isDirExists(_dirName))
            std::filesystem::remove_all(directory_path_string);

        std::filesystem::create_directories(directory_path_string);
    }

    bool Manager::exportVoronoiDiagram(
        const VoronoiDiagram &_voronoi_diagram,
        std::string _dirName, std::string _fileName)
    {
        if (not(isDirExists(_dirName)))
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

            exportPolygonData(voronoi_cell.second, "polygon", cell);

            diagram.push_back(cell);
        }

        std::string file_path_string = "../" + _dirName + "/" + _fileName + ".yaml";
        std::ofstream result(file_path_string);

        result << diagram;

        return true;
    }

    bool Manager::exportBufferedVoronoiDiagram(
        const VoronoiDiagram &_voronoi_diagram,
        const VoronoiDiagram &_buffered_voronoi_diagram,
        std::string _dirName, std::string _fileName)
    {
        if (not(isDirExists(_dirName)))
        {
            std::cerr << "BufferedVoronoiDiagram::Manager::exportBufferedVoronoiDiagram: "
                      << "There is no such directory named " << _dirName << std::endl;
            return false;
        }

        YAML::Node diagram;

        assert(_voronoi_diagram.size() == _buffered_voronoi_diagram.size());

        for (size_t idx = 0; idx < _voronoi_diagram.size(); ++idx)
        {
            VoronoiCell voronoi_cell = _voronoi_diagram[idx];
            VoronoiCell buffered_voronoi_cell = _buffered_voronoi_diagram[idx];

            YAML::Node cell;

            exportVoronoiCellData(voronoi_cell, cell);

            // std::vector<double> point = {voronoi_cell.first.x(), voronoi_cell.first.y()};
            // cell["point"] = point;
            // cell["point"].SetStyle(YAML::EmitterStyle::Flow);

            // exportPolygonData(voronoi_cell.second, "polygon", cell);
            exportPolygonData(buffered_voronoi_cell.second, "offset_polygon", cell);

            diagram.push_back(cell);
        }

        std::string file_path_string = "../" + _dirName + "/" + _fileName + ".yaml";
        std::ofstream result(file_path_string);

        result << diagram;

        return true;
    }

    void Manager::exportVoronoiCellData(
        const VoronoiCell &_voronoi_cell, YAML::Node &_node)
    {
        std::vector<double> point = {_voronoi_cell.first.x(), _voronoi_cell.first.y()};
        _node["point"] = point;
        _node["point"].SetStyle(YAML::EmitterStyle::Flow);

        exportPolygonData(_voronoi_cell.second, "polygon", _node);
    }

    bool Manager::isDirExists(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;
        std::filesystem::path directory_path(directory_path_string);

        return std::filesystem::exists(directory_path);
    }

    void Manager::exportPolygonData(const CGAL::Polygon_2<Kernel> &_polygon, std::string _label, YAML::Node &_node)
    {
        for (auto viter = _polygon.vertices_begin(); viter != _polygon.vertices_end(); ++viter)
        {
            YAML::Node edge;

            std::vector<double> from = {viter->x(), viter->y()};
            edge["from"] = from;
            edge["from"].SetStyle(YAML::EmitterStyle::Flow);

            std::vector<double> to;
            if (viter + 1 != _polygon.vertices_end())
                to = {(viter + 1)->x(), (viter + 1)->y()};
            else
                to = {_polygon.vertices_begin()->x(), _polygon.vertices_begin()->y()};
            edge["to"] = to;
            edge["to"].SetStyle(YAML::EmitterStyle::Flow);

            _node[_label].push_back(edge);
        }
    }
} // namespace SubgoalGenerator::BufferedVoronoiDiagram