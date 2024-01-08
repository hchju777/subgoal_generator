#pragma once

#include <iostream>
#include <fstream>
#include <memory>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "subgoal_generator/bvc_generator.h"

namespace SubgoalGenerator::BufferedVoronoiDiagram
{
    class Manager
    {
    public:
        typedef std::unique_ptr<Manager> UniquePtr;
        typedef std::shared_ptr<Manager> SharedPtr;

    public:
        static void printPolygon(const CGAL::Polygon_2<Kernel> &_polygon);

    public:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static void clearResultDir(std::string _dirName = "result");

        static bool exportVoronoiDiagram(
            const VoronoiDiagram &_voronoi_diagram,
            std::string _dirName = "result",
            std::string _fileName = "voronoi");

        static bool exportBufferedVoronoiDiagram(
            const VoronoiDiagram &_voronoi_diagram,
            const VoronoiDiagram &_buffered_voronoi_diagram,
            std::string _dirName = "result",
            std::string _fileName = "buffered_voronoi");

        static void exportVoronoiCellData(
            const VoronoiCell &_voronoi_cell, YAML::Node &_node);

        static void exportPolygonData(
            const CGAL::Polygon_2<Kernel> &_polygon, std::string _label, YAML::Node &_node);

    protected:
        static bool isDirExists(std::string _dirName);

    }; // class Manager
} // namespace namespace SubgoalGenerator::BufferedVoronoiDiagram