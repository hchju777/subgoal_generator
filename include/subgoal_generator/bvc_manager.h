#pragma once

// standard includes
#include <iostream>
#include <fstream>
#include <memory>
#include <filesystem>

// yaml-cpp includes
// Need to install libyaml-cpp-dev
#include <yaml-cpp/yaml.h> 

// CGAL includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Polygon_2.h>

// typedefs for defining the adaptor
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT> AT;

// typedef for the result type of the point location
typedef AT::Point_2 Point_2;

namespace SubgoalGenerator::BufferedVoronoiDiagram
{
    class Manager
    {
    public:
        typedef std::unique_ptr<Manager> UniquePtr;
        typedef std::shared_ptr<Manager> SharedPtr;

    public:
        typedef std::pair<Point_2, CGAL::Polygon_2<Kernel>> VoronoiCell;
        typedef std::vector<VoronoiCell> VoronoiDiagram;

    public:
        Manager();

        ~Manager();

    public:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static void clearResultDir(std::string _dirName = "result");

        static bool exportVoronoiDiagram(
            const VoronoiDiagram &_voronoi_diagram,
            std::string _dirName = "result",
            std::string _fileName = "voronoi");

        static void printPolygon(const CGAL::Polygon_2<Kernel> &_polygon);

    }; // class Manager
} // namespace namespace SubgoalGenerator::BufferedVoronoiDiagram