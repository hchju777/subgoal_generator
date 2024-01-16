#pragma once

#include <string>
#include <map>
#include <fstream>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "subgoal_generator/pibt.h"
#include "subgoal_generator/bvc_manager.h"
#include "subgoal_generator/velocity_obstacle_manager.h"

namespace SubgoalGenerator::PIBT
{
    class Manager
    {
    public:
        struct ExportDataContainer
        {
        public:
            Agent agent_;

            BufferedVoronoiDiagram::VoronoiCell voronoi_cell_;
            BufferedVoronoiDiagram::VoronoiCell buffered_voronoi_cell_;
        }; // struct ExportDataContainer

        typedef std::map<std::string, ExportDataContainer> ExportDB;

    public:
        static void printAgents(const Agents &_agents);

        static void printGraph(const DynamicGraph::Graph::SharedPtr &_graph);

        static void printPriority(const std::stack<std::string> &_priority_list);

    public:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static void clearResultDir(std::string _dirName = "result");

        static bool exportDBList(
            const std::list<Manager::ExportDB>  &_DBList,
            std::string _dirName = "result", std::string _fileName = "pibt_environment_result");

        static void exportDBData(const ExportDB &_exportDB, YAML::Node &_node);

        static void exportDataContainerData(const ExportDataContainer &_exportData, YAML::Node &_node);

    protected:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static bool isDirExists(std::string _dirName);
    }; // class Manager
} // namespace SubgoalGenerator