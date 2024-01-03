#pragma once

// standard includes
#include <list>
#include <map>
#include <stack>
#include <iostream>
#include <fstream>
#include <filesystem>

// yaml-cpp includes
// Need to install libyaml-cpp-dev
#include <yaml-cpp/yaml.h>

#include <subgoal_generator/vertex.h>
#include <subgoal_generator/dynamic_graph.h>

namespace SubgoalGenerator::DynamicGraph
{
    class Manager
    {
    public:
        static void printGraph(const Graph::SharedPtr &_dynamic_graph);

        static void printVertices(const std::map<std::string, Vertex> &_vertices);

        static void printAdjList(const std::map<std::string, std::list<Vertex>> &_adj_list);

        static void printAdjPriorityList(const std::map<std::string, std::list<std::string>> &_adj_priority_list);

        static void printPriority(const std::stack<std::string> &_priority_list);

    public:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static void clearResultDir(std::string _dirName = "result");

        static bool exportDynamicGraph(
            const Graph::SharedPtr &_graph,
            std::string _dirName = "result", std::string _fileName = "dynamic_graph");

        static void exportVerticesData(const std::map<std::string, Vertex> &_vertices, YAML::Node &_node);

        static void exportAdjListData(const std::map<std::string, std::list<Vertex>> &_adj_list, YAML::Node &_node);

        static void exportAdjPriorityListData(const std::map<std::string, std::list<std::string>> &_adj_priority_list, YAML::Node &_node);

        static void exportPriorityData(const std::stack<std::string> &_priority_list, YAML::Node &_node);

    protected:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static bool isDirExists(std::string _dirName);

    }; // class Manager
} // namespace SubgoalGenerator::DynamicGraph