#pragma once

#include <iostream>
#include <fstream>
#include <filesystem>

#include "subgoal_generator/velocity_obstacle.h"

namespace SubgoalGenerator::VelocityObstacle
{
    class Manager
    {
    public:
        static void printAgents(const std::map<std::string, Agent *> &_agents);

    public:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static void clearResultDir(std::string _dirName = "result");

        static bool exportDynamicGraph(
            const std::map<std::string, Agent *> &_agents,
            std::string _dirName = "result", std::string _fileName = "velocity_obstacle");

        static void exportAgentInfoData(const Agent &_agent, YAML::Node &_node);

        static void exportAgentVOData(const Agent &_agent, YAML::Node &_node);

    protected:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static bool isDirExists(std::string _dirName);
    }; // class Manager
} // namespace SubgoalGenerator::VelocityObstacle