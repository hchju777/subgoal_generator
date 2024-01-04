#include "subgoal_generator/velocity_obstacle_manager.h"

namespace SubgoalGenerator::VelocityObstacle
{
    void Manager::printAgents(const std::map<std::string, Agent *> &_agents)
    {
        std::cout << "<agents>" << std::endl;
        for (const auto &agentPair : _agents)
        {
            auto &agent = agentPair.second;

            std::cout << "\to " << *agent << std::endl;

            for (const auto &neighborPair : agent->neighbors())
            {
                std::cout << "\t   - " << *neighborPair.second << std::endl;
            }
            std::cout << std::endl;
        }
    }

    void Manager::clearResultDir(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;

        if (isDirExists(_dirName))
            std::filesystem::remove_all(directory_path_string);

        std::filesystem::create_directories(directory_path_string);
    }

    bool Manager::exportDynamicGraph(
        const std::map<std::string, Agent *> &_agents,
        std::string _dirName, std::string _fileName)
    {
        if (not(isDirExists(_dirName)))
        {
            std::cerr << "DynamicGraph::Manager::exportDynamicGraph: "
                      << "There is no such directory named " << _dirName << std::endl;
            return false;
        }

        YAML::Node node;

        for (const auto &agentPair : _agents)
        {
            YAML::Node agent;

            exportAgentInfoData(*agentPair.second, agent);
            exportAgentVOData(*agentPair.second, agent);

            node.push_back(agent);
        }

        std::string file_path_string = "../" + _dirName + "/" + _fileName + ".yaml";
        std::ofstream result(file_path_string);

        result << node;

        return true;
    }

    void Manager::exportAgentInfoData(const Agent &_agent, YAML::Node &_node)
    {
        YAML::Node info;

        info["name"] = _agent.name();
        info["radius"] = _agent.radius();

        std::vector<double> pose = {_agent.pose().position().x(), _agent.pose().position().y(), _agent.pose().theta()};
        std::vector<double> goal = {_agent.goal().position().x(), _agent.goal().position().y(), _agent.goal().theta()};
        std::vector<double> velocity = {_agent.velocity().x(), _agent.velocity().y()};

        info["pose"] = pose;
        info["pose"].SetStyle(YAML::EmitterStyle::Flow);
        info["goal"] = goal;
        info["goal"].SetStyle(YAML::EmitterStyle::Flow);
        info["velocity"] = velocity;
        info["velocity"].SetStyle(YAML::EmitterStyle::Flow);

        _node["info"] = info;
    }

    void Manager::exportAgentVOData(const Agent &_agent, YAML::Node &_node)
    {
        for (const auto &VOCone : _agent.VOCones())
        {
            YAML::Node Cone;

            std::vector<double> point = {VOCone.point_.x(), VOCone.point_.y()};
            std::vector<double> left_direction = {VOCone.left_direction_.x(), VOCone.left_direction_.y()};
            std::vector<double> right_direction = {VOCone.right_direction_.x(), VOCone.right_direction_.y()};

            Cone["point"] = point;
            Cone["point"].SetStyle(YAML::EmitterStyle::Flow);
            Cone["radius"] = VOCone.radius_;
            Cone["left_direction"] = left_direction;
            Cone["left_direction"].SetStyle(YAML::EmitterStyle::Flow);
            Cone["right_direction"] = right_direction;
            Cone["right_direction"].SetStyle(YAML::EmitterStyle::Flow);

            _node["vo"].push_back(Cone);
        }
    }

    bool Manager::isDirExists(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;
        std::filesystem::path directory_path(directory_path_string);

        return std::filesystem::exists(directory_path);
    }
} // namespace SubgoalGenerator::VelocityObstacle