#include "subgoal_generator/dynamic_graph_manager.h"

namespace SubgoalGenerator::DynamicGraph
{
    void Manager::printGraph(const Graph::SharedPtr &_dynamic_graph)
    {
        Manager::printVertices(_dynamic_graph->vertices());
        std::cout << std::endl;

        Manager::printAdjList(_dynamic_graph->adj_list());
        std::cout << std::endl;

        Manager::printAdjPriorityList(_dynamic_graph->adj_priority_list());
        std::cout << std::endl;
    }

    void Manager::printVertices(const std::map<std::string, Vertex> &_vertices)
    {
        if (_vertices.empty())
            return;

        std::cout << "<vertices>" << std::endl;
        for (const auto &vertexPair : _vertices)
            std::cout << "\t o " << vertexPair.second << std::endl;
    }

    void Manager::printAdjList(const std::map<std::string, std::list<Vertex>> &_adj_list)
    {
        if (_adj_list.empty())
            return;

        std::cout << "<adjacent list>" << std::endl;
        for (const auto &adjPair : _adj_list)
        {
            if (adjPair.second.empty())
                continue;

            std::cout << "\t o [" << adjPair.first << "]" << std::endl;
            for (const auto &vertex : adjPair.second)
                std::cout << "\t\t - " << vertex << std::endl;
        }
    }

    void Manager::printAdjPriorityList(const std::map<std::string, std::list<std::string>> &_adj_priority_list)
    {
        if (_adj_priority_list.empty())
            return;

        std::cout << "<adjacent priority list>" << std::endl;
        for (const auto &adjPair : _adj_priority_list)
        {
            if (adjPair.second.empty())
                continue;

            std::cout << "\t o [" << adjPair.first << "]" << std::endl;
            for (const auto &vertexName : adjPair.second)
                std::cout << "\t\t - " << vertexName << std::endl;
        }
    }

    void Manager::printPriority(const std::stack<std::string> &_priority_list)
    {
        std::stack<std::string> stack = _priority_list;

        while (not(stack.empty()))
        {
            std::cout << stack.top() << std::endl;
            stack.pop();
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
        const Graph::SharedPtr &_graph,
        std::string _dirName, std::string _fileName)
    {
        if (not(isDirExists(_dirName)))
        {
            std::cerr << "DynamicGraph::Manager::exportDynamicGraph: "
                      << "There is no such directory named " << _dirName << std::endl;
            return false;
        }

        YAML::Node node;
        exportVerticesData(_graph->vertices(), node);
        exportAdjListData(_graph->adj_list(), node);
        exportAdjPriorityListData(_graph->adj_priority_list(), node);
        exportPriorityData(_graph->topologicalSort(), node);

        std::string file_path_string = "../" + _dirName + "/" + _fileName + ".yaml";
        std::ofstream result(file_path_string);

        result << node;

        return true;
    }

    void Manager::exportVerticesData(const std::map<std::string, Vertex> &_vertices, YAML::Node &_node)
    {
        for (const auto &vertexPair : _vertices)
        {
            YAML::Node vertexNode;

            const Vertex &vertex = vertexPair.second;

            std::vector<double> current = {vertex.current().position().x(), vertex.current().position().y(), vertex.current().theta()};
            std::vector<double> goal = {vertex.goal().position().x(), vertex.goal().position().y(), vertex.goal().theta()};

            vertexNode["name"] = vertex.name();
            vertexNode["current"] = current;
            vertexNode["current"].SetStyle(YAML::EmitterStyle::Flow);
            vertexNode["goal"] = goal;
            vertexNode["goal"].SetStyle(YAML::EmitterStyle::Flow);

            _node["vertices"].push_back(vertexNode);
        }
    }

    void Manager::exportAdjListData(const std::map<std::string, std::list<Vertex>> &_adj_list, YAML::Node &_node)
    {
        for (const auto &adjPair : _adj_list)
        {
            YAML::Node adjListNode;
            adjListNode["name"] = adjPair.first;

            for (const auto &vertex : adjPair.second)
            {
                YAML::Node vertexNode;

                std::vector<double> current = {vertex.current().position().x(), vertex.current().position().y(), vertex.current().theta()};
                std::vector<double> goal = {vertex.goal().position().x(), vertex.goal().position().y(), vertex.goal().theta()};

                vertexNode["name"] = vertex.name();
                vertexNode["current"] = current;
                vertexNode["current"].SetStyle(YAML::EmitterStyle::Flow);
                vertexNode["goal"] = goal;
                vertexNode["goal"].SetStyle(YAML::EmitterStyle::Flow);

                adjListNode["neighbor"].push_back(vertexNode);
            }

            _node["adjacent_list"].push_back(adjListNode);
        }
    }

    void Manager::exportAdjPriorityListData(const std::map<std::string, std::list<std::string>> &_adj_priority_list, YAML::Node &_node)
    {
        for (const auto &adjPair : _adj_priority_list)
        {
            YAML::Node adjPriorityListNode;
            adjPriorityListNode["higher"] = adjPair.first;

            for (const auto &low : adjPair.second)
                adjPriorityListNode["lower"].push_back(low);

            _node["adjacent_priority_list"].push_back(adjPriorityListNode);
        }
    }

    void Manager::exportPriorityData(const std::stack<std::string> &_priority_list, YAML::Node &_node)
    {
        std::stack<std::string> stack = _priority_list;

        while (not(stack.empty()))
        {
            _node["priority"].push_back(stack.top());
            stack.pop();
        }
    }

    bool Manager::isDirExists(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;
        std::filesystem::path directory_path(directory_path_string);

        return std::filesystem::exists(directory_path);
    }

} // namespace SubgoalGenerator::DynamicGraph