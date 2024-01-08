#include "subgoal_generator/dynamic_graph.h"

namespace SubgoalGenerator::DynamicGraph
{
    Graph::Graph()
    {
        reset();

        std::cout << "Graph has been initilized." << std::endl;
    }

    Graph::~Graph()
    {
        std::cout << "Graph has been terminated." << std::endl;
    }

    void Graph::addVertex(const Vertex &_newVertex, double _range)
    {
        for (const auto &vertexPair : vertices_)
        {
            const Vertex &vertex = vertexPair.second;

            if (Vertex::getDistance(vertex, _newVertex) > _range)
                continue;

            if (vertex.isGoal() and _newVertex.isGoal())
                continue;

            auto hasHigherPriority = [&](const Vertex &_low, const Vertex _high) -> bool
            {
                if (_high.isGoal())
                    return false;

                if (_low.goalDistance() < _high.goalDistance())
                    return false;

                return true;
            };

            if (vertex.isGoal() or hasHigherPriority(vertex, _newVertex))
            {
                adj_priority_list_[_newVertex.name()].push_back(vertex.name());
            }
            else if (_newVertex.isGoal() or hasHigherPriority(_newVertex, vertex))
            {
                adj_priority_list_[vertex.name()].push_back(_newVertex.name());
            }
            else
            {
                continue;
            }

            adj_list_[vertex.name()].push_back(_newVertex);
            adj_list_[_newVertex.name()].push_back(vertex);
        }

        vertices_.emplace(std::make_pair(_newVertex.name(), _newVertex));
    }

    void Graph::deleteVertex(std::string _name)
    {
        for (const auto &vertex : adj_list_[_name])
        {
            std::list<Vertex> &adj_list = adj_list_[vertex.name()];
            std::list<std::string> &adj_priority_list_list = adj_priority_list_[vertex.name()];

            auto isDeleteVertex = [&](const Vertex &_vertex) -> bool
            {
                return _vertex.name() == _name;
            };

            auto isDeleteName = [&](std::string _vertexName) -> bool
            {
                return _vertexName == _name;
            };

            adj_list.remove_if(isDeleteVertex);
            adj_priority_list_list.remove_if(isDeleteName);

            if (adj_list.empty())
                adj_list_.erase(vertex.name());
        }

        vertices_.erase(_name);
        adj_list_.erase(_name);
        adj_priority_list_.erase(_name);
    }

    std::stack<std::string> Graph::topologicalSort()
    {
        std::stack<std::string> stack;

        std::map<std::string, bool> visited;
        for (const auto &vertexPair : vertices_)
        {
            const std::string &vertexName = vertexPair.first;
            visited.emplace(std::make_pair(vertexName, false));
        }

        for (const auto &vertexPair : vertices_)
        {
            const std::string &vertexName = vertexPair.first;

            if (visited[vertexName] == false)
                topologicalSortUtil(vertexName, visited, stack);
        }

        return stack;
    }

    std::list<Vertices> Graph::generateGroupList()
    {
        std::list<Vertices> groupList;

        std::map<std::string, bool> visited;
        for (const auto &vertexPair : vertices_)
        {
            const std::string &vertexName = vertexPair.first;
            visited.emplace(std::make_pair(vertexName, false));
        }

        std::stack<std::string> priority_graph = topologicalSort();

        while (not(priority_graph.empty()))
        {
            auto vertexName = priority_graph.top();
            priority_graph.pop();

            if (visited[vertexName] == true)
                continue;

            Vertices group;
            generateGroupListUtil(vertexName, visited, group);

            groupList.emplace_back(group);
        }

        return groupList;
    }

    void Graph::reset()
    {
        std::map<std::string, std::list<Vertex>> empty_graph;
        std::map<std::string, Vertex> empty_vertices;
        std::map<std::string, std::list<std::string>> empty_adj_priority;

        vertices_.swap(empty_vertices);
        adj_list_.swap(empty_graph);
        adj_priority_list_.swap(empty_adj_priority);

        assert(vertices_.empty());
        assert(adj_list_.empty());
        assert(adj_priority_list_.empty());
    }

    void Graph::topologicalSortUtil(
        std::string _name, std::map<std::string, bool> &_visited,
        std::stack<std::string> &_stack)
    {
        _visited[_name] = true;

        for (const auto &low : adj_priority_list_[_name])
        {
            if (_visited[low] == false)
                topologicalSortUtil(low, _visited, _stack);
        }

        _stack.push(_name);
    }

    void Graph::generateGroupListUtil(
        std::string _name, std::map<std::string, bool> &_visited,
        Vertices &_group)
    {
        _visited[_name] = true;

        for (const auto &neighbor : adj_list_[_name])
        {
            if (_visited[neighbor.name()] == false)
                generateGroupListUtil(neighbor.name(), _visited, _group);
        }

        _group.emplace(_name, vertices_[_name]);
    }
} // namespace SubgoalGenerator