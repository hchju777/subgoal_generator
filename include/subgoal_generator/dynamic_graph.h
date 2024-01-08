#pragma once

#include <memory>
#include <list>
#include <map>
#include <stack>
#include <iostream>

#include <subgoal_generator/vertex.h>

namespace SubgoalGenerator::DynamicGraph
{
    class Graph
    {
    public:
        typedef std::unique_ptr<Graph> UniquePtr;
        typedef std::shared_ptr<Graph> SharedPtr;

    public:
        Graph();

        Graph(const Graph &_graph)
        {
            vertices_ = _graph.vertices_;
            adj_list_ = _graph.adj_list_;
            adj_priority_list_ = _graph.adj_priority_list_;
        }

        ~Graph();

    public:
        void addVertex(const Vertex &_newVertex, double _range);

        void deleteVertex(std::string _name);

        std::stack<std::string> topologicalSort();

        std::list<Vertices> generateGroupList();

    public:
        Graph &operator=(const Graph &_rhs)
        {
            if (&_rhs != this)
            {
                vertices_ = _rhs.vertices_;
                adj_list_ = _rhs.adj_list_;
                adj_priority_list_ = _rhs.adj_priority_list_;
            }

            return *this;
        }

    public:
        const std::map<std::string, Vertex> vertices() const
        {
            return vertices_;
        }

        const std::map<std::string, std::list<Vertex>> adj_list() const
        {
            return adj_list_;
        }

        const std::map<std::string, std::list<std::string>> adj_priority_list() const
        {
            return adj_priority_list_;
        }

    public:
        void reset();

    protected:
        void topologicalSortUtil(
            std::string _name, std::map<std::string, bool> &_visited,
            std::stack<std::string> &_stack);

        void generateGroupListUtil(
            std::string _name, std::map<std::string, bool> &_visited,
            Vertices &_group);

    protected:
        std::map<std::string, Vertex> vertices_;
        std::map<std::string, std::list<Vertex>> adj_list_;
        std::map<std::string, std::list<std::string>> adj_priority_list_;
    }; // class DynamicGraph
} // namespace SubgoalGenerator