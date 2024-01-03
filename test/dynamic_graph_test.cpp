#include <gtest/gtest.h>

#include <iostream>
#include <random>

#include "subgoal_generator/dynamic_graph.h"
#include "subgoal_generator/dynamic_graph_manager.h"

namespace SubgoalGenerator
{
    class DynamicGraphTest : public testing::Test
    {
    };

    TEST(DynamicGraphTest, DynamicGraphHandle_TEST)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> position_dis(-5, 5);
        std::uniform_real_distribution<double> theta_dis(-1 * M_PI, M_PI);

        DynamicGraph::Graph::SharedPtr graph = std::make_shared<DynamicGraph::Graph>();
        int vertex_num = 9;
        double range = 5.0;

        std::cout << "====================================================================================================================" << std::endl;
        for (int idx = 0; idx < vertex_num; ++idx)
        {
            DynamicGraph::Vertex vertex(
                "robot" + std::to_string(idx + 1),
                Pose(position_dis(gen), position_dis(gen), theta_dis(gen)),
                Pose(position_dis(gen), position_dis(gen), theta_dis(gen)));

            graph->addVertex(vertex, range);
        }
        DynamicGraph::Manager::printGraph(graph);

        std::cout << "====================================================================================================================" << std::endl;
        std::uniform_int_distribution<int> int_dis(1, vertex_num);
        for (int cnt = 0; cnt < std::floor(vertex_num / 2); ++cnt)
        {
            std::string vertex_name_to_delete;

            do
            {
                vertex_name_to_delete = "robot" + std::to_string(int_dis(gen));
            } while (not(graph->vertices().contains(vertex_name_to_delete)));

            graph->deleteVertex(vertex_name_to_delete);

            std::cout << "After delete " << vertex_name_to_delete << std::endl;
            DynamicGraph::Manager::printGraph(graph);
            std::cout << "====================================================================================================================" << std::endl;
        }

        graph->reset();
        DynamicGraph::Manager::printGraph(graph);
        std::cout << "====================================================================================================================" << std::endl;
    }

    TEST(DynamicGraphTest, ExportGraph_TEST)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> position_dis(-5, 5);
        std::uniform_real_distribution<double> theta_dis(-1 * M_PI, M_PI);

        DynamicGraph::Graph::SharedPtr graph = std::make_shared<DynamicGraph::Graph>();
        int vertex_num = 9;
        double range = 5.0;

        std::cout << "====================================================================================================================" << std::endl;
        for (int idx = 0; idx < vertex_num; ++idx)
        {
            DynamicGraph::Vertex vertex(
                "robot" + std::to_string(idx + 1),
                Pose(position_dis(gen), position_dis(gen), theta_dis(gen)),
                Pose(position_dis(gen), position_dis(gen), theta_dis(gen)));

            graph->addVertex(vertex, range);
        }
        DynamicGraph::Manager::printGraph(graph);
        std::cout << "====================================================================================================================" << std::endl;

        DynamicGraph::Manager::exportDynamicGraph(graph);
    }

} // namespace SubgoalGenerator

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    SubgoalGenerator::DynamicGraph::Manager::clearResultDir();

    return RUN_ALL_TESTS();
}