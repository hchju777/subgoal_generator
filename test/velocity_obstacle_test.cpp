#include <gtest/gtest.h>

#include <iostream>
#include <random>

#include "subgoal_generator/dynamic_graph.h"
#include "subgoal_generator/dynamic_graph_manager.h"
#include "subgoal_generator/velocity_obstacle.h"
#include "subgoal_generator/velocity_obstacle_manager.h"

namespace SubgoalGenerator
{
    class VOTest : public testing::Test
    {
    };

    TEST(VOTest, VOConeGen_TEST)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> position_dis(-2.5, 2.5);
        std::uniform_real_distribution<double> theta_dis(-1 * M_PI, M_PI);
        std::uniform_real_distribution<double> theta_diff_dis(-0.125 * M_PI, 0.125*M_PI);
        std::uniform_real_distribution<double> velocity_dis(0.0, 1.0);
        std::uniform_int_distribution<int> radius_dis(4, 6);

        DynamicGraph::Graph::SharedPtr graph = std::make_shared<DynamicGraph::Graph>();
        int vertex_num = 4;
        double range = 5.0;

        std::map<std::string, Agent> agents;
        for (int idx = 0; idx < vertex_num; ++idx)
        {
            Agent agent;
            agent.name() = "robot" + std::to_string(idx + 1);
            agent.radius() = 0.1 * radius_dis(gen);

            Pose goal(position_dis(gen), position_dis(gen), theta_dis(gen));
            agent.goal() = goal;

            bool collision_condition;
            do
            {
                collision_condition = false;

                Pose pose(position_dis(gen), position_dis(gen), theta_dis(gen));
                for (const auto &agentPair : agents)
                {
                    const auto &other = agentPair.second;
                    Eigen::Vector2d relativePosition = other.pose().position() - pose.position();

                    if (relativePosition.norm() < agent.radius() + other.radius())
                    {
                        collision_condition = true;
                        break;
                    }
                }
                Eigen::Vector2d toGoal = goal.position() - pose.position();
                pose.theta() = std::atan2(toGoal.y(), toGoal.x());
                pose += Pose(0, 0, theta_diff_dis(gen));

                agent.pose() = pose;
            } while (collision_condition);

            double velocity = velocity_dis(gen);
            agent.velocity().coeffRef(0) = velocity * agent.pose().orientationUnitVec().coeffRef(0);
            agent.velocity().coeffRef(1) = velocity * agent.pose().orientationUnitVec().coeffRef(1);

            agents.emplace(agent.name(), agent);
        }

        std::cout << "====================================================================================================================" << std::endl;
        for (const auto &agentPair : agents)
        {
            const auto &agent = agentPair.second;

            DynamicGraph::Vertex vertex(
                agentPair.first, agent.pose(), agent.goal());

            graph->addVertex(vertex, range);
        }

        DynamicGraph::Manager::printGraph(graph);
        std::cout << "====================================================================================================================" << std::endl;

        VelocityObstacle::Generator::SharedPtr vo_generator = std::make_shared<VelocityObstacle::Generator>();
        for (const auto &vertexPair : graph->vertices())
        {
            Agent *agent = new Agent();
            VelocityObstacle::Generator::toAgent(vertexPair.first, graph->vertices(), agent);

            agent->velocity() = agents[agent->name()].velocity();
            agent->radius() = agents[agent->name()].radius();

            vo_generator->emplaceAgent(agent);
        }
        vo_generator->updateAgentsNeighbors(graph->adj_list());

        VelocityObstacle::Manager::printAgents(vo_generator->agents());
        std::cout << "====================================================================================================================" << std::endl;

        for (const auto &agentPair : vo_generator->agents())
        {
            vo_generator->updateVOCones(agentPair.first);
        }

        VelocityObstacle::Manager::exportDynamicGraph(vo_generator->agents());
    }

} // namespace SubgoalGenerator

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    SubgoalGenerator::VelocityObstacle::Manager::clearResultDir();

    return RUN_ALL_TESTS();
}