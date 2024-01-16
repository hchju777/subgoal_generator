#include <gtest/gtest.h>

#include <iostream>
#include <random>

#include "subgoal_generator/subgoal_generator.h"
#include "subgoal_generator/pibt_manager.h"

namespace SubgoalGenerator::PIBT
{
    std::map<std::string, Agent> generate_random_agents(int _num)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> position_dis(-2.5, 2.5);
        std::uniform_real_distribution<double> theta_dis(-1 * M_PI, M_PI);
        std::uniform_real_distribution<double> theta_diff_dis(-0.125 * M_PI, 0.125 * M_PI);
        std::uniform_real_distribution<double> velocity_dis(0.0, 1.0);
        std::uniform_int_distribution<int> radius_dis(4, 6);

        DynamicGraph::Graph::SharedPtr graph = std::make_shared<DynamicGraph::Graph>();

        std::map<std::string, Agent> agents;
        for (int idx = 0; idx < _num; ++idx)
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

        return agents;
    }

    class PIBTTest : public testing::Test, public PIBT::Solver
    {
    };

    TEST(PIBTTest, EnvironmentGeneration)
    {
        for (int i = 0; i < 100; ++i)
        {
            std::cout << std::endl;
            Generator::SharedPtr subgoal_generator;
            subgoal_generator = std::make_shared<Generator>();

            SubgoalGenerator::Agents agents = generate_random_agents(4);

            for (const auto &agentPair : agents)
            {
                subgoal_generator->emplaceAgent(agentPair.second);
            }

            std::cout.precision(3);
            std::cout << "==========================================================================="
                      << "===========================================================================" << std::endl;

            Manager::printAgents(subgoal_generator->agents());

            std::cout << "==========================================================================="
                      << "===========================================================================" << std::endl;

            Manager::printGraph(subgoal_generator->graph());

            std::cout << "==========================================================================="
                      << "===========================================================================" << std::endl;

            std::list<Manager::ExportDB> exportDBList;
            const auto &groupList = subgoal_generator->graph()->generateGroupList();
            std::stack<std::string> priority_graph = subgoal_generator->graph()->topologicalSort();

            for (const auto &group : groupList)
            {
                EXPECT_EQ(subgoal_generator->updateVOCones(group), true);

                Agents agentsGroup;
                for (const auto &vertexPair : group)
                {
                    std::string name = vertexPair.first;

                    agentsGroup.emplace(name, agents[name]);
                }

                Solver::SharedPtr solver = std::make_shared<Solver>(agentsGroup, priority_graph);

                std::map<std::string, BufferedVoronoiDiagram::VoronoiCell> voronoi_diagram, buffered_voronoi_diagram;
                EXPECT_EQ(solver->generateBVC(voronoi_diagram, buffered_voronoi_diagram), true);

                {
                    Manager::ExportDB exportDB;
                    for (const auto &vcPair : voronoi_diagram)
                    {
                        Manager::ExportDataContainer exportData;
                        {
                            exportData.agent_ = subgoal_generator->agents()[vcPair.first];
                            exportData.voronoi_cell_ = vcPair.second;
                        }
                        exportDB.emplace(exportData.agent_.name(), exportData);
                    }

                    for (const auto &bvcPair : buffered_voronoi_diagram)
                    {
                        exportDB[bvcPair.first].buffered_voronoi_cell_ = bvcPair.second;
                    }

                    exportDBList.emplace_back(exportDB);
                }
            }

            EXPECT_EQ(Manager::exportDBList(exportDBList), true);
        }
    }

    TEST(PIBTTest, FindSubgoal_TEST)
    {
        Solver::SharedPtr pibt_solver = std::make_shared<Solver>();

        //! Input
        // Define the goal point
        const std::vector<Point_2> goals = {
            Point_2(2, 2),
            Point_2(4, 4),
            Point_2(7, 0),
            Point_2(10, 4),
            Point_2(6, 10),
            Point_2(0, 7),
            Point_2(6, 6)};

        const std::vector<Point_2> subgoals = {
            Point_2(6, 2),
            Point_2(5, 5),
            Point_2(7, 2),
            Point_2(8, 4),
            Point_2(6, 8),
            Point_2(2, 7),
            Point_2(6, 6)};

        // Define the polygon vertices
        const std::vector<Point_2> polygonVertices = {
            Point_2(2, 6),
            Point_2(4, 6),
            Point_2(6, 4),
            Point_2(6, 2),
            Point_2(8, 2),
            Point_2(8, 8),
            Point_2(2, 8)};

        const CGAL::Polygon_2<Kernel> polygon(polygonVertices.begin(), polygonVertices.end());

        //! Process
        CGAL::Polygon_triangulation_decomposition_2<Kernel> triangular_decomp;

        std::list<CGAL::Polygon_2<Kernel>> triangular_decomp_poly_list;
        triangular_decomp(polygon, std::back_inserter(triangular_decomp_poly_list));

        for (size_t i = 0; i < goals.size(); ++i)
        {
            Point_2 subgoal;

            const Point_2 &goal = goals[i];
            const Point_2 &subgoal_answer = subgoals[i];

            EXPECT_EQ(PIBT::SubgoalUtil::find_subgoal(goal, triangular_decomp_poly_list, subgoal), true);
            EXPECT_EQ(subgoal, subgoal_answer);
        }
    }

} // namespace SubgoalGenerator::PIBT

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}