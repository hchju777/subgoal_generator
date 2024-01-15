#include <gtest/gtest.h>

#include <iostream>
#include <random>

#include "subgoal_generator/subgoal_generator.h"
#include "subgoal_generator/subgoal_generator_manager.h"

namespace SubgoalGenerator
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

    class SubgoalGenTest : public testing::Test
    {
    };

    TEST(SubgoalGenTest, SubgoalGen_TEST)
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

            SubgoalGenerator::Manager::printAgents(subgoal_generator->agents());

            std::cout << "==========================================================================="
                      << "===========================================================================" << std::endl;

            SubgoalGenerator::Manager::printGraph(subgoal_generator->graph());

            std::cout << "==========================================================================="
                      << "===========================================================================" << std::endl;

            std::list<Manager::ExportDB> exportDBList;
            const auto &groupList = subgoal_generator->graph()->generateGroupList();

            for (const auto &group : groupList)
            {
                BufferedVoronoiDiagram::Generator::SharedPtr bvc_generator;
                EXPECT_EQ(subgoal_generator->get_BVC_Generator(group, bvc_generator), true);

                std::map<std::string, BufferedVoronoiDiagram::VoronoiCell> voronoi_diagram, buffered_voronoi_diagram;
                EXPECT_EQ(subgoal_generator->generateBVC(group, bvc_generator, voronoi_diagram, buffered_voronoi_diagram), true);

                EXPECT_EQ(subgoal_generator->updateVOCones(group), true);
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

    // TEST(SubgoalGenTest, FindSubgoal_TEST)
    // {
    //     Generator::SharedPtr subgoal_generator;
    //     subgoal_generator = std::make_shared<Generator>();

    //     //! Input
    //     // Define the goal point
    //     const std::vector<Point_2> goals = {
    //         Point_2(2, 2),
    //         Point_2(4, 4),
    //         Point_2(7, 0),
    //         Point_2(10, 4),
    //         Point_2(6, 10),
    //         Point_2(0, 7),
    //         Point_2(6, 6)};

    //     const std::vector<Point_2> subgoals = {
    //         Point_2(6, 2),
    //         Point_2(5, 5),
    //         Point_2(7, 2),
    //         Point_2(8, 4),
    //         Point_2(6, 8),
    //         Point_2(2, 7),
    //         Point_2(6, 6)};

    //     // Define the polygon vertices
    //     const std::vector<Point_2> polygonVertices = {
    //         Point_2(2, 6),
    //         Point_2(4, 6),
    //         Point_2(6, 4),
    //         Point_2(6, 2),
    //         Point_2(8, 2),
    //         Point_2(8, 8),
    //         Point_2(2, 8)};

    //     const CGAL::Polygon_2<Kernel> polygon(polygonVertices.begin(), polygonVertices.end());

    //     //! Process
    //     std::list<CGAL::Polygon_2<Kernel>> convex_subPolygons = subgoal_generator->get_convex_subPolygons(polygon);

    //     for (size_t i = 0; i < goals.size(); ++i)
    //     {
    //         Point_2 subgoal;

    //         const Point_2 &goal = goals[i];
    //         const Point_2 &subgoal_answer = subgoals[i];

    //         EXPECT_EQ(subgoal_generator->find_subgoal(goal, convex_subPolygons, subgoal), true);
    //         EXPECT_EQ(subgoal, subgoal_answer);
    //     }
    // }

    // TEST(SubgoalGenTest, FindGarrison_TEST)
    // {
    //     std::vector<Point_2> robot_positions = {
    //         Point_2(0, 0),
    //         Point_2(1, 0),
    //         Point_2(1, 1),
    //         Point_2(0, 1)};

    //     std::vector<Point_2> subgoals = {
    //         Point_2(-2, -2),
    //         Point_2(0.25, 0),
    //         Point_2(0.75, 0),
    //         Point_2(0.75, 1),
    //         Point_2(0.25, 1)};

    //     std::vector<std::pair<bool, Point_2>> answers = {
    //         {false, Point_2()},
    //         {true, Point_2(1, 0)},
    //         {true, Point_2(0, 0)},
    //         {true, Point_2(0, 1)},
    //         {true, Point_2(1, 1)}};

    //     Generator::UniquePtr subgoal_generator;
    //     BufferedVoronoiDiagram::Generator::UniquePtr bvc_generator =
    //         std::make_unique<BufferedVoronoiDiagram::Generator>(robot_positions);

    //     size_t answer_idx = 0;
    //     for (const auto &subgoal : subgoals)
    //     {
    //         Point_2 current_position;

    //         Locate_result lr = bvc_generator->vd().locate(subgoal);
    //         if (Face_handle *f = boost::get<Face_handle>(&lr))
    //         {
    //             CGAL::Polygon_2<Kernel> vn_poly;
    //             bvc_generator->get_raw_voronoi_polygon(subgoal, vn_poly);

    //             //! Make a vector from current position to subgoal
    //             current_position = (*f)->dual()->point();
    //         }

    //         Point_2 garrison_point;
    //         bool answer_flag = subgoal_generator->find_garrison_point_from_voronoi_diagram(
    //             current_position, subgoal, bvc_generator, garrison_point);

    //         EXPECT_EQ(answer_flag, answers[answer_idx].first);
    //         if (answer_flag)
    //             EXPECT_EQ(garrison_point, answers[answer_idx].second);

    //         ++answer_idx;
    //     }
    // }

    // TEST(SubgoalGenTest, TruncatedPolyGen_TEST)
    // {
    //     std::vector<Point_2> polygon_vertices = {
    //         Point_2(-2, -2),
    //         Point_2(2, -2),
    //         Point_2(2, 2),
    //         Point_2(-2, 2)};

    //     CGAL::Polygon_2<Kernel> polygon(polygon_vertices.begin(), polygon_vertices.end());

    //     Agent::Cone cone1;
    //     {
    //         cone1.point_ = Eigen::Vector2d(0, 0);
    //         cone1.left_direction_ = Eigen::Vector2d(0, 1);
    //         cone1.right_direction_ = Eigen::Vector2d(1, 0);
    //     }

    //     Agent::Cone cone2;
    //     {
    //         cone2.point_ = Eigen::Vector2d(0, 0);
    //         cone2.left_direction_ = Eigen::Vector2d(1, 0);
    //         cone2.right_direction_ = Eigen::Vector2d(0, 1);
    //     }

    //     SubgoalGenerator::Generator::UniquePtr subgoal_generator;
    //     CGAL::Polygon_2<Kernel> truncated_polygon;
    //     EXPECT_EQ(subgoal_generator->get_truncated_polygon(polygon, {cone1}, truncated_polygon), true);
    //     EXPECT_EQ(subgoal_generator->get_truncated_polygon(polygon, {cone2}, truncated_polygon), true);
    //     EXPECT_EQ(subgoal_generator->get_truncated_polygon(polygon, {cone1, cone2}, truncated_polygon), false);
    // }

} // namespace SubgoalGenerator

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    SubgoalGenerator::Manager::clearResultDir();

    return RUN_ALL_TESTS();
}