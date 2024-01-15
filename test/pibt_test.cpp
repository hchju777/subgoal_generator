#include <gtest/gtest.h>

#include <iostream>
#include <random>

#include "subgoal_generator/pibt.h"

namespace SubgoalGenerator::PIBT
{
    class PIBTTest : public testing::Test
    {
    };

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
        std::list<CGAL::Polygon_2<Kernel>> convex_subPolygons = pibt_solver->get_triangular_subPolygons(polygon);

        for (size_t i = 0; i < goals.size(); ++i)
        {
            Point_2 subgoal;

            const Point_2 &goal = goals[i];
            const Point_2 &subgoal_answer = subgoals[i];

            EXPECT_EQ(pibt_solver->find_subgoal(goal, convex_subPolygons, subgoal), true);
            EXPECT_EQ(subgoal, subgoal_answer);
        }
    }

    TEST(PIBTTest, FindGarrison_TEST)
    {
        std::vector<Point_2> robot_positions = {
            Point_2(0, 0),
            Point_2(1, 0),
            Point_2(1, 1),
            Point_2(0, 1)};

        std::vector<Point_2> subgoals = {
            Point_2(-2, -2),
            Point_2(0.25, 0),
            Point_2(0.75, 0),
            Point_2(0.75, 1),
            Point_2(0.25, 1)};

        std::vector<std::pair<bool, Point_2>> answers = {
            {true, Point_2(0, 0)},
            {true, Point_2(1, 0)},
            {true, Point_2(0, 0)},
            {true, Point_2(0, 1)},
            {true, Point_2(1, 1)}};

        Solver::SharedPtr pibt_solver = std::make_shared<Solver>();
        BufferedVoronoiDiagram::Generator::SharedPtr bvc_generator =
            std::make_shared<BufferedVoronoiDiagram::Generator>(robot_positions);

        size_t answer_idx = 0;
        for (const auto &subgoal : subgoals)
        {
            Point_2 current_position;

            Locate_result lr = bvc_generator->vd().locate(subgoal);
            if (Face_handle *f = boost::get<Face_handle>(&lr))
            {
                CGAL::Polygon_2<Kernel> vn_poly;
                bvc_generator->get_raw_voronoi_polygon(subgoal, vn_poly);

                //! Make a vector from current position to subgoal
                current_position = (*f)->dual()->point();
            }

            Point_2 garrison_point;
            bool answer_flag = pibt_solver->find_garrison_point_from_voronoi_diagram(
                current_position, subgoal, bvc_generator, garrison_point);

            EXPECT_EQ(answer_flag, answers[answer_idx].first);
            if (answer_flag)
                EXPECT_EQ(garrison_point, answers[answer_idx].second);

            ++answer_idx;
        }
    }

    TEST(PIBTTest, TruncatedPolyGen_TEST)
    {
        std::vector<Point_2> polygon_vertices = {
            Point_2(-2, -2),
            Point_2(2, -2),
            Point_2(2, 2),
            Point_2(-2, 2)};

        CGAL::Polygon_2<Kernel> polygon(polygon_vertices.begin(), polygon_vertices.end());

        Agent::Cone cone1;
        {
            cone1.point_ = Eigen::Vector2d(0, 0);
            cone1.left_direction_ = Eigen::Vector2d(0, 1);
            cone1.right_direction_ = Eigen::Vector2d(1, 0);
        }

        Agent::Cone cone2;
        {
            cone2.point_ = Eigen::Vector2d(0, 0);
            cone2.left_direction_ = Eigen::Vector2d(1, 0);
            cone2.right_direction_ = Eigen::Vector2d(0, 1);
        }

        Solver::SharedPtr pibt_solver = std::make_shared<Solver>();
        CGAL::Polygon_2<Kernel> truncated_polygon;
        EXPECT_EQ(pibt_solver->get_truncated_polygon(polygon, {cone1}, truncated_polygon), true);
        EXPECT_EQ(pibt_solver->get_truncated_polygon(polygon, {cone2}, truncated_polygon), true);
        EXPECT_EQ(pibt_solver->get_truncated_polygon(polygon, {cone1, cone2}, truncated_polygon), false);
    }
} // namespace SubgoalGenerator::PIBT

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}