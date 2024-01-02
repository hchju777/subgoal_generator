#include <gtest/gtest.h>

#include <iostream>

#include "subgoal_generator/bvc_generator.h"
#include "subgoal_generator/bvc_manager.h"

namespace SubgoalGenerator::BufferedVoronoiDiagram
{
  class BVCGenTest : public testing::Test
  {
  };

  TEST(BVCGenTest, VD_GEN_TEST)
  {
    std::vector<Site_2> points;
    points.clear();

    points.push_back(Site_2(0, 0));
    points.push_back(Site_2(10, 0));
    points.push_back(Site_2(10, 10));
    points.push_back(Site_2(0, 10));
    points.push_back(Site_2(20, 0));
    points.push_back(Site_2(30, 0));
    points.push_back(Site_2(35, 0));
    points.push_back(Site_2(15, 15));
    points.push_back(Site_2(10, 15));
    points.push_back(Site_2(30, 32.5));

    Generator::UniquePtr bvc_generator = std::make_unique<Generator>(points);

    Manager::VoronoiDiagram voronoi_diagram;

    for (const auto &point : points)
    {
      std::cout << "=================================================================" << std::endl;
      Point_2 p(point.x(), point.y());
      std::cout << "Point: " << p << std::endl;

      CGAL::Polygon_2<Kernel> polygon;

      EXPECT_EQ(bvc_generator->get_polygon(p, polygon), true);

      voronoi_diagram.push_back(std::make_pair(p, polygon));
      Manager::printPolygon(polygon);

      std::cout << "=================================================================" << std::endl;
      std::cout << std::endl;
    }

    Manager::exportVoronoiDiagram(
        voronoi_diagram, "result", "voronoi");

    EXPECT_EQ(0, 0);
  }

  TEST(BVCGenTest, BVD_GEN_TEST)
  {
    std::vector<Site_2> points;
    points.clear();

    points.push_back(Site_2(0, 0));
    points.push_back(Site_2(10, 0));
    points.push_back(Site_2(10, 10));
    points.push_back(Site_2(0, 10));
    points.push_back(Site_2(20, 0));
    points.push_back(Site_2(30, 0));
    points.push_back(Site_2(35, 0));
    points.push_back(Site_2(15, 15));
    points.push_back(Site_2(10, 15));
    points.push_back(Site_2(30, 32.5));

    Generator::UniquePtr bvc_generator = std::make_unique<Generator>(points);

    Manager::VoronoiDiagram voronoi_diagram;
    Manager::VoronoiDiagram buffered_voronoi_diagram;

    for (const auto &point : points)
    {
      std::cout << "=================================================================" << std::endl;
      Point_2 p(point.x(), point.y());
      std::cout << "Point: " << p << std::endl;

      CGAL::Polygon_2<Kernel> polygon;

      EXPECT_EQ(bvc_generator->get_polygon(p, polygon), true);
      voronoi_diagram.push_back(std::make_pair(p, polygon));

      bvc_generator->convert_to_bvc(polygon, 3.0);
      buffered_voronoi_diagram.push_back(std::make_pair(p, polygon));
      Manager::printPolygon(polygon);

      std::cout << "=================================================================" << std::endl;
      std::cout << std::endl;
    }

    Manager::exportBufferedVoronoiDiagram(
        voronoi_diagram, buffered_voronoi_diagram, "result", "buffered_voronoi");

    EXPECT_EQ(0, 0);
  }
} // namespace SubgoalGenerator::BufferedVoronoiDiagram

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  SubgoalGenerator::BufferedVoronoiDiagram::Manager::clearResultDir();

  return RUN_ALL_TESTS();
}