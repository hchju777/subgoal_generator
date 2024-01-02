#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Straight_skeleton_2/Straight_skeleton_aux.h>
#include <CGAL/Polygon_offset_builder_2.h>

#include <memory>

#include <vector>
#include <cassert>

#include "subgoal_generator/bvc_manager.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K ;

typedef K::Point_2                   Point ;
typedef CGAL::Polygon_2<K>           Polygon_2 ;
typedef CGAL::Straight_skeleton_2<K> Ss ;

typedef std::shared_ptr<Polygon_2> PolygonPtr ;
typedef std::shared_ptr<Ss> SsPtr ;

typedef std::vector<PolygonPtr> PolygonPtrVector ;

int main()
{
  Polygon_2 poly ;
  poly.push_back( Point(-17.5,-16.25) ) ;
  poly.push_back( Point(5,-16.25) ) ;
  poly.push_back( Point(5,5) ) ;
  poly.push_back( Point(-17.5,5) ) ;
  assert(poly.is_counterclockwise_oriented());

  auto ss = CGAL::create_interior_straight_skeleton_2(poly);

  double lOffset = 1 ;
  auto offset_polygons = CGAL::create_offset_polygons_2<Polygon_2>(lOffset,*ss);
  for (const auto &polygon : offset_polygons)
  {
    auto foo = *polygon;
    std::cout << "Polygon: ";
    SubgoalGenerator::BufferedVoronoiDiagram::Manager::printPolygon(foo);
  }

  return EXIT_SUCCESS;
}