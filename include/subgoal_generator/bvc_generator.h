#pragma once

// standard includes
#include <memory>
#include <limits>
#include <cassert>

// CGAL includes
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Voronoi_diagram_2.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/bounding_box.h>

#include <CGAL/Random.h>
#include <CGAL/Triangulation_utils_2.h>
#include <CGAL/Voronoi_diagram_2/Face.h>
#include <CGAL/Voronoi_diagram_2/Handle_adaptor.h>
#include <CGAL/Voronoi_diagram_2/Vertex.h>
#include <CGAL/Voronoi_diagram_2/basic.h>
#include <CGAL/Voronoi_diagram_2/Accessor.h>

// typedefs for defining the adaptor
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT> AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT, AT, AP> VD;

// typedef for the result type of the point location
typedef AT::Site_2 Site_2;
typedef AT::Point_2 Point_2;
typedef VD::Locate_result Locate_result;
typedef VD::Vertex_handle Vertex_handle;
typedef VD::Face_handle Face_handle;
typedef VD::Halfedge_handle Halfedge_handle;
typedef VD::Ccb_halfedge_circulator Ccb_halfedge_circulator;

namespace SubgoalGenerator::BufferedVoronoiDiagram
{
    class Generator
    {
    public:
        typedef std::unique_ptr<Generator> UniquePtr;
        typedef std::shared_ptr<Generator> SharedPtr;

    public:
        Generator(const std::vector<Site_2> &_points);

        ~Generator();

    public:
        bool get_polygon(const Point_2 &_point, CGAL::Polygon_2<Kernel>& _poly);

    public:
        inline VD &vd() { return vd_; }
        inline const VD &vd() const { return vd_; }

        inline void reset()
        {
            vd_.clear();
        }

        inline bool is_valid()
        {
            return vd_.is_valid();
        }

    protected:
        Kernel::Segment_2 convert_to_seg(const CGAL::Object _seg_obj, bool _outgoing);

    protected:
        VD vd_;

        CGAL::Iso_rectangle_2<CGAL::Epick> bbox_;
    }; // class Generator

} // namespace SubgoalGenerator::BufferedVoronoiDiagram