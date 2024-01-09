#include "subgoal_generator/bvc_generator.h"

namespace SubgoalGenerator::BufferedVoronoiDiagram
{
    Generator::Generator(const std::vector<Site_2> &_points)
    {
        bbox_ = CGAL::bounding_box(_points.begin(), _points.end());

        for (const auto &point : _points)
            vd_.insert(point);
    }

    Generator::~Generator()
    {
        reset();
    }

    bool Generator::get_polygon(const Point_2 &_point, CGAL::Polygon_2<Kernel> &_poly)
    {
        CGAL::Polygon_2<Kernel> vn_poly;

        if (not(get_raw_voronoi_polygon(_point, vn_poly)))
            return false;

        CGAL::Polygon_2<Kernel> box_poly;
        {
            auto offset_x = CGAL::abs(bbox_.xmax() - bbox_.xmin()) / 2;
            auto offset_y = CGAL::abs(bbox_.ymax() - bbox_.ymin()) / 2;

            box_poly.push_back(Point_2(bbox_.xmin() - offset_x, bbox_.ymin() - offset_y));
            box_poly.push_back(Point_2(bbox_.xmax() + offset_x, bbox_.ymin() - offset_y));
            box_poly.push_back(Point_2(bbox_.xmax() + offset_x, bbox_.ymax() + offset_y));
            box_poly.push_back(Point_2(bbox_.xmin() - offset_x, bbox_.ymax() + offset_y));
        }

        std::list<CGAL::Polygon_with_holes_2<Kernel>> cropped_vn_poly;
        CGAL::intersection(vn_poly, box_poly, std::back_insert_iterator(cropped_vn_poly));

        assert(cropped_vn_poly.size() == 1);

        CGAL::Polygon_with_holes_2<Kernel> &poly_w_holes = cropped_vn_poly.front();
        _poly = poly_w_holes.outer_boundary();

        return true;
    }

    bool Generator::get_raw_voronoi_polygon(const Point_2 &_point, CGAL::Polygon_2<Kernel> &_poly)
    {
        assert(is_valid());

        Locate_result lr = vd_.locate(_point);

        if (Face_handle *f = boost::get<Face_handle>(&lr))
        {
            Ccb_halfedge_circulator ec_start = (*f)->ccb();
            Ccb_halfedge_circulator ec = ec_start;

            do
            {
                const CGAL::Object cur_seg_dual = vd_.dual().dual(ec->dual());

                const auto cur_seg = convert_to_seg(cur_seg_dual, ec->has_target());
                _poly.push_back(cur_seg.source());

                if (not(ec->has_target()))
                {
                    const CGAL::Object next_seg_dual = vd_.dual().dual(ec->next()->dual());
                    const auto next_seg = convert_to_seg(next_seg_dual, ec->next()->has_target());
                    _poly.push_back(next_seg.target());
                }
            } while (++ec != ec_start);

            return true;
        }

        return false;
    }

    bool Generator::convert_to_bvc(CGAL::Polygon_2<Kernel> &_poly, double _offset)
    {
        assert(_poly.is_counterclockwise_oriented());

        // To get a buffered voronoi cell, CGAL::create_interior_straight_skeleton_2 should be used.
        // However, to use CGAL::create_interior_straight_skeleton_2, inexact_kernel should be used.
        // See: https://doc.cgal.org/latest/Straight_skeleton_2/group__PkgStraightSkeleton2SkeletonFunctions.html
        typedef CGAL::Exact_predicates_inexact_constructions_kernel InExact_Kernel;
        typedef InExact_Kernel::Point_2 InExact_Point_2;

        CGAL::Polygon_2<InExact_Kernel> twin_poly;
        for (auto viter = _poly.vertices_begin(); viter != _poly.vertices_end(); ++viter)
        {
            twin_poly.push_back(InExact_Point_2(CGAL::to_double(viter->x()), CGAL::to_double(viter->y())));
        }

        auto ss = CGAL::create_interior_straight_skeleton_2(twin_poly);

        auto offset_polygon = CGAL::create_offset_polygons_2<CGAL::Polygon_2<Kernel>>(_offset, *ss);

        assert(offset_polygon.size() == 1);

        if (offset_polygon.empty())
            return false;

        _poly = *offset_polygon.front();

        return true;
    }

    Kernel::Segment_2 Generator::convert_to_seg(const CGAL::Object _seg_obj, bool _outgoing)
    {
        const Kernel::Segment_2 *dseg = CGAL::object_cast<Kernel::Segment_2>(&_seg_obj);
        const Kernel::Ray_2 *dray = CGAL::object_cast<Kernel::Ray_2>(&_seg_obj);

        if (dseg)
        {
            return *dseg;
        }
        else
        {
            const auto &source = dray->source();
            const auto dsx = source.x();
            const auto dsy = source.y();
            const auto &dir = dray->direction();
            const auto tpoint = Kernel::Point_2(
                dsx + 1000 * dir.dx(), dsy + 1000 * dir.dy());

            if (_outgoing)
            {
                return Kernel::Segment_2(dray->source(), tpoint);
            }
            else
            {
                return Kernel::Segment_2(tpoint, dray->source());
            }
        }
    }
} // SubgoalGenerator::BufferedVoronoiDiagram