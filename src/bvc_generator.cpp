#include "subgoal_generator/bvc_generator.h"

namespace SubgoalGenerator::BufferedVoronoiDiagram
{
    Generator::Generator(const std::vector<Site_2> &_points)
    {
        bbox_ = CGAL::bounding_box(_points.begin(), _points.end());

        for (const auto &point : _points)
            vd_.insert(point);

        std::cout << "Buffered Voronoi Diagram Generator has been initilized." << std::endl;
    }

    Generator::~Generator()
    {
        reset();

        std::cout << "Buffered Voronoi Diagram Generator has been terminated." << std::endl;
    }

    bool Generator::get_polygon(const Point_2 &_point, CGAL::Polygon_2<Kernel>& _poly)
    {
        assert(is_valid());

        Locate_result lr = vd_.locate(_point);

        if (Face_handle *f = boost::get<Face_handle>(&lr))
        {
            Ccb_halfedge_circulator ec_start = (*f)->ccb();
            Ccb_halfedge_circulator ec = ec_start;

            CGAL::Polygon_2<Kernel> vn_poly;
            do
            {
                const CGAL::Object cur_seg_dual = vd_.dual().dual(ec->dual());

                const auto cur_seg = convert_to_seg(cur_seg_dual, ec->has_target());
                vn_poly.push_back(cur_seg.source());

                if (not(ec->has_target()))
                {
                    const CGAL::Object next_seg_dual = vd_.dual().dual(ec->next()->dual());
                    const auto next_seg = convert_to_seg(next_seg_dual, ec->next()->has_target());
                    vn_poly.push_back(next_seg.target());
                }
            } while (++ec != ec_start);

            CGAL::Polygon_2<Kernel> box_poly;
            {
                double offset_x = (bbox_.xmax() - bbox_.xmin()) / 2;
                double offset_y = (bbox_.ymax() - bbox_.ymin()) / 2;

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

        return false;
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