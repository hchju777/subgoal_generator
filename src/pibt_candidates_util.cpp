#include "subgoal_generator/pibt_candidates_util.h"

namespace SubgoalGenerator::PIBT
{
    std::list<std::pair<Point_2, CGAL::Polygon_2<Kernel>>> CandidatesUtil::createRawCandidates(
        const Agent &_agent, const VoronoiCell &_bvc, const VD &_vd)
    {
        const Point_2 &site = _bvc.first;
        const CGAL::Polygon_2<Kernel> &bvc = _bvc.second;

        std::list<std::pair<Point_2, CGAL::Polygon_2<Kernel>>> raw_candidates;
        for (std::size_t i = 0; i < bvc.size(); ++i)
        {
            const Point_2 &p1 = bvc[i];
            const Point_2 &p2 = bvc[(i + 1) % bvc.size()];

            const Eigen::Vector3d v1(CGAL::to_double(site.x() - p1.x()), CGAL::to_double(site.y() - p1.y()), 0.0);
            const Eigen::Vector3d v2(CGAL::to_double(p2.x() - p1.x()), CGAL::to_double(p2.y() - p1.y()), 0.0);
            const double distance_between_edge_and_site = std::fabs(v1.cross(v2).z()) / v2.norm();

            const double sign = bvc.is_clockwise_oriented() ? 1.0 : -1.0;
            const double dx = sign * CGAL::to_double(p2.x() - p1.x());
            const double dy = sign * CGAL::to_double(p2.y() - p1.y());
            const Eigen::Vector2d unitOrthogonal = Eigen::Vector2d(dx, dy).unitOrthogonal();

            const Eigen::Vector2d vec_self_to_neighbor = 2 * (distance_between_edge_and_site + _agent.radius()) * unitOrthogonal;
            const Point_2 estimated_neighbor_site(site.x() + vec_self_to_neighbor.x(), site.y() + vec_self_to_neighbor.y());

            Locate_result neighbor_lr = _vd.locate(estimated_neighbor_site);
            Face_handle *neighbor_fh = boost::get<Face_handle>(&neighbor_lr);

            Point_2 neighbor_site;
            if (neighbor_fh)
                neighbor_site = (*neighbor_fh)->dual()->point();
            else
                continue;

            if (std::pow(CGAL::to_double(neighbor_site.x() - estimated_neighbor_site.x()), 2) +
                    std::pow(CGAL::to_double(neighbor_site.y() - estimated_neighbor_site.y()), 2) >
                1e-1)
            {
                neighbor_site = site;
            }

            CGAL::Polygon_2<Kernel> triangular_subpolygon;

            triangular_subpolygon.push_back(site);
            triangular_subpolygon.push_back(p1);
            triangular_subpolygon.push_back(p2);

            raw_candidates.push_back(std::make_pair(neighbor_site, triangular_subpolygon));
        }

        sort_raw_candidates(raw_candidates, _agent);

        return raw_candidates;
    }

    std::list<CGAL::Polygon_2<Kernel>> CandidatesUtil::get_truncated_polygon(const CGAL::Polygon_2<Kernel> &_polygon, const std::vector<Agent::Cone> &_cones)
    {
        std::list<CGAL::Polygon_2<Kernel>> polygon_list;
        polygon_list.push_back(_polygon);

        for (const auto &cone : _cones)
        {
            CGAL::Polygon_2<Kernel> cone_polygon;
            cone_polygon.push_back(Point_2(cone.point_.x(), cone.point_.y()));

            double right_angle = std::atan2(cone.right_direction_.y(), cone.right_direction_.x());
            double left_angle = std::atan2(cone.left_direction_.y(), cone.left_direction_.x());
            left_angle = right_angle < left_angle ? left_angle : left_angle + 2 * M_PI;

            for (double angle = right_angle - 1e-8; angle < left_angle; angle = angle + M_PI / 18)
            {
                cone_polygon.push_back(Point_2(cone.point_.x() + 1000 * std::cos(angle),
                                               cone.point_.y() + 1000 * std::sin(angle)));
            }
            cone_polygon.push_back(Point_2(cone.point_.x() + 1000 * std::cos(left_angle + 1e-8),
                                           cone.point_.y() + 1000 * std::sin(left_angle + 1e-8)));

            std::list<CGAL::Polygon_2<Kernel>> new_polygon_list;
            for (const auto &poly : polygon_list)
            {
                std::list<CGAL::Polygon_with_holes_2<Kernel>> truncated_poly_w_holes;
                CGAL::difference(poly, cone_polygon, std::back_inserter(truncated_poly_w_holes));

                for (const auto &poly_w_holes : truncated_poly_w_holes)
                    new_polygon_list.push_back(poly_w_holes.outer_boundary());
            }
            polygon_list = new_polygon_list;
        }

        return polygon_list;
    }

    void CandidatesUtil::sort_raw_candidates(std::list<CandidatesUtil::RawCandidate> &_rawCandidates, const Agent &_agent)
    {
        _rawCandidates.sort(
            [_agent](const CandidatesUtil::RawCandidate &_c1, const CandidatesUtil::RawCandidate &_c2)
            {
                const Eigen::Vector3d goal_vec(_agent.goal().x() - _agent.pose().x(), _agent.goal().y() - _agent.pose().y(), 0);

                const std::vector<RawCandidate> rawCandidateGroup = {_c1, _c2};
                std::vector<double> closest_leg_dots;

                for (size_t i = 0; i < rawCandidateGroup.size(); ++i)
                {
                    std::list<Eigen::Vector3d> legs;
                    for (const auto &p : rawCandidateGroup[i].second.container())
                    {
                        if (std::fabs(CGAL::to_double(p.x()) - _agent.pose().x()) < 1e-8 and
                            std::fabs(CGAL::to_double(p.y()) - _agent.pose().y()) < 1e-8)
                        {
                            continue;
                        }

                        legs.push_back(
                            Eigen::Vector3d(CGAL::to_double(p.x()) - _agent.pose().x(), CGAL::to_double(p.y()) - _agent.pose().y(), 0));
                    }

                    if (legs.front().cross(goal_vec).z() * legs.front().cross(legs.back()).z() > 0 and
                        legs.front().cross(goal_vec).z() * legs.back().cross(goal_vec).z() < 0)
                        return i == 0;

                    closest_leg_dots.push_back(std::max(
                        legs.front().dot(goal_vec) / legs.front().norm(),
                        legs.back().dot(goal_vec) / legs.back().norm()));
                }

                return closest_leg_dots.front() > closest_leg_dots.back();
            });
    }
} // namespace SubgoalGenerator::PIBT