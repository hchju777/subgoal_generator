#pragma once

#include <list>
#include <utility>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <CGAL/Polygon_triangulation_decomposition_2.h>

#include "subgoal_generator/agent.h"
#include "subgoal_generator/bvc_generator.h"

namespace SubgoalGenerator::PIBT
{
    class CandidatesUtil
    {
    protected:
        typedef BufferedVoronoiDiagram::VoronoiCell VoronoiCell;

    public:
        typedef std::pair<Point_2, CGAL::Polygon_2<Kernel>> RawCandidate;
        typedef std::pair<std::string, std::list<CGAL::Polygon_2<Kernel>>> Candidate;

    public:
        static std::list<RawCandidate> createRawCandidates(
            const Agent &_agent, const VoronoiCell &_bvc, const VD &_vd);

        static std::list<CGAL::Polygon_2<Kernel>> get_truncated_polygon(const CGAL::Polygon_2<Kernel> &_polygon, const std::vector<Agent::Cone> &_cones);

    protected:
        static void sort_raw_candidates(std::list<RawCandidate> &_rawCandidates, const Agent &_agent);

    public:
        static std::list<CGAL::Polygon_2<Kernel>> triangular_decompose(const CGAL::Polygon_2<Kernel> &_cell);

        static std::list<CGAL::Polygon_2<Kernel>> triangular_decompose(const CGAL::Polygon_with_holes_2<Kernel> &_cell_w_holes);

    }; // class CandidatesUtil

} // namespace SubgoalGenerator::PIBT