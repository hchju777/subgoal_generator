#pragma once

#include <memory>
#include <map>

#include <Eigen/Core>

#include "subgoal_generator/pose.h"

namespace SubgoalGenerator
{
    class Agent
    {
    public:
        typedef std::unique_ptr<Agent> UniquePtr;
        typedef std::shared_ptr<Agent> SharedPtr;

    public:
        typedef std::map<double, Agent *, std::greater<double>> Neighbors;

        struct Cone
        {
            Cone() {}

            Cone(const Cone &_cone)
            {
                neighbor_ = _cone.neighbor_;
                point_ = _cone.point_;
                radius_ = _cone.radius_;
                left_direction_ = _cone.left_direction_;
                right_direction_ = _cone.right_direction_;
            }

            Cone &operator=(const Cone &_rhs)
            {
                if (&_rhs != this)
                {
                    neighbor_ = _rhs.neighbor_;
                    point_ = _rhs.point_;
                    radius_ = _rhs.radius_;
                    left_direction_ = _rhs.left_direction_;
                    right_direction_ = _rhs.right_direction_;
                }

                return *this;
            }

            std::string neighbor_;
            Eigen::Vector2d point_;
            double radius_;
            Eigen::Vector2d left_direction_;
            Eigen::Vector2d right_direction_;
        }; // struct Cone

    public:
        Agent() {}

        Agent(const Agent &_agent);

    public:
        inline std::string &name() { return name_; }
        inline const std::string &name() const { return name_; }

        inline Pose &pose() { return pose_; }
        inline const Pose &pose() const { return pose_; }

        inline Pose &goal() { return goal_; }
        inline const Pose &goal() const { return goal_; }

        inline Pose &subgoal() { return subgoal_; }
        inline const Pose &subgoal() const { return subgoal_; }

        inline Eigen::Vector2d &velocity() { return velocity_; }
        inline const Eigen::Vector2d &velocity() const { return velocity_; }

        inline double &radius() { return radius_; }
        inline const double &radius() const { return radius_; }

        inline Neighbors &neighbors() { return neighbors_; }
        inline const Neighbors &neighbors() const { return neighbors_; }

        inline std::vector<Cone> &VOCones() { return VOCones_; }
        inline const std::vector<Cone> &VOCones() const { return VOCones_; }

        inline double &timeHorizon() { return timeHorizon_; }
        inline const double &timeHorizon() const { return timeHorizon_; }

        inline bool &subgoal_fixed() { return subgoal_fixed_; }
        inline const bool &subgoal_fixed() const { return subgoal_fixed_; }

    public:
        Agent &operator=(const Agent &_rhs);

    public:
        void init();

    public:
        friend std::ostream &operator<<(std::ostream &_os, const Agent &_agent)
        {
            _os << "[" << _agent.name_ << "]"
                << "\t Radius: " << _agent.radius_
                << "\t Pose: " << _agent.pose_
                << "\t Goal: " << _agent.goal_
                << "\t Velocity: " << _agent.velocity_.coeffRef(0) << ", " << _agent.velocity_.coeffRef(1);

            return _os;
        }

    protected:
        std::string name_;

        Pose pose_;
        Pose goal_;
        Pose subgoal_;

        Eigen::Vector2d velocity_;
        double radius_{0.0};

        Neighbors neighbors_;
        std::vector<Cone> VOCones_;

        double timeHorizon_{0.05};

        bool subgoal_fixed_{false};

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; // class Agent

    typedef std::map<std::string, Agent> Agents;

} // namespace SubgoalGenerator