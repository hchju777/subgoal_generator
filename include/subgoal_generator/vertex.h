#pragma once

#include <memory>
#include <iostream>

#include <subgoal_generator/pose.h>

namespace SubgoalGenerator::DynamicGraph
{
    class Vertex
    {
    public:
        Vertex() {}

        Vertex(std::string _name, const Pose &_current, const Pose &_goal)
        {
            name_ = _name;
            current_ = _current;
            goal_ = _goal;
        }

        Vertex(const Vertex &_vertex)
        {
            name_ = _vertex.name_;
            current_ = _vertex.current_;
            goal_ = _vertex.goal_;
        }

        ~Vertex() {}

    public:
        inline std::string &name() { return name_; }
        const inline std::string &name() const { return name_; }

        inline Pose &current() { return current_; }
        const inline Pose &current() const { return current_; }

        inline Pose &goal() { return goal_; }
        const inline Pose &goal() const { return goal_; }

    public:
        inline void setCurrent(const Pose &_current)
        {
            current_ = _current;
        }

        inline void setGoal(const Pose &_goal)
        {
            goal_ = _goal;
        }

        inline static double getDistance(const Vertex &_lhs, const Vertex &_rhs)
        {
            Eigen::Vector2d v = _lhs.current().position() - _rhs.current().position();

            return v.norm();
        }

    public:
        Vertex &operator=(const Vertex &_rhs)
        {
            if (&_rhs != this)
            {
                name_ = _rhs.name_;
                current_ = _rhs.current_;
                goal_ = _rhs.goal_;
            }

            return *this;
        }

        friend std::ostream &operator<<(std::ostream &_os, const Vertex &_vertex)
        {
            _os << "[" << _vertex.name_ << "]"
                << "\t Current Pose: " << _vertex.current_
                << "\t Goal Pose: " << _vertex.goal_;

            return _os;
        }

    public:
        inline bool isGoal() const
        {
            return goalDistance() < 1e-8;
        }

        inline double goalDistance() const
        {
            Eigen::Vector2d v = current_.position() - goal_.position();

            return v.norm();
        }

    protected:
        std::string name_;
        Pose current_;
        Pose goal_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; // class Vertex
} // namespace SubgoalGenerator