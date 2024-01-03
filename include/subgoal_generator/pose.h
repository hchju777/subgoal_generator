#pragma once

#include <Eigen/Core>

namespace SubgoalGenerator
{
    class Pose
    {
    public:
        Pose()
        {
            setZero();
        }

        Pose(const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta)
        {
            position_ = _position;
            theta_ = _theta;
        }

        Pose(double _x, double _y, double _theta)
        {
            position_.coeffRef(0) = _x;
            position_.coeffRef(1) = _y;
            theta_ = _theta;
        }

        Pose(const Pose &_pose)
        {
            position_ = _pose.position_;
            theta_ = _pose.theta_;
        }

        ~Pose() {}

    public:
        inline Eigen::Vector2d &position() { return position_; }
        inline const Eigen::Vector2d &position() const { return position_; }

        inline double &x() { return position_.coeffRef(0); }
        const inline double &x() const { return position_.coeffRef(0); }

        inline double &y() { return position_.coeffRef(1); }
        const inline double &y() const { return position_.coeffRef(1); }

        inline double &theta() { return theta_; }
        const inline double &theta() const { return theta_; }

    public:
        inline void setZero()
        {
            position_.setZero();
            theta_ = 0.0;
        }

        inline Eigen::Vector2d orientationUnitVec() const
        {
            return Eigen::Vector2d(std::cos(theta_), std::sin(theta_));
        }

        static double normalize_theta(double _theta)
        {
            const double result = std::fmod(_theta + M_PI, 2.0 * M_PI);

            if (result <= 0.0)
                return result + M_PI;

            return result - M_PI;
        }

    public:
        bool operator==(const Pose &_rhs)
        {
            return (position_ == _rhs.position_ and std::fabs(theta_ - _rhs.theta_) < 1e-8);
        }

        Pose &operator=(const Pose &_rhs)
        {
            if (&_rhs != this)
            {
                position_ = _rhs.position_;
                theta_ = _rhs.theta_;
            }

            return *this;
        }

        Pose &operator+=(const Pose &_rhs)
        {
            position_ += _rhs.position_;
            theta_ = Pose::normalize_theta(theta_ + _rhs.theta_);

            return *this;
        }

        friend Pose operator+(Pose _lhs, const Pose &_rhs)
        {
            return _lhs += _rhs;
        }

        Pose &operator-=(const Pose &_rhs)
        {
            position_ -= _rhs.position_;
            theta_ = Pose::normalize_theta(theta_ - _rhs.theta_);

            return *this;
        }

        friend Pose operator-(Pose _lhs, const Pose &_rhs)
        {
            return _lhs -= _rhs;
        }

        friend Pose operator*(Pose _pose, double _scaler)
        {
            _pose.position_ *= _scaler;
            _pose.theta_ = Pose::normalize_theta(_pose.theta_ * _scaler);

            return _pose;
        }

        friend Pose operator*(double _scaler, Pose _pose)
        {
            _pose.position_ *= _scaler;
            _pose.theta_ = Pose::normalize_theta(_pose.theta_ * _scaler);

            return _pose;
        }

        friend std::ostream &operator<<(std::ostream &_os, const Pose &_pose)
        {
            _os << "x: " << _pose.position_[0] << " y: " << _pose.position_[1] << " theta: " << _pose.theta_;

            return _os;
        }

    protected:
        Eigen::Vector2d position_;
        double theta_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }; // class Position
} // namespace SubgoalGenerator
