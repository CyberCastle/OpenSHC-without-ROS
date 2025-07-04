/*!
 * @file Pose.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _GEOMETRY_MSGS_POSE_HPP_
#define _GEOMETRY_MSGS_POSE_HPP_

namespace geometry_msgs {

class Vector3 {
  public:
    double x;
    double y;
    double z;
};

class Quaternion {
  public:
    double x;
    double y;
    double z;
    double w;
};

class Twist {
  public:
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
};

class Point {
  public:
    double x;
    double y;
    double z;
};

class Pose {
  public:
    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;
};

class Transform {
  public:
    geometry_msgs::Vector3 translation;
    geometry_msgs::Quaternion rotation;
};

class PoseStamped {
  public:
    geometry_msgs::Pose pose;
};

class TransformStamped {
  public:
    char child_frame_id[64];
    geometry_msgs::Transform transform;
};

template <class ContainerAllocator>
struct Point_ {
    typedef Point_<ContainerAllocator> Type;

    Point_()
        : x(0.0), y(0.0), z(0.0) {
    }

    Point_(const ContainerAllocator &_alloc)
        : x(0.0), y(0.0), z(0.0) {
    }

    typedef double _x_type;
    double x;

    typedef double _y_type;
    double y;

    typedef double _z_type;
    double z;
};

template <class ContainerAllocator>
struct Quaternion_ {
    typedef Quaternion_<ContainerAllocator> Type;

    Quaternion_()
        : x(0.0), y(0.0), z(0.0), w(0.0) {
    }

    Quaternion_(const ContainerAllocator &_alloc)
        : x(0.0), y(0.0), z(0.0), w(0.0) {
    }

    typedef double _x_type;
    double x;

    typedef double _y_type;
    double y;

    typedef double _z_type;
    double z;

    typedef double _w_type;
    double w;
};

template <class ContainerAllocator>
struct Vector3_ {
    typedef Vector3_<ContainerAllocator> Type;

    Vector3_()
        : x(0.0), y(0.0), z(0.0) {
    }

    Vector3_(const ContainerAllocator &_alloc)
        : x(0.0), y(0.0), z(0.0) {
    }

    typedef double _x_type;
    double x;

    typedef double _y_type;
    double y;

    typedef double _z_type;
    double z;
};

template <class ContainerAllocator>
struct Pose_ {
    typedef Pose_<ContainerAllocator> Type;

    Pose_()
        : position(), orientation() {
    }

    Pose_(const ContainerAllocator &_alloc)
        : position(_alloc), orientation(_alloc) {
    }

    typedef ::geometry_msgs::Point_<ContainerAllocator> _position_type;
    ::geometry_msgs::Point_<ContainerAllocator> position;

    typedef ::geometry_msgs::Quaternion_<ContainerAllocator> _orientation_type;
    ::geometry_msgs::Quaternion_<ContainerAllocator> orientation;
};

template <class ContainerAllocator>
struct PoseStamped_ {
    typedef PoseStamped_<ContainerAllocator> Type;

    PoseStamped_()
        : pose() {
    }

    PoseStamped_(const ContainerAllocator &_alloc)
        : pose(_alloc) {
    }

    typedef ::geometry_msgs::Pose_<ContainerAllocator> _pose_type;
    ::geometry_msgs::Pose_<ContainerAllocator> pose;
};

template <class ContainerAllocator>
struct Wrench_ {
    typedef Wrench_<ContainerAllocator> Type;

    Wrench_()
        : force(), torque() {
    }

    Wrench_(const ContainerAllocator &_alloc)
        : force(_alloc), torque(_alloc) {
    }

    typedef ::geometry_msgs::Vector3_<ContainerAllocator> _force_type;
    ::geometry_msgs::Vector3_<ContainerAllocator> force;

    typedef ::geometry_msgs::Vector3_<ContainerAllocator> _torque_type;
    ::geometry_msgs::Vector3_<ContainerAllocator> torque;
};

template <class ContainerAllocator>
struct Twist_ {
    typedef Twist_<ContainerAllocator> Type;

    Twist_()
        : linear(), angular() {
    }

    Twist_(const ContainerAllocator &_alloc)
        : linear(_alloc), angular(_alloc) {
    }

    typedef ::geometry_msgs::Vector3_<ContainerAllocator> _linear_type;
    ::geometry_msgs::Vector3_<ContainerAllocator> linear;

    typedef ::geometry_msgs::Vector3_<ContainerAllocator> _angular_type;
    ::geometry_msgs::Vector3_<ContainerAllocator> angular;
};

template <class ContainerAllocator>
struct TwistStamped_ {
    typedef TwistStamped_<ContainerAllocator> Type;

    TwistStamped_() : twist() {
    }

    TwistStamped_(const ContainerAllocator &_alloc) : twist(_alloc) {
    }

    typedef ::geometry_msgs::Twist_<ContainerAllocator> _twist_type;
    ::geometry_msgs::Twist_<ContainerAllocator> twist;
};

} // namespace geometry_msgs

#endif // _GEOMETRY_MSGS_POSE_HPP_
