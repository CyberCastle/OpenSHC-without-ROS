// Generated by gencpp from file syropod_highlevel_controller/TipState.msg
// DO NOT EDIT!

#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_MESSAGE_TIPSTATE_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_MESSAGE_TIPSTATE_H

#include <memory>
#include <string>
#include <vector>

namespace syropod_highlevel_controller {
template <class ContainerAllocator>
struct TipState_ {

    typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _name_type;
    _name_type name;

    typedef std::vector<::geometry_msgs::Wrench_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<::geometry_msgs::Wrench_<ContainerAllocator>>> _wrench_type;
    _wrench_type wrench;

    typedef std::vector<::geometry_msgs::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<::geometry_msgs::Vector3_<ContainerAllocator>>> _step_plane_type;
    _step_plane_type step_plane;

}; // struct TipState_

typedef ::syropod_highlevel_controller::TipState_<std::allocator<void>> TipState;

} // namespace syropod_highlevel_controller

#endif // SYROPOD_HIGHLEVEL_CONTROLLER_MESSAGE_TIPSTATE_H
