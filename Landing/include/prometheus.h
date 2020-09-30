#ifndef PROMETHEUS_H
#define PROMETHEUS_H

#include <string>
#include <vector>
#include <map>

// #include <boost/thread/mutex.hpp>
// #include <boost/thread/shared_mutex.hpp>
// #include <boost/serialization/array_wrapper.hpp>

// #include <std_msgs/Header.h>

// namespace prometheus_msgs
// {
//     template <class ContainerAllocator>

//     struct DetectionInfo_
//     {
//         typedef DetectionInfo_<ContainerAllocator> Type;

//         DetectionInfo_()
//             : //header(),
//               detected(false), frame(0), position(), attitude(), sight_angle(), yaw_error(0.0), category(0)
//         {
//             position.fill(0.0);

//             attitude.fill(0.0);

//             sight_angle.fill(0.0);
//         }
//         DetectionInfo_(const ContainerAllocator &_alloc)
//             : //header(_alloc),
//               detected(false), frame(0), position(), attitude(), sight_angle(), yaw_error(0.0), category(0)
//         {
//             (void)_alloc;
//             position.fill(0.0);

//             attitude.fill(0.0);

//             sight_angle.fill(0.0);
//         }

//         //    typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
//         //   _header_type header;

//         typedef uint8_t _detected_type;
//         _detected_type detected;

//         typedef int32_t _frame_type;
//         _frame_type frame;

//         typedef std::array<float, 3> _position_type;
//         _position_type position;

//         typedef std::array<float, 3> _attitude_type;
//         _attitude_type attitude;

//         typedef std::array<float, 2> _sight_angle_type;
//         _sight_angle_type sight_angle;

//         typedef float _yaw_error_type;
//         _yaw_error_type yaw_error;

//         typedef int32_t _category_type;
//         _category_type category;

//         typedef boost::shared_ptr<::prometheus_msgs::DetectionInfo_<ContainerAllocator>> Ptr;
//         typedef boost::shared_ptr<::prometheus_msgs::DetectionInfo_<ContainerAllocator> const> ConstPtr;

//     }; // struct DetectionInfo_
// } // namespace prometheus_msgs
// typedef ::prometheus_msgs::DetectionInfo_<std::allocator<void>> DetectionInfo;

// typedef boost::shared_ptr< ::prometheus_msgs::DetectionInfo > DetectionInfoPtr;
// typedef boost::shared_ptr< ::prometheus_msgs::DetectionInfo const> DetectionInfoConstPtr;

// constants requiring out of line definition

// template<typename ContainerAllocator1, typename ContainerAllocator2>
// bool operator==(const ::prometheus_msgs::DetectionInfo_<ContainerAllocator1> & lhs, const ::prometheus_msgs::DetectionInfo_<ContainerAllocator2> & rhs)
// {
//   return //lhs.header == rhs.header &&
//     lhs.detected == rhs.detected &&
//     lhs.frame == rhs.frame &&
//     lhs.position == rhs.position &&
//     lhs.attitude == rhs.attitude &&
//     lhs.sight_angle == rhs.sight_angle &&
//     lhs.yaw_error == rhs.yaw_error &&
//     lhs.category == rhs.category;
// }

// template<typename ContainerAllocator1, typename ContainerAllocator2>
// bool operator!=(const ::prometheus_msgs::DetectionInfo_<ContainerAllocator1> & lhs, const ::prometheus_msgs::DetectionInfo_<ContainerAllocator2> & rhs)
// {
//   return !(lhs == rhs);
// }

#endif