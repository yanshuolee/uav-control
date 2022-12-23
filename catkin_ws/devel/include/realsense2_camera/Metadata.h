// Generated by gencpp from file realsense2_camera/Metadata.msg
// DO NOT EDIT!


#ifndef REALSENSE2_CAMERA_MESSAGE_METADATA_H
#define REALSENSE2_CAMERA_MESSAGE_METADATA_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace realsense2_camera
{
template <class ContainerAllocator>
struct Metadata_
{
  typedef Metadata_<ContainerAllocator> Type;

  Metadata_()
    : header()
    , json_data()  {
    }
  Metadata_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , json_data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _json_data_type;
  _json_data_type json_data;





  typedef boost::shared_ptr< ::realsense2_camera::Metadata_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::realsense2_camera::Metadata_<ContainerAllocator> const> ConstPtr;

}; // struct Metadata_

typedef ::realsense2_camera::Metadata_<std::allocator<void> > Metadata;

typedef boost::shared_ptr< ::realsense2_camera::Metadata > MetadataPtr;
typedef boost::shared_ptr< ::realsense2_camera::Metadata const> MetadataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::realsense2_camera::Metadata_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::realsense2_camera::Metadata_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::realsense2_camera::Metadata_<ContainerAllocator1> & lhs, const ::realsense2_camera::Metadata_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.json_data == rhs.json_data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::realsense2_camera::Metadata_<ContainerAllocator1> & lhs, const ::realsense2_camera::Metadata_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace realsense2_camera

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::realsense2_camera::Metadata_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense2_camera::Metadata_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense2_camera::Metadata_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense2_camera::Metadata_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense2_camera::Metadata_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense2_camera::Metadata_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::realsense2_camera::Metadata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4966ca002be16ee67fe4dbfb2f354787";
  }

  static const char* value(const ::realsense2_camera::Metadata_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4966ca002be16ee6ULL;
  static const uint64_t static_value2 = 0x7fe4dbfb2f354787ULL;
};

template<class ContainerAllocator>
struct DataType< ::realsense2_camera::Metadata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "realsense2_camera/Metadata";
  }

  static const char* value(const ::realsense2_camera::Metadata_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::realsense2_camera::Metadata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"string json_data\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::realsense2_camera::Metadata_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::realsense2_camera::Metadata_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.json_data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Metadata_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::realsense2_camera::Metadata_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::realsense2_camera::Metadata_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "json_data: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.json_data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REALSENSE2_CAMERA_MESSAGE_METADATA_H
