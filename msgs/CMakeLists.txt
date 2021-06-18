find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(msgs
  Thrust.proto
)

PROTOBUF_GENERATE_CPP(ERCR_PROTO_SRCS ERCR_PROTO_HDRS ${msgs})
add_library(ercr_msgs SHARED ${ERCR_PROTO_SRCS})
target_link_libraries(ercr_msgs ${PROTOBUF_LIBRARY})      
