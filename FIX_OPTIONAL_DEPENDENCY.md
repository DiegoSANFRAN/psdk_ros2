# Fix: Optional Dependency Compilation Issue

## Problem
The build was failing with:
```
fatal error: ivaq_finder_search_msgs/msg/ivaq_finder_video_streaming.hpp: No such file or directory
```

## Root Cause
The header file was trying to use a templated type (`rclcpp::Subscription<T>`) with an incomplete/missing type, which doesn't work in C++.

## Solution
Changed from strongly-typed subscription to base class pointer:

### Before (Broken):
```cpp
#ifdef HAS_IVAQ_FINDER_MSGS
  rclcpp::Subscription<ivaq_finder_search_msgs::msg::IvaqFinderVideoStreaming>::SharedPtr
      video_streaming_control_sub_;
#endif
```

### After (Fixed):
```cpp
// Use base class pointer - no template parameter needed
rclcpp::SubscriptionBase::SharedPtr video_streaming_control_sub_;
```

## Why This Works
- `rclcpp::SubscriptionBase` is the non-templated base class
- Doesn't require knowing the message type at compile time
- The actual message type is only needed in the `.cpp` file where we `#include` it
- This is a standard pattern in ROS2 for optional dependencies

## Build Command
```bash
cd ~/Development/psdk_ros2
colcon build --merge-install --packages-select psdk_wrapper --symlink-install --allow-overriding psdk_wrapper
```

## Files Modified
1. **liveview.hpp** - Changed subscription type to `SubscriptionBase::SharedPtr`
2. **liveview.cpp** - Conditional `#include` of ivaq_finder_search_msgs (already done)
3. **CMakeLists.txt** - Optional find_package check (already done)

## Result
- ✅ Compiles WITHOUT `ivaq_finder_search_msgs` installed (webapp control disabled)
- ✅ Compiles WITH `ivaq_finder_search_msgs` installed (webapp control enabled)
- ✅ No errors, no warnings about missing headers
