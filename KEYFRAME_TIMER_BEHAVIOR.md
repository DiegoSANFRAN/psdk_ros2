# Automatic Keyframe Timer Behavior

## Critical Design: Timer Only Runs During Active Streaming

The automatic keyframe request timer is **stream-aware** and only runs when the camera is actively streaming data.

## Why This Matters

❌ **Bad Design (previous):**
- Timer starts when node activates
- Requests keyframes even when no stream is running
- Wastes resources
- Can cause errors if camera isn't ready

✅ **Good Design (current):**
- Timer starts **only** when streaming begins
- Stops automatically when streaming ends
- No wasted keyframe requests
- No errors from requesting frames when camera is off

## Lifecycle Flow

```
┌─────────────────────────────────────────────────────────┐
│ Node Startup                                             │
├─────────────────────────────────────────────────────────┤
│ 1. on_configure()                                        │
│    ├─ Read auto_keyframe_interval parameter             │
│    ├─ auto_keyframe_enabled_ = (interval > 0)           │
│    └─ Log: "Automatic keyframe requests configured"     │
│                                                          │
│ 2. on_activate()                                         │
│    ├─ Activate publishers                               │
│    ├─ DO NOT START TIMER YET ❌                         │
│    └─ Log: "will start when streaming begins"           │
└─────────────────────────────────────────────────────────┘
                        ↓
        ⏱️  Waiting for streaming request...
                        ↓
┌─────────────────────────────────────────────────────────┐
│ Streaming Starts                                         │
├─────────────────────────────────────────────────────────┤
│ 3. camera_setup_streaming(start_stop=true) called       │
│    ├─ start_camera_stream() → DjiLiveview_StartH264...()│
│    ├─ is_streaming_active_ = true ✅                    │
│    ├─ IF auto_keyframe_enabled_:                        │
│    │   ├─ Create timer ✅                               │
│    │   └─ Log: "Started automatic keyframe timer"       │
│    └─ Timer now runs every keyframe_request_interval    │
└─────────────────────────────────────────────────────────┘
                        ↓
        🔄 Timer ticks every 0.25s (for 4Hz)
                        ↓
┌─────────────────────────────────────────────────────────┐
│ Timer Callback (running)                                 │
├─────────────────────────────────────────────────────────┤
│ 4. auto_request_keyframe_callback()                     │
│    ├─ Check is_module_initialized_ ✅                   │
│    ├─ Check is_streaming_active_ ✅                     │
│    ├─ DjiLiveview_RequestIntraframeFrameData()          │
│    └─ Log (debug): "Auto keyframe requested"            │
└─────────────────────────────────────────────────────────┘
                        ↓
        🔄 Continues until streaming stops
                        ↓
┌─────────────────────────────────────────────────────────┐
│ Streaming Stops                                          │
├─────────────────────────────────────────────────────────┤
│ 5. camera_setup_streaming(start_stop=false) called      │
│    ├─ Cancel timer ✅                                   │
│    ├─ timer_.reset() ✅                                 │
│    ├─ is_streaming_active_ = false ✅                   │
│    ├─ stop_main_camera_stream() → DjiLiveview_Stop...() │
│    └─ Log: "Stopped automatic keyframe timer"           │
└─────────────────────────────────────────────────────────┘
                        ↓
        ⏱️  Timer is destroyed, no more requests
```

## Code Flow Example

### Scenario: FPV Camera at 4Hz

```cpp
// 1. Node configures
auto_keyframe_interval_ = 0.25;  // From ROS2 parameter
auto_keyframe_enabled_ = true;   // Since interval > 0
is_streaming_active_ = false;    // Not streaming yet

// 2. Node activates - timer NOT started
RCLCPP_INFO("Automatic keyframe requests configured at 4.00 Hz - will start when streaming begins");

// 3. camera_to_rtp_bridge_node calls camera_setup_streaming service
Request: payload_index=7 (FPV), camera_source=0, start_stop=true

// 4. camera_setup_streaming_cb executes
start_camera_stream(...);  // Start actual DJI camera stream
if (success) {
    is_streaming_active_ = true;  ✅
    
    if (auto_keyframe_enabled_ && !keyframe_request_timer_) {
        // CREATE TIMER HERE ✅
        keyframe_request_timer_ = create_wall_timer(
            std::chrono::duration<double>(0.25),
            &auto_request_keyframe_callback
        );
        RCLCPP_INFO("🎯 Started automatic keyframe request timer: 4.00 Hz");
    }
}

// 5. Timer fires every 250ms
void auto_request_keyframe_callback() {
    if (!is_module_initialized_ || !is_streaming_active_) {
        return;  // Safety check
    }
    
    DjiLiveview_RequestIntraframeFrameData(
        DJI_LIVEVIEW_CAMERA_POSITION_FPV,  // payload_index_
        DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT  // selected_camera_source_
    );
    // Drone generates I-frame
}

// 6. Later, camera_to_rtp_bridge_node stops streaming
Request: payload_index=7, camera_source=0, start_stop=false

// 7. camera_setup_streaming_cb executes (stop)
if (keyframe_request_timer_) {
    keyframe_request_timer_->cancel();  ✅
    keyframe_request_timer_.reset();    ✅
    RCLCPP_INFO("Stopped automatic keyframe request timer");
}
is_streaming_active_ = false;  ✅
stop_main_camera_stream(...);
```

## State Checks in Timer Callback

The timer callback has **two safety checks** before requesting a keyframe:

```cpp
void auto_request_keyframe_callback() {
    // Check 1: Module initialized (DjiLiveview_Init() called)
    if (!is_module_initialized_) {
        return;  // Don't request if DJI SDK not ready
    }
    
    // Check 2: Streaming active (camera actually running)
    if (!is_streaming_active_) {
        return;  // Don't request if no stream running
    }
    
    // Both checks passed - safe to request keyframe
    DjiLiveview_RequestIntraframeFrameData(payload_index_, selected_camera_source_);
}
```

This prevents errors like:
- Requesting keyframes before DJI SDK is initialized
- Requesting keyframes when camera is off
- Requesting keyframes for wrong payload_index

## Comparison: Before vs After

### Before (Incorrect)

```
Node Activate → START TIMER ❌
                    ↓
    Timer runs before streaming starts ❌
    Wastes CPU requesting keyframes ❌
    May cause DJI API errors ❌
                    ↓
Streaming Starts → Camera already getting requests
                    ↓
Streaming Stops → Timer keeps running ❌
                    ↓
    Continues wasting resources ❌
```

### After (Correct)

```
Node Activate → Timer configured but not started ✅
                    ↓
    No CPU waste, no errors ✅
                    ↓
Streaming Starts → START TIMER NOW ✅
                    ↓
    Timer runs only when needed ✅
    Requests go to active camera ✅
                    ↓
Streaming Stops → STOP TIMER ✅
                    ↓
    No wasted resources ✅
```

## Benefits of Stream-Aware Timer

1. **Resource Efficiency**
   - No keyframe requests when camera is off
   - Timer only allocates resources when needed
   - CPU cycles saved

2. **Error Prevention**
   - No API errors from requesting frames on inactive camera
   - payload_index and camera_source always valid when timer fires
   - DJI SDK always ready when requests are made

3. **Logical Consistency**
   - Keyframes only requested when stream exists to benefit from them
   - Timer lifecycle matches stream lifecycle
   - Cleaner state management

4. **Debug Clarity**
   - Easy to see when timer starts/stops in logs
   - No confusing "why is timer running?" scenarios
   - Clear correlation between streaming and keyframes

## Testing Verification

### Test 1: Timer doesn't start prematurely

```bash
# Start psdk_wrapper with auto_keyframe_interval=0.25
ros2 run psdk_wrapper psdk_wrapper_node --ros-args -p auto_keyframe_interval:=0.25

# Expected logs:
# ✅ "Automatic keyframe requests configured at 4.00 Hz - will start when streaming begins"
# ❌ NOT: "Started automatic keyframe request timer" (not yet!)

# Verify timer not running
ros2 topic echo /wrapper/psdk_ros2/fpv_camera_stream
# Should be: No data (no streaming yet)
```

### Test 2: Timer starts with streaming

```bash
# Start streaming via service
ros2 service call /wrapper/psdk_ros2/camera_setup_streaming \
  psdk_interfaces/srv/CameraSetupStreaming \
  "{payload_index: 7, camera_source: 0, start_stop: true, decoded_output: false}"

# Expected logs:
# ✅ "Starting streaming..."
# ✅ "🎯 Started automatic keyframe request timer: 4.00 Hz (every 0.250 seconds)"
# ✅ (with debug) "Auto keyframe requested for payload=7, source=0"

# Verify stream is running
ros2 topic hz /wrapper/psdk_ros2/fpv_camera_stream
# Should show: average rate: ~30.000
```

### Test 3: Timer stops with streaming

```bash
# Stop streaming via service
ros2 service call /wrapper/psdk_ros2/camera_setup_streaming \
  psdk_interfaces/srv/CameraSetupStreaming \
  "{payload_index: 7, camera_source: 0, start_stop: false, decoded_output: false}"

# Expected logs:
# ✅ "Stopping camera streaming..."
# ✅ "Stopped automatic keyframe request timer"

# Verify stream stopped
ros2 topic hz /wrapper/psdk_ros2/fpv_camera_stream
# Should show: no new messages
```

## Integration with camera_to_rtp_bridge_node

The Python node now benefits from automatic keyframe generation:

```python
# camera_to_rtp_bridge_node.py

# 1. Start camera streaming (triggers timer start in C++)
start_req = CameraSetupStreaming.Request()
start_req.payload_index = 7  # FPV
start_req.camera_source = 0
start_req.start_stop = True
start_req.decoded_output = False
self.setup_client.call_async(start_req)
# → C++ timer starts here ✅

# 2. Receive frames with I-frames at 4Hz (thanks to C++ timer)
def on_image(self, msg: Image):
    # Filter and forward frames
    # I-frames arrive at 4Hz automatically ✅
    pass

# 3. Stop camera streaming (triggers timer stop in C++)
stop_req = CameraSetupStreaming.Request()
stop_req.start_stop = False
self.setup_client.call_async(stop_req)
# → C++ timer stops here ✅
```

No timer management needed in Python! The C++ module handles everything based on streaming state.

## Summary

✅ **Timer lifecycle tied to streaming lifecycle**
✅ **No wasted keyframe requests**
✅ **No errors from inactive cameras**
✅ **Automatic start/stop with streaming**
✅ **Clean state management**
✅ **Debug-friendly logging**

The automatic keyframe timer is now **stream-aware** and only operates when the camera is actively streaming, making it both efficient and error-free! 🎯
