# GOP-Aware Dropping Explained

## The Relationship Between GOPs and Frame Rate

### Basic Formula
```
Output FPS = (GOPs accepted per second) × (frames per GOP)
```

### Visual Example with 30fps Camera

#### Scenario 1: Keyframe interval = 0.1s (10 GOPs/second)
```
Camera output: 30 fps
┌─────────────────────────────────────────────────────────┐
│  GOP#1   GOP#2   GOP#3   GOP#4   GOP#5   GOP#6   ...   │
│  I--P-P  I--P-P  I--P-P  I--P-P  I--P-P  I--P-P   ...   │
│  (3 fr)  (3 fr)  (3 fr)  (3 fr)  (3 fr)  (3 fr)   ...   │
└─────────────────────────────────────────────────────────┘
    0.1s     0.2s     0.3s     0.4s     0.5s     0.6s

Each GOP = 30fps / 10 GOPs/s = 3 frames
```

**Output options:**
- Accept **ALL** GOPs → 10 GOPs/s × 3 frames = **30 fps** (original)
- Accept **every 2nd** GOP → 5 GOPs/s × 3 frames = **15 fps**
- Accept **every 3rd** GOP → 3.33 GOPs/s × 3 frames = **10 fps** ✅ **RECOMMENDED**
- Accept **every 5th** GOP → 2 GOPs/s × 3 frames = **6 fps**

#### Scenario 2: Keyframe interval = 0.2s (5 GOPs/second)
```
Camera output: 30 fps
┌───────────────────────────────────────────────────┐
│     GOP#1         GOP#2         GOP#3        ...  │
│  I--P-P-P-P-P  I--P-P-P-P-P  I--P-P-P-P-P   ...  │
│    (6 frames)    (6 frames)    (6 frames)   ...  │
└───────────────────────────────────────────────────┘
       0.2s           0.4s           0.6s

Each GOP = 30fps / 5 GOPs/s = 6 frames
```

**Output options:**
- Accept **ALL** GOPs → 5 GOPs/s × 6 frames = **30 fps** (original)
- Accept **every 2nd** GOP → 2.5 GOPs/s × 6 frames = **15 fps**
- Accept **every 3rd** GOP → 1.67 GOPs/s × 6 frames = **10 fps**
- Accept **every 5th** GOP → 1 GOP/s × 6 frames = **6 fps**

---

## Why GOP Dropping Solves Your Lag Problem

### The Lag Accumulation Issue

```
Camera → [RTP Network] → [Decoder] → [Display]
  30fps      ???           ???          ???

Problem: If any component can't keep up with 30fps:
┌─────────────────────────────────────────┐
│  Frame Buffer (growing over time)      │
│  ┌────────────────────────────────┐    │
│  │ F1 F2 F3 F4 F5 F6 F7 F8 F9 ... │ ← Frames accumulating
│  └────────────────────────────────┘    │
│       ↑                                 │
│    Lag increases (delay between         │
│    capture and display grows)           │
└─────────────────────────────────────────┘
```

### The Solution: Match Input Rate to Pipeline Capacity

```
Camera → [GOP Drop] → [RTP Network] → [Decoder] → [Display]
  30fps     10fps         10fps          10fps       10fps

Result: All components keep up, buffer stays small, NO LAG!
```

---

## Current Configuration (RECOMMENDED)

### Settings
```cpp
auto_keyframe_interval = 0.1s    // 10 GOPs per second
direct_rtp.fps = 10.0            // Target 10fps output
direct_rtp.iframes_only = false  // Keep P-frames (no artifacts)
```

### How It Works
1. **Camera encodes**: 30fps → 10 GOPs/s, each GOP has 3 frames (I+P+P)
2. **GOP counter**: Tracks received GOPs (1, 2, 3, 4, 5, ...)
3. **Modulo dropping**: Accept GOP if `(gop_number % 3 == 0)`
   - GOP #1: 1 % 3 = 1 → ❌ Skip (drop all 3 frames)
   - GOP #2: 2 % 3 = 2 → ❌ Skip (drop all 3 frames)
   - GOP #3: 3 % 3 = 0 → ✅ **Accept** (send all 3 frames intact)
   - GOP #4: 4 % 3 = 1 → ❌ Skip
   - GOP #5: 5 % 3 = 2 → ❌ Skip
   - GOP #6: 6 % 3 = 0 → ✅ **Accept**
4. **Output**: 3.33 GOPs/s × 3 frames = **10 fps**
5. **Bandwidth**: ~300-500 Kbps (67% reduction from 30fps)

### Benefits
- ✅ **No artifacts**: Entire GOPs kept intact, P-frames preserved
- ✅ **Smooth playback**: All frames in accepted GOPs play correctly
- ✅ **No lag**: Output rate (10fps) matches processing capacity
- ✅ **Low bandwidth**: Only ~300-500 Kbps vs 2-4 Mbps at 30fps
- ✅ **Real-time**: Video displays at capture speed (no delay accumulation)

---

## Tuning Options

### If 10fps is too slow:
```cpp
auto_keyframe_interval = 0.1s
direct_rtp.fps = 15.0      // Accept every 2nd GOP → 15fps
```

### If 10fps is still causing lag (very slow network):
```cpp
auto_keyframe_interval = 0.1s
direct_rtp.fps = 6.0       // Accept every 5th GOP → 6fps
```

### For maximum quality (if network can handle it):
```cpp
auto_keyframe_interval = 0.067s  // 15 GOPs/s
direct_rtp.fps = 15.0            // Accept all GOPs → 15fps
```

---

## Expected Telemetry Output

```
📊 RTP Stats [5s]: Received=30.0 fps (50 GOPs), Pushed=10.0 fps (17 GOPs), Drop=66.7%
```

**What this means:**
- **Received**: Camera sent 30fps (50 GOPs in 5 seconds)
- **Pushed**: We sent 10fps (17 GOPs in 5 seconds = every 3rd GOP)
- **Drop**: 67% of frames dropped (2 out of every 3 GOPs)

---

## Troubleshooting

### Still seeing lag?
1. **Reduce FPS further**: Try 6fps or 8fps
2. **Check network**: Use `iftop` or `nload` to monitor bandwidth
3. **Check decoder**: Verify receiver can decode 10fps smoothly

### Video stuttering?
1. **Increase FPS**: Try 12fps or 15fps
2. **Check GOP size**: Ensure keyframe_interval matches (0.1s for smooth 10fps)

### Bandwidth too high?
1. **Reduce FPS**: Lower target FPS drops more GOPs
2. **Reduce resolution**: If DJI API allows (check camera settings)

