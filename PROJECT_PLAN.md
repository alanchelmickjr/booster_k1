# K-1 Robot Project Plan
**Date**: 2025-11-17  
**Status**: Camera pipeline working (received 1 frame before crash)  
**Goal**: Complete testing and make K-1 fully interactive using AI

---

## 🎯 Project Objectives

1. **Stabilize Camera Pipeline**: Fix the crash after first frame
2. **Complete Testing Suite**: Ensure all components work reliably
3. **Integrate AI Features**: Make K-1 interactive and intelligent
4. **Production Ready**: Deploy stable, maintainable system

---

## 📊 Current Status

### ✅ Completed
- [x] Camera feed diagnostics and root cause analysis
- [x] Fixed camera topic routing (`/image_left_raw` vs `/booster_camera_bridge/image_left_raw`)
- [x] Consolidated camera code into [`CameraUtils`](src/camera_utils.py) module
- [x] Updated all camera scripts to use common utilities
- [x] Documentation updated ([`CAMERA_FIX_SUMMARY.md`](CAMERA_FIX_SUMMARY.md), [`K1_TEST_RESULTS.md`](K1_TEST_RESULTS.md))

### 🔄 In Progress
- [ ] Camera pipeline stability (crashes after 1 frame)
- [ ] Complete system testing
- [ ] AI integration for interactivity

### ⏳ Not Started
- [ ] Docker containerization analysis
- [ ] Production deployment strategy
- [ ] Performance optimization for Jetson Orin NX 8GB

---

## 🚨 Critical Issues

### 1. Camera Crash After First Frame
**Priority**: 🔴 CRITICAL  
**Description**: Pipeline receives one frame successfully then crashes  
**Impact**: Blocks all vision-based features

**Investigation Steps**:
1. Add comprehensive error handling and logging to camera callbacks
2. Check memory leaks in image conversion (NV12 to BGR)
3. Verify ROS2 QoS settings match publisher
4. Test with frame rate limiting
5. Monitor Jetson resource usage during operation

**Files to Review**:
- [`src/camera_utils.py`](src/camera_utils.py:26-37) - `convert_nv12_to_bgr()` method
- [`src/basic_cam.py`](src/basic_cam.py:55-79) - callback implementation
- [`src/came_yolo.py`](src/came_yolo.py:205-250) - YOLO detection callback

---

## 🗺️ Implementation Roadmap

### Phase 1: Stabilize Core Systems (1-2 days)

#### 1.1 Fix Camera Stability
- [ ] Add error boundaries in all camera callbacks
- [ ] Implement graceful degradation on frame processing errors
- [ ] Add memory usage monitoring
- [ ] Test with reduced frame rates (frame skipping)
- [ ] Verify no resource leaks

**Success Criteria**: Camera runs continuously for 5+ minutes without crashes

#### 1.2 Improve Error Handling
- [ ] Add try-catch blocks to all critical paths
- [ ] Implement logging at key checkpoints
- [ ] Add system health monitoring
- [ ] Create automatic recovery mechanisms

### Phase 2: Complete Testing (2-3 days)

#### 2.1 Unit Tests
- [ ] Test `CameraUtils` image conversion
- [ ] Test ROS2 subscription handling
- [ ] Test face recognition module
- [ ] Test YOLO detection
- [ ] Test TTS integration
- [ ] Test voice listener

#### 2.2 Integration Tests
- [ ] Test camera → YOLO pipeline
- [ ] Test camera → face recognition pipeline  
- [ ] Test smart recognition (YOLO + faces + TTS + voice)
- [ ] Test robot control integration
- [ ] Test emergency stop systems

#### 2.3 System Tests
- [ ] Long-running stability test (8+ hours)
- [ ] Stress test with multiple detections
- [ ] Network resilience test
- [ ] Recovery from errors test

**Success Criteria**: All tests pass with 99%+ reliability

### Phase 3: AI Integration (3-4 days)

#### 3.1 Enhance Recognition System
- [ ] Improve conversation memory (track context)
- [ ] Add natural language understanding
- [ ] Implement task planning capabilities
- [ ] Add learning from interactions

#### 3.2 Behavioral Intelligence
- [ ] Implement attention mechanisms (look at people)
- [ ] Add proactive engagement (greet people approaching)
- [ ] Create context-aware responses
- [ ] Develop personality traits

#### 3.3 Navigation Integration
- [ ] Connect recognition to movement
- [ ] Implement follow-person behavior
- [ ] Add obstacle avoidance with vision
- [ ] Create autonomous exploration

**Success Criteria**: K-1 can have natural conversations and respond contextually

### Phase 4: Optimization & Deployment (2-3 days)

#### 4.1 Performance Optimization
- [ ] Profile CPU/GPU usage on Jetson Orin NX
- [ ] Optimize YOLO inference (TensorRT?)
- [ ] Reduce memory footprint
- [ ] Implement adaptive quality settings

#### 4.2 Docker Analysis
**Question**: Why don't people use Docker on Orin NX 8GB?

**Investigation**:
- [ ] Test Docker overhead on Jetson
- [ ] Measure performance impact
- [ ] Compare bare metal vs containerized
- [ ] Evaluate benefits (reproducibility, isolation) vs costs (performance, memory)

**Hypothesis**: Docker might be too slow due to:
- Additional memory overhead on limited 8GB RAM
- GPU passthrough complexity
- Real-time performance requirements
- Native CUDA/TensorRT integration simpler without containers

**Recommendation**: Consider Docker for development/testing but deploy natively

#### 4.3 Production Deployment
- [ ] Create deployment scripts
- [ ] Set up systemd services for autostart
- [ ] Implement logging and monitoring
- [ ] Create backup/recovery procedures
- [ ] Document deployment process

**Success Criteria**: One-command deployment, auto-recovery, production monitoring

---

## 📁 Code Organization

### Current Structure
```
booster_k1/
├── src/
│   ├── camera_utils.py         # ✅ Common camera utilities
│   ├── basic_cam.py            # ✅ Updated to use CameraUtils
│   ├── cam_face_recognition.py # ✅ Updated to use CameraUtils
│   ├── smart_recognition.py    # ✅ Updated to use CameraUtils  
│   ├── came_yolo.py            # ✅ Updated to use CameraUtils
│   ├── basic_controls.py       # Robot movement control
│   ├── face_recognition.py     # Face recognition module
│   ├── tts_module.py           # Text-to-speech
│   └── voice_listener.py       # Voice input
├── tests/                      # ⚠️ Needs expansion
│   ├── test_camera.py
│   ├── test_recognition.py
│   └── test_integration.py
└── deploy/                     # ⚠️ To be created
    ├── systemd/
    ├── scripts/
    └── config/
```

### Refactoring Recommendations

1. **Create `src/core/` for fundamental modules**:
   - `camera_utils.py`
   - `ros2_utils.py`
   - `error_handling.py`

2. **Create `src/ai/` for AI features**:
   - `recognition.py` (consolidated face + object)
   - `conversation.py`
   - `behavior.py`

3. **Create `src/robot/` for hardware interface**:
   - `motor_control.py`
   - `sensors.py`
   - `emergency_stop.py`

---

## 🐛 Known Issues & Workarounds

### Issue: Camera Crashes After One Frame
**Workaround**: None yet - critical issue  
**Fix**: See Phase 1.1

### Issue: Code Duplication
**Status**: ✅ RESOLVED - Consolidated into `CameraUtils`

### Issue: Incomplete Testing
**Status**: In progress - See Phase 2

---

## 🔧 Technical Decisions

### 1. Camera Topic Selection
**Decision**: Use `/image_left_raw` directly from `mipi_cam` node  
**Rationale**: More reliable than `/booster_camera_bridge/` topics  
**Reference**: [`CAMERA_FIX_SUMMARY.md`](CAMERA_FIX_SUMMARY.md:10-24)

### 2. Code Consolidation
**Decision**: Created [`CameraUtils`](src/camera_utils.py) for common camera operations  
**Rationale**: Reduce duplication, improve maintainability  
**Impact**: All camera scripts now share consistent image handling

### 3. Docker Strategy (Pending)
**Decision**: TBD - See Phase 4.2  
**Considerations**:
- Memory constraints (8GB Orin NX)
- Real-time performance needs
- GPU/CUDA access complexity
- Deployment simplicity

---

## 📈 Success Metrics

### Stability
- [ ] Camera runs 24+ hours without crashes
- [ ] <1% error rate on frame processing
- [ ] Automatic recovery from errors

### Performance  
- [ ] Face detection: 10+ FPS
- [ ] Object detection (YOLO): 5+ FPS
- [ ] Response latency: <500ms
- [ ] Memory usage: <6GB under load

### AI Capabilities
- [ ] Recognizes and remembers people
- [ ] Natural conversation flow
- [ ] Context-aware responses
- [ ] Proactive engagement

### Production Readiness
- [ ] One-command deployment
- [ ] Auto-start on boot
- [ ] Logging and monitoring
- [ ] Remote diagnostics capability

---

## 👥 Team Roles (if applicable)

- **Hardware/Robotics**: Motor control, sensors, emergency stop
- **Vision/AI**: Camera, detection, recognition, AI integration
- **Systems/DevOps**: Deployment, monitoring, optimization
- **Testing**: Test suite, validation, quality assurance

---

## 📚 Resources & References

### Documentation
- [Camera Fix Summary](CAMERA_FIX_SUMMARY.md)
- [Test Results](K1_TEST_RESULTS.md)
- [Safety Checklist](SAFETY_CHECKLIST.md)
- [Roadmap](ROADMAP.md)

### External References
- [Jetson Orin NX Docs](https://developer.nvidia.com/embedded/jetson-orin-nx)
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [YOLOv8](https://docs.ultralytics.com/)
- [DeepFace](https://github.com/serengil/deepface)

---

## 🎯 Next Immediate Actions

1. **TODAY**: Fix camera stability issue
   - Add comprehensive logging to [`src/camera_utils.py`](src/camera_utils.py)
   - Test with frame rate limiting
   - Monitor memory usage

2. **THIS WEEK**: Complete Phase 1 & 2
   - Stabilize all core systems
   - Complete testing suite
   - Document all issues and fixes

3. **NEXT WEEK**: Begin AI integration (Phase 3)
   - Once stability is proven
   - Start with conversation enhancements
   - Then add behavioral intelligence

---

## 💡 Open Questions

1. **Docker on Jetson**: Performance impact? Worth the tradeoff?
2. **Cloud Integration**: Should we offload heavy AI to cloud?
3. **Model Optimization**: TensorRT for YOLO? Quantization?
4. **Power Management**: Battery life with continuous vision processing?
5. **Multi-Robot**: Will K-1 need to coordinate with other robots?

---

## 📞 Support & Escalation

### Critical Issues
- Camera crashes: Highest priority
- Safety system failures: Immediate escalation
- Data loss: Backup and recovery procedures

### Contact Points
- Hardware issues: Check robot documentation
- ROS2 questions: Community forums
- AI/ML questions: Model documentation

---

**Last Updated**: 2025-11-17  
**Next Review**: After Phase 1 completion