# OV²SLAM — Internal Architecture & Developer Documentation

> **Paper:** [arXiv:2102.04060](https://arxiv.org/pdf/2102.04060.pdf)
> **Authors:** M. Ferrera, A. Eudes, J. Moras, M. Sanfourche, G. Le Besnerais (ONERA)

This document provides a deep-dive into the internals of OV²SLAM for developers who need to understand, modify, or extend the system. All references point to actual class names, member variables, and source files in this repository.

---

## 1. System Architecture

### 1.1 Multi-Threaded Pipeline Overview

OV²SLAM uses a **4-thread parallel architecture** where each thread operates at a different rate and communicates through shared data structures protected by mutexes.

```
┌──────────────────────────────────────────────────────────────────┐
│                        ROS2 Node Thread                         │
│  (ov2slam_node.cpp — SensorsGrabber::sync_process)              │
│  Receives images via ROS topics → pushes to SlamManager queues  │
└──────────────────────┬───────────────────────────────────────────┘
                       │ qimg_left_, qimg_right_, qimg_time_
                       ▼
┌──────────────────────────────────────────────────────────────────┐
│  Thread 1: SLAM Main Loop  (SlamManager::run)                   │
│  ├── Calls VisualFrontEnd::visualTracking() at FRAME RATE       │
│  ├── Decides if new KeyFrame is required                        │
│  ├── Pushes KeyFrame struct to Mapper queue                     │
│  └── Spawns detached viz threads (visualizeAtFrameRate)         │
└──────────────────────┬───────────────────────────────────────────┘
                       │ Keyframe struct via Mapper::addNewKf()
                       ▼
┌──────────────────────────────────────────────────────────────────┐
│  Thread 2: Mapper  (Mapper::run — detached at construction)     │
│  ├── Triangulates new MapPoints (temporal + stereo)             │
│  ├── Updates covisibility graph                                 │
│  ├── Matches to local map (descriptor-based)                    │
│  ├── Forwards KF to Estimator for BA                            │
│  └── Forwards KF + image to LoopCloser                          │
└───────────┬──────────────────────────┬───────────────────────────┘
            │                          │
            ▼                          ▼
┌───────────────────────┐  ┌───────────────────────────────────────┐
│ Thread 3: Estimator   │  │ Thread 4: LoopCloser                  │
│ (Estimator::run)      │  │ (LoopCloser::run)                     │
│ ├── Local BA (Ceres)  │  │ ├── Online BoW via iBoW-LCD           │
│ ├── Map filtering     │  │ ├── Loop candidate verification       │
│ └── KF culling        │  │ ├── Local Pose Graph optimization     │
└───────────────────────┘  │ └── Full Pose Graph (post-sequence)   │
                           └───────────────────────────────────────┘
```

**Thread spawning chain (from source):**

| Thread | Created in | Method |
|--------|-----------|--------|
| SLAM Main Loop | `ov2slam_node.cpp` | `std::thread slamthread(&SlamManager::run, &slam)` |
| Mapper | `Mapper` constructor | `std::thread mapper_thread(&Mapper::run, this)` (detached) |
| Estimator | `Mapper::run()` | `std::thread estimator_thread(&Estimator::run, pestimator_)` |
| LoopCloser | `Mapper::run()` | `std::thread lc_thread(&LoopCloser::run, ploopcloser_)` |

### 1.2 Anchored Inverse Depth Formulation

Instead of parameterizing each 3D map point with **3 parameters** (X, Y, Z in world frame), OV²SLAM uses an **anchored inverse depth** representation that reduces each point to **1 optimizable parameter**: the inverse depth `ρ = 1/d` relative to the anchor keyframe.

Given a 3D point observed first in keyframe `KF_a` (the **anchor**), its world position is recovered as:

```
P_world = T_w_a * (bearing_vector / ρ)
```

Where:
- `T_w_a` is the anchor keyframe's world pose (Sophus::SE3d)
- `bearing_vector` is the unit ray from the anchor camera center through the pixel observation
- `ρ` is the scalar inverse depth

**Implementation in code:**

```cpp
// map_point.hpp
class MapPoint {
    int kfid_;           // Anchor keyframe ID
    double invdepth_;    // ρ — the single optimized parameter
    Eigen::Vector3d ptxyz_;  // Cached 3D world position
    // ...
};

// Called during triangulation (mapper.cpp, line ~333):
pmap_->updateMapPoint(vkps.at(i).lmid_, wpt, 1./left_pt.z());
```

**Why 1 parameter instead of 3?**
- The anchor KF pose + the pixel observation already encode 5 of the 6 DoF. Only depth remains unknown.
- Reduces the state space in Bundle Adjustment → fewer Jacobian columns → faster Schur complement.
- Inverse depth handles points at infinity gracefully (ρ → 0) which is important for distant scene structures.
- Controlled by `buse_inv_depth: 1` in the YAML configuration.

### 1.3 Real-Time Constraints

OV²SLAM enforces real-time behavior through several mechanisms:

| Mechanism | Implementation |
|-----------|---------------|
| **Frame dropping** | `bforce_realtime_` flag. When enabled, `getNewImage()` skips all queued frames except the latest. |
| **BA interruption** | `Optimizer::signalStopLocalBA()` / `stopLocalBA()` lets the Mapper signal BA to abort early when a new KF arrives. |
| **Deferred local map matching** | `matchingToLocalMap()` is skipped if `bnewkfavailable_` is true — prioritizing triangulation over matching. |
| **Detached merge threads** | `mergeMatches()` runs in a detached thread to avoid blocking the Mapper. |
| **Thread sleep granularity** | Mapper sleeps 100μs, SLAM main loop sleeps 1ms when idle — balancing CPU use vs. latency. |

---

## 2. Directory & Source File Descriptions

### 2.1 Repository Structure

```
ov2slam/
├── src/                          # C++ implementation files
├── include/                      # Header files (class declarations)
│   └── ceres_parametrization/    # Ceres cost functions & local parameterizations
├── Thirdparty/                   # Vendored dependencies
│   ├── Sophus/                   # SE3/SO3 Lie group library
│   ├── ceres-solver/             # Nonlinear least squares optimizer
│   ├── obindex2/                 # Online visual vocabulary (binary index)
│   └── ibow_lcd/                 # Incremental BoW Loop Closure Detection
├── parameters_files/             # YAML configurations
│   ├── accurate/                 # High accuracy, higher compute
│   ├── average/                  # Balanced mode
│   └── fast/                     # Low latency, fewer features
├── docker/                       # Dockerfile for containerized build
├── benchmark_scripts/            # Evaluation utilities
├── ov2slam_visualization.rviz    # Pre-configured RViz layout
├── CMakeLists.txt                # Build system
└── package.xml                   # ROS2 package manifest
```

### 2.2 Core Classes (src/ & include/)

#### SlamManager (`ov2slam.hpp` / `ov2slam.cpp`)
The **central orchestrator**. Owns all major subsystems and runs the main SLAM loop.

```cpp
class SlamManager {
    std::shared_ptr<Frame> pcurframe_;              // Current frame (shared across threads)
    std::shared_ptr<MapManager> pmap_;              // Global map database
    std::unique_ptr<VisualFrontEnd> pvisualfrontend_;  // Tracking front-end
    std::unique_ptr<Mapper> pmapper_;               // Back-end keyframe processor
    std::queue<cv::Mat> qimg_left_, qimg_right_;    // Image input queues
    // ...
};
```

**Key responsibilities:**
- Receives images from `SensorsGrabber` (ROS2 callback → queue)
- Runs `VisualFrontEnd::visualTracking()` for every frame
- Decides when to create KeyFrames and dispatches them
- Handles system `reset()` when tracking is lost
- Writes trajectory files on completion (`writeResults()`)

#### VisualFrontEnd (`visual_front_end.hpp` / `visual_front_end.cpp`)
The **tracking thread logic**. Processes every incoming frame at camera rate.

```cpp
class VisualFrontEnd {
    cv::Mat cur_img_, prev_img_;                   // Current and previous grayscale images
    std::vector<cv::Mat> cur_pyr_, prev_pyr_;      // KLT image pyramids
    MotionModel motion_model_;                     // Constant-velocity SE3 motion model
    bool bp3preq_;                                 // Flag: force P3P next iteration
    // ...
};
```

**Key methods:**
- `trackMono()` — Full mono tracking pipeline: preprocess → KLT → epipolar filter → PnP
- `kltTracking()` — Forward-backward KLT with motion prior for 3D points
- `epipolar2d2dFiltering()` — 5-point Essential Matrix RANSAC to reject outlier tracks
- `computePose()` — P3P-RANSAC + Ceres PnP (motion-only BA)
- `checkNewKfReq()` — Parallax + co-observation heuristics for KF decision

#### MotionModel (embedded in `visual_front_end.hpp`)
A **constant-velocity model** in the Lie algebra (se3) that predicts the next frame's pose:

```cpp
class MotionModel {
    Sophus::SE3d prevTwc_;
    Eigen::Matrix<double, 6, 1> log_relT_;   // Velocity in se3
    double prev_time_;
    // Prediction: Twc_predicted = Twc_current * exp(log_relT_ * dt)
};
```

#### Mapper (`mapper.hpp` / `mapper.cpp`)
The **back-end keyframe processor**. Runs in its own thread. Spawns the Estimator and LoopCloser threads.

```cpp
class Mapper {
    std::shared_ptr<Estimator> pestimator_;       // BA engine
    std::shared_ptr<LoopCloser> ploopcloser_;     // Loop detection engine
    std::queue<Keyframe> qkfs_;                   // Incoming KF queue
    // ...
};
```

**Pipeline per KeyFrame:**
1. Stereo triangulation (if stereo mode)
2. Temporal triangulation (2D → 3D promotion via multi-view geometry)
3. Mono initialization check (reset if <30 3D points at KF #1)
4. Covisibility graph update
5. Local map matching (descriptor-based, if BRIEF enabled)
6. Forward to Estimator (BA) and LoopCloser

#### Estimator (`estimator.hpp` / `estimator.cpp`)
Runs **local Bundle Adjustment** via Ceres Solver.

```cpp
class Estimator {
    std::unique_ptr<Optimizer> poptimizer_;   // Ceres-based optimizer
    // Key method: applyLocalBA() → poptimizer_->localBA()
};
```

**Pipeline per KeyFrame:**
1. `processKeyframe()` — receives KF from Mapper
2. `applyLocalBA()` — optimizes poses of covisible KFs + 3D points
3. `mapFiltering()` — removes redundant KFs (based on `fkf_filtering_ratio`)

#### LoopCloser (`loop_closer.hpp` / `loop_closer.cpp`)
Detects and corrects **loop closures** using iBoW-LCD.

```cpp
class LoopCloser {
    #ifdef IBOW_LCD
    ibow_lcd::LCDetector lcdetetector_;    // Online BoW detector
    #endif
    std::unique_ptr<Optimizer> poptimizer_;  // Separate optimizer for pose graph
    // ...
};
```

**Pipeline:**
1. `processKeyframe()` — extracts descriptors, queries iBoW-LCD
2. `processLoopCandidate()` — geometric verification (kNN matching → epipolar → P3P → PnP)
3. `localPoseGraph()` — corrects KF poses if loop confirmed

#### Frame (`frame.hpp` / `frame.cpp`)
Represents both **regular frames and keyframes**. This is the fundamental data unit.

```cpp
class Frame {
    int id_, kfid_;                                      // Frame ID, associated KF ID
    Sophus::SE3d Twc_, Tcw_;                            // World↔Camera transforms
    std::unordered_map<int, Keypoint> mapkps_;           // Observed keypoints (by landmark ID)
    std::map<int,int> map_covkfs_;                       // Covisibility: {kf_id → shared_point_count}
    std::unordered_set<int> set_local_mapids_;           // Local map point IDs
    std::vector<std::vector<int>> vgridkps_;             // Spatial grid for fast neighbor lookup
    std::shared_ptr<CameraCalibration> pcalib_leftcam_;  // Intrinsics + distortion model
    // ...
};
```

#### MapPoint (`map_point.hpp` / `map_point.cpp`)
Represents a **3D landmark** in the sparse map.

```cpp
class MapPoint {
    int lmid_;                                    // Unique landmark ID
    bool is3d_;                                   // True once triangulated
    Eigen::Vector3d ptxyz_;                       // 3D world position (cached)
    int kfid_;                                    // Anchor keyframe ID
    double invdepth_;                             // Inverse depth w.r.t. anchor
    std::set<int> set_kfids_;                     // Set of observing KF IDs
    cv::Mat desc_;                                // Representative descriptor
    std::unordered_map<int, cv::Mat> map_kf_desc_; // Per-KF descriptors
    // ...
};
```

#### MapManager (`map_manager.hpp` / `map_manager.cpp`)
The **global map database**. Thread-safe storage for all KeyFrames and MapPoints.

```cpp
class MapManager {
    std::unordered_map<int, std::shared_ptr<Frame>> map_pkfs_;   // All keyframes
    std::unordered_map<int, std::shared_ptr<MapPoint>> map_plms_; // All map points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_;              // PCL point cloud for RViz
    // Protected by: map_mutex_, optim_mutex_, kf_mutex_, lm_mutex_
};
```

#### Optimizer (`optimizer.hpp` / `optimizer.cpp`)
Wraps all **Ceres Solver** optimization problems.

| Method | Purpose |
|--------|---------|
| `localBA()` | Local Bundle Adjustment on covisible KFs |
| `looseBA()` | BA on a KF range (used after loop closure) |
| `fullBA()` | Global BA over all KFs + MapPoints |
| `localPoseGraph()` | Pose graph optimization after loop detection |
| `fullPoseGraph()` | Full trajectory correction with loop constraints |
| `structureOnlyBA()` | Optimize only MapPoint positions (fixed poses) |

#### Other Key Files

| File | Role |
|------|------|
| `feature_extractor.cpp` | FAST/Shi-Tomasi/cornerMinEigenVal keypoint detection + BRIEF/ORB description |
| `feature_tracker.cpp` | Forward-backward KLT optical flow with CLAHE preprocessing |
| `camera_calibration.cpp` | Pinhole + Fisheye models, undistortion maps, stereo rectification |
| `multi_view_geometry.cpp` | Triangulation, 5-pt Essential Matrix, P3P (OpenGV or OpenCV fallback) |
| `camera_visualizer.cpp` | ROS2 publishers for RViz visualization |
| `logger.hpp` | Static trajectory logging in TUM, KITTI, and TartanAir formats |
| `profiler.hpp` | Per-function timing instrumentation (`Profiler::Start/StopAndDisplay`) |
| `slam_params.cpp` | YAML parameter file reader → `SlamParams` struct |
| `ceres_parametrization/` | Ceres cost functors for reprojection errors, anchored inverse depth, pose graph edges |

### 2.3 Thirdparty Libraries

| Library | Version/Branch | Purpose |
|---------|---------------|---------|
| **Sophus** | Vendored copy | Lie group SE3/SO3 representation. Used for all pose storage (`Twc_`, `Tcw_`), motion model, and relative transformations. |
| **Ceres Solver** | Vendored copy | Nonlinear least-squares optimization. Powers all BA variants, PnP refinement, and pose graph optimization. Built with `-march=native` for performance. |
| **iBoW-LCD** | Modified library | Incremental Bag-of-Words loop closure detection. Modified from catkin package to standalone library. Builds vocabulary online from incoming descriptors. |
| **OBIndex2** | Modified library | Binary visual word index — required dependency for iBoW-LCD. Provides efficient nearest-neighbor search over binary descriptors. |

### 2.4 Parameter Files

Three configuration tiers are provided. The differences are primarily in feature extraction density, detector type, and optimization aggressiveness:

| Parameter | `accurate/` | `average/` | `fast/` |
|-----------|------------|-----------|--------|
| **Detector** | `use_singlescale_detector: 1` (cornerMinEigenVal) | Mixed | `use_fast: 1` (FAST corners) |
| **nmaxdist** (cell size) | `35` (more features, denser grid) | ~40 | `50` (fewer features, sparser) |
| **use_clahe** | `1` (histogram equalization ON) | Varies | `0` (OFF — saves CPU) |
| **dop3p** | `0` (skip P3P, rely on PnP only) | Varies | `1` (use P3P-RANSAC every frame) |
| **fkf_filtering_ratio** | `0.95` (keep more KFs) | ~0.92 | `0.9` (cull more KFs) |
| **buse_loop_closer** | `0` or `1` | Varies | `0` (disabled for speed) |

> **Rule of thumb:** Use `fast/` for real-time on constrained hardware. Use `accurate/` for offline processing or when drift matters most.

---

## 3. Key Data Structures & Variables

### 3.1 Keypoint Struct

Every detected feature in a frame is stored as a `Keypoint`:

```cpp
struct Keypoint {
    int lmid_;              // Associated MapPoint (landmark) ID
    cv::Point2f px_;        // Distorted pixel position
    cv::Point2f unpx_;      // Undistorted pixel position
    Eigen::Vector3d bv_;    // Unit bearing vector (in camera frame)
    int scale_;             // Detection scale level
    float angle_;           // Orientation (for descriptors)
    cv::Mat desc_;          // Binary descriptor (BRIEF or ORB)
    bool is3d_;             // True if associated MapPoint is triangulated
    bool is_stereo_;        // True if stereo match exists
    cv::Point2f rpx_;       // Right camera pixel (stereo)
    Eigen::Vector3d rbv_;   // Right camera bearing vector (stereo)
    bool is_retracked_;     // True if re-matched from local map
};
```

### 3.2 Frame / KeyFrame Pose Storage

Poses are stored as **dual SE3 elements** using Sophus:

```cpp
Sophus::SE3d Twc_;  // Camera-to-World (for projecting camera position into world)
Sophus::SE3d Tcw_;  // World-to-Camera (for projecting world points into image)
```

Both are maintained simultaneously and updated together via `setTwc()` / `setTcw()`, which internally computes the inverse. Thread safety is ensured by `pose_mutex_`.

### 3.3 MapPoint Anchored Representation

```cpp
class MapPoint {
    Eigen::Vector3d ptxyz_;   // World position = T_w_anchor * (bv / invdepth_)
    int kfid_;                // Anchor keyframe ID
    double invdepth_;         // ρ = 1/depth_in_anchor_frame

    // Observation tracking
    std::set<int> set_kfids_;                     // Which KFs see this point
    std::unordered_map<int, cv::Mat> map_kf_desc_; // Descriptor per observing KF
};
```

The `invdepth_` is set during triangulation:

```cpp
// mapper.cpp — after temporal triangulation:
left_pt = computeTriangulation(Tcicj, kfkp.bv_, vkps.at(i).bv_);
wpt = pkf->projCamToWorld(left_pt);
pmap_->updateMapPoint(vkps.at(i).lmid_, wpt, 1./left_pt.z());
//                                              ^^^^^^^^^^^^^^
//                                              inverse depth in anchor frame
```

### 3.4 Covisibility Graph

Stored per-frame as an ordered map: `std::map<int,int> map_covkfs_` where:
- **Key:** Covisible keyframe ID
- **Value:** Number of shared MapPoint observations

Updated by `MapManager::updateFrameCovisibility()` after each new KeyFrame is processed. Used for:
- Selecting KFs to include in local BA (`nmin_covscore` threshold)
- Building the local map for descriptor matching
- KF culling decisions

```cpp
// frame.hpp
std::map<int,int> map_covkfs_;  // {kfid → co_observation_count}

// Usage in Estimator — only optimize KFs with enough shared observations:
if (co_obs_count >= pslamstate_->nmin_covscore_) { /* include in BA */ }
```

### 3.5 Spatial Keypoint Grid

Keypoints are binned into a **spatial grid** for O(1) neighbor lookups during matching:

```cpp
std::vector<std::vector<int>> vgridkps_;  // grid_cell_index → [landmark_ids]
size_t ncellsize_;   // Cell size in pixels (= nmaxdist parameter)
size_t nbwcells_;    // Number of horizontal cells
size_t nbhcells_;    // Number of vertical cells
```

Used by `getSurroundingKeypoints()` to find nearby features for local map matching without brute-force search.

---

## 4. Algorithmic Workflow

### 4.1 Front-End: KLT-Based Visual Tracking

The tracking pipeline runs at **camera frame rate**. It uses an **indirect** (feature-based) Lucas-Kanade formulation, not direct (photometric) alignment.

#### Per-Frame Pipeline (`VisualFrontEnd::trackMono`):

```
1. preprocessImage()
   ├── Optional CLAHE histogram equalization
   ├── Swap cur_img_ ↔ prev_img_
   └── Build KLT image pyramid (cv::buildOpticalFlowPyramid)

2. Apply Motion Model
   └── Predict Twc using constant-velocity model in se3

3. kltTracking()
   ├── For 3D keypoints (with prior from motion model):
   │   ├── Project 3D points into predicted frame
   │   ├── Forward KLT with 2-level pyramid (fast, coarse)
   │   └── Backward KLT for error check (fmax_fbklt_dist)
   └── For 2D keypoints (no prior):
       ├── Forward KLT with full pyramid levels
       └── Backward KLT for verification

4. epipolar2d2dFiltering()  [if doepipolar=1]
   ├── Compute 5-point Essential Matrix (OpenGV or OpenCV)
   ├── RANSAC outlier rejection
   └── Optional: use recovered motion if tracking is poor (mono, <30 3D points)

5. computePose()
   ├── P3P-RANSAC (if dop3p=1 or bp3preq_=true)
   │   ├── OpenGV P3P with LMedS
   │   └── Outlier removal
   └── Ceres PnP (motion-only BA)
       ├── Robust Huber cost (robust_mono_th)
       ├── Optional L2 re-optimization after outlier removal
       └── Final pose update

6. updateMotionModel()
   └── Store velocity for next prediction

7. checkNewKfReq()
   ├── Parallax check (median rotation-compensated parallax vs finit_parallax)
   ├── 3D keypoint ratio check (cur vs prev KF)
   ├── Grid occupancy check (noccupcells vs nbmaxkps)
   └── Time-based check (stereo: >1s since last KF)
```

#### Forward-Backward KLT Error Check

The tracker computes optical flow in both directions and rejects tracks where the round-trip error exceeds `fmax_fbklt_dist` (default: 0.5 px):

```
Track forward:   prev_px → cur_px_forward
Track backward:  cur_px_forward → prev_px_backward
Error:           ||prev_px - prev_px_backward|| > fmax_fbklt_dist  →  REJECT
```

### 4.2 Back-End: Mapping & Bundle Adjustment

#### Temporal Triangulation (`Mapper::triangulateTemporal`)

For each 2D keypoint in the new keyframe:
1. Find the **first keyframe** that also observes this landmark
2. Compute relative pose `T_ci_cj` between anchor KF and new KF
3. Check **rotation-compensated parallax** (must exceed threshold)
4. Triangulate using bearing vectors (via OpenGV or midpoint method)
5. Verify: point must be in front of both cameras + reprojection error < `fmax_reproj_err`
6. Update MapPoint with world position and anchor inverse depth

#### Local Bundle Adjustment (`Optimizer::localBA`)

Optimizes a **sliding window** of keyframes and their observed MapPoints:

**State variables:**
- KF poses: `Sophus::SE3d` (6 DoF each, using Ceres local parameterization)
- MapPoints: either `Eigen::Vector3d` (3 DoF) or anchored inverse depth `double` (1 DoF), controlled by `buse_inv_depth`

**Cost function per observation:**
```
e = π(T_cw * P_world) - pixel_observation
```
With Huber robust cost: `ρ(||e||²)` where the threshold is `robust_mono_th` (chi² distribution, default 5% = 5.9915).

**Optimization strategy (Ceres options):**

| Param | Default | Effect |
|-------|---------|--------|
| `use_sparse_schur` | 1 | Sparse Schur complement (recommended) |
| `use_dogleg` | 0 | Dogleg trust-region strategy |
| `use_nonmonotic_step` | 0 | Allow cost to temporarily increase |
| `apply_l2_after_robust` | 1 | Re-optimize without Huber after outlier removal |

#### Map Filtering (`Estimator::mapFiltering`)

After BA, redundant keyframes are culled based on `fkf_filtering_ratio`:
- If ≥ 90% (or 95% in accurate mode) of a KF's 3D observations are also seen by ≥ 4 other KFs → that KF is redundant → removed.

### 4.3 Loop Closure: Online Vocabulary Tree

Unlike ORB-SLAM which uses a **pre-trained** vocabulary (DBoW2), OV²SLAM uses **iBoW-LCD** which builds the vocabulary **online** from incoming descriptors.

#### How Online BoW Works

```
1. For each new KeyFrame:
   ├── Extract binary descriptors (BRIEF or ORB)
   ├── Feed to iBoW-LCD's LCDetector
   │   ├── Descriptors are incrementally added to OBIndex2 (binary tree)
   │   ├── The vocabulary grows with each new KF
   │   └── No pre-training on any dataset required
   └── Query returns loop candidate KF IDs (if visual similarity > threshold)

2. For each loop candidate:
   ├── kNN descriptor matching between candidate and current KF
   ├── Epipolar filtering (5-pt Essential Matrix RANSAC)
   ├── P3P-RANSAC geometric verification
   ├── PnP refinement with Ceres
   ├── Track local map around loop KF for more correspondences
   └── If verified → localPoseGraph() corrects drift
```

**Advantage:** The online vocabulary is always adapted to the current environment. No domain gap between training data and deployment data.

**Trade-off:** The vocabulary is less discriminative initially (few words) and grows in memory over long sequences.

---

## 5. Extractable Data & Outputs

### 5.1 Pose Data

OV²SLAM outputs camera trajectories in multiple formats via the `Logger` class:

| Output File | Format | Content |
|-------------|--------|---------|
| `ov2slam_traj.txt` | TUM format | `timestamp tx ty tz qx qy qz qw` — per-frame poses |
| `ov2slam_traj_kitti.txt` | KITTI format | 3×4 transformation matrix per line — per-frame |
| `ov2slam_kfs_traj.txt` | TUM format | Keyframe-only poses |
| `ov2slam_fullba_kfs_traj.txt` | TUM format | KF poses after optional final full BA |
| `ov2slam_full_traj_wlc.txt` | TUM format | Full trajectory corrected by loop closure pose graph |

**Programmatic access (inside the code):**

```cpp
// Get current frame pose as SE3:
Sophus::SE3d Twc = pcurframe_->getTwc();

// Extract translation and quaternion:
Eigen::Vector3d t = Twc.translation();           // [x, y, z]
Eigen::Quaterniond q = Twc.unit_quaternion();     // [x, y, z, w]
```

### 5.2 Point Clouds

The sparse 3D map is published as a **PCL PointCloud** via ROS2:

```cpp
// map_manager.hpp
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_;

// Published by RosVisualizer::pubPointCloud() on topic: (configurable in rviz)
```

Each point has XYZ position and RGB color. The cloud is updated at keyframe rate.

**Direct access to all MapPoints:**
```cpp
for (const auto& [lmid, plm] : pmap_->map_plms_) {
    if (plm && plm->is3d_) {
        Eigen::Vector3d pt = plm->getPoint();    // World XYZ
        double inv_depth = plm->invdepth_;        // Anchored inverse depth
        int anchor_kf = plm->kfid_;               // Anchor KF ID
        int num_observations = plm->set_kfids_.size();
    }
}
```

### 5.3 Performance Metrics

OV²SLAM does **not** compute ATE/RPE internally. These are computed externally using the output trajectory files with standard tools:

| Metric | Tool | Command Example |
|--------|------|----------------|
| **ATE (Absolute Trajectory Error)** | [evo](https://github.com/MichaelGrupp/evo) | `evo_ape tum groundtruth.txt ov2slam_traj.txt -p` |
| **RPE (Relative Pose Error)** | [evo](https://github.com/MichaelGrupp/evo) | `evo_rpe tum groundtruth.txt ov2slam_traj.txt -p` |
| **Drift** | Manual | Compare start/end pose for closed loops |

### 5.4 Timing Logs

When `log_timings: 1` is set in the YAML, the `Profiler` class outputs per-function timing:

```
Profiler keys (from source):
├── "0.Full-Front_End"              — Total front-end time per frame
├── "1.FE_Track-Mono"               — Mono tracking sub-total
├── "2.FE_TM_KLT-Tracking"          — KLT tracking time
├── "2.FE_TM_EpipolarFiltering"     — Epipolar outlier rejection time
├── "2.FE_TM_computePose"           — P3P + PnP pose estimation time
├── "2.FE_TM_checkNewKfReq"         — KF decision logic time
├── "2.FE_TM_preprocessImage"       — Image preprocessing time
├── "0.Keyframe-Processing_Mapper"  — Total mapper time per KF
├── "1.KF_TriangulateTemporal"      — Temporal triangulation time
├── "1.KF_TriangulateStereo"        — Stereo triangulation time
└── "1.KF_MatchingToLocalMap"       — Local map matching time
```

The profiler accumulates and displays statistics (mean, min, max) per function when `debug: 1` is enabled.

---

## Appendix: Thread Safety Reference

| Mutex | Protects | Used by |
|-------|----------|---------|
| `SlamManager::img_mutex_` | Image queues (`qimg_left_`, `qimg_right_`) | ROS callback ↔ SLAM main loop |
| `MapManager::map_mutex_` | Map read/write operations | Front-end ↔ Mapper ↔ Estimator |
| `MapManager::optim_mutex_` | BA operations (prevents concurrent optimization) | Estimator ↔ Mapper merge thread |
| `Frame::pose_mutex_` | Pose `Twc_`, `Tcw_` | Any thread accessing poses |
| `Frame::kps_mutex_` | Keypoint map `mapkps_` | Front-end ↔ Mapper |
| `Frame::cokfs_mutex_` | Covisibility map `map_covkfs_` | Mapper ↔ Estimator |
| `MapPoint::pt_mutex` | 3D position + inverse depth | Any thread accessing point data |
| `Mapper::qkf_mutex_` | Keyframe queue `qkfs_` | SLAM main loop ↔ Mapper |
| `Optimizer::localba_mutex_` | BA stop signal `bstop_localba_` | Mapper ↔ Estimator |
