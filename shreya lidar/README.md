# 🛡️ Shreya LiDAR Safety Guard & Visualizer System
### Complete RPLIDAR C1 Quad-Zone AGV Active Telemetry Solution

This directory contains the final, standalone production-ready build of the **RPLIDAR C1 Safety System** for AGVs / AMRs. It provides real-time C++ safety zone classification (L1 Warning, L2 Emergency Stop), hardware reflection filtering, bidirectional active cone detection, hatched non-detection visual styling, and an on-demand HTML5 Canvas visualizer.

---

## 📐 Safety Geometry & Zoning Configuration

### 1. Radial Dimensions
* **Chassis Filter (`ignore_inner_mm`)**: `500 mm`
  * Internal reflections, robot frame mounts, and caster wheels `< 500 mm` are ignored to prevent false obstacle triggers.
* **Inner Stop Ring (`l2_mm`)**: `550 mm` (0.55 m)
  * Triggers immediate AGV **L2 Emergency Stop** if an obstacle penetrates this zone.
* **Outer Warning Ring (`l1_mm`)**: `1000 mm` (1.0 m)
  * Triggers AGV **L1 Warning / Deceleration** if an obstacle is within 0.55 m to 1.0 m.

### 2. Angular Apertures & Inverted Detection Cones
* **Front Active Detection Cone**: `315° – 45°` (`±45°` forward aperture, 90° span)
* **Rear Active Detection Cone**: `135° – 225°` (`±45°` backward aperture, 90° span)
* **Muted Flanks (Non-Detection Sectors)**:
  * **Right Flank**: `45° – 135°` (90° span)
  * **Left Flank**: `225° – 315°` (90° span)
  * Rendered as **diagonal hatched lines (`///`)** with dashed border rays.
  * LiDAR points inside muted zones are completely hidden and filtered out.

---

## 📁 File Structure

```
shreya lidar/
├── config.h        # Configuration structures and JSON serialization definitions
├── config.cpp      # Parser and saver for safety thresholds, blank sectors, and chassis map
├── config.json     # Active safety settings and calibrated chassis envelope
├── guard.cpp       # High-performance C++ RPLIDAR C1 daemon & safety state evaluator
├── ui.cpp          # Lightweight C++ HTTP server with CORS & live JSON endpoints
├── index.html      # Responsive Canvas visualizer with manual tuning & hatched pattern
├── Makefile        # Zero-dependency build script (g++ -std=c++17)
├── launch.sh       # Unified launcher script with --debug Web UI toggle
└── README.md       # Documentation
```

---

## 🚀 Building & Running

### 1. Compilation
No external dependencies required (uses standard C++17 library):
```bash
make
```

### 2. Verification Self-Test
Verify that safety zone evaluation and JSON persistence pass:
```bash
./lidar-guard --self-test
# Expected output: self-test ok
```

### 3. Launching
* **Standard Safety Guard Only**:
  ```bash
  ./launch.sh
  ```
* **With Real-Time Web Visualizer (Debug Mode)**:
  ```bash
  ./launch.sh --debug
  ```
  The visualizer will be accessible at:
  * Local: `http://localhost:8080/`
  * Remote device: `http://<DEVICE_IP>:8080/`

---

## 🎛️ Live Visualizer Features

1. **Manual Zone Setting**:
   * Change `L1 Warning`, `L2 Stop`, and `Chassis Ignore` directly using the number inputs or by dragging the rings on the canvas.
2. **Angle Aperture Tuning**:
   * Set Front Cone Half-Angle (±°) and Rear Cone Half-Angle (±°).
   * Quick Preset buttons: `±45° Cones`, `±30° Narrow`, `±60° Wide`, `Full 360°`.
3. **Muted Sectors Display**:
   * Inactive zones are clearly displayed with slate-colored diagonal hatched lines (`///`).
   * Obstacle points inside muted sectors are suppressed.
4. **AGV Direction Switching**:
   * Forward (`▲`), Stopped (`■`), Reverse (`▼`).
