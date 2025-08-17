# Manipulator Drone

A ROS 2 package that integrates **PX4 SITL**, **Gazebo Harmonic**, and **ros2\_control** for simulating and controlling a manipulator-equipped drone.

---

## 🚀 Overview

This package provides:

- PX4 + Gazebo SITL integration.
- ros2\_control interface for actuators.
- Support for custom URDF/Xacro and SDF models.

The goal is to simulate and control a quadrotor drone with an attached manipulator arm using ROS 2 and Gazebo.

---

## 📦 Requirements

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Harmonic (source install)](https://gazebosim.org/docs/harmonic/install)
- [PX4 Autopilot](https://docs.px4.io/main/en/ros2/)
- [ros2\_control](https://control.ros.org/master/index.html)
- [gz\_ros2\_control](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)

---

## ⚙️ Installation

1. Clone PX4 and setup SITL with ROS 2:

   ```bash
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   make px4_sitl gz_x500
   ```

2. Install ROS 2 Humble (desktop version recommended).

3. Source install **Gazebo Harmonic**:

   ```bash
   git clone https://github.com/gazebosim/gz-sim -b harmonic
   colcon build --merge-install
   ```

4. Install **gz\_ros2\_control**:

   ```bash
   git clone https://github.com/ros-controls/gz_ros2_control.git
   colcon build --merge-install
   ```

5. Export required environment variables:

   ```bash
   export GZ_SIM_RESOURCE_PATH=$HOME/iiit_ws/install/darm/share/darm/models:$GZ_SIM_RESOURCE_PATH
   export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
   ```

6. Build this package:

   ```bash
   colcon build --packages-select darm
   source install/setup.bash
   ```

---

## ▶️ Usage

To launch the simulation:

```bash
prime-run ros2 launch darm trial.launch.py
```

This will:

- Start PX4 + Gazebo SITL.
- Spawn the `x500_base` drone model with manipulator.
- Launch ros2\_control with the configured controllers.

---

## 🛠 Troubleshooting

Check the [Error.md](./Error.md) file for common issues and their fixes.

Example:

| **Error**                                               | **Fix**                                                 | **Reason**                              |
| ------------------------------------------------------- | ------------------------------------------------------- | --------------------------------------- |
| `The supplied world name [world] is reserved`           | Rename world in SDF                                     | `world` is a reserved keyword in Gazebo |
| `Could not load resource ... Unable to open file`       | Ensure meshes are installed under `share/darm/urdf/...` | Symlink issues during build             |
| `Failed to load system plugin [gz_ros2_control-system]` | Export `GZ_SIM_SYSTEM_PLUGIN_PATH`                      | Plugin library path not found           |

---

## 📚 Resources

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ros2\_control Documentation](https://control.ros.org/master/index.html)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [PX4 Autopilot Docs](https://docs.px4.io/main/en/)
- [gz\_ros2\_control GitHub](https://github.com/ros-controls/gz_ros2_control)

---

## ✨ TODO

- Add unit tests for controller plugins.
- Extend manipulator control with MoveIt.
- Automate mesh/URDF installation without symlinks.

---

## 📝 License

This project is licensed under the [Apache 2.0 License](LICENSE).
