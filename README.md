# match_cooperative_handling

ROS 2 nodes and a PyQt extension of the shared MuR GUI for moving a virtual
object with selected UR arms. The GUI also contains a lightweight map-frame
top view; RViz remains available for full robot inspection.

## Build and start

Run from the workspace root on the GUI computer:

```bash
cd /home/rosmatch/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to match_cooperative_handling --symlink-install
source install/setup.bash
ros2 run match_cooperative_handling cooperative_handling_gui.py
```

If the workspace dependencies are already built, rebuild just the two GUI
packages:

```bash
colcon build --packages-select match_mur_gui match_cooperative_handling --symlink-install --allow-overriding match_mur_gui
```

The installed executable includes the `.py` suffix. Check it with
`ros2 pkg executables match_cooperative_handling` if `ros2 run` cannot find
it. Build and source the same packages in the remote MuR workspace before
using the GUI's remote actions. The GUI uses SSH host names `mur620a` through
`mur620d`, the **Remote WS** field, and the ROS domain configured by the
shared MuR GUI.

The OAK controls are separate: start their own GUI with
`ros2 run oak_camera_calibration oak_camera_gui`. Starting this cooperative
GUI does not start an OAK camera.

## Basic workflow

1. Select the MuRs and the UR10 left/right checkboxes in the shared GUI.
   Cooperative actions use the first selected MuR in a–d order as the
   **object host**. The base GUI falls back to `mur620d` for actions if no
   MuR is checked; the top view deliberately shows an empty selection then.
2. Provide `map -> <robot>/base_link` TFs for the selected MuRs. Leave
   **Temporary map anchor: OFF** when these TFs exist. For a temporary
   single-host setup without a map pose, switch it on before the next
   **Start Object Nodes**; it publishes an identity
   `map -> <object_host>/base_link` transform.
3. Start hardware/controllers as needed, then click **Start Object Nodes**.
   This cleans up previous cooperative object processes on the selected
   hosts, starts one virtual-object state node on the object host, and
   starts one TCP transform node for each selected MuR/arm pair.
4. Initialize the virtual object with **Set From TCP** (right arm on the
   object host if selected, otherwise left) or **Set Object Center** (both
   arms on the object host). Then use **Set Current Offsets** to capture the
   object-to-TCP relationship for every selected MuR/arm.
5. Wait for each selected arm's status to become `ready`. **START MOTION**
   checks UR readiness and calls each TCP node's start service. Its
   preflight checks current object data, the relative pose, the required
   TFs and controller subscriber, and the initial TCP error (defaults:
   at most 3 cm and 5°). `armed` means the virtual-object controller is
   enabled. **STOP MOTION** sends a zero object twist and disarms the
   selected arms.
6. **Open Object Jog** or **Demos** can command the object after arming.
   The currently implemented demo is **Safe Wiggle**. Tracking logs run
   for up to 300 s at 50 Hz and are written on each selected robot under
   `<Remote WS>/src/match_cooperative_handling/logs/tracking`.

The shared GUI's Home L/R controls use MoveIt separately from cooperative
object control.

## ROS interfaces

| Interface | Type / purpose |
| --- | --- |
| `/virtual_object/set_pose` | `geometry_msgs/PoseStamped`; set the object pose in `map` (transient local). |
| `/virtual_object/object_pose` | `geometry_msgs/PoseStamped`; current object pose. |
| `/virtual_object/object_twist_cmd` | `geometry_msgs/TwistStamped`; jog/demo command in `map`. |
| `/virtual_object/object_twist` | `geometry_msgs/TwistStamped`; effective object twist. |
| `/<robot>/UR10_<side>/virtual_object_tcp_transform_node/status` | `std_msgs/String`; `blocked`, `ready` or `armed`. |
| `/<robot>/UR10_<side>/virtual_object_tcp_transform_node/target_tcp_pose` | `geometry_msgs/PoseStamped`; target in the arm base frame. |
| `/<robot>/UR10_<side>/virtual_object_tcp_transform_node/relative_object_to_tcp_pose` | `geometry_msgs/PoseStamped`; captured object-to-TCP offset. |
| `/<robot>/UR10_<side>/virtual_object_tcp_transform_node/start`, `stop` | `std_srvs/Trigger`; arm/disarm virtual-object following. |

Here `<robot>` is, for example, `mur620a`, and `<side>` is `l` or
`r`. The main frames are:

| TF frame | Source |
| --- | --- |
| `map` and `<robot>/base_link` | Shared map pose supplied externally; the optional temporary anchor covers only the object host. |
| `<robot>/UR10_<side>/base_link`, joint links, `tool0` | MuR robot state/TF. |
| `virtual_object/base_link` | Virtual-object state node. |
| `<robot>/UR10_<side>/virtual_object_target_tcp` | TCP transform node after its required inputs and TFs are available. |

## Top view

The panel to the right of the button groups and above the GUI log projects
TF positions into `map`: +X points right and +Y up. Its 0.5 m grid, Fit
button, mouse-wheel zoom and drag make the relative placement visible without
a 3D renderer. Drag the splitters to resize the map and log areas. A simplified
MiR-600 footprint (about 1.35 × 0.91 m) carries a distinct colored,
bold **a/b/c/d** and a front marker. Selected UR arms are drawn from
their TF joint positions; their TCP targets and the virtual object have
small coordinate axes. Only checked MuRs and checked arm sides appear.
The TF worker refreshes the view at about 5 Hz. Missing or more than
2 s old dynamic TFs are listed in the panel and are not given invented
positions.

This view assumes that all selected MuRs already have correct poses in
one shared `map` frame. The temporary identity anchor does **not**
compute their relative poses. Do not enable it alongside an existing
map-to-host transform, because that would publish a competing TF. The
source of real MuR map poses is a separate follow-up task.

## Diagnosis

Check the common frame tree and object data before trying motion:

```bash
ros2 run tf2_ros tf2_echo map mur620a/base_link
ros2 run tf2_ros tf2_echo map mur620a/UR10_l/tool0
ros2 run tf2_ros tf2_echo map virtual_object/base_link
ros2 topic echo --once /virtual_object/object_pose
ros2 topic echo --once /mur620a/UR10_l/virtual_object_tcp_transform_node/status
ros2 service list
```

Substitute the selected MuR and side. A missing target in the top view
usually means the TCP node is still missing an object pose/twist, a
relative offset, or a required TF. The arm status and GUI log give the
corresponding reason.
