# omnibot_description

URDF for the OmniBot mecanum base and SO-101 arm (`urdf/omnibot.urdf.xacro`).

## Web model

The browser console (Garage → Add robot → OmniBot Pro → Connect → Simulation) will load this robot with three.js `urdf-loader`. This package only builds the files. Rendering them is a separate console change; nothing in the console repo is modified here.

The chassis, standoffs, wheels, and cameras are box and cylinder primitives in the xacro. There is no base CAD, and this export does not invent any. Nominal tag36h11 cards in `urdf/apriltags.urdf.xacro` stay on the Gazebo model and are omitted here. `urdf-loader` draws those primitives. The SO-101 STL meshes in `meshes/so101/` are real. The xacro still shows the arm as primitives so Gazebo stays light; the web export swaps those arm visuals for the meshes, using the visual origins from `so101_new_calib.urdf` (the same joint frames the xacro already copies).

### Tooling

No ROS install. Expansion uses the PyPI `xacro` package (2.x). Meshes are decimated with `trimesh` and `fast-simplification`, then written as binary GLB. `package://omnibot_description/...` paths are rewritten to URDF-relative `meshes/<name>.glb`.

```bash
python -m pip install xacro trimesh fast-simplification
python omnibot-ros2/omnibot_description/scripts/export_web_model.py
```

Default xacro args match the desktop model: `drive:=mecanum`, `lidar:=false`, `minimal_sensors:=false`. Gazebo plugins and sensors are omitted from the browser URDF. Wheel and arm joint limits are unchanged.

### Output

| Path | Role |
|---|---|
| `build/web-model/omnibot-pro/omnibot-pro.urdf` | Plain URDF. Mesh paths are relative to this file. |
| `build/web-model/omnibot-pro/meshes/*.glb` | Decimated SO-101 parts. The STS3215 servo GLB is shared. |
| `build/web-model/omnibot-pro/manifest.json` | Every joint name, type, parent, child, axis, and limit, plus `arm_joints`, `gripper_joint`, and `mecanum_wheel_joints`. |
| `build/web-model/omnibot-pro.zip` | Zip of that folder. The script prints uncompressed and zip sizes and exits non-zero if the zip exceeds 10 MiB (10485760 bytes). |

`build/` is gitignored. Copy `build/web-model/omnibot-pro/` (the URDF, `meshes/`, and `manifest.json`, not the zip) into the console as `public/models/omnibot-pro/`. `urdf-loader` should fetch `omnibot-pro.urdf`; the GLB paths resolve next to it. Animate `arm_joints` (the six SO-101 joints, including `arm_gripper`) and the four `mecanum_wheel_joints`.

A reference run of the script produced a 1.40 MiB folder and a 0.77 MiB zip, under the 5 MB mark where `omnibot_description/web/` could have been committed. The folder is not committed. The GLBs are a build product of `trimesh` / `fast-simplification` and would churn when those packages change. The script and `test/test_export_web_model.py` are the source of truth. Regenerate after editing the xacro or the STLs.

### Check

`pytest omnibot-ros2/omnibot_description/test` runs the export in a temp directory. It checks the six arm joints (including the gripper) and the four mecanum wheel joints have limits, that every mesh URI resolves to a GLB, that the chassis stays primitive, and that the zip is inside the budget. The CPU CI job runs that test. It does not need ROS.
