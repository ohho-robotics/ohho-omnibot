# omnibot_description

URDF for the OmniBot mecanum base and SO-101 arm (`urdf/omnibot.urdf.xacro`).

## Web model

The browser console (Garage → Add robot → OmniBot Pro → Connect → Simulation) will load this robot with three.js `urdf-loader`. This package only builds the files. Rendering them is a separate console change; nothing in the console repo is modified here.

The SO-101 STL meshes in `meshes/so101/` are the arm `<visual>` geometry. Their origins live in `urdf/omnibot.urdf.xacro` (the poses previously hard-coded in `scripts/export_web_model.py`, from `so101_new_calib.urdf`). Collision on the arm stays the box and cylinder primitives.

There is no chassis or wheel CAD in this repo. `meshes/base/` holds simplified procedural STLs generated from the plate and wheel dimensions by `scripts/generate_base_meshes.py`. They are metres, Z-up, origin at the link frame, and labelled `simplified_` in the STL solid name. STEP for the base is a follow-up; see `meshes/base/README.md`. Standoffs and cameras stay box and cylinder visuals. Collision for the plates and wheels stays the box and the cylinder. Nominal tag36h11 cards in `urdf/apriltags.urdf.xacro` stay on the Gazebo model and are omitted from the browser export.

The web export reads mesh origins from the expanded URDF. It does not keep a second origin table. It decimates the STLs to GLB. `urdf-loader` draws those GLBs plus the remaining primitives.

### Tooling

No ROS install. Expansion uses the PyPI `xacro` package (2.x). Meshes are decimated with `trimesh` and `fast-simplification`, then written as binary GLB. `package://omnibot_description/...` paths are rewritten to URDF-relative `meshes/<name>.glb`.

```bash
python -m pip install xacro trimesh fast-simplification
python omnibot-ros2/omnibot_description/scripts/export_web_model.py
```

Default xacro args match the desktop model: `drive:=mecanum`, `lidar:=false`, `minimal_sensors:=false`. Gazebo plugins and sensors are omitted from the browser URDF. Joint position stops are unchanged. Arm effort and velocity are the Feetech STS3215 7.4 V output ratings cited in the xacro. Wheel effort and velocity remain the estimates cited there (this repo has no gear ratio or wheel-motor datasheet).

### Output

| Path | Role |
|---|---|
| `build/web-model/omnibot-pro/omnibot-pro.urdf` | Plain URDF. Mesh paths are relative to this file. |
| `build/web-model/omnibot-pro/meshes/*.glb` | Decimated SO-101 parts, plus the simplified chassis plate and two mecanum wheels. The STS3215 servo GLB is shared. |
| `build/web-model/omnibot-pro/manifest.json` | Every joint name, type, parent, child, axis, and limit, plus `arm_joints`, `gripper_joint`, and `mecanum_wheel_joints`. |
| `build/web-model/omnibot-pro.zip` | Zip of that folder. The script prints uncompressed and zip sizes and exits non-zero if the zip exceeds 10 MiB (10485760 bytes). |

`build/` is gitignored. Copy `build/web-model/omnibot-pro/` (the URDF, `meshes/`, and `manifest.json`, not the zip) into the console as `public/models/omnibot-pro/`. `urdf-loader` should fetch `omnibot-pro.urdf`; the GLB paths resolve next to it. Animate `arm_joints` (the six SO-101 joints, including `arm_gripper`) and the four `mecanum_wheel_joints`.

The folder is not committed. The GLBs are a build product of `trimesh` / `fast-simplification` and would churn when those packages change. The script and `test/test_export_web_model.py` are the source of truth. Regenerate the browser folder after editing the xacro or the STLs. Regenerate `meshes/base/*.stl` with `scripts/generate_base_meshes.py` after editing the plate or wheel size properties.

### Check

`pytest omnibot-ros2/omnibot_description/test` runs the export in a temp directory. It checks the six arm joints (including the gripper) and the four mecanum wheel joints have limits, that mesh origins in the browser URDF match the xacro, that every mesh URI resolves to a GLB, that collision stays primitive, and that the zip is inside the budget. A second test fails if a URDF mesh URI is missing on disk or a file under `meshes/` is not referenced (`.md` notes are the exception). The CPU CI job runs these tests. They do not need ROS. `check_urdf` runs in the ROS CI job when that tool is installed.
