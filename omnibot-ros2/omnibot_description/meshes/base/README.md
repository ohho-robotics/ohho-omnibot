# Simplified chassis and wheel meshes

These STLs are generated procedural stand-ins. They are not measured CAD
and they are not a scan of the robot.

| File | Link frame | What it stands in for |
|---|---|---|
| `chassis_plate.stl` | `base_link` and `top_plate_link` | The aluminium plate box (`chassis_length` × `chassis_width` × `plate_thickness`) |
| `mecanum_wheel_cs_pos.stl` | `front_left_wheel_link`, `rear_right_wheel_link` | Hub plus eight rollers, `\` chirality |
| `mecanum_wheel_cs_neg.stl` | `front_right_wheel_link`, `rear_left_wheel_link` | Hub plus eight rollers, `/` chirality |

- Units: metres.
- Axis convention: Z up. The mesh origin is the link origin. Wheel axle is link Y, matching the joint axis `0 1 0`.
- Source of the dimensions: numeric `xacro:property` values in `urdf/omnibot.urdf.xacro` (`chassis_length`, `chassis_width`, `plate_thickness`, `wheel_radius`, `wheel_width`, `hub_radius_scale`, `roller_radius`, `roller_length`).
- Generator: `scripts/generate_base_meshes.py`. Regenerate after those properties change. The solid name of each file starts with `simplified_`.
- Licence: Apache-2.0, same as this package. The geometry is original to that script.
- STEP: there is no chassis or wheel STEP file in this repo. A dimensioned CAD export (STEP, metres, with this same licence stated next to the file) is a follow-up. Do not treat these STLs as that CAD.

Collision geometry in the URDF stays the plate box and the wheel cylinder. Cameras, standoffs, and the other non-arm links stay primitive visuals.
