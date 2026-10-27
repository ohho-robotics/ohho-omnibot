# OmniBot AprilTags (OHH-223)

Nominal mounts for headset detection. The numbers below are **nominal, to be measured**. Nothing in this file was measured on the robot.

## Why these tags

OHH-213 (`OhhO-VR` branch `ohh-213-apriltag-spike`, `docs/adr/213-apriltag-passthrough.md`) detects the robot from the Quest 3/3S passthrough camera. `AprilTagSpikeConfig.Family` is `tag36h11`. Pose uses AprilRobotics `estimate_tag_pose` with the measured black-square side in metres. The spike's protocol is a matte print (not a monitor), distances 0.5 m to 3 m, and a gate at 1.5 m. A person standing at that gate looks down at the robot at roughly 40 degrees, so each tag's outward normal is 45 degrees above horizontal.

The black square is 80 mm, inside the 80–100 mm range from OHH-223. With the one-module white quiet zone that `tag36h11` needs, the paper is 100 mm square, which fits on A4 and on the back-left pad beside the arm. A 100 mm black square would be a 125 mm card and would not clear the pedestal.

IDs are the first three `tag36h11` codes (the same pictures as `tag36_11_00000.png`, `00001.png`, and `00002.png` in AprilRobotics `apriltag-imgs`):

| ID | Link | Where the face points |
|---:|---|---|
| 0 | `apriltag_front_link` | Forward and up, centred on the front lip. The jig overhangs the lip so the face clears the arm. |
| 1 | `apriltag_rear_link` | Backward and up, on the back-right. Shifted right so it is not collinear with the front tag and does not cover the arm-base tag. |
| 2 | `apriltag_arm_base_link` | Left and up, on the back-left of the pedestal. The waveshare plate is the arm mount, so the tag sits beside the base rather than on that plate. |

## Frame and `T_tag_base`

`ohho-sdk` on `ohh-214-robot-descriptor` already defines `apriltags`. Each entry has exactly three keys: `id` (integer >= 0), `size` (metres, greater than 0 and at most 1), and `T_tag_base` (16 finite numbers). No schema addition is required. This repo's YAML uses that contract. The issue name `T_tag_base_link` is this field: the tag frame relative to `base_link`.

`T_tag_base` is row-major. It maps a point in the tag frame into `base_link`:

```text
p_base = T_tag_base * p_tag
```

Elements 3, 7, and 11 are the tag origin in `base_link` (x, y, z). The tag frame matches AprilRobotics `estimate_tag_pose` (`apriltag_pose.c` object points):

- Origin at the centre of the black square, on the printed face.
- +X to the right when the tag is upright and facing the camera.
- +Y down, toward the bottom edge of that upright tag.
- +Z into the tag, away from the camera.

The outward normal (toward a person looking at the tag) is the negation of the matrix's third column. The OHH-213 spike currently uses the tag origin only. Keeping this frame means a later pose chain does not need an extra tag-frame rotation.

`size` is the black-square side in metres, the value passed to `estimate_tag_pose`. Nominal size is `0.080`.

## Nominal poses

`base_link` is the centre of the bottom plate. The top-plate centre is 166 mm above that. The top surface is 169 mm above `base_link`. Tag centres below are 39.4 mm above the top surface, which includes the 4 mm jig.

These poses are **nominal, to be measured**.

| ID | Link | Location | Size (m) | Origin in base_link (m) |
|---:|---|---|---:|---|
| 0 | apriltag_front_link | top plate front, centred, overhanging the front lip | 0.080 | 0.137855 0.000000 0.208355 |
| 1 | apriltag_rear_link | top plate back-right, low edge 5 mm in from the rear lip | 0.080 | -0.092145 -0.070000 0.208355 |
| 2 | apriltag_arm_base_link | beside the arm base, back-left, facing left and up | 0.080 | -0.078000 0.079855 0.208355 |

Tape check, black-square centre, still **nominal, to be measured**:

| ID | Forward of the rear lip, or past the front lip | Across the plate | Above the top surface |
|---:|---|---|---|
| 0 | 5.4 mm in front of the front lip, on the centreline | centred (125 mm from either side edge) | 39.4 mm |
| 1 | 40.4 mm forward of the rear lip | 55 mm in from the right edge | 39.4 mm |
| 2 | 54.5 mm forward of the rear lip | 45.1 mm in from the left edge | 39.4 mm |

Full `T_tag_base` matrices, row-major, **nominal, to be measured**:

ID 0

```text
 0.000000000  0.707106781 -0.707106781  0.137855339
 1.000000000  0.000000000  0.000000000  0.000000000
 0.000000000 -0.707106781 -0.707106781  0.208355339
 0.000000000  0.000000000  0.000000000  1.000000000
```

ID 1

```text
 0.000000000 -0.707106781  0.707106781 -0.092144661
-1.000000000  0.000000000  0.000000000 -0.070000000
 0.000000000 -0.707106781 -0.707106781  0.208355339
 0.000000000  0.000000000  0.000000000  1.000000000
```

ID 2

```text
-1.000000000  0.000000000  0.000000000 -0.078000000
 0.000000000  0.707106781 -0.707106781  0.079855339
 0.000000000 -0.707106781 -0.707106781  0.208355339
 0.000000000  0.000000000  0.000000000  1.000000000
```

The plate is 265 mm front-to-rear and 250 mm left-to-right. The same matrices are in `omnibot-ros2/omnibot_bringup/config/robot_descriptor.yaml`. The URDF joints are in `omnibot-ros2/omnibot_description/urdf/apriltags.urdf.xacro`. The geometry is computed in `omnibot-ros2/omnibot_description/apriltags/apriltag_mounts.py` from the chassis literals in `omnibot.urdf.xacro`.

## Print

From the repo root:

```text
python omnibot-ros2/omnibot_description/apriltags/generate_printables.py
```

That rewrites:

- `omnibot-ros2/omnibot_description/apriltags/print/omnibot_tag36h11.pdf`
- `omnibot-ros2/omnibot_description/apriltags/print/apriltag_mount_jig.stl` (millimetres)
- `omnibot-ros2/omnibot_description/apriltags/print/apriltag_mount_jig.scad` (same solid)
- `omnibot-ros2/omnibot_description/meshes/apriltags/tag36h11_id{0,1,2}.{png,obj,mtl}`
- `omnibot-ros2/omnibot_description/urdf/apriltags.urdf.xacro`

Print the PDF at 100% (actual size) on matte paper. Do not scale to fit. Do not print onto a monitor, and do not laminate: OHH-213 calls out a matte print because glare breaks detection. Each page has a 100 mm scale bar. The black square on each tag page must measure 80 mm. If either check fails, fix the printer scale before mounting. The top of each tag page is the top of the tag.

## Jig and mount

The STL is one ramp, reused three times. Units are millimetres. The flat underside (`z = 0`) tapes to the top plate. The low end (`x = 0`, the 4 mm face) points outward, toward the person who should see that tag. The tall end points inward. The slope is 100 mm at 45 degrees, the same size as the paper. Centre the card on the slope with the top of the printed tag at the tall end.

- ID 0: low end forward. The tall end starts 10 mm ahead of the arm body, and the ramp overhangs the front lip. Tape the part of the underside that still sits on the plate.
- ID 1: low end backward, on the back-right. The low edge of the card is 5 mm in from the rear lip.
- ID 2: low end to the robot's left, on the back-left of the pedestal, tall end toward the pedestal.

Home the arm (every joint at 0) before measuring, so the arm matches the URDF. The three cards stay clear of the pedestal in that pose. A raised arm can hide a tag from some directions; the other two faces cover the other sides.

## Calibration (tape, then camera)

Do this on the real robot. Do not write a measured matrix until both checks agree.

1. Confirm the scale bar is 100 mm and each black square is 80.0 mm, plus or minus 0.5 mm. If the square is a different length, the descriptor `size` for that tag becomes the measured length in metres, not 0.080.
2. Mount the three jigs as above. Photograph all three faces in one view if you can, and one close view of each ID. That photo is the OHH-223 mounting evidence.
3. Tape the black-square centre against the millimetre table above (plate edges and the top surface). Record each residual. A residual larger than 2 mm means the jig moved or the print scaled; do not treat the CAD matrix as measured.
4. Camera check, one tag at a time. Place the camera 1.50 m from the black-square centre along the outward normal (45 degrees up and outward). The outward normal is the negation of `T_tag_base` column 2 (elements 2, 6, and 10, counting from 0). For the front tag that direction is about `(0.707, 0, 0.707)`, so the camera is about 1.06 m forward of the tag and 1.06 m above it.
5. Run `estimate_tag_pose` with `tagsize` equal to the measured black-square side. With the optical axis on the normal and the tag upright, the tag origin in the camera frame should be near `(0, 0, 1.50)` m. A miss larger than 2 cm in range or sideways means the mount or the nominal pose is wrong. This is a CAD check. It is not the OHH-213 detector gate (that gate compares the detector with a manually placed transform).
6. If the picture is upside down, or a full pose shows a yaw near 180 degrees, the top of the sheet is at the low end. Remount it. The OHH-213 ADR treats a yaw near 180 degrees as a frame error, not as a passing measurement.
7. When tape and camera agree, replace that tag's `T_tag_base` in `robot_descriptor.yaml` with the measured row-major matrix, set `size` to the measured black-square side, and remove the nominal wording for that tag. Leave the other two nominal until they are measured too.

## Gazebo

`omnibot.urdf.xacro` includes `apriltags.urdf.xacro` once. That file is only the three textured planes (OBJ + PNG, matte). It adds no cameras and no sensors. Launch files and the `vr:=true` profile are unchanged. A chase camera in sim should see the same IDs at these nominal poses.

## Tests

```text
pytest omnibot-ros2/omnibot_bringup/test/test_descriptor.py
```

The descriptor test loads the YAML, checks the ohho-sdk tag keys, and checks that each `T_tag_base` matches the URDF joint chain back to `base_link`.
