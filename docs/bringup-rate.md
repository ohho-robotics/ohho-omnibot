# Bring-up control rates (OHH-97)

Pi and GPU workstation share one ROS domain (`ROS_DOMAIN_ID` in
[`omnibot-ros2/omnibot_bringup/config/ros_domain.env`](../omnibot-ros2/omnibot_bringup/config/ros_domain.env)).
This document records measured topic rates from a **real** dual-machine session.

## Session metadata

| Field | Value |
|---|---|
| Date (UTC) | TODO: Varun — session date |
| `ROS_DOMAIN_ID` | TODO: confirm matches env file |
| Pi IP | TODO: `OMNIBOT_PI_IP` |
| Workstation IP | TODO: `OMNIBOT_WORKSTATION_IP` |
| Pi launch | TODO: e.g. `ros2 launch omnibot_bringup master/omnibot_pi.launch.py` |
| Workstation launch | TODO: e.g. `ros2 launch omnibot_bringup master/omnibot_workstation.launch.py` |
| Policy node / rate | TODO: e.g. `policy_inference.launch.py`, `policy_hz` parameter |

## `ros2 topic hz` measurements

Run on a machine that sees both publishers (typically the workstation), after
both stacks are up and the policy is enabled (`/policy/enable` true).

```bash
# Arm feedback from the Pi (expect ~driver rate, often 10–50 Hz depending on config)
ros2 topic hz /arm/joint_states

# Policy arm output (expect ~policy_hz, default 10 Hz when inference keeps up)
ros2 topic hz /arm/joint_commands
```

### `/arm/joint_states`

| Metric | Value |
|---|---|
| average rate | TODO: Hz from `ros2 topic hz` |
| min / max | TODO |
| window / notes | TODO: command used, duration, any drops |

### Policy output (`/arm/joint_commands`)

| Metric | Value |
|---|---|
| average rate | TODO: Hz from `ros2 topic hz` |
| min / max | TODO |
| window / notes | TODO: policy enabled, missed-period behavior observed |

Optional: also record `/cmd_vel/vla` if base policy output matters for the session.

## Late policy period (expected behavior)

When a policy tick exceeds `policy_period` (1 / `policy_hz`), `policy_node`
must **not** publish a stale arm command. It skips `/arm/joint_commands` for
that tick and publishes zero base velocity on `/cmd_vel` (remapped to
`/cmd_vel/vla` in launch). Confirm under load if inference occasionally exceeds
the budget.

## Human checklist

- [ ] Varun confirmed Pi and workstation IPs in `ros_domain.env`
- [ ] Both machines use the same `ROS_DOMAIN_ID`
- [ ] `ros2 topic hz` table above filled from a real session (no fabricated numbers)
