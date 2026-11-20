# omnibot-ai-engines

This directory is a second copy of the six OhhO engines. It is not published
on PyPI, and it is not what `pip install ohho-os` installs.

Do not delete it in this state. The private repository
`ohho-robotics/ohho-ai-engines` is not a superset of this tree. Removing
the directory would drop modules that the ROS nodes import and that the
private tree does not contain.

The open-core recommendation, including the blob comparison from
20 November 2026, is
[ohho-os `docs/adr/0001-open-core-engines.md`](https://github.com/ohho-robotics/ohho-os/blob/main/docs/adr/0001-open-core-engines.md).
That record is proposed. It does not relicense this tree. This directory
stays under the Apache-2.0 licence of ohho-omnibot.

## What the ROS packages import from here

`omnibot_orchestration.agent_node` and `world_state_node` import:

- `agent_engine.core.harness` (this copy's harness blob differs from the private tree)
- `agent_engine.core.loop_trace` (this tree only)
- `agent_engine.navigation.waypoints` (this tree only)
- `agent_engine.reasoners.scripted_brain` (this tree only)
- `learning_engine.data.replay_dataset` (present in both trees)

`omnibot_lerobot` imports:

- `lerobot_engine.models` and `lerobot_engine.models.reference` (`reference` is this tree only)
- `lerobot_engine.serve` (this tree only)
- `data_engine.recording` and `data_engine.quality` (this tree only)
- `vla_engine.trt` (same blob in both trees)

`langchain_agent_node` does not import `agent_engine`. It is a separate
LangChain node. Leave it until the harness node is the supported agent.
Do not remove it in the same change as this directory.

`COLCON_IGNORE` keeps this tree out of the colcon build. The nodes still
load it with `pip install -e` or `PYTHONPATH`.

## Removal plan

1. Accept the ohho-os ADR, including which engines are public and under
   which licence.
2. Put the modules listed above into one package. The in-tree name on
   `agent_engine` is `omnibot-agent-engine` 0.1.0. Nothing from this
   directory is on PyPI today.
3. Point `agent_node`, `world_state_node`, and `omnibot_lerobot` at that
   package and rerun their tests.
4. Delete this directory in a follow-up once those tests pass.

Until step 3 is done, this tree is the copy those nodes run.
