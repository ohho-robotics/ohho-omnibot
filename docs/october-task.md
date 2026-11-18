# October pick-and-place (OHH-98)

Owner: Varun. This file is a fill-in template. It freezes **one** pick-and-place task for October once every `TODO(Varun)` below is replaced. Until then the task is not frozen.

Docs only. Do not change hardware, launch files, or robot parameters from this page. Do not add photos here. Agents must not insert images.

## One task this month

Agents must not invent a second task this month. October has a single pick-and-place task, and it is only what Varun writes into the blanks in this file. Do not add another object, bin, scene, sentence, or dataset task.

## Task string

One sentence. After Varun fills it, that exact sentence is the only task text for recording and for `/policy/task`.

- Recording: LeRobot `meta/tasks.jsonl` field `task` (see the empty example below), and the recorder task description passed into episode save.
- Policy: publish the same sentence on `/policy/task` (`std_msgs/String`, field `data`). `policy_node` stores it as `task_description`.

```
TODO(Varun): one sentence for the single pick-and-place.
```

Do not copy an example sentence from the training guide. Leave this blank until Varun writes it.

## Object

```
TODO(Varun): what is picked up (identity only; no stand-in object).
```

## Bin

```
TODO(Varun): what it is placed into, and where that bin sits.
```

## Table marks

```
TODO(Varun): marks on the table that define the workspace (what they are, where they are).
```

## Camera mounts

```
TODO(Varun): which cameras, and how each mount is fixed for this task.
```

Wrist and bird's-eye streams used by the dataset are `observation.images.wrist` and `observation.images.bev`. Say which physical cameras feed those views. Do not invent mount positions.

## Start pose

```
TODO(Varun): base pose and arm pose at the start of every episode.
```

## Reset instructions (second person)

Stub for the person who resets the scene between episodes. They follow only the steps Varun writes here. They do not choose a different layout.

1. TODO(Varun): put the object back.
2. TODO(Varun): put the bin back.
3. TODO(Varun): check the table marks.
4. TODO(Varun): return the base and arm to the start pose.
5. TODO(Varun): say what "ready" means before the next recording.

## tasks.jsonl example

Empty on purpose (no task line yet):

[`docs/examples/october-task/tasks.jsonl`](examples/october-task/tasks.jsonl)

After the task sentence above is filled, that file gets one line and nothing else:

```json
{"task_index": 0, "task": "<the one sentence from Task string>"}
```
