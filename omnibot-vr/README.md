# omnibot-vr (retired)

This folder was a second copy of the Meta Quest app. It was retired under OHH-192.

The canonical Quest / VR app is [OhhO-VR](https://github.com/ohho-robotics/OhhO-VR).

`Assets/Scripts` in this tree matched OhhO-VR (Unity 6000.5.2f1, changeset `eb73d3b415a1`). No ROS 2 launch file starts it, and CI does not build a Quest APK from it. The Unity project files were removed so this repository carries one pointer instead of a second VR app.

OmniBot paths that used to depend on this folder:

| Path | Former dependency |
|---|---|
| [`README.md`](../README.md) | Hardware table and layout treated this directory as the Unity Quest project |
| [`docs/reference.md`](../docs/reference.md) | Quest teleop row cited `ProjectSettings/ProjectVersion.txt` |
| [`AGENTS.md`](../AGENTS.md) | Told agents to open this folder as a Unity project |
| [`NOTICE`](../NOTICE) | Listed vendored Meta XR SDK, unity-mcp, and TextMesh Pro licences here |
| [`.gitattributes`](../.gitattributes) | Marked the vendored Unity packages as Linguist-vendored |
| [`ruff.toml`](../ruff.toml) | Excluded `omnibot-vr/Packages` from lint |

Those pointers now refer to OhhO-VR, or the exclusion was dropped with the tree. `.gitleaks.toml` still allowlists three old Meta XR SDK paths under this folder; that file was left unchanged.
