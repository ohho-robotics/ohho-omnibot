# Contributing to OmniBot

OmniBot (`ohho-omnibot`) is the open reference robot of OHHO ROBOTICS LTD. It is
licensed under [Apache-2.0](LICENSE) and welcomes outside contributions.

## Licence of contributions

By contributing, you agree that your contribution is licensed under Apache-2.0,
the licence of this repository (section 5 of the Apache License).

## Developer Certificate of Origin (DCO)

Every commit from an outside contributor must be signed off under the
[Developer Certificate of Origin 1.1](https://developercertificate.org/). The
sign-off certifies that you wrote the change, or otherwise have the right to
submit it under Apache-2.0.

Add the sign-off with `git commit -s`. It adds a line like this to the commit
message, using your real name and email:

```text
Signed-off-by: Jane Doe <jane@example.com>
```

To sign off commits you already made, run `git rebase --signoff main` and force
push your branch. Pull requests with unsigned commits are not merged. There is no
separate contributor licence agreement (CLA).

## Boundary with OhhO OS

OhhO OS, the engine behind the OHHO ROBOTICS LTD commercial products, is
proprietary and closed source. OmniBot stays open. Keep them separate:

- Do not copy, vendor, bundle or embed source, binaries or models from closed
  OhhO OS (ohho-os after the `last-apache-2.0` tag, OhhO Planning, OhhO Fleet,
  OhhO Verified) into this repository.
- Talk to OhhO OS only through public interfaces: the documented ROS 2 topics,
  services and actions in [docs/reference.md](docs/reference.md) and
  [docs/architecture.md](docs/architecture.md), and the
  published `ohho` CLI and API of `ohho-os` 1.1.2 (Apache-2.0).
- OmniBot must keep working without a licence for closed OhhO OS. Closed
  integrations stay optional and live outside this repository.

## Before you open a pull request

- Read [AGENTS.md](AGENTS.md) for the build and test commands.
- Run the CPU checks: `ruff check .` and the test suites in
  [`.github/workflows/ci.yml`](.github/workflows/ci.yml).
- Nothing in a pull request may send commands to a real robot from CI.

The code of conduct is in the [org `.github` repository](https://github.com/ohho-robotics/.github).
