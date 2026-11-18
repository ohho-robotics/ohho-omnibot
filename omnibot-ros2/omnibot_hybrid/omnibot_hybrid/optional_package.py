# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Skip launch nodes whose ROS package is not installed.

`omnibot_perception`, `omnibot_metrics`, and `omnibot_ota` are not part of
this repository. Launch files wrap those nodes in :func:`PackageAvailable`
so a public workspace logs one warning and does not call the package index
for a missing install. The same helper covers other packages that cannot be
declared as ``exec_depend`` (rosdep would fail).

Importing this module does not import ``launch`` or ``ament_index``. Those
imports happen when a launch file actually builds an action.
"""

from __future__ import annotations

from collections.abc import Callable

# Call names the launch-package scanner treats as a guard. A ``package=``
# reference is guarded when it sits on one of these calls, or when the same
# call's ``condition`` argument names the package through one of them.
GUARD_CALLS = frozenset(
    {
        "PackageAvailable",
        "optional_include_launch",
        "optional_node",
        "optional_package_condition",
    }
)

_TRUTHY = frozenset({"1", "true", "yes", "on"})


def missing_package_message(package_name: str) -> str:
    """One warning line. The package name is the text operators search for."""
    return (
        f"Skipping optional node: package '{package_name}' is not installed "
        "in this workspace."
    )


def coerce_enabled(value: object) -> bool:
    """ROS launch truthiness for a performed substitution (``true`` / ``1``)."""
    return str(value).strip().lower() in _TRUTHY


def package_is_available(
    package_name: str,
    *,
    lookup: Callable[[str], object] | None = None,
) -> bool:
    """Return whether ``package_name`` is on the ament index.

    ``lookup`` replaces ``get_package_share_directory`` in unit tests. A
    ``PackageNotFoundError`` (matched by class name so tests need not import
    ament) means the package is absent. Any other exception propagates.
    """
    resolve = lookup or _default_share_directory
    try:
        resolve(package_name)
    except Exception as exc:
        if type(exc).__name__ == "PackageNotFoundError":
            return False
        raise
    return True


def _default_share_directory(package_name: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory(package_name)


def should_start_optional_node(
    package_name: str,
    enabled: bool,
    *,
    is_available: Callable[[str], bool] | None = None,
    warn_when_disabled: bool = True,
) -> tuple[bool, str | None]:
    """Decide whether to start the node, and the warning to log if any.

    Missing packages never start. By default a missing package warns even
    when the launch argument is false, so a public checkout reports the skip
    on the default command. Set ``warn_when_disabled`` false when the node
    is intentionally off (for example simulation) and silence is correct.
    An installed package that the argument turns off is skipped with no warning.
    """
    available = (is_available or package_is_available)(package_name)
    if not available:
        if enabled or warn_when_disabled:
            return False, missing_package_message(package_name)
        return False, None
    if not enabled:
        return False, None
    return True, None


class WarnOnce:
    """Emit the first non-empty warning and drop the rest."""

    def __init__(self) -> None:
        self._sent = False

    def __call__(self, warning: str | None, emit: Callable[[str], None]) -> None:
        if warning and not self._sent:
            emit(warning)
            self._sent = True


def _perform_enabled(enabled: object, context: object) -> str:
    if isinstance(enabled, str):
        return enabled
    perform = getattr(enabled, "perform", None)
    if callable(perform):
        return str(perform(context))
    from launch.utilities import (
        normalize_to_list_of_substitutions,
        perform_substitutions,
    )

    return perform_substitutions(
        context, normalize_to_list_of_substitutions(enabled)
    )


def _launch_warning(message: str) -> None:
    from launch.logging import get_logger

    get_logger("omnibot.optional_package").warning(message)


def PackageAvailable(
    package: str,
    enabled: object,
    *,
    warn_when_disabled: bool = True,
):
    """Launch condition: start only if ``enabled`` and ``package`` is installed.

    When the package is missing, log :func:`missing_package_message` once and
    leave the node out of the launch so ROS does not raise
    ``PackageNotFoundError``. ``enabled`` is a ``LaunchConfiguration``,
    another substitution, or the strings ``true`` / ``false``.
    """
    from launch.conditions import Condition

    warn_once = WarnOnce()

    class _PackageAvailable(Condition):
        def __init__(self) -> None:
            super().__init__(predicate=self._predicate)

        def _predicate(self, context: object) -> bool:
            start, warning = should_start_optional_node(
                package,
                coerce_enabled(_perform_enabled(enabled, context)),
                warn_when_disabled=warn_when_disabled,
            )
            warn_once(warning, _launch_warning)
            return start

        def describe(self) -> str:
            return f"package '{package}' available and enabled"

    return _PackageAvailable()


# Scanner alias. Launch files should call PackageAvailable.
optional_package_condition = PackageAvailable


def optional_include_launch(
    *,
    package: str,
    relative_path: tuple[str, ...],
    when: object,
    launch_arguments: dict[str, str] | None = None,
    warn_when_disabled: bool = False,
):
    """Include another package's launch file only when that package exists.

    ``get_package_share_directory`` runs inside the opaque function, after
    the availability check. A missing package cannot fail description parsing.
    """
    from launch.actions import OpaqueFunction

    arguments = dict(launch_arguments or {})
    warn_once = WarnOnce()

    def _include(context, *args, **kwargs):
        del args, kwargs
        start, warning = should_start_optional_node(
            package,
            coerce_enabled(_perform_enabled(when, context)),
            warn_when_disabled=warn_when_disabled,
        )
        warn_once(warning, _launch_warning)
        if not start:
            return []
        import os

        from ament_index_python.packages import get_package_share_directory
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import AnyLaunchDescriptionSource

        path = os.path.join(get_package_share_directory(package), *relative_path)
        include_kwargs = {}
        if arguments:
            include_kwargs["launch_arguments"] = list(arguments.items())
        return [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(path),
                **include_kwargs,
            )
        ]

    return OpaqueFunction(function=_include)
