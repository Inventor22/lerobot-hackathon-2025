"""
This package gathers shared sub-modules such as `motors`, `cameras`, `robots`, etc.
Having an explicit `__init__.py` makes the package visible to static
analysis tools (e.g. Pyright, Pylance) and avoids false positive errors like:
    Import "lerobot.common.motors.feetech" could not be resolved
The file intentionally leaves the public API to the respective
sub-packages to avoid unnecessary import overhead.
"""

# Eagerly import key sub-packages so that static analyzers (e.g. Pyright, Pylance)
# resolve `lerobot.common.<subpkg>` fully. This has negligible runtime cost and
# avoids further "import could not be resolved" warnings.
from importlib import import_module as _import_module

for _sub in ("cameras", "motors", "robots", "teleoperators"):
    try:
        _import_module(f"{__name__}.{_sub}")
    except ModuleNotFoundError:
        # Optional dependency missing – ignore.
        pass 