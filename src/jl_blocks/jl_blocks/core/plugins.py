"""Load researchers' own blocks from .py files, so their @block classes register."""

from __future__ import annotations

import hashlib
import importlib.util
import sys
from pathlib import Path


def load_block_files(paths: list[str]) -> list[str]:
    """Import each .py file in `paths` (files, or the top level of folders).

    Returns one readable error per file or path that failed; never raises, so
    `jl_blocks check` and the runner can print every problem at once.
    """
    errors: list[str] = []
    files: list[Path] = []
    for raw in paths:
        path = Path(raw)
        if path.is_dir():
            files.extend(sorted(path.glob("*.py")))
        elif path.is_file():
            files.append(path)
        else:
            errors.append(f"{path}: no such file or folder")
    for file in files:
        digest = hashlib.sha1(str(file.resolve()).encode()).hexdigest()[:12]
        module_name = f"_jl_user_blocks_{file.stem}_{digest}"
        spec = importlib.util.spec_from_file_location(module_name, file)
        if spec is None or spec.loader is None:
            errors.append(f"{file}: cannot be imported")
            continue
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as err:
            sys.modules.pop(module_name, None)
            errors.append(f"{file}: {type(err).__name__}: {err}")
    return errors
