import ctypes
import os

# torch's Jetson JP6/cu126 build dlopen()s libcudss.so.0 lazily, but the
# nvidia-cudss-cu12 wheel doesn't ship an RPATH pointing at its own bundled
# libs the way the official cu12 wheels do. Preloading them here (triggered
# by the sibling .pth file at every interpreter startup) makes them resolvable
# without requiring LD_LIBRARY_PATH, so `import torch` works under `uv run`,
# a directly-invoked venv python, or a sourced activate script alike.
_LOAD_ORDER = (
    "libcudss_commlayer_openmpi.so.0",
    "libcudss_commlayer_nccl.so.0",
    "libcudss_mtlayer_gomp.so.0",
    "libcudss.so.0",
)


def _preload() -> None:
    try:
        import nvidia.cu12
    except ImportError:
        return

    # nvidia.cu12 is a PEP 420 namespace package (no __init__.py / __file__),
    # so locate it via __path__ instead.
    cu12_dirs = list(nvidia.cu12.__path__)
    if not cu12_dirs:
        return
    lib_dir = os.path.join(cu12_dirs[0], "lib")
    if not os.path.isdir(lib_dir):
        return

    for name in _LOAD_ORDER:
        path = os.path.join(lib_dir, name)
        if os.path.exists(path):
            try:
                ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass


_preload()
