from csmn.const import Phase

__all__ = ["CanSatController", "Phase"]


def __getattr__(name):
    if name == "CanSatController":
        from csmn.ctrl import CanSatController

        return CanSatController
    raise AttributeError(f"module 'csmn' has no attribute {name!r}")
