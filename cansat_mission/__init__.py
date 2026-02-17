from cansat_mission.constants import Phase

__all__ = ["CanSatController", "Phase"]


def __getattr__(name):
    if name == "CanSatController":
        from cansat_mission.controller import CanSatController

        return CanSatController
    raise AttributeError(f"module 'cansat_mission' has no attribute {name!r}")
