"""fusion2urdf - Modern Fusion 360 to URDF / ROS 2 / OpenUSD exporter.

The core package is intentionally stdlib-only so the same code runs both on a
desktop Python and inside Fusion 360's embedded interpreter (vendored into the
add-in folder by scripts/sync_addin.py).
"""

__version__ = "0.1.0"

from .model import Robot, Link, Joint, Inertial, Origin, Material  # noqa: F401
