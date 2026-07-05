"""OpenUSD export via the newton-physics converters.

Two complementary backends, both from https://github.com/newton-physics:

* ``urdf-usd-converter``   URDF -> USD with UsdPhysics (rigid bodies, joints,
  masses). Loads in Isaac Sim, usdview, any OpenUSD runtime.
* ``mujoco-usd-converter`` MJCF -> USD with MuJoCo/Newton attributes (Mjc*)
  on top of UsdPhysics — the Newton-ready flavor.

Post-processing: the emitted assets can be restructured into NVIDIA's Isaac
Sim asset layout (payloads/, physics variants, robot schema) with the Asset
Transformer. We ship a rule-profile JSON and a runner script wired to
Isaac Sim's ``isaacsim.asset.transformer`` extension (Isaac Sim >= 6.0).

A helper script for a classic full physics import (``URDFImporter``) is also
emitted for users who prefer the importer path.
"""

from __future__ import annotations

import json
from pathlib import Path

ISAAC_IMPORT_SCRIPT = '''\
"""Import {robot}.urdf into Isaac Sim with full physics (run inside Isaac Sim).

Usage:
    $ISAAC_SIM_DIR/python.sh isaac_sim_import.py

Requires Isaac Sim >= 5.0 (isaacsim.asset.importer.urdf extension).
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({{"headless": True}})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.asset.importer.urdf")

from isaacsim.asset.importer.urdf import (  # noqa: E402
    URDFImporter,
    URDFImporterConfig,
)

import os  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

config = URDFImporterConfig(
    urdf_path=os.path.join(HERE, "{robot}.urdf"),
    usd_path=os.path.join(HERE, "isaac_usd"),
    merge_fixed_joints=False,
    fix_base={fix_base},
    collision_from_visuals=True,
    collision_type="Convex Decomposition",
    joint_drive_type="force",
    joint_target_type="position",
    robot_type="{robot_type}",
)
output_usd = URDFImporter(config).import_urdf()
print("Imported:", output_usd)

simulation_app.close()
'''

TRANSFORMER_SCRIPT = '''\
"""Restructure {robot}.usda into the Isaac Sim asset layout (payloads/,
physics variants, Isaac robot schema) using the Asset Transformer.

Usage:
    $ISAAC_SIM_DIR/python.sh run_asset_transformer.py

Requires Isaac Sim >= 6.0 (isaacsim.asset.transformer extension). The
"Isaac Sim Structure" profile ships with the isaacsim.asset.transformer.rules
extension; asset_transformer_profile.json in this folder is a trimmed copy
you can customize (rule format per
https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/asset_transformer.html).
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({{"headless": True}})

from isaacsim.asset.transformer import (  # noqa: E402
    AssetTransformerManager,
    RuleProfile,
)

HERE = os.path.dirname(os.path.abspath(__file__))
INPUT = os.path.join(HERE, "{input_usd}")
PROFILE = os.path.join(HERE, "asset_transformer_profile.json")
OUTPUT = os.path.join(HERE, "{robot}_package")

with open(PROFILE, "r", encoding="utf-8") as f:
    profile = RuleProfile.from_json(f.read())

manager = AssetTransformerManager()
report = manager.run(
    input_stage=INPUT,
    profile=profile,
    package_root=OUTPUT,
)
for result in report.results:
    status = "[PASS]" if result.success else "[FAIL]"
    print(status, result.rule.name, result.error or "")
print("Output package:", OUTPUT)

simulation_app.close()
'''

# Trimmed Isaac Sim Structure profile (source: isaacsim.asset.transformer.rules
# data/isaacsim_structure.json, Isaac Sim 6.0): routes physics schemas/prims
# into payload layers, applies the Isaac robot schema, splits geometry and
# materials, and generates the variant interface asset.
ASSET_TRANSFORMER_PROFILE = {
    "profile_name": "fusion2urdf Isaac Sim Structure",
    "version": "1.0",
    "rules": [
        {
            "name": "Flatten Base",
            "type": "isaacsim.asset.transformer.rules.structure.flatten.FlattenRule",
            "destination": "payloads",
            "params": {
                "output_path": "base.usd",
                "selected_variants": {},
                "clear_variants": False,
                "case_insensitive": True,
            },
            "enabled": True,
        },
        {
            "name": "Apply Joint State APIs",
            "type": "isaacsim.asset.transformer.rules.isaac_sim.joint_state_api.JointStateAPIRule",
            "destination": "payloads",
            "params": {},
            "enabled": True,
        },
        {
            "name": "Route Geometries",
            "type": "isaacsim.asset.transformer.rules.perf.geometries.GeometriesRoutingRule",
            "destination": "payloads",
            "params": {
                "geometries_layer": "geometries.usd",
                "instance_layer": "instances.usda",
                "deduplicate": True,
            },
            "enabled": True,
        },
        {
            "name": "Route Materials",
            "type": "isaacsim.asset.transformer.rules.perf.materials.MaterialsRoutingRule",
            "destination": "payloads",
            "params": {
                "materials_layer": "materials.usda",
                "assets_folder": "Textures",
                "download_textures": True,
            },
            "enabled": True,
        },
        {
            "name": "Route MuJoCo Schemas",
            "type": "isaacsim.asset.transformer.rules.core.schemas.SchemaRoutingRule",
            "destination": "payloads/Physics",
            "params": {"stage_name": "mujoco.usda", "schemas": ["Mjc.*", "mjc.*"]},
            "enabled": True,
        },
        {
            "name": "Route MuJoCo Prims",
            "type": "isaacsim.asset.transformer.rules.core.prims.PrimRoutingRule",
            "destination": "payloads/Physics",
            "params": {"stage_name": "mujoco.usda", "prim_types": ["Mjc.*", "mjc.*"]},
            "enabled": True,
        },
        {
            "name": "Route Physics Schemas",
            "type": "isaacsim.asset.transformer.rules.core.schemas.SchemaRoutingRule",
            "destination": "payloads/Physics",
            "params": {
                "stage_name": "physics.usda",
                "schemas": ["Physics.*", "Newton.*"],
                "ignore_schemas": ["PhysicsCollisionAPI"],
                "prim_names": [".*"],
                "ignore_prim_names": [],
            },
            "enabled": True,
        },
        {
            "name": "Route Physics Prims",
            "type": "isaacsim.asset.transformer.rules.core.prims.PrimRoutingRule",
            "destination": "payloads/Physics",
            "params": {
                "stage_name": "physics.usda",
                "prim_types": ["Physics.*", "Newton.*"],
            },
            "enabled": True,
        },
        {
            "name": "Make Robot Schema",
            "type": "isaacsim.asset.transformer.rules.isaac_sim.robot_schema.RobotSchemaRule",
            "destination": "payloads",
            "params": {
                "prim_path": "",
                "stage_name": "robot.usda",
                "add_sites": True,
                "sites_last": False,
                "sublayer": "Physics/physics.usda",
            },
            "enabled": True,
        },
        {
            "name": "Generate Interface",
            "type": "isaacsim.asset.transformer.rules.structure.interface.InterfaceConnectionRule",
            "destination": "",
            "params": {
                "base_layer": "payloads/base.usda",
                "base_connection_type": "Reference",
                "generate_folder_variants": True,
                "payloads_folder": "payloads",
                "connections": [
                    {
                        "asset_path": "payloads/base.usda",
                        "target_path": "payloads/robot.usda",
                        "connection_type": "Sublayer",
                    }
                ],
                "default_variant_selections": {},
            },
            "enabled": True,
        },
    ],
    "interface_asset_name": None,
    "output_package_root": None,
    "flatten_source": False,
    "base_name": "base.usd",
}


def _import_converters() -> dict:
    """Import both newton-physics converters together (best effort).

    PITFALL: each converter registers its USD schema plugin (mjcPhysics,
    newton) at import time, but pxr's ``UsdSchemaRegistry`` is a singleton
    that seals the known API schemas on first use. If ``mujoco_usd_converter``
    is imported *after* a URDF conversion already touched the registry,
    ``ApplyAPI("MjcSceneAPI")`` fails with ``_ReportInvalidSchemaError``.
    Importing every available converter before the first conversion makes the
    call order irrelevant.
    """
    mods = {}
    for name in ("urdf_usd_converter", "mujoco_usd_converter"):
        try:
            mods[name] = __import__(name)
        except ImportError:
            mods[name] = None
    return mods


def convert_urdf_to_usd(
    urdf_path: str | Path,
    output_dir: str | Path,
    comment: str = "Generated by fusion2urdf",
) -> Path:
    """URDF -> USD via newton-physics urdf-usd-converter.

    Returns the main USD layer path. Raises ImportError with an actionable
    message when the optional dependency is missing.
    """
    mods = _import_converters()
    if mods["urdf_usd_converter"] is None:
        raise ImportError(
            "urdf-usd-converter is not installed. "
            "Install with: pip install 'fusion2urdf[usd]' "
            "or: pip install urdf-usd-converter"
        )

    urdf_path = Path(urdf_path).resolve()
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    converter = mods["urdf_usd_converter"].Converter(comment=comment)
    asset = converter.convert(str(urdf_path), str(output_dir))
    return Path(str(asset.path if hasattr(asset, "path") else asset))


def convert_mjcf_to_usd(
    mjcf_path: str | Path,
    output_dir: str | Path,
    comment: str = "Generated by fusion2urdf",
) -> Path:
    """MJCF -> USD via newton-physics mujoco-usd-converter (Newton-ready:
    carries Mjc* attributes on top of UsdPhysics)."""
    mods = _import_converters()
    if mods["mujoco_usd_converter"] is None:
        raise ImportError(
            "mujoco-usd-converter is not installed. "
            "Install with: pip install 'fusion2urdf[usd]' "
            "or: pip install mujoco-usd-converter"
        )

    mjcf_path = Path(mjcf_path).resolve()
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    converter = mods["mujoco_usd_converter"].Converter(comment=comment)
    asset = converter.convert(str(mjcf_path), str(output_dir))
    return Path(str(asset.path if hasattr(asset, "path") else asset))


def write_isaac_import_script(
    robot_name: str,
    output_dir: str | Path,
    fix_base: bool = True,
    robot_type: str = "Manipulator",
) -> Path:
    """Emit a ready-to-run Isaac Sim physics import script next to the URDF."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    script = ISAAC_IMPORT_SCRIPT.format(
        robot=robot_name, fix_base=fix_base, robot_type=robot_type
    )
    path = output_dir / "isaac_sim_import.py"
    path.write_text(script, encoding="utf-8")
    return path


def write_asset_transformer_files(
    robot_name: str,
    output_dir: str | Path,
    input_usd: str | None = None,
) -> tuple[Path, Path]:
    """Emit the Asset Transformer rule profile + runner script next to the
    converted USD. Returns (profile_path, script_path)."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    profile_path = output_dir / "asset_transformer_profile.json"
    profile_path.write_text(
        json.dumps(ASSET_TRANSFORMER_PROFILE, indent=2) + "\n", encoding="utf-8"
    )

    script_path = output_dir / "run_asset_transformer.py"
    script_path.write_text(
        TRANSFORMER_SCRIPT.format(
            robot=robot_name, input_usd=input_usd or f"{robot_name}.usda"
        ),
        encoding="utf-8",
    )
    return profile_path, script_path
