# fusion2urdf

Modern Fusion 360 to URDF / ROS 2 / OpenUSD exporter.

A ground-up rewrite of the classic
[syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf) lineage
(including the [SpaceMaster85](https://github.com/SpaceMaster85/fusion2urdf)
and [runtimerobotics](https://github.com/runtimerobotics/fusion360-urdf-ros2)
forks), designed for current tooling:

* **URDF** — plain, self-contained `robot.urdf` with relative mesh paths,
  directly importable by Isaac Sim's URDF importer.
* **ROS 2** — complete `<robot>_description` package targeting ROS 2 Jazzy:
  xacro, `ros2_control` hardware description, RViz2 + Gazebo Sim (gz-sim)
  launch files, `ros_gz_bridge` config, ament_cmake build files.
* **MJCF** — MuJoCo XML (validated against the real `mujoco` compiler in the
  test suite), the input for the Newton USD path.
* **OpenUSD** — two flavors, both via the
  [newton-physics](https://github.com/newton-physics) converters:
  * `usd`: URDF -> USD with `UsdPhysics` via
    [`urdf-usd-converter`](https://github.com/newton-physics/urdf-usd-converter)
    (Z-up, meters, rigid bodies + joints).
  * `usd-newton`: MJCF -> USD via
    [`mujoco-usd-converter`](https://github.com/newton-physics/mujoco-usd-converter),
    carrying MuJoCo/Newton (`mjc:*`) physics attributes on top of
    `UsdPhysics` — ready for the Newton physics engine.

  Both USD outputs ship with an `asset_transformer_profile.json` +
  `run_asset_transformer.py` so you can restructure them into the official
  [Isaac Sim asset layout](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/asset_transformer.html)
  (payloads/, physics variants, Isaac robot schema) with Isaac Sim >= 6.0.
  A generated `isaac_sim_import.py` (classic `URDFImporter` path) is also
  emitted next to the URDF.

## Architecture

```
Fusion 360 add-in (addin/URDF_Exporter_Plus)
        |  extracts joints, inertials, materials; exports STL meshes
        v
export bundle:  robot.json + meshes/*.stl        <- intermediate representation
        |
        v
fusion2urdf CLI (desktop python)
        |-- out/urdf/robot.urdf + meshes/ + isaac_sim_import.py
        |-- out/ros2/<robot>_description/        (colcon-buildable package)
        |-- out/mjcf/<robot>.xml + meshes/       (MuJoCo)
        |-- out/usd/<robot>.usda + Payload/      (UsdPhysics, urdf-usd-converter)
        `-- out/usd_newton/<robot>.usda          (Mjc*/Newton, mujoco-usd-converter)
```

The core (`src/fusion2urdf/`) is **stdlib-only**, so the exact same code runs
inside Fusion 360's embedded Python (vendored into the add-in) and on the
desktop. The add-in generates URDF and the ROS 2 package immediately; the USD
target needs pip dependencies, so it runs on the desktop via the CLI.

## Installation

### Desktop CLI

```bash
pip install -e ".[usd]"       # or ".[dev]" for tests
```

### Fusion 360 script

Copy `addin/URDF_Exporter_Plus` into Fusion's scripts folder:

Windows (PowerShell):

```powershell
Copy-Item ".\addin\URDF_Exporter_Plus\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
```

macOS:

```bash
cp -r ./addin/URDF_Exporter_Plus "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/"
```

Then in Fusion 360: `UTILITIES -> ADD-INS -> Scripts` -> `URDF_Exporter_Plus`.

## Design requirements (same as classic fusion2urdf)

* One component named `base_link`.
* Joints of type **Rigid**, **Revolute** or **Slider**.
* Revolute/slider joints need **both** limits set (revolute with no limits
  becomes `continuous`).
* Component light bulbs control what gets exported.
* Make sure Z is up in your design if you want the robot upright.

## CLI usage

```bash
# everything (URDF + ROS 2 + MJCF + both USD flavors)
fusion2urdf build myrobot_export/

# selective targets
fusion2urdf build myrobot_export/ --targets urdf,usd -o /tmp/out
fusion2urdf build myrobot_export/ --targets mjcf,usd-newton    # Newton path
fusion2urdf build myrobot_export/ --targets ros2 --package-name myrobot_description

# mobile robots: free-floating base (freejoint in MJCF, no fix_base in Isaac)
fusion2urdf build myrobot_export/ --no-fix-base --robot-type Wheeled

# inspect / validate a bundle
fusion2urdf info myrobot_export/
fusion2urdf validate myrobot_export/
```

## Using the outputs

### Isaac Sim

Option A (no Isaac install needed) — `out/usd/<robot>.usda` (UsdPhysics) or
`out/usd_newton/<robot>.usda` (Newton/Mjc attributes) open directly in
Isaac Sim / usdview.

Option B (full PhysX import with drives + robot schema):

```bash
$ISAAC_SIM_DIR/python.sh out/urdf/isaac_sim_import.py
```

Option C (official Isaac Sim asset layout via the Asset Transformer,
Isaac Sim >= 6.0):

```bash
$ISAAC_SIM_DIR/python.sh out/usd/run_asset_transformer.py
# -> out/usd/<robot>_package/ with payloads/, robot schema, variants
```

### Newton

`out/usd_newton/<robot>.usda` carries the `mjc:*` schema attributes emitted
by `mujoco-usd-converter`, and `out/mjcf/<robot>.xml` loads directly in
MuJoCo (`mujoco.MjModel.from_xml_path`) or in Newton via its MJCF/USD
importers.

### ROS 2 (Jazzy)

```bash
cp -r out/ros2/<robot>_description ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select <robot>_description
source install/setup.bash
ros2 launch <robot>_description display.launch.py     # RViz2
ros2 launch <robot>_description gazebo.launch.py      # Gazebo Sim
```

## Conventions

* SI units everywhere in the intermediate model (m, kg, kg*m^2); Fusion's
  cm / kg*cm^2 are converted at extraction time; STL meshes stay in mm and are
  scaled by `0.001` in the URDF (the classic fusion2urdf convention).
* Every link frame sits at its parent joint's world anchor; meshes and COM
  data are captured in world coordinates and shifted by the link origin.
* Inertia tensors are translated to the COM with the parallel axis theorem.

## Development

```bash
pip install -e ".[dev]"
pytest             # includes a real URDF->USD conversion smoke test
./scripts/sync_addin.sh   # re-vendor the core into the add-in after changes
```

The test suite validates the generated URDF by parsing it with
[yourdfpy](https://github.com/clemense/yourdfpy) (including forward
kinematics checks), expands the ROS 2 xacro with the real `xacro` package,
and converts to USD with `urdf-usd-converter`, asserting the resulting stage
contains the expected `UsdPhysics` joints.

## Credits

Joint/inertia math and the Fusion API workflow build on the work of
syuntoku14, SpaceMaster85, Lentin Joseph (runtimerobotics), Dheena2k2, and
zhbi98. Licensed MIT.
