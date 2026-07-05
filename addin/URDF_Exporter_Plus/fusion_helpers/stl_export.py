"""STL export for all visible top-level occurrences.

Clones visible bodies into a temporary direct-design document so linked and
nested components export correctly without touching the user's design
(SpaceMaster85 approach, kept because it is the only reliable way to bake
nested body transforms into per-link meshes).
"""

from __future__ import annotations

import os
import re

import adsk
import adsk.core
import adsk.fusion

_SANITIZE = re.compile(r"[ :()<>]")


def _clean(name: str) -> str:
    if "base_link" in name:
        return "base_link"
    return _SANITIZE.sub("_", name)


def _visible_bodies(occ):
    bodies = []
    if not occ.isLightBulbOn:
        return bodies
    if occ.component.isBodiesFolderLightBulbOn:
        bodies.extend(b for b in occ.bRepBodies if b.isLightBulbOn)
    for child in occ.childOccurrences:
        bodies.extend(_visible_bodies(child))
    return bodies


def export_link_meshes(app, save_dir: str) -> list[str]:
    """Export one binary STL (mm) per top-level occurrence into
    ``save_dir/meshes``. Returns the list of written files."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent

    groups = []  # (link_name, [bodies])
    for occ in root.occurrences:
        bodies = _visible_bodies(occ)
        if bodies:
            groups.append((_clean(occ.name), bodies))

    if not groups:
        raise RuntimeError("no visible bodies found to export")

    temp_mgr = adsk.fusion.TemporaryBRepManager.get()
    cloned = [
        (name, [temp_mgr.copy(b) for b in bodies]) for name, bodies in groups
    ]

    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    written = []
    try:
        export_design = adsk.fusion.Design.cast(doc.products.itemByProductType(
            "DesignProductType"
        ))
        export_design.designType = adsk.fusion.DesignTypes.DirectDesignType
        export_root = export_design.rootComponent

        identity = adsk.core.Matrix3D.create()
        for name, bodies in cloned:
            occ = export_root.occurrences.addNewComponent(identity)
            occ.component.name = name
            for body in bodies:
                occ.component.bRepBodies.add(body)

        mesh_dir = os.path.join(save_dir, "meshes")
        os.makedirs(mesh_dir, exist_ok=True)

        export_mgr = export_design.exportManager
        for occ in export_root.occurrences:
            path = os.path.join(mesh_dir, f"{_clean(occ.component.name)}.stl")
            opts = export_mgr.createSTLExportOptions(occ, path)
            opts.sendToPrintUtility = False
            opts.meshRefinement = (
                adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
            )
            export_mgr.execute(opts)
            written.append(path)
    finally:
        doc.close(False)

    return written
