from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = REPO_ROOT.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mantis import constants


VR_IK_URDF = (
    WORKSPACE_ROOT
    / "bw_teleoperate_ws"
    / "src"
    / "bw_core"
    / "assets"
    / "mantis_2_0_ik"
    / "mantis_2_0_ik.urdf"
)


def _normalize_urdf(content: str) -> str:
    return (
        content.replace(
            'package://mantis_description/meshes/2.0/visual/',
            '../meshes/visual/',
        )
        .replace(
            'package://mantis_description/meshes/2.0/collision/',
            '../meshes/collision/',
        )
        .strip()
    )


def test_sdk_ik_urdf_matches_vr_ik_source_model():
    assert hasattr(constants, "IK_URDF_FILENAME")

    sdk_ik_urdf = REPO_ROOT / "mantis" / "model" / "urdf" / constants.IK_URDF_FILENAME
    assert sdk_ik_urdf.exists()
    assert VR_IK_URDF.exists()

    sdk_content = _normalize_urdf(sdk_ik_urdf.read_text(encoding="utf-8"))
    vr_content = _normalize_urdf(VR_IK_URDF.read_text(encoding="utf-8"))

    assert sdk_content == vr_content
