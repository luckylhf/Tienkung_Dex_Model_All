import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import mujoco

from script.make_mjcf_torq import restore_urdf_inertials, validate_compiled_inertials


PACKAGE_DIR = Path(__file__).resolve().parent.parent
URDF = PACKAGE_DIR / "urdf" / "tiangong3.urdf"
GENERATOR = PACKAGE_DIR / "script" / "make_mjcf_torq.py"
ORIGINAL_MJCF = PACKAGE_DIR / "mujoco" / "tiangong3_torq.xml"


class GenerateMjcfTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.output = self.directory / "robot.xml"
        self.original = ORIGINAL_MJCF.read_bytes()
        self.output.write_bytes(self.original)

    def run_generator(self, *args, without_mujoco=False):
        command = [sys.executable, "-B"]
        if without_mujoco:
            command += [
                "-c",
                'import runpy, sys; sys.modules["mujoco"] = None; '
                'sys.argv.pop(0); runpy.run_path(sys.argv[0], run_name="__main__")',
            ]
        return subprocess.run(
            [*command, str(GENERATOR), "--mjcf", str(self.output), *args],
            capture_output=True, text=True,
        )

    def assert_rejected(self, result, message):
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn(message, result.stderr)
        self.assertEqual(self.output.read_bytes(), self.original)
        self.assertEqual(list(self.directory.glob("*.xml")), [self.output])

    def test_missing_pelvis_inertial_is_rejected_even_offline(self):
        urdf = ET.parse(URDF)
        pelvis = urdf.find("link[@name='pelvis']")
        pelvis.remove(pelvis.find("inertial"))
        source = self.directory / "missing_pelvis.urdf"
        urdf.write(source)
        for options in ((), ("--skip-mujoco-validation",)):
            with self.subTest(options=options):
                result = self.run_generator("--urdf", str(source), *options)
                self.assert_rejected(result, "'pelvis' must have exactly one <inertial>")

    def test_missing_meshes_preserve_target(self):
        result = self.run_generator("--mesh-subdir", "missing_meshes")
        self.assert_rejected(result, "missing_meshes")

    def test_missing_mujoco_requires_explicit_offline_export(self):
        self.assert_rejected(self.run_generator(without_mujoco=True), "MuJoCo 不可用")
        result = self.run_generator("--skip-mujoco-validation", "--floor-z", "-0.12", without_mujoco=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        root = ET.parse(self.output)
        self.assertEqual(root.find("compiler").get("inertiafromgeom"), "false")
        self.assertEqual(len(root.findall("worldbody//body/inertial")), 39)
        self.assertEqual(root.find("worldbody/geom[@name='floor']").get("pos"), "0 0 -0.120000")

    def test_compiled_inertial_coverage_and_total_mass(self):
        (self.directory / "meshes_convex").symlink_to(ORIGINAL_MJCF.parent / "meshes_convex", target_is_directory=True)
        result = self.run_generator()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("for all 39 bodies (65.817803 kg)", result.stdout)
        model = mujoco.MjModel.from_xml_path(str(self.output))
        _, expected = restore_urdf_inertials(self.output.read_text(), URDF)
        incomplete = expected.copy()
        del incomplete["pelvis"]
        with self.assertRaisesRegex(ValueError, "Compiled body coverage.*pelvis"):
            validate_compiled_inertials(mujoco, model, incomplete)
        with patch.object(mujoco, "mj_getTotalmass", return_value=73.667594):
            with self.assertRaisesRegex(ValueError, "Compiled total mass"):
                validate_compiled_inertials(mujoco, model, expected)


if __name__ == "__main__":
    unittest.main()
