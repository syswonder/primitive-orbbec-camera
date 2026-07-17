# SPDX-License-Identifier: Apache-2.0
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ManifestLifecycleTest(unittest.TestCase):
    def test_shared_lifecycle_driver_is_implicit(self):
        text = (ROOT / "package_manifest.yaml").read_text()
        names = {
            line.strip().removeprefix("- name: ")
            for line in text.splitlines()
            if line.strip().startswith("- name: ")
        }
        self.assertNotIn("robonix/primitive/camera/driver", names)
        self.assertNotIn("robonix/lifecycle/driver", names)


if __name__ == "__main__":
    unittest.main()
