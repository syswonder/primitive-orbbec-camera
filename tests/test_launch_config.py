# SPDX-License-Identifier: Apache-2.0
import unittest

from orbbec_camera.launch_config import device_selector_args


class DeviceSelectorArgsTest(unittest.TestCase):
    def test_omits_selectors_for_single_camera_auto_discovery(self):
        self.assertEqual(device_selector_args({}), [])
        self.assertEqual(
            device_selector_args({"serial_number": "", "usb_port": "  "}),
            [],
        )

    def test_forwards_serial_number(self):
        self.assertEqual(
            device_selector_args({"serial_number": "CP123456"}),
            ["serial_number:=CP123456"],
        )

    def test_forwards_usb_port(self):
        self.assertEqual(
            device_selector_args({"usb_port": "2-3"}),
            ["usb_port:=2-3"],
        )

    def test_preserves_both_selectors_in_upstream_argument_order(self):
        self.assertEqual(
            device_selector_args({"usb_port": "2-3", "serial_number": 1234}),
            ["serial_number:=1234", "usb_port:=2-3"],
        )


if __name__ == "__main__":
    unittest.main()
