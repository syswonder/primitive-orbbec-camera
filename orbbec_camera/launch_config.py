#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Dependency-free helpers for constructing Orbbec ROS launch arguments."""

from __future__ import annotations


def device_selector_args(cfg: dict) -> list[str]:
    """Return explicit Orbbec device selectors from a Robonix config.

    The upstream launch files accept both values as strings. Omitting empty
    values preserves their single-camera auto-discovery behavior.
    """
    args: list[str] = []
    for key in ("serial_number", "usb_port"):
        value = cfg.get(key)
        if value is None:
            continue
        value = str(value).strip()
        if value:
            args.append(f"{key}:={value}")
    return args
