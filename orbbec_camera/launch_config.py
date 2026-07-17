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


def device_preset_args(cfg: dict) -> list[str]:
    """Return an optional named preset without changing the driver default.

    The ROS parameter is string-typed.  In particular, forwarding the integer
    ``1`` makes the Gemini 330 driver reject the override.  Omitting the
    argument lets the upstream launch file use its working ``Default`` preset.
    """
    value = cfg.get("device_preset")
    if value is None:
        return []
    if not isinstance(value, str):
        raise ValueError("device_preset must be a named string preset")
    value = value.strip()
    return [f"device_preset:={value}"] if value else []
