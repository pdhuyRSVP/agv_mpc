#!/usr/bin/env python3
"""Compatibility entry point for the direct USB-RS485 wheel driver."""

from agv_sensors.wheel_vel_node_direct import WheelVelNode, main


__all__ = ["WheelVelNode", "main"]
