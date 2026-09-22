#!/usr/bin/env python3

import xarm
import hid

# Vendor ID and Product ID for Hiwonder xArm S1
VID = 0x0483
PID = 0x5750

# List all HID devices
# If there is permission issues, the input fields will not show up correctly
for device in hid.enumerate():
    if device['vendor_id'] == VID and device['product_id'] == PID:
        print(f"Device path: {device['path'].decode()}")
        print(f"Manufacturer: {device['manufacturer_string']}")
        print(f"Product: {device['product_string']}")
        print(f"Seriala: {device['serial_number']}")
        print(f"Interface Number: {device.get('interface_number', 'N/A')}")
        print("-" * 40)

arm = xarm.Controller('USB')

print('Battery voltage in volts: ', arm.getBatteryVoltage())
