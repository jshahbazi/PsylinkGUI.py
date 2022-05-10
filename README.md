# PsylinkGUI.py

GUI for the [Psylink neural interface](https://psylink.me).

Designed to work with macOS, but should work with Windows and Linux.

---

If you want to include IMU readings and a 3D model that follows the principal axes of the Power Module, you'll need to upload the following modified sketch to your Psylink: \
[AnalogToBLE2.0.ino](./AnalogToBLE2.0/AnalogToBLE2.0.ino)

Its a highly modified version of the Psylink sketch that adds [flash storage](https://github.com/petewarden/arduino_nano_ble_write_flash), magnetometer readings, and principal axes calculations using quaternion code from here: https://github.com/kriswiner/LSM9DS1.

---

### Built with:
[Bleak](https://github.com/hbldh/bleak) \
[DearPyGui](https://github.com/hoffstadt/DearPyGui) \
[Psylink](https://codeberg.org/psylink/psylink) 
