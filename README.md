# PsylinkGUI.py

GUI for the [Psylink neural interface](https://psylink.me).

Designed to work with macOS, but should work with Windows and Linux.

---

If you want to include magnetometer readings, you'll need to upload the following modified sketch to your Psylink: \
[AnalogToBLE1.1.ino](./AnalogToBLE1.1/AnalogToBLE1.1.ino)

Its basically the same as the Psylink sketch, but it adds the magnetometer readings to the end of the BLE data.

---

### Built with:
[Bleak](https://github.com/hbldh/bleak) \
[DearPyGui](https://github.com/hoffstadt/DearPyGui) \
[Psylink](https://codeberg.org/psylink/psylink) 
