# PsylinkGUI.py

GUI for the [Psylink neural interface](https://psylink.me).

Designed to work with macOS, but should work with Windows and Linux.

---

If you want to include magnetometer readings, you'll need to upload the following modified sketch to your Psylink: \
[AnalogToBLE1.1.ino](https://github.com/jshahbazi/PsylinkGUI.py/blob/main/AnalogToBLE1.1.ino)

Its basically the same as the Psylink sketch, but it adds the magnetometer readings to the end of the BLE data.

---

### Bugs:
* Exiting the program sometimes hangs
  * Just CTRL-C the command line to exit
  * It's because of async and multiprocessing - I just need to spend some more time tearing things down properly

---

### Built with:
[Bleak](https://github.com/hbldh/bleak) \
[DearPyGui](https://github.com/hoffstadt/DearPyGui) \
[Psylink](https://codeberg.org/psylink/psylink) 
