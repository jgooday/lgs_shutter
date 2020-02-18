# LGS Beam Shutter ✨

This code runs the beam shutter designed by Jack Gooday for the Laser Guide Star (LGS) system on the EOS 1.8m telescope on Mt. Stromlo. It is written in C++ and was designed to run on the Arduino Nano (Atmega 328).

The repository is split into two [PlatformIO](https://platformio.org/) projects; `shutter` and `testing`. `shutter` contains the code that runs the shutter, `testing` contains code that is designed to mimic a virtual control system, and was used to test the shutter prototype. More info about each can be found in the respective folders.

As PlatformIO has been used, it is very easy to compile each project for any microcontroller. Consult the PlatformIO documentation on how to do this. Alternatively, you can just copy and paste the C++ code in to the [Arduino IDE](https://www.arduino.cc/en/main/software) and remove the library headers.
