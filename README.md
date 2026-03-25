🕹️ Chip4Nano

Chip4Nano is a Open-source lightweight CHIP-8 emulator built for the Arduino Nano.
It uses a joystick for input and an SH1106 OLED display for output, with
no external memory required.

Features: - CHIP-8 emulator optimized for Arduino Nano - EEPROM saving -
Flash Mode via Python - Joystick input - SH1106 display support

Flash Mode: To enter Flash Mode, hold the joystick button and press the
reset button.

Usage: python send_rom.py pong.ch8

Hardware: OLED (I2C): SDA=A4, SCL=A5 Joystick: VRx=A0, VRy=A1, SW=D2

To-Do: - Test Arduino Uno compatibility - Improve SCHIP support - Add
sound - Optimize performance
