#!/usr/bin/env python3
import time
import serial
import sys
from serial.tools import list_ports
from tkinter import Tk, filedialog


def get_port():
    for p in list_ports.comports():
        d = (p.description or "").lower()
        m = (p.manufacturer or "").lower()
        if "arduino" in d or "ch340" in d or "cp210" in d or "usb serial" in d:
            return p.device
        if "arduino" in m or "wch" in m or "silicon" in m:
            return p.device

    ports = [p.device for p in list_ports.comports()]
    return ports[0] if ports else None


def pick_file():
    root = Tk()
    root.withdraw()
    return filedialog.askopenfilename(filetypes=[("CH8", "*.ch8"), ("all", "*.*")])


port = get_port()
if not port:
    print("no device")
    sys.exit(1)

file = pick_file()
if not file:
    print("no file")
    sys.exit(1)

ser = serial.Serial(port, 57600, timeout=2)

with open(file, "rb") as f:
    rom = f.read()

if len(rom) > 1024:
    print("too big")
    sys.exit(1)

while ser.readline().decode(errors="ignore").strip() != "READY":
    time.sleep(0.1)

ser.write(bytes([1, len(rom) >> 8, len(rom) & 0xFF]))

if ser.read(1) != b"\x06":
    print("fail")
    sys.exit(1)

i = 0
while i < len(rom):
    chunk = rom[i:i+32]
    ser.write(chunk)
    ser.write(bytes([sum(chunk) & 0xFF]))
    if ser.read(1) == b"\x06":
        i += len(chunk)

ser.write(bytes([4]))

if ser.read(1) == b"\x06":
    print("ok")

ser.close()
