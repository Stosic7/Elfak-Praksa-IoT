#!/usr/bin/env python3
import base64, serial, sys, time
from PIL import Image

PORT = "/dev/cu.usbmodem101"

def cekaj(ser, pref, t=20):
    rok = time.time() + t
    while time.time() < rok:
        l = ser.readline().decode("utf-8", errors="ignore").strip()
        if not l:
            continue
        if l.startswith("#ERR"):
            raise RuntimeError(l)
        if l.startswith(pref):
            return l
    raise TimeoutError(pref)

ser = serial.Serial(PORT, 115200, timeout=2)
time.sleep(2)
ser.reset_input_buffer()
ser.write(b"ping\n")
cekaj(ser, "#READY", 10)

ime = sys.argv[1] if len(sys.argv) > 1 else "ulaz"

ser.reset_input_buffer()
ser.write(b"cap\n")

res = cekaj(ser, "#RES", 30)
d = res.split()
print(f"indeks={d[1]}  novcanik={d[2]}  prazno={d[3]}")

hdr = cekaj(ser, "#START", 30)
n, w, h = int(hdr.split()[1]), int(hdr.split()[2]), int(hdr.split()[3])

b64 = []
while True:
    l = ser.readline().decode("utf-8", errors="ignore").strip()
    if l == "#END":
        break
    if l:
        b64.append(l)

raw = base64.b64decode("".join(b64))
print(f"primljeno {len(raw)} od {n} bajtova")

img = Image.frombytes("RGB", (w, h), raw[:w*h*3])
img.save(f"{ime}.png")
img.resize((384, 384), Image.NEAREST).save(f"{ime}_veliko.png")
print(f"sacuvano: {ime}.png")
ser.close()