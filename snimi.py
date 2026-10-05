#!/usr/bin/env python3
"""
Snimanje audio uzoraka sa XIAO ESP32S3 Sense preko USB-a.
Cuva .wav fajlove spremne za upload u Edge Impulse.

Upotreba:
    python3 snimi.py --label kamera --count 5
    python3 snimi.py --label stani  --count 5
    python3 snimi.py --label buka   --count 3

Zahteva:  pip3 install pyserial
VAZNO:    Serial Monitor u Arduino IDE mora biti ZATVOREN.
"""

import argparse
import base64
import os
import sys
import time
import wave

try:
    import serial
except ImportError:
    sys.exit("Nedostaje pyserial. Instaliraj sa:  pip3 install pyserial")


DEFAULT_PORT = "/dev/cu.usbmodem101"
OUT_DIR = "uzorci"


def cekaj_liniju(ser, prefiks, timeout=20):
    """Cita linije dok ne naidje na onu koja pocinje sa prefiks."""
    rok = time.time() + timeout
    while time.time() < rok:
        linija = ser.readline().decode("utf-8", errors="ignore").strip()
        if not linija:
            continue
        if linija.startswith("#ERR"):
            raise RuntimeError(linija)
        if linija.startswith(prefiks):
            return linija
    raise TimeoutError(f"Nema odgovora '{prefiks}' u {timeout}s")


def snimi_jedan(ser):
    """Salje 'rec', vraca (pcm_bajtovi, sample_rate)."""
    ser.reset_input_buffer()
    ser.write(b"rec\n")
    ser.flush()

    cekaj_liniju(ser, "#REC", timeout=10)
    zaglavlje = cekaj_liniju(ser, "#START", timeout=30)

    delovi = zaglavlje.split()
    ocekivano = int(delovi[1])
    rate = int(delovi[2])

    b64 = []
    rok = time.time() + 120
    while time.time() < rok:
        linija = ser.readline().decode("utf-8", errors="ignore").strip()
        if linija == "#END":
            break
        if linija.startswith("#ERR"):
            raise RuntimeError(linija)
        if linija:
            b64.append(linija)
    else:
        raise TimeoutError("Prenos nije zavrsen na vreme")

    pcm = base64.b64decode("".join(b64))

    if len(pcm) != ocekivano:
        print(f"  upozorenje: primljeno {len(pcm)} od {ocekivano} bajtova")

    return pcm, rate


def sacuvaj_wav(putanja, pcm, rate):
    with wave.open(putanja, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)


def sledeci_broj(label):
    """Nalazi prvi slobodan redni broj da ne pregazi postojece fajlove."""
    n = 1
    while os.path.exists(os.path.join(OUT_DIR, f"{label}.{n}.wav")):
        n += 1
    return n


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--label", required=True,
                   help="naziv klase, npr. kamera / stani / buka / tisina")
    p.add_argument("--count", type=int, default=5,
                   help="koliko snimaka od 10s (podrazumevano 5)")
    p.add_argument("--port", default=DEFAULT_PORT)
    args = p.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)

    print(f"Otvaram {args.port} ...")
    try:
        ser = serial.Serial(args.port, 115200, timeout=2)
    except serial.SerialException as e:
        sys.exit(f"Ne mogu da otvorim port: {e}\n"
                 f"Proveri da li je Serial Monitor u Arduino IDE zatvoren.")

    time.sleep(2)
    ser.reset_input_buffer()
    ser.write(b"ping\n")
    try:
        cekaj_liniju(ser, "#READY", timeout=10)
    except TimeoutError:
        sys.exit("Ploca se ne javlja. Pritisni RESET pa pokreni ponovo.")

    print(f"Ploca spremna.\n")
    print(f"Klasa: '{args.label}'  |  {args.count} snimaka po 10 sekundi")
    print("Tokom svakog snimka ponovi rec 4-5 puta, sa pauzom od oko sekunde.\n")

    poc = sledeci_broj(args.label)

    for i in range(args.count):
        broj = poc + i
        input(f"[{i+1}/{args.count}]  ENTER za start snimanja...")
        print("  SNIMAM 10s - pricaj sada", flush=True)

        try:
            pcm, rate = snimi_jedan(ser)
        except Exception as e:
            print(f"  greska: {e}")
            continue

        putanja = os.path.join(OUT_DIR, f"{args.label}.{broj}.wav")
        sacuvaj_wav(putanja, pcm, rate)
        trajanje = len(pcm) / 2 / rate
        print(f"  sacuvano: {putanja}  ({trajanje:.1f}s)\n")

    ser.close()
    print(f"Gotovo. Fajlovi su u folderu '{OUT_DIR}/'")
    print("Sledeci korak: uploaduj ih u Edge Impulse > Data acquisition > Upload data")


if __name__ == "__main__":
    main()