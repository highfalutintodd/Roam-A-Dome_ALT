# Phase 0 bench session checklist

One sitting at the droid with a USB cable. Everything here is read-only for the droid —
nothing gets flashed. Do this **before** any v2 firmware goes on the board.

## 1. Identify the board variant

1. Plug the RAD controller into the Mac over USB. Find the port: `ls /dev/cu.usb*`
2. **From the repo root** (so files land in `docs/capture/`), run the capture tool.
   Opening the port resets the board automatically — no button press needed; the
   tool waits out the boot flood and retries each command until it gets a real
   answer:

```bash
python3 tools/capture_config.py --port "/dev/cu.usb*" --capture docs/capture
```

   Make sure the **dome sensor ring is powered and connected** during this — the
   previous session logged "Dome Sensor Not Ready", so we still need to see a
   healthy sensor stream and the configured baud.

3. Photograph both sides of the controller PCB (close enough to read chip markings) and
   drop the photos in `docs/capture/`.
4. Note which build it is: compact (no display) vs display (big-number position screen).

## 2. Capture migration data

The capture tool already saved `docs/capture/config.txt` (`#DPCONFIG`),
`status.txt` (`#DPSTATUS`), `sequences.txt` (`#DPL`), and `boot_banner.txt`.
These are the only migration path to v2 — the old EEPROM layout can't be read directly.

Also try `#DPPWMARC1` then `#DPPWMARC0` in a serial monitor and note whether it
answers or says `Invalid` (resolves an open question in BEHAVIOR.md §9).

## 3. Back up the legacy firmware (rollback insurance)

```bash
pip3 install esptool
```

Run **from the repo root** and create the output folder first (the previous attempt
failed only because `releases/` didn't exist):

```bash
mkdir -p releases && esptool --port /dev/cu.usbmodem* read_flash 0x0 0x800000 releases/legacy-backup.bin
```

(If the board has 4 MB flash instead of 8 MB the command errors — retry with
`0x400000`. esptool 5.x dropped the `.py` suffix; either spelling works if both are
installed.)

## 4. Sensor stream check

With the sensor ring powered and the dome turned by hand, watch the raw feed if you have
a spare USB-UART adapter on the sensor line, or just note the configured
`#DPSENSORBAUD` value from the config capture. We need: baud (57600 vs 115200) and
roughly how many `#DP@` lines per second arrive (tunes the v2 glitch filter).

## 5. WCB fleet facts

Goal: confirm the mesh parameters the new firmware will join with, so `begin()`
doesn't silently land on the wrong channel or checksum setting.

**Most of this is already in your Sabé config** (`NOtes/Sabe Config` in the Sabé
repo): octets **3C/4E**, quantity **3**, checksum **ON**, Sabé at device **20**.
Channel was never changed from the default, so it should be **1**. Device IDs in
use are 1–3 (the three physical WCBs) and 20 (Sabé), which leaves **4 free** for
the dome. So this step is a spot-check, not a research project.

To verify directly, plug USB into **WCB1** (any of the three works) and open a
serial monitor at **115200** — the same console you used for the `?VSTATS` log:

1. **Boot banner**: press the WCB's reset button (or power-cycle it). The banner
   prints its current config — MAC octets, board number, quantity, and channel.
   Jot down what it shows.
2. **Checksum state**: type `?ETM,CHKSM` with no argument. On/off state is echoed
   back. (With an argument — `?ETM,CHKSM,ON` — it *sets* the state, so send it
   bare.) Expected: ON.
3. **Who's on the mesh**: type `?WDP,LIST` (WCB firmware 6.x). It enumerates every
   device the board knows — the other WCBs, Sabé at 20, and later the dome at 4.
   Confirm nothing is listed at ID 4 today.
4. If any of those queries isn't recognized, don't fight it — a phone photo of the
   boot banner is enough, and I can decode it. The WCB Wizard (if you use it) shows
   the same values graphically.

- [ ] Checksum → ON
- [ ] Channel → 1
- [ ] Octets → 3C / 4E
- [ ] Quantity → 3
- [ ] Device ID 4 free
- [ ] Mesh password in hand (entered once on the droid via `#DPWCBPW` — **never
      committed to the repo**)

## Done?

Commit `docs/capture/` (the .gitignore excludes only `*.log`; the captured .txt files
should be committed) and the PCB photos. That unblocks the v2 pin map and the
conformance work.
