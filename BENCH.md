# Phase 0 bench session checklist

One sitting at the droid with a USB cable. Everything here is read-only for the droid —
nothing gets flashed. Do this **before** any v2 firmware goes on the board.

## 1. Identify the board variant

1. Plug the RAD controller into the Mac over USB. Find the port: `ls /dev/cu.usb*`
2. Run the capture tool (it records the boot banner — press the board's RESET button
   when prompted):

```bash
python3 tools/capture_config.py --port "/dev/cu.usb*" --capture docs/capture
```

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

```bash
esptool.py --port /dev/cu.usbmodem* read_flash 0x0 0x800000 releases/legacy-backup.bin
```

(If the board has 4 MB flash instead of 8 MB the command errors — retry with `0x400000`.)

## 4. Sensor stream check

With the sensor ring powered and the dome turned by hand, watch the raw feed if you have
a spare USB-UART adapter on the sensor line, or just note the configured
`#DPSENSORBAUD` value from the config capture. We need: baud (57600 vs 115200) and
roughly how many `#DP@` lines per second arrive (tunes the v2 glitch filter).

## 5. WCB fleet facts (from Sabé's console or the WCB Wizard)

Confirm and jot down:

- [ ] `?ETM,CHKSM` → ON
- [ ] `?WCBCH` → channel 1
- [ ] `?WCBM` → octets 3C / 4E
- [ ] `?WCBQ` → 3
- [ ] Nothing on the mesh already uses device ID 4
- [ ] The mesh password (needed once for `#DPWCBPW` on v2 — don't commit it)

## Done?

Commit `docs/capture/` (the .gitignore excludes only `*.log`; the captured .txt files
should be committed) and the PCB photos. That unblocks the v2 pin map and the
conformance work.
