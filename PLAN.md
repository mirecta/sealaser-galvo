# Rayforge: Gweike G2 Galvo Laser Support

## Goal
Add support for the Gweike G2 (BJJCZ/JCZ LMC controller) galvo fiber laser to the open-source [Rayforge](https://github.com/barebaric/rayforge) laser software.

---

## Context

- **Machine**: Gweike G2 — uses a BJJCZ LMCV4-FIBER controller, USB (idVendor=0x9588, idProduct=0x9899)
- **Protocol**: Proprietary LMC USB protocol, reverse-engineered by the `galvoplotter` and `balor` projects
- **Rayforge**: Python/GTK4 app, MIT license. Currently supports GRBL, Ruida (UDP), OctoPrint
- **Key dependency**: [`galvoplotter`](https://github.com/meerk40t/galvoplotter) — Python lib wrapping the LMC protocol

---

## Repository Setup

```bash
# Clone Rayforge
git clone https://github.com/barebaric/rayforge
cd rayforge

# Install dev dependencies (see Developer Documentation)
pip install -e ".[dev]"

# Install galvoplotter
pip install galvoplotter pyusb
```

On Windows, use [Zadig](https://zadig.akeo.ie/) to replace the BJJCZ USB driver with WinUSB/libusb.

---

## Implementation Plan

### Phase 1 — Explore & Verify

- [ ] Read `rayforge/drivers/` (or equivalent) to understand the existing Ruida driver structure
- [ ] Read `rayforge/machines/` to understand machine profile format
- [ ] Run `galvoplotter` example scripts against the G2 to verify connectivity
- [ ] Confirm USB IDs: `lsusb` or Device Manager should show `0x9588:0x9899`

### Phase 2 — BJJCZ Connection Driver

Create `rayforge/drivers/bjjcz.py`:

- [ ] Implement `BJJCZConnection` class matching Rayforge's driver interface
  - `connect()` — open USB device via `galvoplotter`
  - `disconnect()` — close connection
  - `is_connected()` — return status
  - `send_job(job)` — translate and send a job
  - `jog(x, y, speed)` — move red dot pointer
  - `home()` — center galvo
  - `pause()` / `resume()` / `abort()`
  - `get_status()` — return machine state

### Phase 3 — G-code / Toolpath Translator

Create `rayforge/drivers/bjjcz_translator.py`:

Rayforge outputs G-code; galvo controllers use a command list (not G-code). Translate:

| G-code | Galvo command |
|---|---|
| `G0 X Y` (travel) | `travel_to(x, y)` |
| `G1 X Y F` (feed) | `mark_to(x, y)` |
| `M3 S<power>` | `set_power(power)` + laser on |
| `M5` | laser off |
| `F<speed>` | `set_speed(speed)` |
| Frequency param | `set_frequency(freq)` |

Key considerations:
- Coordinate system: Rayforge uses mm; LMC uses 16-bit integers (0x0000–0xFFFF mapped to field size)
- Scale factor: `lmc_val = int((mm / field_size_mm) * 0xFFFF)`
- Origin: LMC center is `0x8000, 0x8000`

### Phase 4 — Machine Profile

Create `rayforge/machines/gweike_g2.json` (or `.yaml`):

```json
{
  "name": "Gweike G2",
  "driver": "bjjcz",
  "connection": "usb",
  "usb_vendor_id": "0x9588",
  "usb_product_id": "0x9899",
  "laser_type": "fiber",
  "work_area_mm": [110, 110],
  "max_speed_mm_s": 15000,
  "default_frequency_khz": 20,
  "default_power_percent": 50,
  "field_lens_mm": 110
}
```

Add variants for G2 Max (175×175mm field) if needed.

### Phase 5 — UI Integration

- [ ] Register `BJJCZConnection` in Rayforge's driver registry
- [ ] Add "Gweike G2" (and generic "BJJCZ Fiber Galvo") to machine selector
- [ ] Add galvo-specific operation parameters to the layer UI:
  - Pulse frequency (kHz)
  - Q-pulse width (ns) — for MOPA color engraving
  - Jump delay / mark delay
- [ ] Gate G-code arc/curve features that don't apply to galvo

### Phase 6 — Testing

- [ ] Unit tests for coordinate translation (mm ↔ LMC units)
- [ ] Unit tests for G-code parsing → galvo command list
- [ ] Integration test: send a test pattern (square, circle) to machine
- [ ] Test raster engraving path
- [ ] Test contour/cut path

### Phase 7 — Contribution

- [ ] Open a feature request issue on `barebaric/rayforge` before large PRs
- [ ] Follow Rayforge code style (run existing linters/formatters)
- [ ] Add documentation: new page in `docs/machines/gweike-g2.md`
- [ ] Submit PR with description, screenshots, and test results
- [ ] Alternatively: package as `rayforge-addon-bjjcz` using `barebaric/rayforge-addon-template`

---

## Key Files to Read First

```
rayforge/
  drivers/
    grbl.py          # Model for a simple serial driver
    ruida.py         # Model for a UDP/binary protocol driver
  machines/          # Machine profile format
  config/            # App config structure
  README.md
  docs/CONTRIBUTING.md
```

---

## External References

| Resource | URL |
|---|---|
| Rayforge source | https://github.com/barebaric/rayforge |
| Rayforge dev docs | https://rayforge.org/contributing/ |
| galvoplotter | https://github.com/meerk40t/galvoplotter |
| Balor (reverse eng.) | https://gitlab.com/bryce15/balor |
| Balor MeerK40t plugin | https://github.com/tatarize/balor-meerk40t |
| Bryce Schroeder's write-up | https://www.bryce.pw/engraver.html |
| Rayforge Discord | https://rayforge.org (linked on homepage) |
| Rayforge addon template | https://github.com/barebaric/rayforge-addon-template |

---

## Notes for Agent

- Always check the actual Rayforge driver interface by reading existing drivers before implementing — the API may have changed
- `galvoplotter` uses `pyusb` under the hood; USB permissions may need udev rules on Linux
- The G2 communicates over USB only (no WiFi for LMC control, WiFi is only for GLaser's proprietary protocol)
- MOPA color marking requires pulse width control — `galvoplotter` may need extension for this
- Coordinate origin and scaling are the most common source of bugs — write and test this math first
