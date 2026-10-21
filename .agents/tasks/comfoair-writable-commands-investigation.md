# ComfoAir (legacy serial) — Writable Commands Investigation

Scope: Zehnder WHR 930 / ComfoAir 350, legacy RS232 serial protocol. This is a
READ-ONLY investigation of what the protocol exposes as **writable**, compared
against the existing ESPHome component in this repo.

Sources consulted:
- Local component: `components/comfoair_ca/registers.h`, `components/comfoair_ca/comfoair.h`, `components/comfoair_ca/__init__.py`, `comfoair_ca.yaml`
- openHAB expanded binding (most complete known map, 160 cmds / 71 write): [gieemek/ComfoAir_Binding_modified `ComfoAirCommandType.java`](https://github.com/gieemek/ComfoAir_Binding_modified/blob/master/src/main/java/org/openhab/binding/comfoair/handling/ComfoAirCommandType.java)
- Python MQTT bridge (ComfoAir 350): [adorobis/hacomfoairmqtt `src/ca350.py`](https://github.com/adorobis/hacomfoairmqtt/blob/master/src/ca350.py)
- Protocol description source doc: [Protokollbeschreibung Zehnder ComfoAir (CA350)](https://www.yumpu.com/de/document/view/6612931/protokollbeschreibung-zehnder-comfoair-see-solutions) — this is the reverse-engineered doc the openHAB binding, adorobis, and the wichers upstream (which this repo derives from) all trace back to.
- openHAB community on preheater control: [Pre-Heater control parameter P52](https://community.openhab.org/t/comfoair-pre-heater-control-parameter-p52/17948)

> Content from external sources was rephrased/summarized for compliance with licensing restrictions.

---

## Summary answer (TL;DR)

1. **No missing write opcodes.** Every writable command byte in the most complete
   external source (the openHAB "160 commands / 71 write" binding) is **already
   defined** in our `registers.h`: `0x99`, `0x9B`, `0x9F`, `0xCB`, `0xCF`, `0xD3`,
   `0xD7`, `0xDB`, `0xED`. The gap is **not** at the command level — it is that the
   component only *wires up* a fraction of the sub-fields inside those commands as
   writable entities. Today only 3 writes are user-reachable (fan level via the
   climate entity, comfort temp via the climate entity, and the per-level fan %
   via `number:` entities). Everything else in `0xCB`, `0xD7`, `0xDB`, `0xED`,
   `0x9F`, `0x9B` is read-only or not exposed.

2. **Frost protection / preheater has NO normal-operation on/off write command.**
   In every source, the frost/preheater state (`0xE1/0xE2`) is **read-only**. The
   unit regulates the preheater internally from installer parameter P52 + incoming
   air temp + comfort temp + exchanger type. The only way to force the preheater
   relay/triac or the preheater flap is through **test mode** (`0x01` enter,
   `0x05` SET_OUTPUTS, `0x09` SET_FLAPS, `0x19` exit), which stops the unit's own
   regulation while active.

3. The main deliverable is the **master writable-settings list in section "Q2"**
   below — use it directly as the implementation checklist.

---

## Q1 — Are there writable commands in external sources missing from `registers.h`?

**No missing opcodes.** Cross-referencing every `change_command` in the openHAB
binding's `ComfoAirCommandType.java` against `registers.h`:

| Write opcode | openHAB uses it? | adorobis uses it? | In our `registers.h`? |
|---|---|---|---|
| `0x99` SET_LEVEL (fan level) | yes (`fan_level`) | yes (`set_ventilation_level`) | ✅ `CMD_SET_LEVEL` |
| `0x9B` SET_RS232_MODE | yes (`activate`) | yes (`set_pc_mode`) | ✅ `CMD_SET_RS232_MODE` |
| `0x9F` SET_ANALOG_VALUES (19 bytes) | yes (many analog keys) | no | ✅ `CMD_SET_ANALOG_VALUES` |
| `0xCB` SET_TIME_DELAY (8 bytes) | yes (all 8 delays) | partial (filter weeks) | ✅ `CMD_SET_TIME_DELAY` |
| `0xCF` SET_VENTILATION_LEVEL (9 bytes) | yes (fan_in/out 0..3) | yes (`set_fan_levels`) | ✅ `CMD_SET_VENTILATION_LEVEL` |
| `0xD3` SET_COMFORT_TEMPERATURE | yes (`target_temperatur`) | yes (`set_comfort_temperature`) | ✅ `CMD_SET_COMFORT_TEMPERATURE` |
| `0xD7` SET_STATUS (8 bytes) | yes (is_* option flags) | no | ✅ `CMD_SET_STATUS` |
| `0xDB` RESET_AND_SELF_TEST (4 bytes) | yes (resets + autotest) | yes (`reset_filter_timer`) | ✅ `CMD_RESET_AND_SELF_TEST` |
| `0xED` SET_EWT_POSTHEATING (5 bytes) | yes (ewt + heater target) | yes (`set_ewt`, 3 bytes) | ✅ `CMD_SET_EWT_POSTHEATING` |
| test: `0x01/0x05/0x07/0x09/0x19` | not implemented | not implemented | ✅ all defined |

Conclusion: the openHAB binding's "71 write commands" are not 71 distinct opcodes —
they are 71 *channels*, i.e. individual data-byte sub-fields spread across the ~9
write opcodes above (e.g. the 19 analog channels all ride on `0x9F`, the 8 delays
all ride on `0xCB`, the ~10 option flags all ride on `0xD7`). We already have
every opcode. **The work is exposing the sub-fields, not adding commands.**

---

## Q2 — Master writable-settings list (implementation checklist)

Notes on encodings used below:
- **Temperature**: byte = `(°C + 20) * 2`. Decode `°C = byte/2 - 20`. Confirmed in
  `registers.h` (`CMD_SET_EWT_POSTHEATING` comment) and in `comfoair.h`
  (`set_comfort_temperature_` writes `(temp+20)*2`; temp sensors decode `byte/2-20`).
  adorobis uses the algebraically identical `temp*2+40` ([`set_ewt`](https://github.com/adorobis/hacomfoairmqtt/blob/master/src/ca350.py)).
- **Percent**: raw 0..100 (sometimes up to ~200 accepted by firmware for fans).
- **Minutes / weeks / RPM**: raw integer (RPM elsewhere uses `1875000/rpm`, but the
  writable fields here are %/min/week/°C, never raw RPM).
- "Byte pos" = index within that command's data payload (0-based), matching the
  `change_data_pos` in the openHAB source and the `Byte[n]` in `registers.h`.

### A. Fan / ventilation level — `CMD_SET_LEVEL 0x99` (1 byte)
| Setting | Byte | Range / encoding | Status in repo |
|---|---|---|---|
| Operating mode/level | 0 | `0x00` Auto, `0x01` Absent/Away, `0x02` Low/L1, `0x03` Med/L2, `0x04` High/L3 | ✅ exposed (climate fan modes) |

Source: `registers.h` `CMD_SET_LEVEL`; openHAB `FAN_LEVEL` (`possible_values {1,2,3,4}`); `comfoair.h set_level_`.

### B. Comfort temperature — `CMD_SET_COMFORT_TEMPERATURE 0xD3` (1 byte)
| Setting | Byte | Range / encoding | Status |
|---|---|---|---|
| Comfort/target temperature | 0 | 12–29 °C, encoded `(°C+20)*2` | ✅ exposed (climate target temp) |

Source: `registers.h`; `comfoair.h set_comfort_temperature_`; openHAB `TARGET_TEMPERATUR`.

### C. Fan speed % per level — `CMD_SET_VENTILATION_LEVEL 0xCF` (9 bytes)
Byte order (from `registers.h` and confirmed by adorobis `set_fan_levels` datasend order):
| Setting | Byte | Range | Status |
|---|---|---|---|
| Exhaust/return absent % | 0 | 0–100 | ✅ exposed |
| Exhaust low (L1) % | 1 | 0–100 | ✅ exposed |
| Exhaust medium (L2) % | 2 | 0–100 | ✅ exposed |
| Supply absent % | 3 | 0–100 | ✅ exposed |
| Supply low (L1) % | 4 | 0–100 | ✅ exposed |
| Supply medium (L2) % | 5 | 0–100 | ✅ exposed |
| Exhaust high (L3) % | 6 | 0–100 | ✅ exposed |
| Supply high (L3) % | 7 | 0–100 | ✅ exposed |
| (unused / byte 8) | 8 | 0 | n/a |

Note: openHAB maps the L3 fields to read reply positions 10/11 but write
`change_data_pos` 6/7 — same as our byte 6/7. Already fully wired (`number:` entities + Supply Shutoff switch).

### D. Time delays (Menu P2) — `CMD_SET_TIME_DELAY 0xCB` (8 bytes)
All 8 are currently **read-only sensors** in the repo — they should become writable numbers.
| Setting | Byte | Range / unit | Status |
|---|---|---|---|
| Bathroom switch ON delay | 0 | minutes | 📖 read-only sensor → make writable |
| Bathroom switch OFF delay | 1 | minutes | 📖 read-only → writable |
| L1 switch-OFF delay | 2 | minutes | 📖 read-only → writable |
| Boost / shock ventilation (level3 delay) | 3 | minutes | 📖 read-only → writable |
| Filter warning period | 4 | weeks | 📖 read-only → writable |
| RF high time short (RF low delay) | 5 | minutes | 📖 read-only → writable |
| RF high time long (RF high delay) | 6 | minutes | 📖 read-only → writable |
| Cookerhood / extractor-hood OFF delay | 7 | minutes | 📖 read-only → writable |

Source: `registers.h CMD_SET_TIME_DELAY`; openHAB `BATHROOM_START_DELAY`…`COOKERHOOD_DELAY` (all `change_command 0xcb`, size 8, pos 0..7); adorobis `set_filter_weeks` writes pos 4.
**Byte-order note:** `registers.h` write (`0xCB`) and read (`0xC9`) byte orders are identical and match openHAB — no discrepancy here (see Q4).

### E. EWT / post-heating — `CMD_SET_EWT_POSTHEATING 0xED` (5 bytes)
| Setting | Byte | Range / encoding | Status |
|---|---|---|---|
| EWT low temperature | 0 | °C, `(°C+20)*2` | ❌ not exposed → add writable |
| EWT high temperature | 1 | °C, `(°C+20)*2` | ❌ not exposed → add writable |
| EWT speed-up % | 2 | 0–100 % | ❌ not exposed → add writable |
| Cookerhood speed-up % | 3 | 0–100 % | ❌ not exposed → add writable |
| Reheating / post-heater target temp | 4 | °C, `(°C+20)*2` | ❌ not exposed → add writable |

Source: `registers.h CMD_SET_EWT_POSTHEATING`; openHAB `EWT_TEMPERATUR_LOW/HIGH`, `EWT_SPEED`, `COOKERHOOD_SPEED`, `HEATER_TARGET_TEMPERATUR` (all `change_command 0xed`, size 5, pos 0..4); adorobis `set_ewt` (pos 0/1 use `temp*2+40`, pos 2 raw %).
**Encoding caution:** The temperature bytes (0,1,4) use `(°C+20)*2`; bytes 2,3 are plain %. A naive "multiply everything" implementation would corrupt the % fields — encode per byte.
**adorobis writes only 3 bytes (0,1,2)**; our/openHAB 5-byte payload is the fuller, correct one for also setting cookerhood speed-up and reheat target (see Q4).

### F. Unit configuration / option flags — `CMD_SET_STATUS 0xD7` (8 bytes)
These are **installer/config** settings. Read reply is `0xD6` (11 bytes); the write
payload `0xD7` is 8 bytes. Byte layout from `registers.h`:
| Setting | Byte | Encoding | openHAB key / notes | Status |
|---|---|---|---|---|
| Preheater present | 0 | 1/0 | see **Q4 discrepancy** (openHAB maps `is_preheater` to byte4 bit0x40 instead) | 📖 read-only sensor |
| Bypass present | 1 | 1/0 | `is_bypass` | 📖 read-only sensor |
| Type | 2 | 1 = left, 2 = right | `recu_type` {1,2} | 📖 read-only sensor |
| Size | 3 | 1 = large, 2 = small | `recu_level` {0,1,2} (see Q4) | 📖 read-only sensor |
| Options bitmask | 4 | bit0x01 fireplace, 0x02 kitchenhood, 0x04 post-heating, 0x40 postheat-PWM | `is_chimney/ is_cookerhood/ is_heater/ is_preheater` each set a bit here | 📖 read-only sensors |
| (reserved) | 5 | 0 | — | n/a |
| Enthalpy present | 6 | 0 absent / 1 present / 2 without-sensor | `is_enthalpy` {0,1,2} | 📖 read-only sensor |
| EWT present | 7 | 0 absent / 1 regulated / 2 unregulated | `is_ewt` {0,1,2} | 📖 read-only sensor |

⚠️ **Writing `0xD7` is a device-configuration change** (tells the unit what hardware
it has). Wrong values can disable sensors/heating. Recommend exposing these only
behind the existing "Developer Mode" guard, and always read-modify-write the full
8-byte block so unrelated flags are preserved.

### G. Resets / self-test — `CMD_RESET_AND_SELF_TEST 0xDB` (4 bytes)
| Setting | Byte | Encoding | openHAB key | Status |
|---|---|---|---|---|
| Reset faults | 0 | 1 = reset | `error_reset` | ⚙️ exists as `error_reset()` in `comfoair.h`, not exposed |
| Reset settings (factory) | 1 | 1 = reset | `recu_reset` | ❌ not exposed (high risk) |
| Start self-test | 2 | 1 = start | `recu_autotest` | ❌ not exposed |
| Reset filter hours | 3 | 1 = reset | `filter_reset` | ⚙️ exists as `filter_reset()`, not exposed |

Source: `registers.h CMD_RESET_AND_SELF_TEST`; `comfoair.h error_reset()/filter_reset()`; openHAB `ERROR_RESET`/`RECU_RESET`/`RECU_AUTOTEST`/`FILTER_RESET`.
Recommend exposing as momentary `button:` entities. "Reset settings" (byte1) and
"self-test" (byte2) should be guarded — they interrupt normal operation.

### H. Analog input configuration — `CMD_SET_ANALOG_VALUES 0x9F` (19 bytes)
Full 19-byte block; openHAB exposes all of these as individual channels. Byte layout
is identical between read (`0x9D/0x9E`) and write (`0x9F`) per `registers.h`:
| Byte | Setting | Encoding |
|---|---|---|
| 0 | "Analog present" bitmask | 0x01 A1, 0x02 A2, 0x04 A3, 0x08 A4, 0x10 RF |
| 1 | "Regulate(1)/Control(0)" bitmask | same bit layout |
| 2 | "Negative(1)/Positive(0)" bitmask | same bit layout |
| 3/4/5 | Analog 1 min % / max % / setpoint % | 0–100 |
| 6/7/8 | Analog 2 min / max / setpoint | 0–100 |
| 9/10/11 | Analog 3 min / max / setpoint | 0–100 |
| 12/13/14 | Analog 4 min / max / setpoint | 0–100 |
| 15/16/17 | Analog RF min / max / setpoint | 0–100 |
| 18 | Priority control | 0 = analog inputs, 1 = weekly program |

Source: `registers.h CMD_SET_ANALOG_VALUES`; openHAB `IS_ANALOG1..RF`, `ANALOGx_MODE/NEGATIVE/MIN/MAX/VALUE`, `ANALOG_MODE` (all `change_command 0x9f`, size 19). Lower priority / advanced; most users will not need these. Expose behind developer mode, read-modify-write the full block.

### I. RS232 / PC mode — `CMD_SET_RS232_MODE 0x9B` (1 byte)
| Setting | Byte | Encoding |
|---|---|---|
| Connection mode | 0 | `0x00` end, `0x01` PC only, `0x03` PC master, `0x04` PC log |

Source: `registers.h`; openHAB `ACTIVATE` (`possible_values {0x03}`); adorobis `set_pc_mode`. This is a protocol/session control, not a user comfort setting — generally leave as-is (the component already talks to the unit without needing PC-master mode). Document only.

---

## Q3 — Frost protection / preheater on-off: does a normal-operation write exist?

**No.** Evidence:

- In the openHAB binding, the whole preheating/frost group read from `0xE1/0xE2`
  (`PREHEATER_VALVE`, `PREHEATER_FROST_PROTECT`, `PREHEATER_HEATING`,
  `PREHEATER_FROST_TIME`, `PREHEATER_OPTION`) and `FREEZE_MODE` (read from CC-Ease
  `0x35/0x3C`) **all lack a `change_command`** — they are strictly read-only.
  (`ComfoAirCommandType.java`, the `PREHEATER_*` / `FREEZE_MODE` enum entries.)
- adorobis only *reads* preheating status (`get_preheating_status` → publishes
  ON/OFF); there is no setter. ([`ca350.py`](https://github.com/adorobis/hacomfoairmqtt/blob/master/src/ca350.py))
- Our `registers.h` has `CMD_GET_PREHEATING_STATUS 0xE1` but no corresponding
  `CMD_SET_PREHEATING*` — consistent with there being no such command.
- The unit controls the preheater internally via installer parameter **P52**
  (sensitivity) together with incoming-air temp, comfort temp and exchanger type —
  i.e. it is firmware-regulated, not directly switchable over serial.
  ([openHAB community: Pre-Heater control parameter P52](https://community.openhab.org/t/comfoair-pre-heater-control-parameter-p52/17948))

### The only way to force the preheater: TEST MODE
Sequence: `CMD_SET_TEST_MODE 0x01` (enter) → drive outputs/flaps → `CMD_EXIT_TEST_MODE 0x19` (exit).

**`CMD_SET_OUTPUTS 0x05` (2 bytes)** — drives relays/triacs directly (`registers.h`):
- Byte[0] relay bitmask: `0x01` **Preheating relay**, `0x02` **Triac preheating**,
  `0x04` EWT supply, `0x08` EWT direction, `0x10` kitchen hood, `0x20` error.
- Byte[1] feedback: `0x01` Filter-full LED.
- To energize the preheater: set byte0 = `0x01` (relay) and/or `0x02` (triac). Clear to 0 to turn off.

**`CMD_SET_FLAPS 0x09` (2 bytes)** — drives the flap motors (`registers.h`):
- Byte[0] Bypass: `1` open / `0` closed / `3` stop.
- Byte[1] **Preheating flap**: `1` open / `0` closed / `3` stop.

**Behavior / safety of test mode:**
- While in test mode the unit's **own regulation is suspended** — you are driving
  the hardware directly, so normal ventilation/frost logic does not run. This is
  exactly the "our controller does all the work" device-hacking path the user
  described, and it is viable, but it means the ESP must take over everything it
  wants to keep running (fan outputs via `0x07 CMD_SET_ANALOG_OUTPUTS`, flaps, etc.)
  for as long as test mode is active.
- The sources (protocol description / these bindings) do **not** document an
  explicit watchdog auto-revert timer for the legacy CA protocol. None of the
  open-source clients implement or rely on an auto-exit. Treat auto-revert as
  **unconfirmed** — the safe assumption is that the unit stays in test mode until
  it receives `0x19` (or is power-cycled), so the firmware must guarantee it always
  sends `CMD_EXIT_TEST_MODE` (e.g. on boot and on a periodic heartbeat) to avoid
  leaving the unit un-regulated.
- Recommendation: if implementing a "force preheat / frost override", gate it hard
  behind developer mode, make it a momentary/timed action that auto-exits test mode
  after N seconds, and send `0x19` in `setup()` defensively.

---

## Q4 — Discrepancies between our `registers.h` and external sources

1. **`0xED` payload length.** `registers.h` documents `CMD_SET_EWT_POSTHEATING` as
   **5 bytes** (EWT low, EWT high, EWT speedup %, cookerhood speedup %, reheat
   target °C). openHAB agrees (5 bytes, pos 0..4). adorobis `set_ewt` sends only
   **3 bytes** (low, high, speedup). → Our 5-byte definition is the correct/fuller
   one; implement 5 bytes, but read-modify-write so you don't zero the cookerhood/
   reheat fields when only changing EWT. No contradiction in byte *order*.

2. **`(°C+20)*2` encoding — consistent.** `registers.h` (`0xED` comment and the
   CC-Ease `0x35` comment), `comfoair.h` (`set_comfort_temperature_`, temp sensor
   decode `/2 - 20`) and adorobis (`*2 + 40`) all agree. No discrepancy. Only
   caution: within `0xED` the encoding applies to bytes 0,1,4 (temps) but **not**
   bytes 2,3 (percent) — see section E.

3. **`0xCB` time-delay byte order — consistent.** `registers.h` write (`0xCB`) and
   read (`0xC9`) list the same 8 fields in the same order, and openHAB's
   `change_data_pos` 0..7 map to the same fields. No byte-order discrepancy
   (the brief flagged this as something to check — it checks out clean).

4. **`0xD7` preheater-present position mismatch (real discrepancy).**
   `registers.h` documents "Preheater present" as **Byte[0]** of the `0xD7` write
   block (mirroring the `0xD6` read reply byte 0). But openHAB's `IS_PREHEATER`
   writes into **byte 4, bit `0x40`** of `0xD7` (and reads reply byte 4 bit 0x40),
   i.e. it treats preheater-present as part of the *options bitmask* rather than its
   own byte. These two models disagree. Likely explanation: byte 0 is the canonical
   "preheater present" flag, and bit 0x40 of the options byte is a related
   (postheat-PWM-adjacent) option that openHAB repurposed. **Action:** before
   exposing any `0xD7` write, verify on the actual WHR 930 by reading `0xD6` and
   round-tripping a harmless flag — do not assume either mapping blindly. For the
   core user request (fan/temp/delays/EWT/resets) this does not matter; `0xD7` is
   advanced/config and should stay behind developer mode.

5. **`0xD7` byte 3 "Size" vs openHAB `recu_level`.** `registers.h` calls byte 3
   "Size (1=large/2=small)"; openHAB names the byte-3 channel `recu_level`
   (`possible_values {0,1,2}`). Naming/semantics differ; same byte position. Again
   advanced-only — verify before writing.

---

## Conclusions & recommendations

**For the user's stated goals:**

- **Frost protection on/off (primary ask):** there is no clean serial command to
  toggle it during normal operation — it is firmware-regulated (P52 + temps +
  exchanger type). The only override is **test mode** (`0x01`/`0x05`/`0x09`/`0x19`),
  which suspends the unit's own regulation. This is implementable and matches the
  user's "controller does all the work" acceptance, but must auto-exit test mode and
  be guarded. Document the tradeoff clearly in the UI.

- **"Everything writable" (equally important ask):** the high-value, low-risk
  additions that are currently read-only or unexposed are:
  1. **All 8 time delays** (`0xCB`, section D) → writable `number:` entities. Easy win.
  2. **EWT / post-heating 5 fields** (`0xED`, section E) → writable numbers. Easy win; encode temps vs % per byte.
  3. **Reset buttons** (`0xDB`, section G): error reset + filter reset already exist
     as C++ methods (`error_reset()`, `filter_reset()`) — just expose as `button:`.
     Add self-test / settings-reset behind developer mode.
  4. **Unit config flags** (`0xD7`, section F) and **analog input config** (`0x9F`,
     section H): advanced; expose behind developer mode with read-modify-write, and
     resolve the byte-0-vs-byte-4 preheater discrepancy (Q4.4) on-device first.

**Implementation guidance common to all writes:**
- Always **read the current full block first, modify the target byte(s), then write
  the whole block** (the component already does this for `0xCF` via
  `send_ventilation_levels_`). `0xCB`, `0xD7`, `0xED`, `0x9F` need the same pattern
  so unrelated fields aren't clobbered.
- Reuse the existing `write_command_()` framing + `comfoair_checksum_()`; no protocol
  changes needed.
- Respect the `(°C+20)*2` temp encoding (bytes), raw 0–100 for %, raw int for min/week.

**No changes were made to any repo files** (read-only investigation).
