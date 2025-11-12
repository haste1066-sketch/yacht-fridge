# Project: Yacht Fridge – Dual-ESP32 Control (Sender + Controller)

## Goal
Run a Peltier-based fridge on a yacht using two ESP32s:
- **Sender**: reads thermistors, decides fridge ON/OFF, shows temps & state on a 20x4 (2004) I2C LCD, and transmits state via ESP-NOW.
- **Controller**: receives state, drives relays for TECs, fans (fridge HX fan + hot-side Peltier fan), and pumps (coolant + future seawater), shows outputs & health on a display, and enforces safeties.

## High-level Behaviour
- Sender samples T_box, T_hotHX, T_ambient and optionally T_coolant.
- Applies hysteresis with user-set thresholds to compute `fridge_on`.
- Publishes a compact ESP-NOW packet every 1s (or on change), requests ACK.
- Controller actuates:
  - TECs: follow `fridge_on`.
  - Hot-side fan: ON with TECs; optional min idle purge when TECs OFF (e.g., 10–20% duty or low-duty cycle).
  - Fridge HX fan (inside box): PID or stepped duty; **low duty idle** when TECs OFF for circulation/defrost assist.
  - Pump 1 (coolant loop): ON with TECs; optional post-run for heat soak (e.g., 30–60s).
  - Pump 2 (future seawater): mirrors Pump 1 when seawater circuit present; otherwise disabled via config.
- Fallback if packets lost >N seconds: **safe idle** (all TECs OFF, fans low, pumps OFF) and show warning.

## Tasks
1. **Config** – Centralise pins, thresholds, and feature flags in `config.yaml`.
2. **Protocol** – Define a compact ESP-NOW message with CRC and monotonic tick in `protocol.md`.
3. **Sender** – Implement sensor read, debounced hysteresis, LCD view, and transmitter with retry/ACK.
4. **Controller** – Implement receiver with sanity checks, output state machine, post-run timers, defrost helper, LCD view.
5. **Safeties & Diagnostics**
   - Over-temp hot side → force pumps/fan HIGH, TECs OFF, alarm.
   - Probe missing/invalid → fall back and alarm.
   - Supply undervolt (ADC) → reduce load or shut down TECs.
6. **Observability**
   - LCD lines (Sender): T_box, T_hotHX, T_amb, Fridge: ON/OFF, RSSI/last ACK ms.
   - LCD lines (Controller): TEC/FAN/PUMP statuses, mode, alarms, last packet age.
7. **Future**
   - Add seawater loop enable.
   - Add SD logging or serial logging switchable.
   - Add manual override switch with timeout.

## Rules for the Agent
- Never change pin assignments without updating `config.yaml`.
- Propose changes before destructive refactors.
- Keep ISR-free timing; use `millis()`.
- All thresholds in °C; include hysteresis and minimum on/off runtimes.
- Keep ESP-NOW channel and MACs in `config.yaml`.
- Code must compile for **ESP32 Dev Module**, Arduino core 2.x.

## Done When
- Both sketches compile and run.
- Sender LCD shows temps & fridge state.
- Controller LCD shows outputs & health.
- Outputs respond with correct hysteresis and fail-safe on link loss.
