# Dual Footpod (nRF5340, NCS 2.9.1)

## Overview
This firmware runs on a single nRF5340 (Raytac MDBT53-1M, ISM330DHCX IMU) to demo **Session Mode** (500 Hz streaming) and **All-Day Auto-Burst Mode** (26 Hz with 500 Hz bursts on step detection). Data is sent via BLE, with packets including sequence IDs and timestamps for future dual-pod sync. Use nRF Connect for Mobile to log data and a Python script to convert logs to CSV.

**Features**:
- **Session Mode**: 500 Hz accel/gyro streaming, 10-sample batches, started/stopped by short button press.
- **All-Day Auto-Burst Mode**: 26 Hz low-power sampling, auto 500 Hz bursts on ≥8 steps (>1.5 g, 48–210 steps/min), with 1 s pre-trigger, 4 s post-trigger, 10 s cooldown, 20-min daily budget.
- **Button/LED UX**: Short press for Session (LED solid), 3 s press for All-Day (LED slow pulse), bursts (LED fast blink), cooldown/idle (LED off).
- **BLE**: Motion500 (500 Hz, 200 bytes), Motion40 (26 Hz, 20 bytes).
- **Logs**: Serial logs for state transitions; nRF Connect logs packets for CSV.

## Flash/Load Firmware
1. **Setup**: Install nRF Connect SDK v2.9.1 in VS Code (nRF Connect extension).
2. **Build**:
   ```bash
   west build -b nrf5340dk/nrf5340/cpuapp
   ```
3. **Flash**:
   - Connect nRF5340 via USB (or J-Link).
   - Run:
     ```bash
     west flash
     ```
4. **Verify**: Serial terminal (e.g., nRF Terminal in VS Code) shows `IMU BLE Auto-Burst Example Starting...`. Device advertises (UUID: `12345678-1234-5678-1234-1234567890ab`).

## Run the Logger (nRF Connect for Mobile)
1. **Install**: Download nRF Connect for Mobile (Android/iOS) and nRF Logger (optional backup).
2. **Connect**:
   - Open nRF Connect, scan for "nRF5340-IMU" (UUID: `12345678-1234-5678-1234-1234567890ab`).
   - Connect, discover services.
3. **Subscribe**:
   - Subscribe to Motion500 (`12345679-1234-5678-1234-1234567890ab`) and Motion40 (`1234567a-1234-5678-1234-1234567890ab`) characteristics.
   - Enable logging (swipe right > Logger > Enable).
4. **Log Data**:
   - Start Session or All-Day Mode (see below).
   - Logs show hex packets (20 bytes for 26 Hz, 200 bytes for 500 Hz) with timestamps.
   - Save logs: Tap SAVE icon, export as text (`.txt`). Note: nRF Connect Mobile doesn’t export CSV directly.
5. **Convert to CSV**: Use Python script (below) to parse text logs to CSV.

**Note**: 500 Hz may lag nRF Connect; limit to short sessions (<5 min) or use a custom client for longer runs.

## Start/Stop Sessions
- **Session Mode**:
  - **Start**: Press Button 1 (sw0) briefly (<3 s).
    - **LED**: Solid (led2).
    - **Behavior**: Streams 500 Hz (10-sample batches, ~20 ms intervals).
  - **Stop**: Press Button 1 again.
    - **LED**: Off.
    - **Behavior**: Returns to Idle, stops streaming.
- **All-Day Auto-Burst Mode**:
  - **Start**: Hold Button 1 for 3 s.
    - **LED**: Slow pulse (500 ms on/off).
    - **Behavior**: Samples at 26 Hz (~38 ms intervals).
  - **Burst Trigger**: Walk with pod (e.g., on shoe) for ≥8 steps (accel >1.5 g, 48–210 steps/min).
    - **LED**: Fast blink (100 ms on/off).
    - **Behavior**: Switches to 500 Hz (20 s burst, 1 s pre-trigger, 4 s post-trigger), then 10 s cooldown (LED off), back to 26 Hz.
  - **Stop**: Hold Button 1 for 3 s.
    - **LED**: Off.
    - **Behavior**: Returns to Idle.

## Expected CSV Output
- **Fields**: `time_us,ax,ay,az,gx,gy,gz,lr_role,event`
  - `time_us`: Microsecond timestamp (`uint64_t`).
  - `ax,ay,az`: Scaled accel (g × 1671, `int16_t`).
  - `gx,gy,gz`: Scaled gyro (dps × 7500, `int16_t`).
  - `lr_role`: `L` (single pod).
  - `event`: 0 (not implemented).
- **Example Rows** (from Session Mode, 500 Hz):
  ```
  time_us,ax,ay,az,gx,gy,gz,lr_role,event
  1000000,1671,0,-1671,7500,0,0,L,0
  1002000,1650,100,-1700,7400,50,10,L,0
  ```
- **Example Rows** (from All-Day Mode, 26 Hz):
  ```
  time_us,ax,ay,az,gx,gy,gz,lr_role,event
  2000000,0,0,-1671,0,0,0,L,0
  2038462,0,0,-1671,0,0,0,L,0
  ```
- **Conversion Script**:
  ```python
  import struct
  import csv
  def parse_sample(data):
      return struct.unpack('<HQ3h3h', data)
  packets = [...] # Load nRF Connect text log, convert hex to bytes
  with open('imu_data.csv', 'w', newline='') as f:
      writer = csv.writer(f)
      writer.writerow(['time_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'lr_role', 'event'])
      for packet, _ in packets:
          if len(packet) == 20: # 26 Hz
              seq_id, time_us, ax, ay, az, gx, gy, gz = parse_sample(packet)
              writer.writerow([time_us, ax, ay, az, gx, gy, gz, 'L', 0])
          else: # 500 Hz batch
              for i in range(0, len(packet), 20):
                  seq_id, time_us, ax, ay, az, gx, gy, gz = parse_sample(packet[i:i+20])
                  writer.writerow([time_us, ax, ay, az, gx, gy, gz, 'L', 0])
  ```
  - **Note**: Parse hex from nRF Connect text logs (e.g., via `binascii.unhexlify()`). Example input: `0001 0000E8D4A510 03E7 0000 FFF1 1D4C 0000 0000`.

## Notes
- **Stationary Demo**: Use code modification to simulate bursts (no step detection without motion).
- **Logger Limits**: nRF Connect Mobile may lag at 500 Hz; export logs via SAVE or ADB (`adb logcat -d > log.txt`).
