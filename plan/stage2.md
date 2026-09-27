# Stage 2: the gimbal add-on

Part of the [overall plan](plan.md). It builds on [Stage 1](stage1.md): the rover driver, the run
summary script, and the baseline numbers.

**Goal:** use the STorM32 gimbal to make ORB-SLAM3 more robust on the rover, **without changing
the library**. It's used in three ways:

| Feature | What the gimbal does | ORB-SLAM3 problem it targets | Measured on |
|---|---|---|---|
| **G1: stability** | Holds the camera steady and level; yaw follows the rover | Motion blur and jerky frames from vibration and bumps | Route A (and B, C) |
| **G3: coverage sweeps** | At a stop, turns the camera slowly through ±170° | A place is recognised only from roughly the same direction as before, so out-and-back routes don't close loops | Route B |
| **G4: recovery sweeps** | When tracking is lost, turns the camera back toward where it last tracked, then around | After 3 s of failed relocalisation, ORB-SLAM3 starts a new, separate map | Route C |

**G2** is the enabling step: the Jetson controlling the gimbal over serial.

### The rule that makes this "our contribution"

ORB-SLAM3 doesn't know the gimbal exists. It never asks for the camera to move. It sees images,
**copes** with the camera rotating (stereo gives depth in every frame), and **reports** how
tracking is going through its public API.

Our add-on reads those reports (`GetTrackingState()`, `GetTrackedMapPoints()`, the pose from
`TrackStereo()`) and decides when and how to move the gimbal. ORB-SLAM3 then does what it always
does: tracking, storing keyframes, relocalising, closing loops, merging maps.

Nothing under `src/`, `include/` or `Thirdparty/` changes.

---

## 1. Hardware for this stage

| Item | Notes |
|---|---|
| STorM32 BGC **v1.3** board, 3 brushless gimbal motors, gimbal frame | The v1.3 board belongs to the "v1" family. Its documentation is the [STorM32 v1 wiki](https://www.olliw.eu/storm32bgc-v1-wiki/) |
| **IMU1** (the MPU6050 sensor board) | Must sit on the **camera plate**. Then it measures the camera's own orientation, which G4 uses |
| D435 + a thin, flexible USB 3 cable (0.5–1 m) | Stiff cables fight the gimbal motors |
| Gimbal battery or supply | At the board's rated voltage (read the board; typically a 3S LiPo). **Separate from the Jetson's supply** |
| USB cable STorM32 ↔ Jetson | For commands (G2). Any spare Jetson USB port works; it uses almost no bandwidth |
| Windows PC with `o323BGCTool` | STorM32 configuration. **The GUI version must match the board's firmware version** |
| Vibration dampers, M3 screws, spirit level, multimeter, zip ties | For mounting |

---

## 2. Software architecture

Everything new sits in `addons/` and in the Stage 1 rover driver.

```
addons/gimbal/
  storm32_link.h/.cc        serial protocol: frames, CRC, commands, replies
  gimbal_controller.h/.cc   own thread: runs sweep patterns at 30 Hz, polls the gimbal IMU, handles abort
  sweep_policy.h/.cc        decides WHEN: keys, tracking state, point count, stillness (G3, G4)
  PROTOCOL.md               what G2 discovery found for OUR firmware
  config/storm32_settings.* exported GUI configuration (G1)
tools/
  storm32_probe.py          G2 discovery and manual test tool
  summarize_run.py          (from Stage 1, extended with Stage 2 events)
Examples/Stereo/stereo_realsense_D435_rover.cc     gains --gimbal and --auto-recover
```

**Build:** add the three `.cc` files to the rover driver's existing target in `CMakeLists.txt`,
and give it the include path:
```cmake
    add_executable(stereo_realsense_D435_rover
            Examples/Stereo/stereo_realsense_D435_rover.cc
            addons/gimbal/storm32_link.cc
            addons/gimbal/gimbal_controller.cc
            addons/gimbal/sweep_policy.cc)
    target_include_directories(stereo_realsense_D435_rover PRIVATE ${PROJECT_SOURCE_DIR}/addons)
    target_link_libraries(stereo_realsense_D435_rover ${PROJECT_NAME})
```

**Threads and data flow:**
```
 camera frames ──► main loop ──► SLAM.TrackStereo()  (ORB-SLAM3, unchanged)
                      │
                      ├─ after each frame: state, tracked points, pose, latest key
                      ▼
                 sweep_policy  ── commands (non-blocking) ──►  gimbal_controller thread (30 Hz)
                      │                                             │
                      └─ operator messages, events.csv ◄── status ──┤
                                                                    ▼
                                                             storm32_link ──USB──► STorM32
```
- **The main loop never waits on the gimbal.** Serial replies take milliseconds, and a blocked
  loop would drop camera frames. The policy only *posts* commands to a queue; the controller
  thread does all the serial work.
- **Same program, feature flags.** Without `--gimbal`, the driver behaves exactly like the Stage 1
  baseline. Every Stage 2 result is compared against the same program, so the add-on is the only
  difference.

**Command line** (Stage 1 flags plus these):
```
stereo_realsense_D435_rover <voc> <settings> [--out DIR] [--no-viewer]
                            [--gimbal /dev/storm32] [--auto-recover] [--weak-threshold N]
```

**Operator keys** (the Stage 1 keyboard thread already captures keys in `g_key`):

| Key | Action |
|---|---|
| `s` | Coverage sweep (G3). Refused if the rover isn't still or tracking isn't OK |
| `f` | Abort anything, and return the camera to forward / follow mode |
| `a` | Toggle automatic recovery (G4) on or off |
| `q` | Clean exit and save (Stage 1) |

**Operator messages** are printed with an `[ADDON]` prefix, plus a terminal bell (`\a`) for
urgent ones, so they stand out among ORB-SLAM3's output in the SSH window.

**New `events.csv` entries** (same `time,event,detail` format as Stage 1):
`key`, `gimbal_open`, `sweep_refused`, `sweep_start`, `sweep_end`, `sweep_abort`, `weak_on`,
`weak_off`, `lost`, `recover_fast_start`, `relocalized`, `recover_sweep_start`,
`recover_sweep_end`, `imu_yaw` (once per second).

---

## 3. G1: stability

**Why:** ORB-SLAM3 matches features between consecutive frames and needs sharp images whose
content changes smoothly. On a rover, vibration blurs the image and bumps make the view jump.
Both reduce matches, and can break tracking. A gimbal absorbs both and keeps the view level when
the rover rocks. No software is involved: this is purely mechanical and gimbal configuration.

### 3.1 Safety (read first)
1. **Never power the gimbal motors without a balanced camera mounted.** Motors fighting an
   unbalanced or empty load overheat and can burn out.
2. **Keep the gimbal still for ~10 s after powering on.** It calibrates its gyro at start-up, and
   moving it then causes a tilted horizon or drift.
3. Connect the gimbal battery last and disconnect it first, before any mechanical change.
4. LiPo: never below 3.5 V per cell under load; charge on a fire-safe surface, and never leave it
   unattended.
5. Check the motor temperature after 10 minutes of running. Too hot to hold means lower the motor
   power (Vmax) or improve the balance.

### 3.2 Test the gimbal board on its own
1. Connect the STorM32 to the Windows PC over USB. The board's logic powers from USB. Leave the
   motors unpowered (no battery).
2. Open `o323BGCTool`, click **Connect**, then **Read**. Write down the **firmware version** and
   **board version**. Use the GUI version that matches the firmware; don't flash new firmware
   unless something doesn't work.
3. In the live data display, tilt the IMU board by hand, and check that pitch, roll and yaw change.
4. **Back up the original settings** to a file: `addons/gimbal/config/original_settings.*`.

### 3.3 Mount the camera
1. Fix the D435 to the camera plate by its **two M3 holes on the back**: snug, with blue
   (medium) threadlocker, and screws short enough not to bottom out. If only the 1/4"-20 tripod
   thread fits the plate, add an anti-rotation point.
2. Lenses forward. The two IR lenses must be **horizontal**, parallel to the roll axis. Centre the
   camera left-to-right on the plate.
3. Check that nothing of the gimbal frame appears in the IR view at any pitch angle. Tilt by hand
   to both extremes and look in `realsense-viewer`.
4. IMU1 must be **rigidly fixed to the camera plate**. Write down its orientation (arrow
   direction, which face is up) for the GUI setup.

### 3.4 Cable routing (before balancing)
1. Run the USB cable along the arms, from the camera to the pitch motor, then along the roll arm,
   then to the yaw motor and the base.
2. At each joint, leave a **slack loop** that allows that axis's full travel, including **±170°
   yaw** for G3. Tie the cable down on the **fixed side** of each joint.
3. With the power off, move each axis through its full range by hand. The cable must never go
   tight or snag, and must not bend sharply at the connector. The D435's USB-C plug must never be
   levered sideways.

### 3.5 Balancing (motors unpowered)
Good balance lets the motors hold the camera with little power, so they run cooler and stabilise
better. Balance in this order, and at each step let go and see whether the axis stays put:
1. **Pitch, fore and aft:** camera level. If the nose dips or rises, slide the camera or plate
   back or forward.
2. **Pitch, up and down:** point the camera straight down. If it swings back toward level, move
   the camera up or down in the plate's slots.
3. **Roll:** hold the roll arm level. If it rotates to one side, slide it sideways.
4. **Yaw:** tilt the whole gimbal base 20–30°. If the yaw axis swings, adjust the yaw arm.
5. Check all three again at ±45°. Mark the final slider positions with a paint pen.

### 3.6 Gimbal configuration (GUI, battery connected)
1. **IMU orientation:** enter IMU1's mounting orientation, or use the GUI's configuration wizard.
   Check that tilting the plate by hand moves the GUI's pitch and roll the right way.
2. **Motor poles:** from the motor's specification (commonly 14).
3. **Motor direction and start-up position:** use the GUI's automatic detection.
4. **Accelerometer calibration:** level the camera plate (spirit level on the camera's top face),
   then run the calibration. This defines "level".
5. **PID and motor power, one axis at a time** (pitch, then roll, then yaw):
   - **Vmax:** the lowest value that still holds the camera firmly against a gentle push.
   - **P:** raise it until the axis buzzes or oscillates, then reduce by ~30 %.
   - **D:** raise it to damp overshoot after flicking the frame. Too much D gives a high-pitched
     buzz.
   - **I:** raise it until the axis returns exactly to level after a slow push, without slow
     wobble.
6. **Pan mode = HOLDHOLDPAN:** pitch and roll hold level; yaw pans (follows) the rover. Set the
   pan deadband to a few degrees, and the pan speed to medium.
7. **Serial access:**
   - Leave the **MAVLink heartbeat off**. When it's on, the UART accepts only MAVLink. USB accepts
     every command either way.
   - Leave the RC inputs unassigned, or assigned to things that won't interfere with serial
     angle commands.
8. **Write + Store** to the board's memory (settings that aren't stored are lost at power-off).
   Export the settings to `addons/gimbal/config/rover_settings.*`, and take screenshots of each tab.

### 3.7 Mount the gimbal on the rover
- Replace the Stage 1 bracket with the gimbal base, **on rubber dampers**. Put it at the same
  position and height as the Stage 1 camera, so the view matches the baseline as closely as
  possible.
- Re-check that no part of the rover appears in the IR view at any gimbal angle.
- Give the gimbal its own battery or regulator branch. Connect the STorM32's USB to the Jetson.

### 3.8 Acceptance checks
- **Tap test:** tap the frame firmly. The camera returns to level within ~0.5 s, with no bounce.
- **Follow test:** turn the rover 90° slowly, then quickly. The camera follows smoothly and
  settles facing forward.
- **USB test:** `lsusb -t` shows 5000M for the D435 through full gimbal travel.
- **Stream test:** stream both IR cameras in `realsense-viewer` at 640×480 @ 30 for 30 min while
  the gimbal runs and the rover moves, with `sudo dmesg -w | grep -iE "usb|uvc|disconnect|reset"`
  open in another terminal. There must be no disconnects.
- **Thermal:** after 30 min, the motors are warm but you can hold them.

### 3.9 G1 evaluation
Drive routes **A, B and C** (Stage 1, section 4.6) **3 times each** with the gimbal in follow mode
(`--gimbal` not needed yet). Folder names end in `_g1`.

| Compare with the baseline | Expected with G1 |
|---|---|
| `lost_events`, `new_maps` | same or fewer, especially on rough floors |
| `relocalized` | same or fewer (fewer losses to recover from) |
| route A `endpoint_pct` | same or lower |
| `dropped frs` messages | same |

If the floors are smooth indoors, G1's effect may be small. An extra run over rough ground
(outdoor path, carpet edges, thresholds) makes the difference visible.

---

## 4. G2: gimbal control from the Jetson

**Why a separate step:** G3 and G4 need the Jetson to command the gimbal's yaw and read the
camera's orientation. The command set depends on the firmware, so we confirm exactly what our
board does before writing any sweep logic.

### 4.1 What the STorM32 serial protocol offers

Source: [STorM32 v1 wiki, Serial Communication](https://www.olliw.eu/storm32bgc-v1-wiki/Serial_Communication).

- **Frame to the board:** `0xFA`, `LEN` (payload length), `CMD`, `PAYLOAD…`, `CRC_lo`, `CRC_hi`.
- **Replies from the board:** start with `0xFB`, same layout.
- **CRC:** the x25 16-bit CRC used by MAVLink. It covers every byte **except the start byte**, and
  is sent low byte first.
- **Every command is answered**, either with data or with `CMD_ACK`.
- **Ports:** USB, UART and Bluetooth. When the MAVLink heartbeat is on, the UART accepts only
  MAVLink; USB accepts everything. **We use USB.**

| Command | ID | Payload | Use in our add-on |
|---|---|---|---|
| `CMD_GETVERSION` | 1 | none | Discovery: firmware version |
| `CMD_GETDATA` | 5 | `uint8 type` (only `0`) | Live data, including **IMU1 pitch/roll/yaw ×100** (degrees) |
| `CMD_SETYAW` | 12 | `uint16` 700–2300; **0 = recenter** | Return yaw to centre after a sweep |
| `CMD_SETPANMODE` | 13 | `uint8`: 0 off, 1 HOLDHOLDPAN, 2 HOLDHOLDHOLD, 3 PANPANPAN, 4 PANHOLDHOLD, 5 PANHOLDPAN | 1 for follow mode; possibly 2 during sweeps |
| `CMD_SETANGLE` | 17 | `float pitch, float roll, float yaw` (degrees), `uint8 flags`, `uint8 type = 0` | **Sweep motion.** flags bits 0x01 pitch, 0x02 roll, 0x04 yaw: bit set = "limited" (clamped by RcMin/RcMax, absolute only); bit clear = "unlimited" (no clamping, relative or absolute) |
| `CMD_ACK` | 150 (reply) | `uint8`: 0 OK, 1 FAIL, 2 ACCESS_DENIED, 3 NOT_SUPPORTED, 150 TIMEOUT, 151 CRC, 152 PAYLOADLEN | Error checking |

**Not documented, so found out in discovery:**
- which of the 32 `GETDATA` values are the IMU1 angles;
- how `SETANGLE` yaw behaves in each pan mode (what "0°" is relative to, and whether follow mode
  overrides it);
- what the board does when angle commands stop arriving;
- the reply delay.

### 4.2 Connect and name the port
```bash
lsusb                                   # the STorM32 appears as a USB serial device
sudo dmesg | tail                       # -> /dev/ttyACM0 or /dev/ttyUSB0
udevadm info -a -n /dev/ttyACM0 | grep -m2 -E 'idVendor|idProduct'
```
Create `/etc/udev/rules.d/99-storm32.rules`, with the IDs just shown:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", SYMLINK+="storm32", MODE="0666"
```
Then run `sudo udevadm control --reload && sudo udevadm trigger`. The port is now always
`/dev/storm32`, whatever else is plugged in.

### 4.3 The discovery tool: `tools/storm32_probe.py`
```python
#!/usr/bin/env python3
"""STorM32 RC-command probe for G2 discovery.  sudo apt install python3-serial
usage: storm32_probe.py PORT version | data [seconds] | panmode N | angle PITCH ROLL YAW
                        | recenter | sweep DEG RATE_DEG_S"""
import struct, sys, time
import serial

ACK = {0: "OK", 1: "FAIL", 2: "ACCESS_DENIED", 3: "NOT_SUPPORTED", 150: "TIMEOUT", 151: "CRC", 152: "PAYLOADLEN"}

def crc_x25(data: bytes) -> int:                      # MAVLink's crc_accumulate, init 0xFFFF
    crc = 0xFFFF
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([len(payload), cmd]) + payload       # CRC covers LEN, CMD, PAYLOAD
    return b"\xFA" + body + struct.pack("<H", crc_x25(body))

def reply(port, timeout=0.5):
    end = time.time() + timeout
    while time.time() < end:
        if port.read(1) != b"\xFB":
            continue
        hdr = port.read(2)
        if len(hdr) < 2:
            return None
        n, cmd = hdr
        rest = port.read(n + 2)
        if len(rest) < n + 2:
            return None
        payload = rest[:n]
        ok = struct.unpack("<H", rest[n:])[0] == crc_x25(hdr + payload)
        return cmd, payload, ok
    return None

def send(port, cmd, payload=b"", show=True):
    t0 = time.time()
    port.write(frame(cmd, payload))
    r = reply(port)
    if show:
        if r is None:
            print("no reply")
        else:
            c, p, ok = r
            extra = f" ACK={ACK.get(p[0], p[0])}" if c == 150 and p else ""
            print(f"reply cmd={c} len={len(p)} crc_ok={ok} {1000*(time.time()-t0):.1f} ms{extra}  {p.hex()}")
    return r

def set_angle(port, pitch, roll, yaw, show=False):
    return send(port, 17, struct.pack("<fffBB", pitch, roll, yaw, 0, 0), show)   # flags 0 = unlimited

port = serial.Serial(sys.argv[1], 115200, timeout=0.05)
what, args = sys.argv[2], sys.argv[3:]
if what == "version":
    send(port, 1)
elif what == "data":                                  # watch which values change as you tilt by hand
    end = time.time() + (float(args[0]) if args else 10)
    while time.time() < end:
        r = send(port, 5, b"\x00", show=False)
        if r and r[2]:
            p = r[1]
            vals = struct.unpack(f"<{len(p)//2}h", p[:len(p)//2*2])
            print(" ".join(f"{i}:{v}" for i, v in enumerate(vals)))
        time.sleep(0.1)
elif what == "panmode":
    send(port, 13, bytes([int(args[0])]))
elif what == "angle":
    set_angle(port, *map(float, args), show=True)
elif what == "recenter":
    send(port, 12, struct.pack("<H", 0))
elif what == "sweep":                                 # 0 -> +DEG -> -DEG -> 0 at RATE deg/s, 30 Hz
    deg, rate = float(args[0]), float(args[1])
    for a, b in [(0, deg), (deg, -deg), (-deg, 0)]:
        steps = max(1, int(abs(b - a) / rate * 30))
        for k in range(1, steps + 1):
            set_angle(port, 0.0, 0.0, a + (b - a) * k / steps)
            time.sleep(1 / 30)
```

### 4.4 Discovery procedure

The gimbal is on the rover, powered and balanced, and the rover is still. Record every result in
`addons/gimbal/PROTOCOL.md`.

| Test | Command | What to record |
|---|---|---|
| T1 version | `storm32_probe.py /dev/storm32 version` | Reply bytes, round-trip time. **No reply means stop here:** check the port and cable, and that MAVLink is off |
| T2 data fields | `... data 30`, while tilting and turning the **camera** slowly by hand: pitch, then roll, then yaw | Which indices change by ~100 per degree for IMU1 pitch, roll and yaw; the sign convention; how many replies per second the loop achieves |
| T3 angle in follow mode | `... panmode 1`, then `... angle 0 0 30` | Does the camera turn 30° and hold? Does it drift back (follow mode overriding)? |
| T4 angle in hold mode | `... panmode 2`, then `... angle 0 0 30`; then turn the **rover** by hand | Does the camera hold 30° from the rover's heading, or in the world? |
| T5 return | `... panmode 1`, then `... recenter` | Does the camera return to forward and resume following? |
| T6 sweep | `... sweep 170 30`, then `... sweep 90 60` | Smooth motion? Does the IMU yaw from T2 (in a second terminal) track the command? Lag? |
| T7 commands stop | `... angle 0 0 45`, then wait 10 s | Does it hold, or return to follow / centre after a timeout? |
| T8 limits | an `angle` beyond the GUI's RcMin/RcMax with flags 0 | Confirms "unlimited" bypasses the RC limits. **Our code enforces ±170° itself** because of the cable |

**Decide the command sequence** from these results. The expected outcome, confirmed or corrected
by T3–T5:
- **enter sweep mode:** `SETPANMODE 2` (HOLDHOLDHOLD), so follow mode doesn't pull the camera
  back;
- **move:** stream `SETANGLE` yaw targets at 30 Hz;
- **leave:** `SETPANMODE 1` (HOLDHOLDPAN), then `SETYAW 0` (recenter).

Write the final sequence into `PROTOCOL.md`; `gimbal_controller` implements exactly that.

**Fallback** if T1 or T3–T5 fail (the firmware won't take angle commands): G3 and G4 become
**operator-assisted**. The policy still decides *when*, and prints instructions ("sweep now: turn
the camera slowly left to 170°, right to −170°, back"). The operator moves the camera by hand or
through the gimbal's RC input. That is weaker (speed isn't controlled), but still testable. Check
this early: it's the main risk of Stage 2.

### 4.5 `storm32_link` (C++)
It mirrors the probe tool in C++, using POSIX `termios`:
- open the port with `O_RDWR | O_NOCTTY`, `cfmakeraw`, 115200 baud (ignored by USB serial);
- the same frame builder and x25 CRC as the Python version;
- `readFrame()` with a timeout.
```cpp
class Storm32Link {
public:
    bool open(const std::string& dev);                     // "/dev/storm32"
    bool setPanMode(uint8_t mode);                         // CMD 13; true if ACK == OK
    bool setAngles(float pitch, float roll, float yaw);    // CMD 17, flags 0, type 0
    bool recenterYaw();                                    // CMD 12, value 0
    bool readImuAngles(float& pitch, float& roll, float& yaw);   // CMD 5; indices from PROTOCOL.md, /100
private:
    int fd_ = -1;
    bool transact(uint8_t cmd, const std::vector<uint8_t>& payload,
                  uint8_t& reply_cmd, std::vector<uint8_t>& reply, int timeout_ms = 50);
};
```
Every call is **synchronous**, and is only ever made from the controller thread.

### 4.6 `gimbal_controller` (C++, its own thread)
```cpp
struct GimbalCmd { enum Type { COVERAGE_SWEEP, RECOVER_TURN, RECOVER_WIGGLE, ABORT } type; float arg = 0; };

class GimbalController {
public:
    void start(Storm32Link* link);          // launches the 30 Hz thread
    void stop();
    void post(const GimbalCmd& c);          // non-blocking; ABORT pre-empts anything running
    bool busy() const;                      // a pattern is running
    float imuYaw() const;                   // latest camera yaw from IMU1, degrees (unwrapped)
    int   patternsDone() const;             // incremented when a pattern ends; the policy watches it
};
```
**What the thread does each 33 ms tick:**
1. If a pattern is running: move the yaw target one step toward the next waypoint at the
   pattern's speed. Clamp it to **±170°** (cable limit) and pitch to **[−30°, +20°]**. Send
   `setAngles`.
2. Every 3rd tick (~10 Hz): `readImuAngles`. Store the yaw **unwrapped**: add ±360 when it jumps
   by more than 180°, so differences are always correct.
3. When the pattern ends: run the "leave" sequence from `PROTOCOL.md`, increment
   `patternsDone`, and report it to the policy.
4. `ABORT`: run the "leave" sequence immediately.

**Patterns** (waypoints are yaw angles relative to straight ahead):

| Pattern | Waypoints | Speed | Duration |
|---|---|---|---|
| `COVERAGE_SWEEP` | 0 → +170 → −170 → 0 | 30°/s | ≈ 23 s |
| `COVERAGE_SWEEP` with pitch pass | the same, then again at pitch −15° | 30°/s | ≈ 46 s |
| `RECOVER_TURN(Δ)` | 0 → Δ, hold 1.5 s | 60°/s | ≤ 4.5 s |
| `RECOVER_WIGGLE` | 0 → +45 → −45 → 0 | 60°/s | ≈ 3 s |

**Test** (with ORB-SLAM3 not running): a small test program posts each pattern. Using the probe's
`data` readout, or `imuYaw()` printed, check that:
- the speeds match;
- the limits hold;
- `f` / ABORT always returns the camera to follow mode within 1 s.

---

## 5. G3: coverage sweeps at stops

### 5.1 The problem
ORB-SLAM3 recognises a place by comparing the current image with stored **keyframes**, using the
bag-of-words vocabulary. That only works if a keyframe looked at the place **from roughly the
same direction**.

On route B, the return trip sees the corridor from the other side. No stored keyframe matches,
so no loop closure fires, and the drift from the outward trip stays in the map. The Stage 1
baseline measures how often this happens.

### 5.2 What the sweep does
At a stop, the camera turns slowly through ±170° while ORB-SLAM3 keeps running normally.
ORB-SLAM3 stores keyframes facing every direction, as it always does when the view changes
enough. Coming back from any direction, there's now a stored view to match: the loop closes, or
tracking recovers in the same map instead of starting a new one.

### 5.3 Why these numbers

| Choice | Reasoning |
|---|---|
| **30°/s** | Our config's focal length is `fx ≈ 383` px, which at 640 px width gives ≈ 80° horizontal view and ≈ 6.7 px per degree near the centre. At 30 fps, 30°/s is **1° per frame**: consecutive frames overlap by ~99 % and tracking holds easily. With the driver's 5 ms auto-exposure cap, the camera turns 0.15° during an exposure, ≈ **1 px of blur**, which is negligible |
| **±170°, back and forth** | Continuous turning one way would wind up the USB cable. Returning through 0° unwinds it |
| **Only when still** | Turning in place is safe for *stereo* SLAM: every frame has its own depth, unlike single-camera SLAM, where pure rotation gives nothing to triangulate. Combining it with driving mixes two motions and raises the risk of losing tracking |
| **Optional −15° pitch pass** | Adds nearby floor and low objects. Useful in sparse spaces; it doubles the time |

**What each sweep keyframe contains:** ORB-SLAM3 builds new keyframes from stereo points, using
all points closer than `Stereo.ThDepth` (40 × baseline ≈ 2 m), and at least the 100 nearest
points (`maxPoint = 100` in [src/Tracking.cc](../src/Tracking.cc#L3365)). So sweeps add the most
in rooms and corridors, with surfaces within a few metres. In large open spaces, sweep keyframes
hold fewer, farther points.

### 5.4 Behaviour, step by step
1. The operator stops the rover at a chosen spot and presses **`s`**.
2. **The policy checks** before starting. If either check fails, it logs `sweep_refused` and
   prints why ("move less" / "wait for tracking"):
   - tracking state is **OK (2)**;
   - **the rover is still**: the camera centre (from the `TrackStereo` pose) moved less than
     **5 cm over the last 1 s**. The policy keeps a 1 s ring buffer of positions.
3. The policy posts `COVERAGE_SWEEP`, logs `sweep_start`, and prints "[ADDON] sweeping: keep the
   rover still (~23 s)".
4. The controller enters sweep mode and runs the pattern. **ORB-SLAM3 keeps tracking
   throughout.**
5. In the Map Viewer (if open): a **ring of blue keyframes pointing outward** around the stop, and
   new points in every direction.
6. At the end, the camera returns to follow mode. The policy logs `sweep_end` and prints
   "[ADDON] sweep done: drive on".
7. **If tracking is lost during a sweep:** the policy aborts it (`sweep_abort`), and G4 takes over
   if enabled. With the numbers above this shouldn't happen, and each occurrence is recorded.

### 5.5 Where to sweep
- the start point;
- turnarounds and dead ends;
- junctions and doorways;
- any spot the route will pass again **from a different direction**.

On route B: the start and the turnaround.

### 5.6 G3 evaluation

| Runs | Setup |
|---|---|
| Route B × 3, `_g3` | G1 gimbal + sweeps at the start and at the turnaround |
| Route B × 3, `_g1` | Reference without sweeps (from G1) |
| Route A × 3, `_g3a` | Sweep at the start only: checks that sweeps cause no harm |

| Measure | Expected with G3 (route B) |
|---|---|
| `loops` (on the return trip) | ≥ 1 in most runs; rare without |
| `endpoint_pct` | lower, since the loop correction removes drift |
| `new_maps`, `lost_events` | same or fewer |
| `sweep_abort` events | 0 |

**Extension of `summarize_run.py`:** count `sweep_start`, `sweep_end` and `sweep_abort` from
`events.csv`, and print the time of each `*Loop detected` relative to the turnaround sweep's end.
Record the time in the console log with a tagged line, or read it from the `map_changed` hint.

---

## 6. G4: automatic recovery sweeps

### 6.1 What ORB-SLAM3 does when tracking fails
Taken from [src/Tracking.cc](../src/Tracking.cc), in the `RECENTLY_LOST` handling for visual-only
modes:
1. **If the current map has more than 10 keyframes**, the state becomes `RECENTLY_LOST` (3).
   Every frame, ORB-SLAM3 tries to **relocalise**: it compares the image with stored keyframes.
2. If relocalisation doesn't succeed within **3 seconds** of camera time, the state becomes `LOST`
   (4). The old map is **stored**, and a **new map** starts (`Stored map with ID` in the log).
   ORB-SLAM3 may later **merge** them, if the new map sees a place the old one knows
   (`*Merge detected`).
3. **If the map has 10 or fewer keyframes**, there's no 3-second window. It goes straight to
   `LOST`, and the map is **discarded**.
4. The viewer draws **only the active map**, so after a new map starts, the old one vanishes from
   view, even though it still exists.

### 6.2 What G4 does
Aim the camera so ORB-SLAM3's own relocalisation or merging succeeds:
- **phase A**, inside the 3-second window: turn the camera **back to the direction it faced when
  tracking was last OK**, which is exactly what relocalisation needs;
- **phase B**, if the window is missed and a new map has started: a coverage sweep, so the new
  map sees places the stored map knows and ORB-SLAM3 can merge them.

**How "back to where it was" works:**
- IMU1 sits on the camera plate, so its yaw follows the camera's heading in the world. It includes
  the rover's own turns.
- The policy remembers `yaw_ok`, the IMU1 yaw at the last frame with state OK.
- On losing tracking, the camera has turned by Δ = `yaw_now − yaw_ok` since then. Turning the
  gimbal by **−Δ** points the camera back at the last well-tracked view.
- The IMU yaw drifts slowly, but over a few seconds that's negligible, and only the *difference*
  is used.

### 6.3 Policy state machine (`sweep_policy`)

| State | Enter when | Actions | Leave when |
|---|---|---|---|
| `NORMAL` | start; after any recovery | follow mode. Every frame with state OK: store `yaw_ok` | `WARN` / `RECOVER_FAST` / `RECOVER_SWEEP` below |
| `WARN` | state OK, but tracked points < **W** for > 0.5 s | print "[ADDON] tracking weak: slow down / stop" (once), log `weak_on` | points > 1.2·W: log `weak_off`, back to `NORMAL` |
| `RECOVER_FAST` (phase A) | state changes to **3** (RECENTLY_LOST) and `--auto-recover` is on | bell + "[ADDON] TRACKING LOST: stop the rover", log `lost` and `recover_fast_start`. Δ = unwrap(`yaw_now − yaw_ok`). If \|Δ\| ≥ 10°: post `RECOVER_TURN(−Δ)` (clamped to ±170°). Otherwise post `RECOVER_WIGGLE` (the camera didn't turn; look around nearby) | state **2**: log `relocalized` with the time since `lost`, post ABORT (back to follow), print "[ADDON] relocalized: continue", go to `NORMAL`. State **4**: go to `RECOVER_SWEEP` |
| `RECOVER_SWEEP` (phase B) | state 3 → 4 (a map was stored, so there's something to merge with) | print "[ADDON] new map started: stop for a recovery sweep". Once still and OK in the new map (same checks as G3): post `COVERAGE_SWEEP`, log `recover_sweep_start` | pattern finished: log `recover_sweep_end`, go to `NORMAL`. Whether a merge happened is read from `console.log` afterwards |
| (none) | state 2 → 4 directly | the map had ≤ 10 keyframes and was discarded: log `lost` with detail `small_map` | nothing to recover toward. Stay in `NORMAL` |

The operator can **always** override: `f` aborts to follow mode, `a` turns G4 off.

**Threshold W.** Set it from the Stage 1 baseline: roughly **one third of the median `tracked`
count** while tracking is OK, adjusted using route C, where the count falls just before a loss.
Pass it as `--weak-threshold N`, and record it in `run_info.txt`.

**Why 60°/s in phase A:** the window is 3 s, so speed matters. At 60°/s a 90° turn takes 1.5 s,
and blur is ≈ 2 px with a 5 ms exposure, which is still fine for matching.

### 6.4 G4 evaluation

| Runs | Setup |
|---|---|
| Route C × 3, `_g1` | Reference: gimbal, no auto-recovery |
| Route C × 3, `_g4` | Gimbal + `--auto-recover` |
| Route A × 3, `_g4a` | Checks that G4 doesn't trigger or harm normal driving |

| Measure | Expected with G4 |
|---|---|
| relocalised within 3 s (`relocalized` event, and no `Stored map` line) | more often |
| time from `lost` to `relocalized` | shorter |
| `new_maps` | fewer |
| `merges` after a new map | more often (phase B) |
| `split` | less often |
| G4 triggers on route A | 0, or only the early warning |

---

## 7. Stage 2 demo

One route shows everything: out along a corridor, into a room, and back.
1. **G1:** drive over a rough threshold; the image stays steady and level.
2. **G3:** sweep at the start and at the turnaround. On the return, `*Loop detected` fires and the
   trajectory visibly snaps into place.
3. **G4:** approach a blank wall until tracking is lost. The camera turns back on its own, and
   `Relocalized!!` appears.
4. **Results slide:** the table of baseline vs G1/G3/G4 numbers from sections 3.9, 5.6 and 6.4.

Record the viewer (screen recording, over VNC) plus a phone video of the rover and gimbal moving.

---

## 8. Stage 2 is done when

- G1 is mounted, tuned and passes 3.8; its settings are exported to `addons/gimbal/config/`.
- `PROTOCOL.md` records our firmware's behaviour (T1–T8), including the final command sequence.
- G3 and G4 run from the rover driver with `--gimbal` / `--auto-recover`, and every event is
  logged.
- The evaluation tables in 3.9, 5.6 and 6.4 are filled in (3 runs per condition).

## 9. Risks and troubleshooting

| Risk / symptom | Mitigation |
|---|---|
| **Firmware doesn't accept angle commands, or ignores them in pan mode** | Found in T1–T5 before any code is written. Operator-assisted fallback (4.4) |
| No reply on `/dev/storm32` | MAVLink heartbeat on (only affects UART; use USB); wrong port; check `dmesg` |
| Camera drifts back during a sweep | Follow mode overriding the command: use the "enter sweep" pan mode found in T3/T4 |
| Tracking lost during sweeps | Lower the sweep speed to 20°/s; check the lighting; check the exposure cap |
| Cable pulls at ±170° | Increase the slack loops, or reduce the limit to ±150° (a constant in `gimbal_controller`) |
| IMU yaw jumps at ±180° | Unwrap (4.6); use differences only |
| Phase A too slow for the 3 s window | The early warning gets the operator to stop *before* losing tracking; phase B still aims for a merge |
| Differences too small to show | 3 runs per condition; same routes, operator and lighting; add a rough-ground route for G1 |
| Terminal stops echoing after a crash | Type `reset` (the keyboard thread didn't restore the terminal) |
