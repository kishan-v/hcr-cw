## Unity Setup

#### 1. Open the project through Unity Hub

If there are any compile errors on startup, click `ignore`

#### 2. Select control method

To switch between joystick control and virtuix control, enable/disable `virtuixDummyObject` at the top of the inspector and disable/enable `TeleopJoystickCommunication` in a single controller (currently, only a single websocket works at one time)

#### 3. Run

Before starting one of the server nodes, the unity program needs to be run

#### 4. Stop, edit, restart to switch control method

Repeat 2 onwards to change control method

---

#### Error Checklist

```
Check everything is on (!)
```

- `SteamVR_Actions`
    - Try running once
    - Are the controllers binded to the trigger (rotate) and the trackpad (teleop)?

- Running with no errors but not sending
    - Are the correct objects enabled?
    - Are the correct scripts enabled?
    - Is data being received from the virtuix? (check via `virtuixDummyObject`'s inspector)

- Not updating virtuix data
    - Is the raw data changing in inspector?
    - Adjust thresholds

- Laptop crashing
    - Reduce computation?
    - Close everything else
    - Restart
