## Unity Setup

#### 1. Open the project through Unity Hub

If there are any compile errors on startup, click `ignore`

#### 2. Equipment

Ensure all equipment is turned on and controller bindings are set to `Bernie` configuration in `Window > SteamVR Input > Open Bindings` (map touchpad and trigger).

#### 3. Run

After starting one of the server nodes, the unity program needs to be run

#### 4. Change control method

| Key   | Action Description              |
|-------|---------------------------------|
| X     | No movement (default)           |
| V     | Virtuix                         |
| J     | Joystick                        |

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
