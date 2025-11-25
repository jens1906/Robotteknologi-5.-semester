# UR3e Launch Options

## Quick Start

### For Simulation (No Real Robot) - DEFAULT
```bash
./launch_ur3e_driver.sh
```
This uses `mock_hardware` for simulation without a physical robot.

### For Real Robot Connection
```bash
./launch_ur3e_real.sh
```
This connects to the actual robot at 192.168.56.2.
**Important:** Start the External Control program on the teach pendant before launching!

## Launch MoveIt (Same for Both)

After starting either driver, launch MoveIt in a second terminal:
```bash
./launch_ur_moveit.sh
```

## Known Issues

### Mock Hardware Crash on Shutdown
- **Problem:** Using mock hardware causes segmentation faults when you press Ctrl-C
- **Why:** This is a known bug in ros2_control's mock_components plugin
- **Workaround:** 
  - Run `./stop_robot_safely.sh` before pressing Ctrl-C
  - Or just force-close the terminals (the crash happens during cleanup, not during normal operation)
  - The crash is harmless - it only occurs when shutting down

### Planning Stops Working After Several Movements
- **Cause:** Mock hardware state can become inconsistent after multiple movements
- **Solution:** 
  - Restart both the driver and MoveIt
  - Use real hardware for production/long-running tests
  - Check MoveIt logs for "Unable to solve the planning problem"

## Recommended Workflow

1. **Development/Testing:** Use `./launch_ur3e_driver.sh` (mock/simulation)
2. **Integration Testing:** Use `./launch_ur3e_real.sh` (real robot)
3. **Production:** Always use real robot

## File Summary

- `launch_ur3e_driver.sh` - Simulation mode (mock hardware)
- `launch_ur3e_real.sh` - Real robot mode
- `launch_ur_moveit.sh` - MoveIt interface (works with both)
- `stop_robot_safely.sh` - Graceful shutdown helper
