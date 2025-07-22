# Qualisys MOCAP Bridge for ArduCopter
This project allows relaying Qualisys motion capture data to ArduCopter using the MAVLink ATT_POS_MOCAP message.

- [Qualisys MOCAP Bridge for ArduCopter](#qualisys-mocap-bridge-for-arducopter)
  - [QUICKSTART:](#quickstart)
  - [Setup/Guide](#setupguide)
    - [Prerequisites](#prerequisites)
    - [Qualisys Setup](#qualisys-setup)
    - [ArduCopter Setup](#arducopter-setup)
    - [Python Setup](#python-setup)
  - [Usage](#usage)
    - [Verify ArduCopter recived external nav data](#verify-arducopter-recived-external-nav-data)
  - [References](#references)


## QUICKSTART: 
```
python mocap_bridge.py -b "<name_of_body>" -qtm "<qtm_address>" -mav "<mavlink_address>" -f <update_frequency> -v <True|False>
```
Example:
```
python mocap_bridge.py -b "mav" -qtm "127.0.0.1" -mav "tcp:192.168.2.1:5760" -f 10 -v True
```

## Setup/Guide
### Prerequisites
- Qualisys cameras mounted and powered
- ArduCopter >= 4.0
  - A working MAV in *stabilized* mode
- Method for delivering MAVLink commands to the MAV (e.g. [DroneBridge](https://github.com/DroneBridge/ESP32))
- Correctly placed tracking markers
  - [Placing markers on a rigid body](https://docs.qualisys.com/getting-started/content/17_rigid_body_series/17a_how_to_track_rigid_bodies/placing_markers_on_a_rigid.htm)
- Windows 10/11 ([System requirements](https://docs.qualisys.com/qtm/content/general_info/system_requirements.htm)) 
- Python >= 3.12

### Qualisys Setup
This section contains links to the official documentation. I have added comments based on my experience where I think they might be useful. Depending on your expertise, these comments might be obvious, so feel free to ignore.

The main resources are [Getting Started](https://docs.qualisys.com/getting-started/content/getting_started/introduction.htm) and [QTM Documentation](https://docs.qualisys.com/qtm/content/welcome_to_qtm.htm).

1. [Installing QTM and QDS](https://docs.qualisys.com/getting-started/content/getting_started/setting_up_your_system/configuring_the_network/installing_qtm_and_qds.htm)
   - To process the camera information, we need the *Qualisys Track Manager* (QTM), and its sub-component *Qualisys DHCP Server* (QDS). QTM allows us to define which markers compose the MAV, and stream information on the local network. QDS allows the QTM to identify and communicate with the cameras ⇒ 

2. [Running the Configuration Wizard](https://docs.qualisys.com/getting-started/content/34_how_to_configure_qds/running_the_configuration.htm)
   - If you are connected through a router, you will need to look at the [Advanced Network Settings](https://docs.qualisys.com/getting-started/content/34_how_to_configure_qds/advanced_network_settings.htm). To get this to work, while retaining your internet connection, only tick the *Enable QDS operation for the network connection* - don't press the *Autoconfig* button.

3. [Viewing the cameras in QTM](https://docs.qualisys.com/getting-started/content/6_connecting_cameras_series/6d_how_to_set_your_cameras_up_in_qtm/viewing_the_cameras_in_qtm.htm)
   - The red "record" button in the action bar will also look for your cameras. This will also tell you if the system needs calibration.

4. [Wand Calibration](https://docs.qualisys.com/qtm/content/project_options/wand_calibration.htm?Highlight=calibration)
   - It might make your life easier, if you place the reference L-bracket in an accessible place - this will make it easier to align the MAV axis later.

5. [Defining a rigid body in QTM](https://docs.qualisys.com/getting-started/content/17_rigid_body_series/17a_how_to_track_rigid_bodies/defining_a_rigid_body_in_qtm.htm)
   - Changing to the 3D view ([Types of Windows](https://docs.qualisys.com/getting-started/content/16_how_to_set_up_your_qtm_workspace/types_of_windows.htm)) will allow you to define a *rigid body*, i.e. the collection of markers which correspond to the MAV without recording first.
   - There might be other reflective surfaces in the environment, and as such, the system might see more markers than actually present. You can try to optimize this with the [Exposure & Flash Time and Marker Threshold](https://docs.qualisys.com/getting-started/content/getting_started/setting_up_your_system/optimizing_the_camera_settings/exposure___flash_time_and_marker_threshold.htm).

6. Align your MAV with the Qualisys reference such that the the MAV forward is in the x direction. Then reset *Reset Rotation* ([Rigid Body Settings](https://docs.qualisys.com/getting-started/content/17_rigid_body_series/17a_how_to_track_rigid_bodies/rigid_body_settings.htm)) to tell QTM how it should interpret the orientation.

### ArduCopter Setup

Configure ArduCopter as described in [Configuration the drone](https://ardupilot.org/copter/docs/common-optitrack.html#configuration-the-drone) (Don't mind that this article refers to a different MOCAP system) :
- [x] set `AHRS_EKF_TYPE=3` 
- [x] set `EK3_ENABLE=1` and `EK2_ENABLE=0`
- [x] set `COMPASS_USE=COMPASS_USE2=COMPASS_USE3=0`.
- [x] set `VISO_TYPE=1`
- [x] set `VISO_POS_M_NSE <=0.3` and `VISO_YAW_M_NSE <= 0.2`
-  Lower to increase the weighting of position measurements from motion capture system.
- [x] set `EK3_SRC1_POSXY=6` and `EK3_SRC1_POSZ=6`
- [x] set `EK3_SRC1_VELXY=0` and `EK3_SRC1_VELZ=0`
- [x] set `EK3_SRC1_YAW=6`


### Python Setup
Install PyMavLink and Qualisys Python SDK:
```
python -m pip install pymavlink qtm-rt
```

## Usage
Ensure QTM is running, is calibrated, and has a rigid body defined. Then, navigate to the folder containing `mocap_bridge.py` using
```
cd C:\path\to\python\file\qualisys_arducopter_bridge
```
Then run 
```
python mocap_bridge.py -b "<name_of_body>" -qtm "<qtm_address>" -mav "<mavlink_address>" -f <update_frequency> -v <True|False>
```
where
| Command             | Shorthand | Type   | Description                                                                       |
| ------------------- | --------- | ------ | --------------------------------------------------------------------------------- |
| `--body`            | `-b`      | `str`  | Name of the rigid body to track. The name is defined in QTM.                      |
| `--qtm_address`     | `-qtm`    | `str`  | Address to listen for QTM stream. QTM defaults to `127.0.0.1`.                    |
| `--mavlink_address` | `-mav`    | `str`  | Address to send MAVLink messages. DroneBridge defaults to "tcp:192.168.2.1:5760". |
| `--frequency`       | `-f`      | `int`  | Frequency (Hz) which to send position and attitude updates.                       |
| `--verbose`         | `-v`      | `bool` | Print additional information, and prevent overwriting console output              |


### Verify ArduCopter recived external nav data
*IMPORTANT:* Ensure that the MAV has EKF3 Origin set! This can easily be verified in Mission Planner or MavProxy by looking for the quadcopter icon on the map. If it isn't there, don't try to fly the drone, it will almost certainly crash! This should be done automatically by `mocap_bridge.py`, but it is worth verifying before takeoff.

Lastly, if you see following message appearing, then the MAV is receiving pose data from Qualisys:
```
EKF3 IMU<X> is using external nav data EKF3 IMU0 initial pos NED = <X>,<X>,<X> (m) EKF3 IMU<X> ext nav yaw alignment complete
```
(Use e.g. Mission Planner to check messages)

## References
- [Qualisys Docs - Getting Started](https://docs.qualisys.com/getting-started/content/getting_started/introduction.htm)
- [Qualisys Docs - QTM Documentation](https://docs.qualisys.com/qtm/content/welcome_to_qtm.htm).
- [Qualisys Docs - Python SDK](https://qualisys.github.io/qualisys_python_sdk/index.html)
- [ArduPilot Docs - Optitrack for Non-GPS Navigation](https://ardupilot.org/copter/docs/common-optitrack.html)
- [ArduPilot Forum - Indoor Flight with External Navigation Data](https://discuss.ardupilot.org/t/indoor-flight-with-external-navigation-data/29980)
- [ArduPilot Docs - Complete Parameter List](https://ardupilot.org/copter/docs/parameters.html)
- [MAVLink Docs - Common Message Set](https://mavlink.io/en/messages/common.html)