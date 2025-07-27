import argparse
import asyncio
import xml.etree.ElementTree as ET
from math import isnan
from pprint import pformat
from time import time
from functools import partial

import qtm_rt
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R


def bodyToIndex(params: str) -> dict:
    xml = ET.fromstring(params)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


def eulerToQuat(roll: float, pitch: float, yaw: float) -> list[float,]:
    r = R.from_euler("xyz", [roll, pitch, yaw], degrees=True)
    return r.as_quat(scalar_first=True).tolist()


def initMav(mavlink_address: str, lat: int = 0, lon: int = 0, alt: int = 0) -> mavutil.mavtcp | mavutil.mavtcpin | mavutil.mavudp | mavutil.mavwebsocket | mavutil.mavmcast:
    """Initilize the MAVLink connection with ArduCopter.

    Args:
        mavlink_address (str): The address to look communicate with ArduCopter. For example Dronebridge defaults to "tcp:192.168.2.1:5760".
        lat (int, optional): degE7, Latitude (WGS84). Defaults to 0.
        lon (int, optional): degE7, Latitude (WGS84). Defaults to 0.
        alt (int, optional): mm, Altitude (MSL). Positive for up. Defaults to 0.

    Returns:
        mavutil.mavXXX: Serial, UDP, TCP, etc. mavlink connection
    """

    print("Connecting to MAV...")
    conn = mavutil.mavlink_connection(mavlink_address)

    print("Waiting for Heartbeat...")
    conn.wait_heartbeat()

    # print("Relaying system time...")
    # msg = mavutil.mavlink.MAVLink_system_time_message(time_unix_usec=int(time()), time_boot_ms=0)
    # conn.mav.send(msg)

    lat, lon, alt = 55.270884, 14.755208, 0
    print(f"Setting GPS Origin to {lat=}, {lon=}, {alt=}")
    msg = conn.mav.command_long_encode(
            conn.target_system,
            conn.target_component,
            48,#mavutil.mavlink.MAV_CMD_SET_GLOBAL_ORIGIN,  # ID of command to send
            0,              # Confirmation
            0,              # (Use Current)
            int(lat*1e7),   # (Latitude)
            int(lon*1e7),   # (Longitude)
            alt,            # (Altitude)
            0,              # (Unused)
            0,              # (Unused)
            0,              # (Unused)
        )
    conn.mav.send(msg)

    print("MavLink Initilized!")
    return conn


def initQTM(qtm_address):
    conn = qtm_rt.connect(qtm_address)

    if conn is None:
        raise Exception("Failed to connect to QTM!")
    
    # Get 6dof settings from qtm
    params = conn.get_parameters(parameters=["6d"])
    body_index = bodyToIndex(params)
    print("Found rigid bodies:", pformat(body_index))

    print("QTM Initilized!")
    return conn, body_index


def sendPosAtt(packet, body_index, conn_mav):
        _, bodies = packet.get_6d_euler()
        print(f"Errors:")
        print(f"Framenumber: {packet.framenumber}")

        wanted_index = body_index[RIGID_BODY]
        position, rotation = bodies[wanted_index]
        x, y, z = position.x, position.y, position.z
        roll, pitch, yaw = rotation.a1, rotation.a2, rotation.a3
        print(f"x={x:>11.5f}, y={y:>11.5f}, z={z:>11.5f}")
        print(f"roll={roll:>11.5f}, pitch={pitch:>11.5f}, yaw={yaw:>11.5f}")
        print(f"qw={qw:>11.5f}, qx={qx:>11.5f}, qy={qy:>11.5f}, qz={qz:>11.5f}")

        if isnan(x) or isnan(y) or isnan(z) or isnan(roll) or isnan(pitch) or isnan(yaw):
            print("\33[5A", end="")
            print(f'Errors: QTM LOST TRACK OF BODY "{RIGID_BODY}" AT FRAME {packet.framenumber}!', end="")
            print("\33[5B", end="")
            return

        qw, qx, qy, qz = eulerToQuat(roll, pitch, yaw)
        msg = mavutil.mavlink.MAVLink_att_pos_mocap_message(
            time_usec=int(time()),
            q=[qw, qy, qx, qz],
            x=x / 1000, # Qualisys reports in cm
            y=y / 1000,
            z=z / 1000,
        )
        conn_mav.mav.send(msg)
        if not VERBOSE:
            # Move cursor n (\33[nA) lines up => Overwrites output and does not spam console
            print("\33[5A", end="")

async def main():

    conn_mav = initMav(MAVLINK_ADDRESS)
    conn_qtm, body_index = await initQTM(QTM_ADDRESS)

    if RIGID_BODY is None and RIGID_BODY not in body_index:
        raise Exception(f"Rigid Body {RIGID_BODY} was not found!")
    
    
    await conn_qtm.stream_frames(frames=f"frequency:{MAVLINK_FREQUENCY}", components=["6d"], on_packet=partial(sendPosAtt, body_index, conn_mav))


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        prog="Qualisys to ArduCopter Bridge using MAVLink",
        description="Allows sending Qualisys Motion Capture data through the mavlink protocol to ArduCopter. For Copter settings, follow https://ardupilot.org/copter/docs/common-optitrack.html",
        epilog="Developed by Andreas Larsen (ankula@mp.aau.dk)",
    )

    parser.add_argument("-b", "--body", default="mav")
    parser.add_argument("-qtm", "--qtm_address", default="127.0.0.1")
    parser.add_argument("-mav", "--mavlink_address", default="tcp:192.168.2.1:5760")
    parser.add_argument("-f", "--frequency", default=10)
    parser.add_argument("-v", "--verbose", default=False)

    args = parser.parse_args()

    RIGID_BODY = args.body
    QTM_ADDRESS = args.qtm_address
    MAVLINK_ADDRESS = args.mavlink_address
    MAVLINK_FREQUENCY = args.frequency
    VERBOSE = args.verbose

    asyncio.ensure_future(main())
    asyncio.get_event_loop().run_forever()
