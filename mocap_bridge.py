import asyncio
import qtm_rt
from time import sleep, time
from math import copysign
from pymavlink import mavutil
import xml.etree.ElementTree as ET
from pprint import pformat
import argparse


def bodyToIndex(params: str) -> dict:
    xml = ET.fromstring(params)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


def rotToQuat(mat: tuple) -> tuple[float, ...]:
    """Converts rotation matrix to quaternions.

    Args:
        mat (tuple): Rotation matrix as tuple.

    Returns:
        tuple: Normalized quaternion (r, i, j, k).
    """
    r11, r12, r13 = mat[0], mat[1], mat[2]
    r21, r22, r23 = mat[3], mat[4], mat[5]
    r31, r32, r33 = mat[6], mat[7], mat[8]

    # Numerical errors leads to domain errors, thus, we max to avoid negatives
    r = 0.5 * max(0, 1 + r11 + r22 + r33) ** 0.5
    i = 0.5 * max(0, 1 + r11 - r22 - r33) ** 0.5 * copysign(1.0, r32 - r23)
    j = 0.5 * max(0, 1 - r11 + r22 - r33) ** 0.5 * copysign(1.0, r13 - r31)
    k = 0.5 * max(0, 1 - r11 - r22 + r33) ** 0.5 * copysign(1.0, r21 - r12)
    return (r, i, j, k)


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
    print("Relaying system time...")
    msg = mavutil.mavlink.MAVLink_system_time_message(time_unix_usec=int(time()), time_boot_ms=0)
    conn.mav.send(msg)
    lat, lon, alt = 0, 0, 0
    print(f"Setting GPS Origin to {lat=}, {lon=}, {alt=}")
    msg = mavutil.mavlink.MAVLink_gps_global_origin_message(
        latitude=lat,
        longitude=lon,
        altitude=alt,
        time_usec=int(time()),
    )
    conn.mav.send(msg)

    print("MavLink Initilized!")
    return conn


async def initQTM(qtm_address):
    conn = await qtm_rt.connect(qtm_address)
    if conn is None:
        raise Exception("Failed to connect to QTM!")

    # Get 6dof settings from qtm
    params = await conn.get_parameters(parameters=["6d"])
    body_index = bodyToIndex(params)

    print("Found rigid bodies:", pformat(body_index))

    print("QTM Initilized!")
    return conn, body_index


async def main():

    conn_mav = initMav(MAVLINK_ADDRESS)
    conn_qtm, body_index = initQTM(QTM_ADDRESS)

    def on_packet(packet):
        _, bodies = packet.get_6d()
        print(f"Framenumber: {packet.framenumber}")

        if RIGID_BODY is not None and RIGID_BODY in body_index:

            wanted_index = body_index[RIGID_BODY]
            position, rotation = bodies[wanted_index]
            x, y, z = position.x, position.y, position.z
            mat = rotation.matrix
            r, i, j, k = rotToQuat(mat)
            print(f"x={x:>10.5f}, y={y:>10.5f}, z={z:>10.5f}")
            print(f"q={r:>10.5f}, i={i:>10.5f}, j={j:>10.5f}, k={k:>10.5f}")
            if VERBOSE:
                print(f"Rot: {mat[0]:>10.5f}, {mat[1]:>10.5f}, {mat[2]:>10.5f}")
                print(f"     {mat[3]:>10.5f}, {mat[4]:>10.5f}, {mat[5]:>10.5f}")
                print(f"     {mat[6]:>10.5f}, {mat[7]:>10.5f}, {mat[8]:>10.5f}")
            else:
                # Move cursor n (\33[nA) lines up => Overwrites output and does not spam console
                print("\33[3A", end="")

            msg = mavutil.mavlink.MAVLink_att_pos_mocap_message(
                time_usec=int(time()),
                q=[r, i, j, k],
                x=x,
                y=y,
                z=z,  # TODO: Check if z should be inverted
            )
            conn_mav.mav.send(msg)

        else:
            raise Exception(f"Rigid Body {RIGID_BODY} was not found!")

    await conn_qtm.stream_frames(frames=f"frequency:{MAVLINK_FREQUENCY}", components=["6d"], on_packet=on_packet)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        prog="Qualisys to ArduCopter Bridge using MAVLink",
        description="Allows sending Qualisys Motion Capture data through the mavlink protocol to ArduCopter. For Copter settings, follow https://ardupilot.org/copter/docs/common-optitrack.html",
        epilog="Developed by Andreas Larsen (ankula@mp.aau.dk)",
    )

    parser.add_argument("-b", "--body", default="mav")
    parser.add_argument("-qtm", "--qtm_address", default="127.0.0.1")
    parser.add_argument("-mav", "--mavlink_address", default="tcp:192.168.2.1:5760")
    parser.add_argument("-f", "--frequency", default=5)
    parser.add_argument("-v", "--verbose", default=True)

    args = parser.parse_args()

    RIGID_BODY = args.body  # "mav"  As named in QTM
    QTM_ADDRESS = args.qtm_address  # "127.0.0.1"
    MAVLINK_ADDRESS = args.mavlink_address  # "tcp:192.168.2.1:5760"
    MAVLINK_FREQUENCY = args.frequency  # 5 Hz
    VERBOSE = args.verbose

    asyncio.ensure_future(main())
    asyncio.get_event_loop().run_forever()
