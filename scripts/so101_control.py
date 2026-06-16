"""
Desc: This is a helper script to perform various tasks such as calibrating the arm, recording episode and replaying episode. 
"""

import argparse
import time
import json
import threading
import sys
from so101_ros2.lerobot.so101 import SO101

def wait_for_enter(stop_event):
    input()
    stop_event.set()

def record_episode(robot, filename, rate):
    print(f"Recording episode at {rate} Hz. Press Enter to stop recording.")
    episode = []
    start_time = time.time()
    interval = 1.0 / rate
    stop_event = threading.Event()
    listener = threading.Thread(target=wait_for_enter, args=(stop_event,))
    listener.daemon = True
    listener.start()
    try:
        while not stop_event.is_set():
            timestamp = time.time() - start_time
            state = robot.get_device_state()
            episode.append({'timestamp': timestamp, 'joint_states': state})
            # sys.stdout.write(f'\rWrote - Timestamp:{timestamp}, JointStates: {state}%')
            # sys.stdout.flush()
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    with open(filename, "w") as f:
        json.dump(episode, f)
    print(f"\nEpisode recorded to {filename}")

def play_episode(robot, filename):
    with open(filename, "r") as f:
        episode = json.load(f)
    print(f"Replaying episode from {filename}")
    start_time = time.time()
    for entry in episode:
        timestamp = entry['timestamp']
        joint_states = entry['joint_states']
        now = time.time() - start_time
        wait_time = timestamp - now
        if wait_time > 0:
            time.sleep(wait_time)
        robot._bus.sync_write("Goal_Position", joint_states)
    print("Replay finished.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Initialize LeRobot arm with specified parameters.')

    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                        help='Serial port for the LeRobot arm (e.g., /dev/ttyACM0)')
    parser.add_argument('--name', type=str, default='so101_follower',
                        help='Name of the LeRobot arm.')
    parser.add_argument('--recalibrate', action='store_true',
                        help='Set to True to recalibrate the LeRobot arm (default False).')
    parser.set_defaults(recalibrate=False) 
    parser.add_argument('--record', metavar='FILENAME', help='Record episode to file')
    parser.add_argument('--rate', type=float, default=10.0, help='Recording rate in Hz (default: 10)')
    parser.add_argument('--replay', metavar='FILENAME', help='Replay episode from file')

    args = parser.parse_args()
    robot = SO101(port=args.port, name=args.name, recalibrate=args.recalibrate)

    try:
        if not args.recalibrate:
            print("Connecting to LeRobot arm...")
            robot.connect()

        if args.record:
            record_episode(robot, args.record, args.rate)
        elif args.replay:
            play_episode(robot, args.replay)

    except Exception as e:
        print(f"Failed to connect or operate LeRobot arm: {e}")
    finally:
        if ('robot' in locals()) and robot.is_connected: # Assuming is_connected() method
            print("Disconnecting LeRobot arm...")
            robot.disconnect()
            print("LeRobot arm disconnected.")
