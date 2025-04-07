import socket
import json
import os
import math
import sys
import time

def send_tcp_packet(server_ip, server_port, message):
    """
    Sends a TCP packet to the specified server with the given message.
    Returns the server's response as a string.
    """
    response = ""
    try:
        # Create a TCP socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the server
        client_socket.connect((server_ip, server_port))
        print(f"Connected to {server_ip}:{server_port}")
        # Send the message
        client_socket.sendall(message.encode('utf-8'))
        print(f"Sent: {message}")
        # Receive the response (if server sends one)
        response = client_socket.recv(1024).decode('utf-8')
        print(f"Received: {response}")
    except socket.error as e:
        print(f"Socket error: {e}")
    finally:
        # Close the connection
        client_socket.close()
        print("Connection closed.")
    return response

def wait_for_command_completion(server_ip, server_port):
    """
    Calls wait_command_done() repeatedly until the response indicates the command is complete.
    If the response is "wait_command_done:-1" (command still in progress), it waits and polls again.
    It returns True once "wait_command_done:0" is received.
    """
    while True:
        response = send_tcp_packet(server_ip, server_port, "wait_command_done()")
        if "wait_command_done:0" in response:
            print("Command completed successfully.")
            return True
        elif "wait_command_done:-1" in response:
            print("Command still in progress, waiting...")
            time.sleep(1)
            continue
        else:
            print("Unexpected response received:")
            print(response)
            time.sleep(1)
            continue

def radians_to_degrees(radians_list):
    """
    Converts a list of joint angles in radians to degrees, rounded to 3 decimals.
    """
    return [round(math.degrees(angle), 3) for angle in radians_list]

def load_joint_angles_from_json(json_path):
    """
    Loads a list of joint angle sets (in radians) from a JSON file.
    Returns a list of lists, or an empty list on error.
    """
    if not os.path.exists(json_path):
        print(f"Joint JSON file not found: {json_path}")
        return []
    try:
        with open(json_path, 'r') as f:
            joint_sets = json.load(f)
            print(f"Loaded {len(joint_sets)} joint sets from {json_path}")
            return joint_sets
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        return []

def format_angles_command(joint_degrees, speed=500):
    """
    Formats the joint angles list into a set_angles command string.
    Example: set_angles(0.000, -90.000, 0.000, -90.000, 0.000, 0.000, 500)
    """
    joint_str = ', '.join(f"{angle:.3f}" for angle in joint_degrees)
    return f"set_angles({joint_str}, {speed})"

if __name__ == "__main__":
    SERVER_IP = "192.168.1.159"
    SERVER_PORT = 5001
    SPEED = 500
    JSON_PATH = 'joint_trajectory_log.json'

    # 1. Set home position
    home_command = "set_angles(0, -90, 0, -90, 0, 0, 500)"
    send_tcp_packet(SERVER_IP, SERVER_PORT, home_command)
    time.sleep(1)  # short delay before checking completion
    if not wait_for_command_completion(SERVER_IP, SERVER_PORT):
        print("Exiting due to error in completion of home command.")
        sys.exit(1)

    # 2. Set workspace position
    workspace_command = "set_angles(40.396, -129.906, -130.695, -10.986, -89.824, -13.096, 500)"
    send_tcp_packet(SERVER_IP, SERVER_PORT, workspace_command)
    if not wait_for_command_completion(SERVER_IP, SERVER_PORT):
        print("Exiting due to error in completion of workspace command.")
        sys.exit(1)

    # 3. Load trajectory from JSON file and send each joint set
    joint_sets = load_joint_angles_from_json(JSON_PATH)
    if not joint_sets:
        print("No joint trajectory data found. Exiting.")
        sys.exit(1)

    for idx, joint_set in enumerate(joint_sets, start=1):
        joint_degrees = radians_to_degrees(joint_set)
        command = format_angles_command(joint_degrees, SPEED)
        print(f"\nTrajectory Step {idx}: Sending command: {command}")
        send_tcp_packet(SERVER_IP, SERVER_PORT, command)
        if not wait_for_command_completion(SERVER_IP, SERVER_PORT):
            print(f"Exiting due to error in completion of trajectory step {idx}.")
            sys.exit(1)
