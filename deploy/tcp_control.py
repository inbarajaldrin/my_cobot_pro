import socket
import time

def send_tcp_packet(server_ip, server_port, messages, delay_between=0.5):
    try:
        # Create a TCP socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_ip, server_port))
        print(f"Connected to {server_ip}:{server_port}")

        for i, message in enumerate(messages):
            try:
                client_socket.sendall(message.encode('utf-8'))
                print(f"[{i+1}/{len(messages)}] Sent: {message}")

                # Optionally wait for a response
                response = client_socket.recv(1024).decode('utf-8')
                print(f"Received: {response}")

                # Small delay to avoid flooding the robot
                time.sleep(delay_between)

            except socket.error as e:
                print(f"Error sending message {i+1}: {e}")
                break

    except socket.error as e:
        print(f"Connection error: {e}")

    finally:
        client_socket.close()
        print("Connection closed.")

if __name__ == "__main__":
    SERVER_IP = "192.168.1.159"
    SERVER_PORT = 5001

    MESSAGES = [
        "set_angles(-172.116, 14.703, -110.525, 10.615, -91.26, -18.24, 500)",
        "set_angles(45.095, -84.195, -110.604, -79.781, 91.891, 18.842, 500)",
        "set_angles(45.095, -84.195, -110.604, -79.781, 91.892, 18.842, 500)",
        "set_angles(-172.116, 14.703, -110.525, 10.615, -91.26, -18.24, 500)",
        "set_angles(-172.116, -95.822, 110.525, -99.91, -91.26, -18.24, 500)",
        "set_angles(45.095, -84.195, -110.604, -79.781, 91.892, 18.843, 500)",
        "set_angles(45.094, -84.195, -110.605, -79.78, 91.892, 18.842, 500)",
        "set_angles(-172.116, -95.822, 110.525, -99.91, -91.26, -18.241, 500)",
        "set_angles(-172.116, 1.024, -44.393, 138.163, 91.259, 161.76, 500)",
        "set_angles(45.095, -84.195, -110.604, -79.781, 91.892, 18.842, 500)"
    ]

    send_tcp_packet(SERVER_IP, SERVER_PORT, MESSAGES)
