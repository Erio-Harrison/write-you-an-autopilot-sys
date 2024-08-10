import zmq
import json

def main():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")

    # Create a simulated vehicle status message
    vehicle_state = {
        "position_x": 1.0,
        "position_y": 2.0,
        "yaw": 0.5,
        "velocity": 10.0,
        "acceleration": 1.0
    }

    # Convert the message to a JSON string
    message = json.dumps(vehicle_state)

    print(f"Sending message: {message}")
    socket.send_string(message)

    # Wait for a reply, set a 5 second timeout
    socket.RCVTIMEO = 5000  # 5000 milliseconds = 5 seconds
    try:
        reply = socket.recv_string()
        print(f"Received reply: {reply}")
    except zmq.error.Again:
        print("No reply received within timeout period")

    socket.close()
    context.term()

if __name__ == "__main__":
    main()