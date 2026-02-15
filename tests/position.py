import requests
import json
import time

def main():
    # Replace with your RoArm's actual IP address
    ip_addr = '192.168.4.1'
    
    # The JSON command for "Get Status/Feedback"
    # T:105 requests current position (x,y,z,t) and joint angles (b,s,e,h)
    command = {"T": 105}
    json_command = json.dumps(command)
    
    url = f"http://{ip_addr}/js?json={json_command}"

    print(f"Monitoring RoArm-M2 at {ip_addr}...")
    print(f"{'TIMESTAMP':<10} | {'BASE':<6} {'SHLDR':<6} {'ELBOW':<6} {'HAND':<6} | {'X':<6} {'Y':<6} {'Z':<6}")
    print("-" * 75)

    try:
        while True:
            try:
                response = requests.get(url, timeout=2)
                response.raise_for_status()
                
                # Parse the JSON response
                # Expected format example: {"T":105, "b":0, "s":0, "e":0, "h":0, "x":150, "y":0, "z":100, ...}
                data = json.loads(response.text)
                
                # Extract Angles
                b = data.get('b', 0) # Base
                s = data.get('s', 0) # Shoulder
                e = data.get('e', 0) # Elbow
                h = data.get('h', 0) # Hand
                
                # Extract Position
                x = data.get('x', 0)
                y = data.get('y', 0)
                z = data.get('z', 0)

                # Get current time for logging
                timestamp = time.strftime("%H:%M:%S", time.localtime())

                # Print formatted row
                print(f"{timestamp:<10} | {b:<6} {s:<6} {e:<6} {h:<6} | {x:<6} {y:<6} {z:<6}")

            except requests.exceptions.RequestException as err:
                print(f"Connection Error: {err}")
            except json.JSONDecodeError:
                print("Data Error: Could not decode JSON response.")
            
            # Poll every 0.2 seconds (adjust as needed)
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()