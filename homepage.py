import gradio as gr
import time
import subprocess
import sys
import os
import requests
import json
import math
from typing import Optional, Dict, Any

# ==========================================
# ROARM CONTROLLER CLASS
# ==========================================
class RoArmController:
    """
    An efficient controller for the RoArm-M2 that synchronizes Python execution 
    with physical arm movement.
    """

    def __init__(self, ip_address: str, port: int = 80, protocol: str = "http", timeout: int = 10):
        self.base_url = f"{protocol}://{ip_address}:{port}/js?json="
        self.timeout = timeout
        self.last_response = None
        # Tolerance for deciding if the arm has "stopped" (radians/mm change per check)
        self.motion_tolerance = 0.02 
        print(f"[RoArm] Initialized. Endpoint: {self.base_url}")

    def _send_command(self, command_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Sends command and parses the immediate JSON acknowledgement.
        """
        try:
            json_payload = json.dumps(command_dict)
            url = f"{self.base_url}{json_payload}"
            
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            
            try:
                data = response.json()
            except json.JSONDecodeError:
                # Fallback for raw text responses
                data = {"status": "ok", "raw": response.text}
            
            return data
        except Exception as e:
            print(f"[RoArm] Comm Error: {e}")
            return None

    def get_feedback(self) -> Optional[Dict[str, float]]:
        """
        Queries the arm's current status (T:105).
        Returns a dictionary of current joint angles/coordinates.
        """
        cmd = {"T": 105}
        resp = self._send_command(cmd)
        # RoArm usually returns keys like 'b', 's', 'e', 'h', 'x', 'y', 'z' in the response
        return resp

    def wait_for_motion_completion(self, check_interval: float = 0.2, stability_required: int = 3):
        """
        BLOCKS execution until the arm physically stops moving.
        
        Strategy: Poll status repeatedly. If the position hasn't changed 
        significantly for 'stability_required' checks in a row, we assume it stopped.
        """
        print("[RoArm] Waiting for motion to complete...", end="", flush=True)
        
        stable_count = 0
        last_values = {}
        
        start_time = time.time()
        
        while True:
            current_status = self.get_feedback()
            
            if not current_status:
                break # Comm failure, don't block indefinitely

            # Extract relevant movement metrics (joints b, s, e, h)
            # We filter for keys that are likely numeric position data
            current_values = {k: v for k, v in current_status.items() if k in ['b', 's', 'e', 'h', 'x', 'y', 'z'] and isinstance(v, (int, float))}
            
            if not last_values:
                last_values = current_values
                time.sleep(check_interval)
                continue

            # Calculate maximum change across all joints/axes
            max_delta = 0.0
            for key, val in current_values.items():
                prev_val = last_values.get(key, val)
                delta = abs(val - prev_val)
                if delta > max_delta:
                    max_delta = delta
            
            # Check if change is within "stopped" tolerance
            if max_delta < self.motion_tolerance:
                stable_count += 1
            else:
                stable_count = 0 # Reset if we detect movement
                
            # If stable for enough consecutive checks, we are done
            if stable_count >= stability_required:
                print(" Done.")
                break
                
            # Safety timeout (e.g., 15 seconds max wait)
            if time.time() - start_time > 15:
                print(" Timeout (Movement took too long).")
                break

            last_values = current_values
            time.sleep(check_interval)

    def move_cartesian(self, x: float, y: float, z: float, t: float, speed: float = 0.25, wait: bool = True):
        """
        Move to X,Y,Z coords (Inverse Kinematics).
        If wait=True, code blocks until move is finished.
        """
        cmd = {"T": 104, "x": x, "y": y, "z": z, "t": t, "spd": speed}
        print(f"\n[RoArm] Moving Cartesian: {x}, {y}, {z}")
        self._send_command(cmd)
        if wait:
            self.wait_for_motion_completion()

    def set_joint(self, joint_id: int, angle: float, speed: float = 0.25, wait: bool = True):
        """
        Move single joint. 1=Base, 2=Shoulder, 3=Elbow, 4=Hand.
        """
        cmd = {"T": 101, "joint": joint_id, "angle": angle, "spd": speed}
        print(f"\n[RoArm] Moving Joint {joint_id} to {angle}")
        self._send_command(cmd)
        if wait:
            self.wait_for_motion_completion()

    def set_torque(self, enable: bool):
        """Enables/Disables motors."""
        cmd = {"T": 210, "cmd": 1 if enable else 0}
        self._send_command(cmd)
        print(f"[RoArm] Torque set to {enable}")
        time.sleep(0.5) # Small buffer for hardware relay/activation


class RobotMock:
    """Robot object for teleop controls using RoArmController."""
    def __init__(self, ip_address: str = "192.168.4.1"):
        try:
            self.arm = RoArmController(ip_address=ip_address)
            self.arm.set_torque(True)
            self.use_real_arm = True
            print("[Robot] Real arm connected")
        except Exception as e:
            print(f"[Robot] Failed to connect to real arm: {e}. Using mock mode.")
            self.arm = None
            self.use_real_arm = False
    
    def teleop_move(self, direction: str) -> str:
        """Move robot based on direction."""
        if not self.use_real_arm or self.arm is None:
            return f"🤖 Moving {direction} (Mock)"
        
        try:
            # Define movement vectors for each direction
            movements = {
                'Forward': (50, 0, 0),
                'Backward': (-50, 0, 0),
                'Left': (0, 50, 0),
                'Right': (0, -50, 0),
                'Up': (0, 0, 50),
                'Down': (0, 0, -50),
            }
            
            if direction in movements:
                # Get current position
                feedback = self.arm.get_feedback()
                if feedback:
                    current_x = float(feedback.get('x', 0))
                    current_y = float(feedback.get('y', 0))
                    current_z = float(feedback.get('z', 0))
                    current_t = float(feedback.get('t', 3.14))
                    
                    # Apply movement
                    dx, dy, dz = movements[direction]
                    new_x = current_x + dx
                    new_y = current_y + dy
                    new_z = current_z + dz
                    
                    # Move arm
                    self.arm.move_cartesian(new_x, new_y, new_z, current_t, wait=False)
                    return f"🤖 Moving {direction}"
            
            return f"❌ Invalid direction: {direction}"
        except Exception as e:
            print(f"[Robot] Movement error: {e}")
            return f"❌ Movement failed: {e}"
    
    def drop_block(self) -> str:
        """Drop block by controlling gripper."""
        if not self.use_real_arm or self.arm is None:
            return "📦 Block dropped (Mock)"
        
        try:
            # Joint 4 is the gripper
            # Open gripper (high angle)
            self.arm.set_joint(joint_id=4, angle=3.14, wait=True)
            time.sleep(0.5)
            # Close gripper (low angle)
            self.arm.set_joint(joint_id=4, angle=1.57, wait=True)
            return "📦 Block dropped"
        except Exception as e:
            print(f"[Robot] Drop error: {e}")
            return f"❌ Drop failed: {e}"

def system_logic():
    """
    Main application logic container.
    """
    
    # Define default state: Not calibrated, Not disabled
    # We use a dictionary to allow mutable state passing
    default_state = {"calibrated": False, "disabled": False}
    
    # Initialize robot - will try real arm first, fall back to mock
    robot = RobotMock(ip_address="192.168.4.1")
    
    # Map keyboard keys to robot commands
    teleop_commands = {
        'w': lambda: robot.teleop_move('Forward'),
        's': lambda: robot.teleop_move('Backward'),
        'a': lambda: robot.teleop_move('Left'),
        'd': lambda: robot.teleop_move('Right'),
        'u': lambda: robot.teleop_move('Up'),
        'j': lambda: robot.teleop_move('Down'),
        'o': lambda: robot.drop_block(),
    }

    def process_chat(user_message, history, state):
        """
        Handles chat interaction and inference generation.
        """
        # Ensure history is initialized
        if history is None:
            history = []

        # Normalize older tuple-format histories to messages format
        if len(history) > 0 and isinstance(history[0], (list, tuple)):
            normalized = []
            for user_msg, bot_msg in history:
                normalized.append({"role": "user", "content": user_msg})
                normalized.append({"role": "assistant", "content": bot_msg})
            history = normalized

        if state["disabled"]:
            # If system is disabled, prevent chat and return warning in messages format
            history.append({"role": "user", "content": user_message})
            history.append({"role": "assistant", "content": "⛔ SYSTEM DISABLED. MESSAGE REJECTED."})
            return history, "System is offline.", ""

        if not user_message.strip():
            return history, "", ""

        # Simulate chatbot logic
        bot_response = f"I received: {user_message}"

        # Append messages in the dict format expected by newer Gradio versions
        history.append({"role": "user", "content": user_message})
        history.append({"role": "assistant", "content": bot_response})

        # Simulate "Inference" processing (Feature 3)
        inference_data = f"ANALYSIS: Input length {len(user_message)} chars.\nINTENT: General Query.\nSTATUS: Processed successfully."

        return history, inference_data, ""

    def run_calibration(state):
        """
        Runs the external `arm_calibrate.py` script and waits for it to finish.
        """
        if state["disabled"]:
            return "❌ System Disabled. Calibration Failed.", state
        try:
            script_path = os.path.join(os.path.dirname(__file__), "calibration", "arm_calibrate.py")
            # Run the calibration script using the same Python executable and wait for completion
            result = subprocess.run([sys.executable, script_path], check=True, capture_output=True, text=True)
            state["calibrated"] = True
            timestamp = time.strftime("%H:%M:%S")
            stdout = result.stdout.strip()
            if stdout:
                return f"✅ System Calibrated at {timestamp}\n\n{stdout}", state
            return f"✅ System Calibrated at {timestamp}", state
        except subprocess.CalledProcessError as e:
            err = e.stderr.strip() if e.stderr else str(e)
            return f"❌ Calibration script failed: {err}", state
        except Exception as e:
            return f"❌ Calibration failed: {e}", state

    def check_calibration_file_exists():
        """
        Checks if the calibration matrix file exists in the calibration directory.
        """
        calib_file = os.path.join(os.path.dirname(__file__), "calibration", "calibration_matrix.npy")
        return os.path.isfile(calib_file)

    def auto_calibrate_on_load(state):
        """
        Triggered automatically when the page loads (Feature 2).
        Checks if calibration file exists, and runs calibration if missing.
        """
        if not check_calibration_file_exists() and not state["disabled"]:
            msg, new_state = run_calibration(state)
            return msg, new_state
        if not state["disabled"]:
            state["calibrated"] = True
            return "System Ready (Calibration file detected)", state
        return "System Ready (Cached)", state

    def handle_signal(signal_code, state):
        """
        Handles the specific signal to disable functionality (Feature 4).
        Specific Signal is: 'STOP'
        """
        if signal_code.strip().upper() == "STOP":
            state["disabled"] = True
            
            # Return values to update UI components:
            # 1. Status Message
            # 2. State
            # 3. Chat Input (interactive=False)
            # 4. Calibrate Button (interactive=False)
            # 5. Signal Button (interactive=False)
            return (
                "⚠ SYSTEM SHUTDOWN SIGNAL RECEIVED. ALL OPERATIONS CEASED.", 
                state,
                gr.update(interactive=False, placeholder="System Disabled"),
                gr.update(interactive=False, value="Disabled"),
                gr.update(interactive=False)
            )
        
        return f"Signal '{signal_code}' ignored.", state, gr.update(), gr.update(), gr.update()

    def execute_teleop_command(key_or_direction):
        """
        Execute teleop command based on key or direction button press.
        """
        if key_or_direction in teleop_commands:
            result = teleop_commands[key_or_direction]()
            return f"✅ {result}"
        return "❌ Invalid command"

    # --- GUI Layout ---
    with gr.Blocks(title="Control Interface") as demo:
        # State variable to hold system status across interactions within a session
        system_state = gr.State(default_state)
        
        gr.Markdown("# 🤖 System Control Interface")
        
        with gr.Row():
            with gr.Column(scale=2):
                # Feature 1: Chatbot
                # Removed 'type="messages"' to fix TypeError. 
                # This component will now expect [[user_msg, bot_msg], ...] format.
                chatbot = gr.Chatbot(label="Conversation Log", height=400)
                msg_input = gr.Textbox(
                    label="User Input", 
                    placeholder="Type a message...",
                    interactive=True
                )
                clear = gr.ClearButton([msg_input, chatbot])

            with gr.Column(scale=1):
                # Feature 2 & 4: Controls and Status
                gr.Markdown("### System Status")
                
                # Feature 2: Calibration Button
                calibrate_btn = gr.Button("📡 Send Calibration Signal", variant="primary")
                
                # Feature 3: Inference Display
                inference_output = gr.TextArea(
                    label="Inference / Processing Output", 
                    interactive=False,
                    lines=5
                )

                gr.Markdown("---")
                gr.Markdown("### Admin Controls")
                
                # Feature 4: Disable Functionality
                signal_input = gr.Textbox(
                    label="Control Signal", 
                    placeholder="Enter signal code (e.g. STOP)"
                )
                signal_btn = gr.Button("Transmit Signal", variant="stop")

        # --- Robot Teleop Control Panel ---
        gr.Markdown("### 🎮 Robot Teleop Controls")
        gr.Markdown("**Keyboard:** W/A/S/D (Move), U (Up), J (Down), O (Drop)")
        
        with gr.Row():
            teleop_forward = gr.Button("⬆️ W", size="lg")
            teleop_output = gr.Textbox(label="Command Output", interactive=False, lines=2)
        
        with gr.Row():
            teleop_left = gr.Button("⬅️ A", size="lg")
            teleop_down = gr.Button("⬇️ S", size="lg")
            teleop_right = gr.Button("➡️ D", size="lg")
        
        with gr.Row():
            teleop_up = gr.Button("⬆️ U", size="lg")
            teleop_drop = gr.Button("📦 O", size="lg")

        # --- Event Wiring ---

        # 1. Chat Interaction
        msg_input.submit(
            process_chat, 
            inputs=[msg_input, chatbot, system_state], 
            outputs=[chatbot, inference_output, msg_input]
        )

        # 2. Manual Calibration
        calibrate_btn.click(
            run_calibration,
            inputs=[system_state],
            outputs=[inference_output, system_state]
        )

        # 3. Automatic Calibration on Page Load
        # This triggers immediately when the browser loads the interface
        demo.load(
            auto_calibrate_on_load,
            inputs=[system_state],
            outputs=[inference_output, system_state]
        )

        # 4. Signal Handling (Disable functionality)
        signal_btn.click(
            handle_signal,
            inputs=[signal_input, system_state],
            outputs=[
                inference_output, # Update status area
                system_state,     # Update internal state
                msg_input,        # Disable chat input
                calibrate_btn,    # Disable calibrate button
                signal_btn        # Disable signal button itself
            ]
        )

        # 5. Teleop Button Controls
        teleop_forward.click(execute_teleop_command, inputs=gr.State('w'), outputs=teleop_output)
        teleop_left.click(execute_teleop_command, inputs=gr.State('a'), outputs=teleop_output)
        teleop_down.click(execute_teleop_command, inputs=gr.State('s'), outputs=teleop_output)
        teleop_right.click(execute_teleop_command, inputs=gr.State('d'), outputs=teleop_output)
        teleop_up.click(execute_teleop_command, inputs=gr.State('u'), outputs=teleop_output)
        teleop_drop.click(execute_teleop_command, inputs=gr.State('o'), outputs=teleop_output)

        # 6. Keyboard input handler with JavaScript
        def register_keyboard_handler():
            return """
            <script>
            document.addEventListener('keydown', (e) => {
                const key = e.key.toLowerCase();
                const commands = ['w', 'a', 's', 'd', 'u', 'j', 'o'];
                if (commands.includes(key)) {
                    const buttons = document.querySelectorAll('[data-testid="button"]');
                    const buttonMap = {
                        'w': 0,
                        'a': 1,
                        's': 2,
                        'd': 3,
                        'u': 4,
                        'j': 5,
                        'o': 6
                    };
                    if (buttonMap[key] !== undefined && buttons[buttonMap[key]]) {
                        buttons[buttonMap[key]].click();
                    }
                }
            });
            </script>
            """
        
        demo.load(lambda: None, outputs=gr.HTML(register_keyboard_handler()))

    return demo

if __name__ == "__main__":
    app = system_logic()
    app.launch()