# import subprocess
# import sys

# # def run_script(script_name):
# #     try:
# #         subprocess.run(['bash', script_name], check=True)
# #     except subprocess.CalledProcessError as e:
# #         print(f"Error while running {script_name}: {e}")

# def run_script_in_terminal(script_name):
#     try:
#         with open('logs.txt', 'a') as log_file:
#             subprocess.Popen(['bash', script_name], stdout=log_file, stderr=log_file)
#     except Exception as e:
#         print(f"Failed to run {script_name}: {e}")


# def toggle_mode(current_mode):
#     if current_mode == 'manual':
#         print("Switching to autonomous mode.")
#         return 'autonomous'
#     elif current_mode == 'autonomous':
#         print("Switching to teleop mode.")
#         return 'teleop'
#     elif current_mode == 'teleop':
#         print("Switching to manual mode.")
#         return 'manual'
#     return current_mode

# project_launched = False  # Define at the global level or pass as argument

# def select_destination(destination):
#     global project_launched  # to modify the flag inside the function
#     if not project_launched:
#         print("Launching project stack.")
#         run_script_in_terminal('launch_project.sh')
#         project_launched = True
#     else:
#         print("Project already running. Skipping launch.")

#     if destination == 'assembly_shop':
#         print("Running Assembly Shop Waypoints.")
#         run_script_in_terminal('assembly_shop_waypoints.sh')
#     elif destination == 'u_turn':
#         print("Running U-turn Waypoints.")
#         run_script_in_terminal('u_turn_waypoints.sh')
#     elif destination == 'bumper_shop':
#         print("Running Bumper Shop Waypoints.")
#         run_script_in_terminal('bumper_shop_waypoints.sh')

# def transition_to_outdoor():
#     print("Switching to outdoor mode with GPS.")
#     run_script_in_terminal('gps.sh')

# def transition_to_indoor():
#     print("Switching to indoor mode with LiDAR.")
#     run_script_in_terminal('odometry.sh')

# def reset_motor_power():
#     print("Resetting motor power.")
#     run_script_in_terminal('motor_power_reset.sh')

# def reset_logic_power():
#     print("Resetting logic power.")
#     run_script_in_terminal('logic_power_reset.sh')

# def main():
#     current_mode = 'manual'
#     current_destination = 'assembly_shop'
#     inside = True  # Assume we're indoors at the start
    
#     while True:
#         # Replace with actual button input handling in your UI
#         button_input = input("Enter button press: ")

#         if button_input == 'toggle_mode':
#             current_mode = toggle_mode(current_mode)
#             print(f"Current mode: {current_mode}")
        
#         elif button_input == 'select_destination':
#             destination = input("Enter destination (assembly_shop, u_turn, bumper_shop): ")
#             select_destination(destination)
#             current_destination = destination
        
#         elif button_input == 'switch_to_outdoor':
#             transition_to_outdoor()
#             inside = False
        
#         elif button_input == 'switch_to_indoor':
#             transition_to_indoor()
#             inside = True
        
#         elif button_input == 'reset_motor':
#             reset_motor_power()
        
#         elif button_input == 'reset_logic':
#             reset_logic_power()
        
#         elif button_input == 'launch_project':
#             run_script_in_terminal('launch_project.sh')
        
#         elif button_input == 'exit':
#             print("Exiting the program.")
#             break
#         else:
#             print("Invalid input. Try again!")

# if __name__ == "__main__":
#     main()

import serial
import subprocess
import time

SERIAL_PORT = '/dev/ttyACM0'  # Change if needed
BAUD_RATE = 9600

# UI State
screen = 0  # 0 = destination, 1 = trailers
current_option = ''
mode = 'A'
project_launched = False

# List of destination scripts
destination_scripts = {
    'Assemble Area': 'assembly_shop_waypoints.sh',
    'U-Turn': 'u_turn_waypoints.sh',
    'Charging Station': 'charging_station_waypoints.sh',
    'Bumper Shop': 'bumper_shop_waypoints.sh',
}

def run_script_in_terminal(script_name):
    try:
        with open('logs.txt', 'a') as log_file:
            subprocess.Popen(['bash', script_name], stdout=log_file, stderr=log_file)
    except Exception as e:
        print(f"Failed to run {script_name}: {e}")

def launch_project_stack():
    global project_launched
    if not project_launched:
        print("Launching main project stack...")
        run_script_in_terminal("launch_project.sh")
        project_launched = True
    else:
        print("Project stack already running.")

def main():
    global screen, current_option, mode

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Reading data from Arduino... Press Ctrl+C to stop.")
        time.sleep(2)
    except Exception as e:
        print(f"Could not connect to serial: {e}")
        return

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    continue
                print(f"Received: {line}")

                if line.startswith("Screen:"):
                # Example line: "Screen: 0, Option: Assemble Area"
                    try:
                        parts = line.split(",")
                        screen_part = parts[0].split(":")[1].strip()
                        option_part = parts[1].split(":")[1].strip()

                        screen = int(screen_part)
                        current_option = option_part

                        print(f">>> Switched to Screen: {screen} ({'Destination' if screen == 0 else 'Trailers'})")
                        print(f">>> Current Option Selected: {current_option}")
                    except Exception as e:
                        print(f"Error parsing screen/option: {e}")


                elif line.startswith("Option:"):
                    current_option = line.split(":", 1)[1].strip()
                    print(f">>> Current Option Selected: {current_option}")

                elif line == "3":
                    print(">>> Confirmed Selection")
                    if screen == 0:
                        # Launch destination
                        script = destination_scripts.get(current_option)
                        if script:
                            launch_project_stack()
                            run_script_in_terminal(script)
                        else:
                            print(f"No script mapped for: {current_option}")
                    elif screen == 1:
                        print(f"Number of trailers confirmed: {current_option}")
                        # You can trigger trailer-specific logic here

                elif line in ["A", "T", "M"]:
                    mode = line
                    
                    print(f">>> Mode changed to: {mode}")

                elif line == "C":
                    print(">>> Confirmation timeout. Auto-cleared.")

    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == "__main__":
    main()