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

# def run_script_in_terminal(script_name):
#     try:
#         with open('logs.txt', 'a') as log_file:
#             subprocess.Popen(['bash', script_name], stdout=log_file, stderr=log_file)
#     except Exception as e:
#         print(f"Failed to run {script_name}: {e}")

def run_script_in_terminal(command):
    try:
        with open('logs.txt', 'a') as log_file:
            subprocess.Popen(command, stdout=log_file, stderr=log_file, shell=True)
    except Exception as e:
        print(f"Failed to run command: {command}\nError: {e}")


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
                    try:
                        run_script_in_terminal(f'./mode_switch.sh {mode}')
                    except Exception as e:
                        print(f"Failed to run mode_switch.sh: {e}")



                elif line == "C":
                    print(">>> Confirmation timeout. Auto-cleared.")

    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == "__main__":
    main()