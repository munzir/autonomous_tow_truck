import subprocess
import sys

# def run_script(script_name):
#     try:
#         subprocess.run(['bash', script_name], check=True)
#     except subprocess.CalledProcessError as e:
#         print(f"Error while running {script_name}: {e}")

def run_script_in_terminal(script_name):
    try:
        with open('logs.txt', 'a') as log_file:
            subprocess.Popen(['bash', script_name], stdout=log_file, stderr=log_file)
    except Exception as e:
        print(f"Failed to run {script_name}: {e}")


def toggle_mode(current_mode):
    if current_mode == 'manual':
        print("Switching to autonomous mode.")
        return 'autonomous'
    elif current_mode == 'autonomous':
        print("Switching to teleop mode.")
        return 'teleop'
    elif current_mode == 'teleop':
        print("Switching to manual mode.")
        return 'manual'
    return current_mode

project_launched = False  # Define at the global level or pass as argument

def select_destination(destination):
    global project_launched  # to modify the flag inside the function
    if not project_launched:
        print("Launching project stack.")
        run_script_in_terminal('launch_project.sh')
        project_launched = True

        # First time, let it read from CSV
        if destination == 'assembly_shop':
            run_script_in_terminal('assembly_shop_waypoints.sh')
        elif destination == 'u_turn':
            run_script_in_terminal('u_turn_waypoints.sh')
        elif destination == 'bumper_shop':
            run_script_in_terminal('bumper_shop_waypoints.sh')
    else:
        # Use current pose on all future runs
        if destination == 'assembly_shop':
            run_script_in_terminal('assembly_shop_waypoints.sh --use_current_pose')
        elif destination == 'u_turn':
            run_script_in_terminal('u_turn_waypoints.sh --use_current_pose')
        elif destination == 'bumper_shop':
            print("Going to bumper shop with current_pose")
            run_script_in_terminal('bumper_shop_waypoints.sh --use_current_pose')


def transition_to_outdoor():
    print("Switching to outdoor mode with GPS.")
    run_script_in_terminal('gps.sh')

def transition_to_indoor():
    print("Switching to indoor mode with LiDAR.")
    run_script_in_terminal('odometry.sh')

def reset_motor_power():
    print("Resetting motor power.")
    run_script_in_terminal('motor_power_reset.sh')

def reset_logic_power():
    print("Resetting logic power.")
    run_script_in_terminal('logic_power_reset.sh')

def main():
    current_mode = 'manual'
    current_destination = 'assembly_shop'
    inside = True  # Assume we're indoors at the start
    
    while True:
        # Replace with actual button input handling in your UI
        button_input = input("Enter button press: ")

        if button_input == 'toggle_mode':
            current_mode = toggle_mode(current_mode)
            print(f"Current mode: {current_mode}")
        
        elif button_input == 'select_destination':
            destination = input("Enter destination (assembly_shop, u_turn, bumper_shop): ")
            select_destination(destination)
            current_destination = destination
        
        elif button_input == 'switch_to_outdoor':
            transition_to_outdoor()
            inside = False
        
        elif button_input == 'switch_to_indoor':
            transition_to_indoor()
            inside = True
        
        elif button_input == 'reset_motor':
            reset_motor_power()
        
        elif button_input == 'reset_logic':
            reset_logic_power()
        
        elif button_input == 'launch_project':
            run_script_in_terminal('launch_project.sh')
        
        elif button_input == 'exit':
            print("Exiting the program.")
            break
        else:
            print("Invalid input. Try again!")

if __name__ == "__main__":
    main()

