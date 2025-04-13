import subprocess
import sys

def run_script(script_name):
    try:
        subprocess.run(['bash', script_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error while running {script_name}: {e}")

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

def select_destination(destination):
    if destination == 'assembly_shop':
        print("Running Assembly Shop Waypoints.")
        run_script('launch_project.sh')
        # run_script('assembly_shop_waypoints.sh')
    elif destination == 'charging_station':
        print("Running Charging Station Waypoints.")
        run_script('launch_project.sh')
        run_script('charging_station_waypoints.sh')
    elif destination == 'bumper_shop':
        print("Running Bumper Shop Waypoints.")
        run_script('launch_project.sh')
        run_script('bumper_shop_waypoints.sh')

def transition_to_outdoor():
    print("Switching to outdoor mode with GPS.")
    run_script('gps.sh')

def transition_to_indoor():
    print("Switching to indoor mode with LiDAR.")
    run_script('odometry.sh')

def reset_motor_power():
    print("Resetting motor power.")
    run_script('motor_power_reset.sh')

def reset_logic_power():
    print("Resetting logic power.")
    run_script('logic_power_reset.sh')

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
            destination = input("Enter destination (assembly_shop, charging_station, bumper_shop): ")
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
            run_script('launch_project.sh')
        
        elif button_input == 'exit':
            print("Exiting the program.")
            break
        else:
            print("Invalid input. Try again!")

if __name__ == "__main__":
    main()

