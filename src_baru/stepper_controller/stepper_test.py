import lgpio
import time
import sys

# Hardware Pinout
STEP_PIN = 17
DIR_PIN  = 27
EN_PIN   = 22
GPIOCHIP = 4  # Raspberry Pi 5

def main():
    print("Initializing GPIO...")
    try:
        h = lgpio.gpiochip_open(GPIOCHIP)
        lgpio.gpio_claim_output(h, STEP_PIN)
        lgpio.gpio_claim_output(h, DIR_PIN)
        lgpio.gpio_claim_output(h, EN_PIN)
    except Exception as e:
        print(f"Failed to initialize GPIO: {e}")
        sys.exit(1)

    # Enable motor (Active Low)
    lgpio.gpio_write(h, EN_PIN, 0)
    print("Motor Enabled. Holding torque applied.")

    try:
        while True:
            print("\n" + "="*40)
            print("Stepper Calibration Menu:")
            print("1. Enter positive steps (e.g., 200) to spin Clockwise")
            print("2. Enter negative steps (e.g., -200) to spin Counter-Clockwise")
            print("3. Enter 'q' to quit and disable motor")
            
            user_input = input("\nEnter steps: ")
            
            if user_input.lower() == 'q':
                break
                
            try:
                target_steps = int(user_input)
            except ValueError:
                print("Invalid input. Please enter a whole number.")
                continue
                
            # Set Direction
            direction = 1 if target_steps > 0 else 0
            lgpio.gpio_write(h, DIR_PIN, direction)
            
            # Use a slightly slower delay for testing to ensure no skipped steps
            delay = 0.005 
            steps_to_move = abs(target_steps)
            
            print(f"Executing {steps_to_move} steps...")
            
            for _ in range(steps_to_move):
                lgpio.gpio_write(h, STEP_PIN, 1)
                time.sleep(delay / 2.0)
                lgpio.gpio_write(h, STEP_PIN, 0)
                time.sleep(delay / 2.0)
                
            print("Movement complete.")

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        # Safely disable motor and release GPIO pins
        lgpio.gpio_write(h, EN_PIN, 1)
        lgpio.gpiochip_close(h)
        print("Motor disabled. GPIO released.")

if __name__ == '__main__':
    main()
