import pygame

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected!")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Joystick name: {joystick.get_name()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print(f"Number of hats: {joystick.get_numhats()}")
    print(f"Number of axes: {joystick.get_numaxes()}")

    try:
        while True:
            pygame.event.pump()

            # Check buttons
            for button_id in range(joystick.get_numbuttons()):
                if joystick.get_button(button_id):
                    print(f"Button {button_id} pressed.")

            # Check D-Pad (HAT)
            hat_x, hat_y = joystick.get_hat(0)  # Usually only one hat
            if hat_x == -1:
                print("D-Pad LEFT pressed.")
            elif hat_x == 1:
                print("D-Pad RIGHT pressed.")
            if hat_y == 1:
                print("D-Pad UP pressed.")
            elif hat_y == -1:
                print("D-Pad DOWN pressed.")

            # Check Triggers (LT and RT are AXES usually 2 and 5)
            lt_axis = joystick.get_axis(2)  # Left Trigger
            rt_axis = joystick.get_axis(5)  # Right Trigger

            if lt_axis > 0.5:
                print("Left Trigger (LT) pressed.")
            if rt_axis > 0.5:
                print("Right Trigger (RT) pressed.")

            pygame.time.wait(100)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
