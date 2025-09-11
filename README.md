TO DO :

- Hardware Command Mode switching
    -> use prepare_command_mode_switch & perform_command_mode_switch methods

- Adapt CAN_Comms for native CAN (jetson) and socket CAN (raspberry pi via Waveshare USB-CAN converter)

- Implement homing upon start up (limit switches)

- Implement seperate hardware interface for claw
    -> optional : implement effort control via current or pressure sensor

- Write teleop launch file for gamepad joint control
    -> config files with preset parameters

- Write controller switching between teleop vel and teleop pos

- Integrate into Moveit2

- Integrate Camera sensor
    -> object detection
    -> hand tracking
    -> map object detection to task generation
    -> map hand tracking to end effect IK solving (moveit_servo)
