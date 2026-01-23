"""
This script allows the user to control the drone manually using zqsd and ijkl inputs to control translation and yaw/elevation.

To operate, a controller with the UART0 peripheral set to 115200 baud rate must be connected to the computer via USB/UART.

Upon pressing buttons on the keyboard, a byte with the command is sent to the controller. The controller then sends the received byte through its UART1 peripheral which should be connected to the RX/TX pins of the DFRobot Gravity UART Fiber Optic Transceiver Module.

The module sends the byte over optic fiber which is received by the second module before being sent via UART to the drone's controller.

The byte is then converted into usable data for the drone's flight.
"""

import serial
import keyboard
import time

# Scan for COM ports
SERIAL_PORT_INDEX = 0
BAUD_RATE = 115200
ser = None

print("Scanning for ESP32...")
while SERIAL_PORT_INDEX < 30:
    port = f"COM{SERIAL_PORT_INDEX}"
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.02) # Low timeout for responsiveness
        print(f"Successfully connected to {port}")
        break
    except (serial.SerialException, PermissionError):
        SERIAL_PORT_INDEX += 1

if not ser:
    print("Could not find an open COM port. Check your USB connection.")
    exit()

print("---")
print("Controls: ZQSD (Move) | IJKL (Height/Yaw) | Ctrl+C to Exit")
print("Telemetry Monitor Active...")
print("---")

last_command = -1

try:
    while True:
        # 1. Create a bitmask (8 bits for 8 keys)
        command_byte = 0
        if keyboard.is_pressed('z'): command_byte |= (1 << 0)
        elif keyboard.is_pressed('s'): command_byte |= (1 << 2)
        if keyboard.is_pressed('q'): command_byte |= (1 << 1)
        elif keyboard.is_pressed('d'): command_byte |= (1 << 3)
        if keyboard.is_pressed('i'): command_byte |= (1 << 4)
        elif keyboard.is_pressed('k'): command_byte |= (1 << 5)
        if keyboard.is_pressed('j'): command_byte |= (1 << 6)
        elif keyboard.is_pressed('l'): command_byte |= (1 << 7)

        # 2. Send Start Byte (0xFE) + Command Byte + End Byte (0xFF)
        packet = bytearray([0xFE, command_byte, 0xFF])
        ser.write(packet)
        
        # Only print command when it changes to keep the console clean
        if command_byte != last_command:
            print(f"[SENT] Key Mask: {bin(command_byte)}")
            last_command = command_byte
        
        # 3. Read and print incoming data (Non-blocking Monitor)
        if ser.in_waiting > 0:
            try:
                # Read whatever is in the buffer
                incoming_data = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                # Use end="" because the incoming data usually already has newlines
                print(incoming_data, end="")
            except Exception as e:
                print(f"\n[READ ERROR]: {e}")

        # Send at ~50Hz
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nExiting Gracefully...")
    ser.close()