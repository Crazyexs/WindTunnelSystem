import serial
import time
import os
import datetime
import threading
import sys

# --- CONFIGURATION ---
COM_PORT = 'COM7' 
BAUD_RATE = 115200
SAVE_FOLDER = 'WindTunnelData'

def read_from_port(ser):
    global current_file, is_recording
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Always print the line to the terminal so menus are visible!
                print(line)

                if "--- Starting New Run" in line:
                    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(SAVE_FOLDER, f"WindTunnelTest_{timestamp}.csv")
                    if current_file:
                        current_file.close()
                    current_file = open(filename, 'w')
                    is_recording = True
                    print(f"\n[>>>] AUTO-SAVE STARTED: Created {filename} on laptop.")
                
                elif "[ TEST COMPLETE" in line:
                    if current_file:
                        current_file.close()
                        current_file = None
                    is_recording = False
                    print(f"[|||] AUTO-SAVE STOPPED. File saved.\n")

                elif is_recording:
                    if current_file:
                        current_file.write(line + '\n')
                        current_file.flush()
    except Exception as e:
        print(f"Read thread error: {e}")

def main():
    global current_file, is_recording
    current_file = None
    is_recording = False

    if not os.path.exists(SAVE_FOLDER):
        os.makedirs(SAVE_FOLDER)

    print(f"Connecting to {COM_PORT} at {BAUD_RATE} baud...")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    print("Connected! Listening for Wind Tunnel data...")
    print("--------------------------------------------------")
    print("-> YOU CAN TYPE COMMANDS HERE AND PRESS ENTER <-")
    print("--------------------------------------------------")

    # Start a background thread to read from the serial port
    read_thread = threading.Thread(target=read_from_port, args=(ser,), daemon=True)
    read_thread.start()

    try:
        # Main thread handles user input and sends it to the Teensy
        while True:
            user_input = input() 
            if user_input:
                ser.write((user_input + '\n').encode('utf-8'))
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if current_file:
            current_file.close()
        ser.close()
        sys.exit(0)

if __name__ == '__main__':
    main()