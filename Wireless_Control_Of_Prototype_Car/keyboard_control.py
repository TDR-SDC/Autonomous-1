import keyboard
import serial
import time


arduino=serial.Serial('COM10',baudrate=9600,timeout=1)   #"COM10" -> port being used

    
while True:
    try:
        key_event = keyboard.read_event(suppress=True)
        if key_event.event_type == keyboard.KEY_DOWN:
            # Process the key pressed
            print(key_event.name)  # Print the key press
            char=key_event.name
            if(char=='w'):
                arduino.write(b'w')
            elif(char=='a'):
                arduino.write(b'a')
            elif(char=='s'):
                arduino.write(b's')
            elif(char=='d'):
                arduino.write(b'd')
            
    except KeyboardInterrupt:
        break