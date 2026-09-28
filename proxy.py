#!/usr/bin/env python3
import serial
import struct
import threading
import time
import argparse
import sys

# Global state for the vehicle commands
state_lock = threading.Lock()
vehicle_steer = 0
vehicle_throttle = 0
vehicle_brake = 0
vehicle_blinkers = 0
keep_running = True

def reader_thread(ser):
    global keep_running
    buffer = bytearray()
    while keep_running:
        try:
            b = ser.read(1)
            if not b:
                continue
            
            buffer.extend(b)
            
            # Check for the binary heartbeat frame [0xBB ... 0x66]
            if len(buffer) >= 9 and buffer[-9] == 0xBB and buffer[-1] == 0x66:
                frame = buffer[-9:]
                state = frame[1]
                l_curr = struct.unpack('>h', frame[2:4])[0]
                r_curr = struct.unpack('>h', frame[4:6])[0]
                s_curr = struct.unpack('>h', frame[6:8])[0]
                
                state_str = "NORMAL" if state == 0 else "FAIL_SAFE"
                # Overwrite the current line with the heartbeat status
                sys.stdout.write(f"\r\033[K[HEARTBEAT] {state_str} | L_curr: {l_curr}mV | R_curr: {r_curr}mV | S_curr: {s_curr}mV | Cmd> ")
                sys.stdout.flush()
                buffer = buffer[:-9] # Remove parsed frame
                
            # If we see a newline, it's likely a debug printk from Zephyr
            elif b == b'\n':
                try:
                    text = buffer.decode('ascii', errors='ignore').strip()
                    if text and not all(c in [0xBB, 0x66] for c in buffer):
                        sys.stdout.write(f"\r\033[K[STM32 LOG] {text}\nCmd> ")
                        sys.stdout.flush()
                except:
                    pass
                buffer.clear()
            
            # Clear buffer if it gets wildly large (shouldn't happen)
            if len(buffer) > 100:
                buffer.clear()
                
        except Exception as e:
            if keep_running:
                print(f"\nSerial read error: {e}")
            break

def sender_thread(ser):
    global keep_running, vehicle_steer, vehicle_throttle, vehicle_brake, vehicle_blinkers
    
    while keep_running:
        with state_lock:
            # Assembly packet: [0xAA, Steer(L,H), Throttle(L,H), Brake(L,H), Blinkers, Checksum, 0x55]
            packet = bytearray([0xAA])
            
            # Encode signed 16-bit values as little-endian
            steer_bytes = struct.pack('<h', vehicle_steer)
            throttle_bytes = struct.pack('<h', vehicle_throttle)
            brake_bytes = struct.pack('<h', vehicle_brake)
            
            packet.extend(steer_bytes)
            packet.extend(throttle_bytes)
            packet.extend(brake_bytes)
            packet.append(vehicle_blinkers)
            
            # Calculate checksum (sum of payload bytes)
            chksum = sum(packet[1:]) & 0xFF
            packet.append(chksum)
            packet.append(0x55)
        
        try:
            ser.write(packet)
        except:
            break
            
        # STM32 watchdog is 150ms. Send every 50ms.
        time.sleep(0.05)

def main():
    global keep_running, vehicle_steer, vehicle_throttle, vehicle_brake, vehicle_blinkers
    
    parser = argparse.ArgumentParser(description="STM32 UART Proxy")
    parser.add_argument('-p', '--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('-b', '--baud', default=115200, type=int, help='Baud rate')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port {args.port}: {e}")
        sys.exit(1)

    print(f"Connected to {args.port} at {args.baud} baud.")
    print("Controls:")
    print("  w/s : Throttle Forward/Reverse")
    print("  a/d : Steer Left/Right")
    print("  space : Brake")
    print("  x : Stop (release throttle and brake)")
    print("  c : Center steering")
    print("  1/2/3 : Left Blinker / Right Blinker / Hazards (0 to clear)")
    print("  q : Quit")
    print("-" * 50)

    rt = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    st = threading.Thread(target=sender_thread, args=(ser,), daemon=True)
    rt.start()
    st.start()

    try:
        while True:
            # We use a simple blocking input.
            cmd = input("\r\033[KCmd> ").strip().lower()
            
            with state_lock:
                if cmd == 'q':
                    keep_running = False
                    break
                elif cmd == 'w':
                    vehicle_throttle = min(vehicle_throttle + 8000, 32767)
                    vehicle_brake = 0
                elif cmd == 's':
                    vehicle_throttle = max(vehicle_throttle - 8000, -32768)
                    vehicle_brake = 0
                elif cmd == 'a':
                    vehicle_steer = max(vehicle_steer - 10000, -32768)
                elif cmd == 'd':
                    vehicle_steer = min(vehicle_steer + 10000, 32767)
                elif cmd == ' ':
                    vehicle_brake = 32767
                    vehicle_throttle = 0
                elif cmd == 'x':
                    vehicle_throttle = 0
                    vehicle_brake = 0
                elif cmd == 'c':
                    vehicle_steer = 0
                elif cmd == '1':
                    vehicle_blinkers = 0x01
                elif cmd == '2':
                    vehicle_blinkers = 0x02
                elif cmd == '3':
                    vehicle_blinkers = 0x03
                elif cmd == '0':
                    vehicle_blinkers = 0x00
                
    except KeyboardInterrupt:
        keep_running = False

    ser.close()
    print("\nDisconnected.")

if __name__ == '__main__':
    main()
