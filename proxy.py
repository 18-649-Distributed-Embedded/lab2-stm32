#!/usr/bin/env python3
import serial
import struct
import threading
import time
import argparse
import sys

# Global state for the vehicle commands
state_lock = threading.Lock()
vehicle_steer = 0        # -128 to 127
vehicle_throttle = 127   # 0 to 255 (127 is center/idle)
vehicle_brake = 0        # 0 or 1
vehicle_blinkers = 0     # 0=Off, 1=Left, 2=Right, 3=Hazard
test_button_active = 0   # 0 or 0xFF

keep_running = True
seq_num = 0

def build_can_frame(can_id, payload):
    packet = bytearray([0xAA])
    packet.extend(struct.pack('<H', can_id))  # ID L, ID H
    packet.append(len(payload))               # DLC
    packet.extend(payload)                    # Payload bytes
    chksum = sum(packet[1:]) & 0xFF           # Checksum
    packet.append(chksum)
    packet.append(0x55)                       # END
    return packet

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
    global keep_running, seq_num
    
    while keep_running:
        with state_lock:
            # Generate CAN-over-UART frames
            t = time.time()
            
            # 1Hz Turn Sync (50% duty)
            turn_sync_led = 1 if (t % 1.0) < 0.5 else 0
            # 2Hz Hazard Sync (50% duty)
            hazard_sync_led = 1 if (t % 0.5) < 0.25 else 0
            
            # Msg_Heartbeat_Cockpit (ID: 0x020)
            hb_payload = struct.pack('<BB', seq_num & 0xFF, test_button_active)
            hb_frame = build_can_frame(0x020, hb_payload)
            
            # Cmd_Brake (ID: 0x010)
            brake_payload = struct.pack('<BB', seq_num & 0xFF, vehicle_brake)
            brake_frame = build_can_frame(0x010, brake_payload)
            
            # Cmd_Motion (ID: 0x100) -> [Seq, Throttle, Steer, TurnReq]
            motion_payload = struct.pack('<BBbB', seq_num & 0xFF, vehicle_throttle, vehicle_steer, vehicle_blinkers)
            motion_frame = build_can_frame(0x100, motion_payload)
            
            # Cmd_Turn_Sync (ID: 0x200)
            turn_payload = struct.pack('<BB', seq_num & 0xFF, turn_sync_led)
            turn_frame = build_can_frame(0x200, turn_payload)
            
            # Cmd_Hazard_Sync (ID: 0x011)
            hazard_payload = struct.pack('<BB', seq_num & 0xFF, hazard_sync_led)
            hazard_frame = build_can_frame(0x011, hazard_payload)
            
            seq_num += 1
        
        try:
            # Send all multiplexed CAN frames
            ser.write(hb_frame)
            ser.write(brake_frame)
            ser.write(motion_frame)
            ser.write(turn_frame)
            ser.write(hazard_frame)
        except:
            break
            
        # Send everything every 50ms (20Hz)
        time.sleep(0.05)

def main():
    global keep_running, vehicle_steer, vehicle_throttle, vehicle_brake, vehicle_blinkers, test_button_active
    
    parser = argparse.ArgumentParser(description="STM32 CAN-over-UART Proxy")
    parser.add_argument('-p', '--port', default='/dev/ttyACM1', help='Serial port')
    parser.add_argument('-b', '--baud', default=115200, type=int, help='Baud rate')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port {args.port}: {e}")
        sys.exit(1)

    print(f"Connected to {args.port} at {args.baud} baud.")
    print("Controls (Mapped to CAN Dictionary Limits):")
    print("  w/s : Throttle Forward/Reverse (0 to 255)")
    print("  a/d : Steer Left/Right (-128 to 127)")
    print("  space/b : Brake (Aggressive)")
    print("  x : Stop (Center throttle and brake)")
    print("  c : Center steering")
    print("  1/2/3 : Left / Right / Hazards (0 to clear)")
    print("  t : Toggle Local Test Button (E-Stop)")
    print("  q : Quit")
    print("-" * 50)

    rt = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    st = threading.Thread(target=sender_thread, args=(ser,), daemon=True)
    rt.start()
    st.start()

    try:
        while True:
            cmd = input("\r\033[KCmd> ").lower()
            
            with state_lock:
                if cmd == 'q':
                    keep_running = False
                    break
                elif cmd == 'w':
                    vehicle_throttle = min(vehicle_throttle + 32, 255)
                    vehicle_brake = 0
                elif cmd == 's':
                    vehicle_throttle = max(vehicle_throttle - 32, 0)
                    vehicle_brake = 0
                elif cmd == 'a':
                    vehicle_steer = max(vehicle_steer - 32, -128)
                elif cmd == 'd':
                    vehicle_steer = min(vehicle_steer + 32, 127)
                elif cmd == ' ' or cmd == 'b':
                    vehicle_brake = 1
                    vehicle_throttle = 127
                elif cmd == 'x':
                    vehicle_throttle = 127
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
                elif cmd == 't':
                    test_button_active = 0xFF if test_button_active == 0 else 0
                    if test_button_active == 0:
                        # Releasing test mode: reset everything to idle!
                        vehicle_throttle = 127
                        vehicle_steer = 0
                        vehicle_brake = 0
                
    except KeyboardInterrupt:
        keep_running = False

    ser.close()
    print("\nDisconnected.")

if __name__ == '__main__':
    main()
