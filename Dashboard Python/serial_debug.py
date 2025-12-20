#!/usr/bin/env python3
"""
Serial Debug - See exactly what's coming from Arduino
"""

import sys
import serial
import serial.tools.list_ports
import time
import struct

def list_ports():
    print("Available ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}: {p.description}")

def hex_dump(data, prefix=""):
    hex_str = ' '.join(f'{b:02X}' for b in data)
    ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
    print(f"{prefix}{hex_str}  |{ascii_str}|")

def crc16_modbus(data):
    """CRC-16 MODBUS - same as STM32/Arduino"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def main():
    if len(sys.argv) < 2:
        list_ports()
        print("\nUsage: python serial_debug.py <port>")
        return
    
    port = sys.argv[1]
    
    print(f"Opening {port} at 115200 baud...")
    ser = serial.Serial(port, 115200, timeout=0.1)
    print("Connected! Waiting for data...\n")
    
    buffer = bytearray()
    frame_count = 0
    
    try:
        while True:
            data = ser.read(256)
            if data:
                buffer.extend(data)
                
                # Try to find and parse frames
                while len(buffer) >= 6:
                    # Find sync bytes
                    try:
                        idx = 0
                        while idx < len(buffer) - 1:
                            if buffer[idx] == 0xAA and buffer[idx+1] == 0x55:
                                break
                            idx += 1
                        
                        if idx > 0:
                            print(f"[SKIP] {idx} bytes before sync")
                            buffer = buffer[idx:]
                        
                        if len(buffer) < 6:
                            break
                        
                        # Parse header
                        length = (buffer[2] << 8) | buffer[3]
                        total_len = 4 + length + 2
                        
                        if length > 256:
                            print(f"[ERR] Bad length: {length}")
                            buffer = buffer[2:]
                            continue
                        
                        if len(buffer) < total_len:
                            break  # Wait for more data
                        
                        # Extract frame
                        frame = bytes(buffer[:total_len])
                        buffer = buffer[total_len:]
                        
                        frame_count += 1
                        msg_type = frame[4]
                        payload = frame[5:-2]
                        frame_crc = (frame[-2] << 8) | frame[-1]
                        
                        # Calculate expected CRC
                        crc_data = bytes([msg_type]) + payload
                        calc_crc = crc16_modbus(crc_data)
                        
                        type_names = {
                            0x01: "FC_FAST",
                            0x02: "FC_SLOW", 
                            0x03: "FC_EVENT",
                            0x10: "GS_STATUS",
                            0x11: "GS_ACK",
                            0x12: "GS_PONG",
                            0x13: "GS_STATS",
                            0x20: "GS_LOG",
                        }
                        type_name = type_names.get(msg_type, f"0x{msg_type:02X}")
                        
                        crc_ok = "OK" if frame_crc == calc_crc else f"FAIL (got {frame_crc:04X}, calc {calc_crc:04X})"
                        
                        print(f"\n[FRAME #{frame_count}] Type={type_name} Len={length} PayloadLen={len(payload)} CRC={crc_ok}")
                        
                        if msg_type == 0x20:  # Log message
                            print(f"  LOG: {payload.decode('utf-8', errors='replace')}")
                        else:
                            hex_dump(payload, "  Payload: ")
                        
                        # For FC packets, check the inner CRC too
                        if msg_type in [0x01, 0x02, 0x03] and len(payload) >= 2:
                            # Inner packet has CRC at end (little-endian)
                            inner_crc = payload[-2] | (payload[-1] << 8)
                            inner_calc = crc16_modbus(payload[:-2])
                            inner_ok = "OK" if inner_crc == inner_calc else f"FAIL (got {inner_crc:04X}, calc {inner_calc:04X})"
                            print(f"  Inner CRC (FC packet): {inner_ok}")
                            
                            if msg_type == 0x01:  # Fast telemetry
                                if len(payload) >= 33:
                                    state = payload[6]
                                    substate = payload[7]
                                    print(f"  State={state} Substate={substate}")
                        
                    except Exception as e:
                        print(f"[ERR] Parse error: {e}")
                        buffer = buffer[1:]
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print(f"\n\nReceived {frame_count} frames")
        ser.close()

if __name__ == '__main__':
    main()
