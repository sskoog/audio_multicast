import sys
import time
import serial

def probe_hci(port="COM121", baud=2000000):
    print(f"=== Probing ESP32-C6 HCI Controller on {port} (Baud: {baud}) ===")
    
    # Initialize serial port without asserting RTS/DTR resets
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 1.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    
    # Let ROM boot header pass and flush buffer
    time.sleep(0.6)
    ser.reset_input_buffer()
    
    commands = [
        ("1. HCI_Reset (0x0C03)", bytes([0x01, 0x03, 0x0C, 0x00])),
        ("2. HCI_Read_Local_Version_Information (0x1001)", bytes([0x01, 0x01, 0x10, 0x00])),
        ("3. HCI_LE_Read_Local_Supported_Features (0x2003)", bytes([0x01, 0x03, 0x20, 0x00])),
        ("4. HCI_Read_BD_ADDR (0x1009)", bytes([0x01, 0x09, 0x10, 0x00]))
    ]
    
    for name, cmd in commands:
        print(f"\n{name}...")
        ser.write(cmd)
        ser.flush()
        
        # Read response event
        time.sleep(0.1)
        resp = ser.read(ser.in_waiting or 32)
        if resp:
            hex_str = ' '.join(f'{b:02X}' for b in resp)
            print(f"   Response ({len(resp)} bytes): {hex_str}")
            if resp[0] == 0x04:
                evt_code = resp[1]
                param_len = resp[2]
                print(f"   -> Valid HCI Event 0x{evt_code:02X}, param_len={param_len}")
        else:
            print("   -> No response received within timeout.")

    ser.close()
    print("\n=== ESP32-C6 HCI Controller Diagnostic Complete ===")

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 'COM121'
    probe_hci(port)
