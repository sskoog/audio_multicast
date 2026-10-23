import serial
import time

s_src = serial.Serial('COM16', 2000000, timeout=0.1, rtscts=False, dsrdtr=False)
s_src.dtr = False
s_src.rts = False
time.sleep(0.5)

s_src.read_all()

commands = [b"sr 48k\r\n", b"sr 32k\r\n", b"sr 24k\r\n", b"sr 16k\r\n", b"sr 8k\r\n"]
for cmd in commands:
    print(f"Sending: {cmd}")
    s_src.write(cmd)
    time.sleep(1.0)
    out = s_src.read_all().decode('utf-8', errors='ignore')
    print(out)
    
s_src.close()
