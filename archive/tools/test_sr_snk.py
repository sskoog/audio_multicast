import serial
import time
import sys

s_src = serial.Serial('COM16', 2000000, timeout=0.1, rtscts=False, dsrdtr=False)
s_snk = serial.Serial('COM23', 921600, timeout=0.1, rtscts=False, dsrdtr=False)
s_src.dtr = False
s_src.rts = False
s_snk.dtr = False
s_snk.rts = False
time.sleep(0.5)

s_src.write(b"synth\r\n")
s_src.write(b"start\r\n")
time.sleep(2)
s_src.read_all()
s_snk.read_all()

s_src.write(b"sr 24k\r\n")
time.sleep(1)
out = s_snk.read_all().decode('utf-8', errors='ignore')
print("SINK OUTPUT:")
print(out)
s_src.close()
s_snk.close()
