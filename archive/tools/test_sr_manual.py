import serial
import time

s_src = serial.Serial('COM16', 2000000, timeout=0.1, rtscts=False, dsrdtr=False)
s_snk1 = serial.Serial('COM23', 921600, timeout=0.1, rtscts=False, dsrdtr=False)
s_src.dtr = False
s_src.rts = False
s_snk1.dtr = False
s_snk1.rts = False
time.sleep(0.5)

s_src.write(b"synth\r\n")
s_src.write(b"start\r\n")
time.sleep(2)
s_src.read_all()
s_snk1.read_all()

s_src.write(b"sr 24k\r\n")
t_end = time.time() + 5.0
while time.time() < t_end:
    while s_snk1.in_waiting:
        line = s_snk1.readline().decode('utf-8', errors='ignore').strip()
        if 'LC3' in line:
            print("SINK1:", line)
    time.sleep(0.1)

s_src.close()
s_snk1.close()
