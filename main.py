import threespace_api as ts_api
import stimulator
import serial
import struct
import math
import numpy

IMU_controller_address = 2
IMU_subject_address = 7
IMU_port = 'COM6'
stimulator_port = 'COM4'
running = False
filter_size = 5
angle = []
counter = 0

def read_sensors():
    global counter
    serial_port = serial.Serial(port=IMU_port, baudrate=115200, timeout=0.001)
    while running:
        bytes_to_read = serial_port.inWaiting()
        if bytes_to_read > 0:
            data = bytearray(serial_port.read(bytes_to_read))

            print data

            continue

            # angle
            b = ''.join(chr(i) for i in data[8:12])  # angle y
            ang = struct.unpack('>f', b)
            x = ang[0]
            # print(x)

            # angle
            b = ''.join(chr(i) for i in data[12:16])  # angle y
            ang = struct.unpack('>f', b)
            ang = ang[0]
            if ang >= 0:
                ang = (ang / math.pi) * 180
                if abs(x) > (math.pi*0.75):
                    ang = ang + 2*(90-ang)
            else:
                ang = 360 + ((ang / math.pi) * 180)
                if abs(x) > (math.pi*0.75):
                    ang = ang - 2*(ang-270)

            # print(ang)
            angle.append(ang)
            if counter >= filter_size and ((ang > 60 and ang < 120) or (ang > 240 and ang < 300)):
                angle[-1] = numpy.mean(angle[-filter_size:])
            # print(angle[-1])

            counter += 1
    serial_port.close()

dng_device = ts_api.TSDongle(com_port=IMU_port)
IMU_controller = dng_device[IMU_controller_address]
IMU_subject = dng_device[IMU_subject_address]
IMU_controller.setEulerAngleDecompositionOrder(3)
IMU_controller.setEulerAngleDecompositionOrder(3)
IMU_controller.setCompassEnabled(0)
IMU_controller.setFilterMode(1)
IMU_controller.setStreamingTiming(interval=0, delay=0, duration=0, timestamp=False)
IMU_controller.setStreamingSlots(slot0='getTaredOrientationAsEulerAngles', slot1='getNormalizedGyroRate')
IMU_controller.tareWithCurrentOrientation()
IMU_controller.startStreaming()
IMU_subject.setEulerAngleDecompositionOrder(3)
IMU_subject.setCompassEnabled(0)
IMU_subject.setFilterMode(1)
IMU_subject.setStreamingTiming(interval=0, delay=0, duration=0, timestamp=False)
IMU_subject.setStreamingSlots(slot0='getTaredOrientationAsEulerAngles', slot1='getNormalizedGyroRate')
IMU_subject.tareWithCurrentOrientation()
IMU_subject.startStreaming()
dng_device.close()
