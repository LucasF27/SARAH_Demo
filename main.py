import threespace_api as ts_api
import stimulator
import serial
import struct
import math
import thread
import time

IMU_controller_address = 1
IMU_subject_address = 7
# IMU_port = 'COM6'
IMU_port = '/dev/tty.usbmodemFA131'
stimulator_port = '/dev/tty.usbserial-HMQYVD6B'
running = True
stimulation = True
filter_size = 5
angle_controller = []
angle_subject = []
counter = 0
pw_max = 500
current = [4]


def read_sensors():
    global counter
    serial_port = serial.Serial(port=IMU_port, baudrate=115200, timeout=0.001)
    while running:
        bytes_to_read = serial_port.inWaiting()
        if bytes_to_read > 0:
            data = bytearray(serial_port.read(bytes_to_read))
            IMU_id = int(''.join('{:02x}'.format(data[6])))


            # angle
            b = ''.join(chr(i) for i in data[8:12])  # angle y
            ang = struct.unpack('>f', b)
            ang = ang[0]
            # print(x)

            # angle
            # b = ''.join(chr(i) for i in data[12:16])  # angle y
            # ang = struct.unpack('>f', b)
            # ang = ang[0]
            if ang >= 0:
                ang = (ang / math.pi) * 180
                ang = ang + 180
                # if abs(x) > (math.pi*0.75):
                #     ang = ang + 2*(90-ang)
            else:
                ang = 360 + ((ang / math.pi) * 180)
                ang = ang - 180
                # if abs(x) > (math.pi*0.75):
                #     ang = ang - 2*(ang-270)

            # print(ang)
            # print(ang)
            if IMU_id == 1:
                # print 'controller'
                angle_controller.append(ang)
                # if counter >= filter_size and ((ang > 60 and ang < 120) or (ang > 240 and ang < 300)):
                # angle_controller[-1] = numpy.mean(angle_controller[-filter_size:])
            elif IMU_id == 7:
                # print 'subject'
                angle_subject.append(ang)
                # if counter >= filter_size and ((ang > 60 and ang < 120) or (ang > 240 and ang < 300)):
                # angle_subject[-1] = numpy.mean(angle_subject[-filter_size:])
            # else:
            #     print 'neigher'

            # angle.append(ang)

            # print(angle[-1])

            counter += 1
    serial_port.close()


def control(angle_controller_in, angle_subject_in):
    kp = 5
    return -(angle_controller_in-angle_subject_in)*kp


def main():
    while running:
        # print(angle_controller[-1],angle_subject[-1])
        control_signal = control(angle_controller[-1], angle_subject[-1])
        print(control_signal)
        if control_signal < 0:
            control_signal = 0
        elif control_signal > 500:
            control_signal = 500
        pw = control_signal
        if stimulation:
            stim.update(1, [pw], current)
        # time.sleep(0.2)
    stim.stop()
    serialPortStimulator.close()


# IMU setup
dng_device = ts_api.TSDongle(com_port=IMU_port)
IMU_controller = dng_device[IMU_controller_address]
IMU_subject = dng_device[IMU_subject_address]
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

# Stimulator setup
if stimulation:
    serialPortStimulator = serial.Serial(stimulator_port, timeout=1, baudrate=115200)
    stim = stimulator.Stimulator(serialPortStimulator)
    stim.initialization(50, 1)


thread.start_new_thread(read_sensors, ())
time.sleep(0.1)
thread.start_new_thread(main, ())

raw_input('Press ENTER to stop')
running = False
time.sleep(0.2)

dng_device = ts_api.TSDongle(com_port=IMU_port)
IMU_controller = dng_device[IMU_controller_address]
IMU_subject = dng_device[IMU_subject_address]
IMU_controller.stopStreaming()
IMU_subject.stopStreaming()
dng_device.close()


