from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave
from fmpy.util import plot_result, download_test_file
import socket
import time
import fmpy
import pygame
import threading
import serial

py_serial = serial.Serial(port = 'COM24', baudrate = 115200)

def num_mapping(input, a, b, c, d):
    result = ((d-c)/(b-a))*(input-a)+c

    return(result)

def threaded_input():
    global gear, accel, handle, WheelFLz, WheelFRz, WheelRLz, WheelRRz, car_break

    while True:
        start = time.time()

        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                handle = pygame.joystick.Joystick(0).get_axis(0)
                accel = pygame.joystick.Joystick(0).get_axis(1)
                car_break = pygame.joystick.Joystick(0).get_axis(2)

                accel = num_mapping(accel, -1, 1, 1, 0)
                car_break = num_mapping(car_break, -1, 1, 1, 0)

            if event.type == pygame.JOYBUTTONDOWN:
                if pygame.joystick.Joystick(0).get_button(13):
                    gear += 1.0

                    if gear > 7:
                        gear = 7.0

                if pygame.joystick.Joystick(0).get_button(12):
                    gear -= 1.0

                    if gear < 0:
                        gear = 0.0

        fmu.setReal([input_gear], [gear])
        fmu.setReal([input_accel], [accel])
        fmu.setReal([input_handle], [handle])
        fmu.setReal([input_break], [car_break])

        fmu.doStep(currentCommunicationPoint=start_time, communicationStepSize=step_size)

        while time.time() - start < 0.0005:
            continue



def threaded_TCP(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        sock.connect((host, port))

        while True:
            data_arr[0] = fmu.getReal([output_Vx])[0]-1
            data_arr[1] = fmu.getReal([output_Vy])[0]
            data_arr[2] = fmu.getReal([output_Z])[0]
            data_arr[3] = fmu.getReal([output_roll])[0]
            data_arr[4] = fmu.getReal([output_pitch])[0]
            data_arr[5] = fmu.getReal([output_Wz])[0]

            data = "".join(str(data_arr))
            data = data.strip('[')
            data = data.strip(']')

            sock.sendall(data.encode("utf-8"))
            response = sock.recv(1024).decode("utf-8")

            WheelFLz = float(response.split(':')[0])
            fmu.setReal([input_WheelFL], [WheelFLz])
            data_arr[6] = fmu.getReal([output_FL])[0]
            data_arr[7] = fmu.getReal([ackermann_left])[0] #FL handle
            data_arr[8] = wheel_RPM #FL RPM

            WheelFRz = float(response.split(':')[1])
            fmu.setReal([input_WheelFR], [WheelFRz])
            data_arr[9] = fmu.getReal([output_FR])[0]
            data_arr[10] = fmu.getReal([ackermann_right])[0] #FR handle
            data_arr[11] = wheel_RPM #FR RPM

            WheelRLz = float(response.split(':')[2])
            fmu.setReal([input_WheelRL], [WheelRLz])
            data_arr[12] = fmu.getReal([output_RL])[0]
            data_arr[13] = wheel_RPM #RL RPM

            WheelRRz = float(response.split(':')[3])
            fmu.setReal([input_WheelRR], [WheelRRz])
            data_arr[14] = fmu.getReal([output_RR])[0]
            data_arr[15] = wheel_RPM #RR RPM

            min_wheel_z = min(WheelRLz, WheelRRz, WheelFLz, WheelFRz)

            A_motor = WheelRRz - min_wheel_z
            B_motor = WheelFRz - min_wheel_z
            C_motor = WheelFLz - min_wheel_z
            D_motor = WheelRLz - min_wheel_z

            command = '*' + str(round(A_motor, 2)) + ',' + str(round(B_motor, 2)) + ','+ str(round(C_motor, 2)) +',' + str(round(D_motor, 2)) + ',' + str(round(data_arr[4], 2)) + '\n'
            py_serial.write(command.encode())
            
            # response = py_serial.readline()
            
            # print(response.decode())

    finally:
        sock.close()

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
pygame.init()

handle = 0
accel = 0
gear = 0.0
WheelFLz = 0
WheelFRz = 0
WheelRLz = 0
WheelRRz = 0
car_break = 0

fmu_file = 'All_simulation.fmu'
start_time = 0
threshold = 2.0
step_size = 1e-3
forward = 0
left_right = 0

model_description = read_model_description(fmu_file)

vrs = {}

for variable in model_description.modelVariables:
    vrs[variable.name] = variable.valueReference

input_gear = vrs['gear']
input_handle = vrs['handle']
input_accel = vrs['accel']
input_break = vrs['break_input']
input_WheelFL = vrs['WheelFL']
input_WheelFR = vrs['WheelFR']
input_WheelRL = vrs['WheelRL']
input_WheelRR = vrs['WheelRR']

output_Vx = vrs['body.Vx']
output_Vy = vrs['body.Vy']
output_Wz = vrs['body.Wz']
output_Z = vrs['body.Current_length']
output_roll = vrs['body.Final_phi']
output_pitch = vrs['body.Final_theta']

output_FL = vrs['tire_Sim2.z']
output_FR = vrs['tire_Sim1.z']
output_RL = vrs['rear_Wheel_l.z']
output_RR = vrs['rear_Wheel_r.z']
wheel_RPM = vrs['rear_Wheel_l.wheel_RPM']
ackermann_left = vrs['body.Left_angle']
ackermann_right = vrs['body.Right_angle']

unzipdir = extract(fmu_file)

fmu = FMU2Slave(guid = model_description.guid, unzipDirectory = unzipdir, modelIdentifier=model_description.coSimulation.modelIdentifier, instanceName='instance1')

fmu.instantiate()
fmu.setupExperiment(startTime = start_time)
fmu.enterInitializationMode()
fmu.exitInitializationMode()

host, port = "127.0.0.1", 25001
data_arr = [0]*16

t1 = threading.Thread(target=threaded_TCP, args=(host, port))
t2 = threading.Thread(target=threaded_input, args=())

t1.start()
t2.start()
