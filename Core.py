import sys
import serial
import time
from multiprocessing import Process, Queue
import http_viewer

import os
import sys, tty, termios
from math import sin, cos, fmod, atan2, sqrt, pow

###################### !!! W A R N I N G !!! ########################
port_web_server = int(sys.argv[1])
#####################################################################

# Funciones de operaciones con matrices
def matrix_sum(A, B):
    C = []
    for i in range(len(A)):
        row = []
        for j in range(len(A[i])):
            row.append(A[i][j] + B[i][j])
        C.append(row)
    return C

def matrix_dif(A, B):
    C = []
    for i in range(len(A)):
        row = []
        for j in range(len(A[i])):
            row.append(A[i][j] - B[i][j])
        C.append(row)
    return C

def matrix_zeros(rows, columns):
    C = []
    for i in range(rows):
        aux = []
        for j in range(columns):
            aux.append(0)
        C.append(aux)
    return C

def matrix_mul(A, B):
    C = matrix_zeros(len(A),len(B[0]))
    for i in range(len(C)):
        for j in range(len(C[0])):
            for k in range(len(A[0])):
                C[i][j] += A[i][k] * B[k][j]
    return C

def transpose(A):
    C = matrix_zeros(len(A[0]), len(A))
    for i in range(len(C)):
        for j in range(len(C[0])):
            C[i][j] = A[j][i]
    return C

def diag(V):
    C = matrix_zeros(len(V), 1)
    for i in range(len(C)):
            C[i][0] = V[i][i]
    return C

# Enviar a Neato un comando, substraida de test_NeatoCommands.py
def envia(ser, missatge,temps=0.1, show_time = False):
    first_time=time.time()
    rbuffer = ''
    resp = ''
    ser.write(missatge + chr(10))
    time.sleep(temps) # giving a breath to pi

    while ser.inWaiting() > 0:
        resp = ser.readline()
        rbuffer = rbuffer + resp

    if show_time:
        t = time.time() - first_time
        print("Round time: ",t)

    return rbuffer

# Devuelve la distancia recorrida de cada rueda
def get_motors():
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])
    return (L, R)

# Devuelve informacion de los laseres
def get_laser():
    msg = envia(ser, 'GetLDSScan', 0.05)
    laser_values = []
    for line in msg.split('\r\n')[2:362]:
        s = line.split(',')
        row = [s[0], s[1], s[2], s[3]]
        laser_values.append(row)
    return laser_values

# Odometria con ruido
def odometry(Lnew, Rnew):
    global L_ini, R_ini, S, V, Pk0
    global pose_est1, pose_t, Pk1, x_w, y_w, sum_theta

    L, R = Lnew - L_ini, Rnew - R_ini

    delta_d = (L + R) / 2
    delta_th = (R - L) / (2 * S)

    F_x = [[1., 0., -(delta_d * sin(sum_theta + delta_th))], [0., 1., (delta_d * cos(sum_theta+delta_th))], [0., 0., 1.]];
    F_v = [[cos(sum_theta + delta_th), -delta_d * sin(sum_theta + delta_th)], [sin(sum_theta+delta_th), delta_d*cos(sum_theta+delta_th)], [0., 1.]];

    # Pose_est1 = Pose_est1 + F_x*(Pose_t - Pose_est1) + F_v*diag(V);
    pose_est1 = matrix_sum(pose_est1, matrix_sum(matrix_mul(F_x, matrix_dif(pose_t, pose_est1)), matrix_mul(F_v, diag(V))))
    # Pk1 = F_x*Pk1*F_x' + F_v*V*F_v';
    Pk1 = matrix_sum(matrix_mul(F_x, matrix_mul(Pk1, transpose(F_x))), matrix_mul(F_v, matrix_mul(V,transpose(F_v))))

    x_w = x_w + (delta_d + V[0][0]) * cos(sum_theta + delta_th + V[1][1])
    y_w = y_w + (delta_d + V[0][0]) * sin(sum_theta + delta_th + V[1][1])
    sum_theta = sum_theta + delta_th + V[1][1]
    if (sum_theta < 0):
        sum_theta += 2 * 3.1415
    elif (sum_theta >= 2 * 3.1415):
        sum_theta -= 2 * 3.1415
    pose_t = transpose([[x_w, y_w, sum_theta]])

# Input
def getch():
    sys.stdin = open('/dev/tty')
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print(ch)
    return ch


# Segun la entrada modifica la direccion y velocidad del robot.
# imprime datos (d)
# y finaliza la ejecucion (q)
def set_next_key(tecla):
    global tiempo
    global direccion
    global speed
    global tita_dot
    global x_w, y_w, sum_theta, Pk1

    if tecla == '8' or tecla == '2':
        if tecla == '8':
            speed = speed + 50
        else:
            speed = speed - 50
        if speed >= 0:
            direccion = 0
        else:
            direccion = 1

        if speed == 0:

            envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.1)
            envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.1)

        else:
            distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
            distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

            comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
            envia(ser,comando, 0.1)


    elif tecla == '4' or tecla == '6':
        if tecla == '4':
            tita_dot = tita_dot + (3.1415/10)
        else:
            tita_dot = tita_dot - (3.1415/10)


        distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
        distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

        comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
        envia(ser,comando, 0.1)

    elif tecla == '5':

        direccion = 0
        speed = 0
        tita_dot = 0

        envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.1)
        envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.1)
    elif tecla == "d":
        print ("Odometry:")
        print ('x_w: ' + str(x_w) + ' y_w: ' +str(y_w) + ' theta: ' + str(sum_theta))
        print ("Covariance Matrix:")
        print ('Pk_xx: ' + str(Pk1[0][0]) + ' Pk_yy: ' + str(Pk1[1][1]) + ' Pk_thth: ' + str(Pk1[2][2]))

# Lo ejecuta el thread auxiliar que se encarga de gestionar el input sin bloquear el programa
def f(q):
    r = ''
    while r != "q" and r != "b":
        r = getch()
        q.put(r)

if __name__ == "__main__":
    # Initialize WebServer
    laser_queue = Queue()
    pose_queue = Queue()
    viewer = http_viewer.HttpViewer(port_web_server, laser_queue, pose_queue)
    print ("To open the viewer go to: http:\\\\192.168.100.1:" + str(port_web_server))

    # Initialize Robot
    global ser
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
    envia(ser, 'TestMode On')
    envia(ser, 'PlaySound 1')

    # Habilitamos motores y laser
    envia(ser ,'SetMotor RWheelEnable LWheelEnable')
    envia(ser, 'SetLDSRotation On')

    #Initialize Robot parameters
    speed = 0 		# en mm/s
    tita_dot = 0
    tiempo = 20
    direccion = 0
    L_ini, R_ini = get_motors() #Distancia inicial recorrida por las ruedas

    # Pedimos por input la pose inicial del robot
    print("Introduce las coordenadas iniciales")
    x_w = input("Coordenada x (mm):")
    y_w = input("Coordenada y (mm):")
    sum_theta = input("Orientacion theta (rad):")

    # Inicializamos los parametros de la odometria
    S = 121.5 # en mm
    V = [[0.01 ** 2, 0.],[0., 0.001 ** 2]]
    Pk0 = [[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001],[0.0001, 0.0001, 0.0001]]
    pose_est1 = transpose([[x_w, y_w, sum_theta]])
    pose_t = transpose([[x_w, y_w, sum_theta]])
    Pk1 = Pk0

    # Creamos un thread que se encargue de la lectura del input, pondra en una cola los caracteres leidos
    q = Queue()
    p = Process(target=f, args=(q,))
    p.start()

    try:
        tecla = ""
        # Mientras no finalicemos ejecucion y/o iniciemos vuelta a base
        while tecla != "q" and tecla != "b":
            # Con la distancia actual calcularemos el incremento restandole la anterior dentro de la funcion odometria
            L, R = get_motors()
            odometry(L, R)

            # Una vez calculada la odometria actualizamos los valores guardamos los incrementos actuales para el siguiente frame
            L_ini, R_ini = L, R

            # Enviamos al servidor la nueva posicion del robot (calculada por la odometria) y los puntos detectados por el laser
            # Esperamos a que el servidor lo reciba
            pose_queue.put([(-y_w, x_w)])
            laser_queue.put(get_laser_data())
            time.sleep(0.1)

            # Hacemos una lectura sin espera en busca de nuevos inputs
            if not q.empty():
                tecla = q.get()
                set_next_key(tecla)
            else:
                tecla = ""
        # Si se sale del bucle hay que dejar de detectar inputs
        p.terminate()
        p.join()

        # Comando de ir automaticamente a la base de carga
        if tecla == "b":
            a = 1
        envia(ser, 'SetLDSRotation Off', 0.1)

    except KeyboardInterrupt:
        envia(ser, 'SetLDSRotation Off', 0.1)
