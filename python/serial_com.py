import time

import serial
import serial.tools.list_ports

STEPS_PER_TURN = 692






class Cupola(object):

    def __init__(self, ref=0):
        self.ser = serial.Serial()
        self.baudrate = 1000000
        self.home = 0
        self.step = 0
        self.azimuth = 0.0
        self.connected = False
        self.ref_azimuth = ref
        self.track_flag = False

    def connect(self, port=None):
        if port is None:
            ports = serial.tools.list_ports.comports(include_links=False)
            if len(ports) != 0:  # on a trouvé au moins un port actif
                if len(ports) > 1:  # affichage du nombre de ports trouvés
                    print(str(len(ports)) + " ports actifs ont ete trouves:")
                else:
                    print("1 port actif a ete trouve:")
                    print(ports[0])
                port = ports[0].device
            else:
                print("Aucun port actif")
                return False
        self.ser.port = port
        self.ser.timeout = 0.01
        try:
            self.ser.close()
            self.ser.open()
            print("Connecté")
            self.connected = True
            return True
        except serial.serialutil.SerialException:
            print("Connexion impossible")
            return False


    def disconnect(self):
        self.ser.close()

    def get_step(self):
        self.ser.write(b's')
        line = self.ser.readline(20)

        try:
            self.step = int(line)
        except ValueError:
            print(f'Invalid message: s --> {line}')
            return False

        return self.step

    def get_home(self):

        return self.home

    def set_home(self):
        self.home = self.step
        return self.home

    def get_track_flag(self):
        self.ser.write(b'k')
        line = self.ser.readline(20)

        try:
            tmp = int(line)
        except ValueError:
            print(f'Invalid message: k --> {line}')
            return False

        #print(f"track = {tmp} {line}")
        if tmp == 1:
            self.track_flag = True
        else:
            self.track_flag = False
        return self.track_flag

    def set_track_flag(self, state):
        if state:
            self.ser.write(b'k1')
        else:
            self.ser.write(b'k0')
        line = self.ser.readline(20)

        try:
            tmp = int(line)
        except ValueError:
            print(f'Invalid message: k --> {line}')
            return False

        if tmp == 1:
            self.track_flag = True
        else:
            self.track_flag = False
        #print(f"track: {line} {tmp}")
        return self.track_flag

    def get_outputs(self):
        self.ser.write(b'p')
        line = self.ser.readline(20)
        tmp = 0
        try:
            tmp = int(line)
        except ValueError:
            print(f'Invalid message: p --> {line}')
        return tmp

    def get_azimuth(self):
        self.get_step()
        time.sleep(0.1)
        self.get_home()
        self.azimuth = self.step2azimuth(self.step, self.home, self.ref_azimuth)
        return self.azimuth

    def turn_left(self):
        self.ser.write(b'l')

    def turn_right(self):
        self.ser.write(b'r')

    def open(self):
        self.ser.write(b'o')

    def close(self):
        self.ser.write(b'c')

    def light_white(self):
        self.ser.write(b'w')

    def light_red(self):
        self.ser.write(b'n')

    def light_off(self):
        self.ser.write(b'b')

    def stop(self):
        self.ser.write(b'x')

    def goto(self, azimuth):
        self.get_home()
        time.sleep(0.1)

        target = int(self.azimuth2step(azimuth,self.home, self.ref_azimuth))
        cmd = f't{target}'
        cmd = cmd.encode()
        self.ser.write(cmd)
        line = self.ser.readline(20)

        try:
            target = int(line)
        except ValueError:
            print(f'Invalid message: {cmd} --> {line}')
        return target

    @staticmethod
    def step2azimuth(step, home, ref_azimuth):
        return ((step-home)/STEPS_PER_TURN*360 + ref_azimuth) % 360

    @staticmethod
    def azimuth2step(azimuth, home, ref_azimuth):
        return ((azimuth - ref_azimuth) / 360 * STEPS_PER_TURN + home) % STEPS_PER_TURN
