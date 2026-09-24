import errno
import glob
import sys
import time
import serial
import serial.tools.list_ports
import threading  # <-- ajouté

STEPS_PER_TURN = 692
DEFAULT_PORT = "COM6" if sys.platform == "win32" else "/dev/ttyACM0"


def list_ports():
    """Ports série disponibles, ports USB en premier.

    Sous Linux, masque les /dev/ttyS* fantômes (hwid 'n/a') et ajoute les liens
    stables /dev/serial/by-id/*, qui ne changent pas d'un branchement à l'autre
    contrairement à /dev/ttyACM0 ou /dev/ttyUSB0."""
    ports = [p for p in serial.tools.list_ports.comports() if p.hwid != "n/a"]
    ports.sort(key=lambda p: p.vid is None)
    return sorted(glob.glob("/dev/serial/by-id/*")) + [p.device for p in ports]


class Cupola(object):

    def __init__(self, ref=0):
        self.ser = serial.Serial()
        self.baudrate = 1_000_000
        self.home = 0
        self.step = 0
        self.azimuth = 0.0
        self.connected = False
        self.ref_azimuth = ref
        self.track_flag = False
        self.lock = threading.Lock()  # <-- verrou pour protéger l'accès série

    # ---------- Connexion ----------

    def connect(self, port=None):
        if port is None:
            ports = list_ports()
            if not ports:
                print("Aucun port actif")
                return False
            port = ports[0]

        self.ser.port = port
        self.ser.baudrate = self.baudrate
        self.ser.timeout = 0.1
        self.ser.write_timeout = 0.1

        try:
            if self.ser.is_open:
                self.ser.close()
            self.ser.open()
            print(f"Connecté à {port}")
            self.connected = True
            return True
        except serial.SerialException as e:
            print(f"Connexion impossible ({e})")
            if getattr(e, "errno", None) == errno.EACCES and sys.platform != "win32":
                print("  -> Accès refusé : ajouter l'utilisateur au groupe dialout "
                      "(sudo usermod -aG dialout $USER) puis se reconnecter")
            self.connected = False
            return False

    def disconnect(self):
        with self.lock:  # peut être appelé depuis l'UI pendant que le worker utilise le port
            if self.ser.is_open:
                self.ser.close()
            self.connected = False

    # ---------- Lecture protégée ----------

    def _readline_int(self, cmd_name: str):
        """Lecture protégée et conversion en entier."""
        with self.lock:  # <-- protection série
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException as e:
                print(f"[ERREUR SERIE] Lecture échouée ({cmd_name}) : {e}")
                return None

        if not line:
            print(f"[WARN] Pas de réponse à {cmd_name}")
            return None

        if not line.replace('-', '').isdigit():
            print(f"[WARN] Réponse non numérique ({cmd_name}): {line!r}")
            return None

        return int(line)

    def _send(self, data: bytes):
        """Écriture protégée sur le port série."""
        with self.lock:
            try:
                self.ser.write(data)
            except serial.SerialException as e:
                print(f"[ERREUR SERIE] Écriture échouée ({data!r}): {e}")
                return False
        return True

    # ---------- Commandes série protégées ----------

    def get_step(self):
        if not self.connected:
            return False
        if not self._send(b's'):
            return False
        value = self._readline_int('s')
        if value is not None:
            self.step = value
            return self.step
        return False

    def get_home(self):
        return self.home

    def set_home(self):
        self.home = self.step
        return self.home

    def get_track_flag(self):
        if not self.connected:
            return False
        if not self._send(b'k'):
            return False
        value = self._readline_int('k')
        if value is None:
            return False
        self.track_flag = (value == 1)
        return self.track_flag

    def set_track_flag(self, state):
        if not self.connected:
            return False
        cmd = b'k1' if state else b'k0'
        if not self._send(cmd):
            return False
        value = self._readline_int('k')
        if value is None:
            return False
        self.track_flag = (value == 1)
        return self.track_flag

    def get_outputs(self):
        if not self.connected:
            return 0
        if not self._send(b'p'):
            return 0
        value = self._readline_int('p')
        return value if value is not None else 0

    def get_azimuth(self):
        if not self.connected:
            return self.azimuth
        self.get_step()
        time.sleep(0.05)
        self.get_home()
        self.azimuth = self.step2azimuth(self.step, self.home, self.ref_azimuth)
        return self.azimuth

    # ---------- Commandes simples ----------

    def _send_simple(self, cmd: bytes):
        if self.connected:
            self._send(cmd)

    def turn_left(self):  self._send_simple(b'l')
    def turn_right(self): self._send_simple(b'r')
    def open(self):       self._send_simple(b'o')
    def close(self):      self._send_simple(b'c')
    def light_white(self): self._send_simple(b'w')
    def light_red(self):   self._send_simple(b'n')
    def light_off(self):   self._send_simple(b'b')
    def stop(self):        self._send_simple(b'x')

    def goto(self, azimuth):
        if not self.connected:
            return False
        self.get_home()
        time.sleep(0.05)
        target = int(self.azimuth2step(azimuth, self.home, self.ref_azimuth))
        cmd = f"t{target}\n".encode()
        if not self._send(cmd):
            return False
        value = self._readline_int(f"t{target}")
        return value if value is not None else False

    # ---------- Conversion ----------

    @staticmethod
    def step2azimuth(step, home, ref_azimuth):
        return ((step - home) / STEPS_PER_TURN * 360 + ref_azimuth) % 360

    @staticmethod
    def azimuth2step(azimuth, home, ref_azimuth):
        return ((azimuth - ref_azimuth) / 360 * STEPS_PER_TURN + home) % STEPS_PER_TURN
