import time
import serial
import serial.tools.list_ports

STEPS_PER_TURN = 692


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

    # ---------- Connexion / déconnexion ----------

    def connect(self, port=None):
        """Essaye de se connecter au port série."""
        if port is None:
            ports = serial.tools.list_ports.comports(include_links=False)
            if not ports:
                print("Aucun port actif")
                return False
            if len(ports) > 1:
                print(f"{len(ports)} ports actifs ont été trouvés :")
                for p in ports:
                    print(f"  - {p.device}")
            else:
                print("1 port actif a été trouvé :")
                print(ports[0])
            port = ports[0].device

        self.ser.port = port
        self.ser.timeout = 0.1  # ← augmenté pour lire les trames complètes
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
            self.connected = False
            return False

    def disconnect(self):
        if self.ser.is_open:
            self.ser.close()
        self.connected = False

    # ---------- Lecture série sécurisée ----------

    def _readline_int(self, cmd_name: str):
        """
        Lit une ligne depuis le port série, convertit en int si possible.
        Retourne None si trame invalide.
        """
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except serial.SerialException as e:
            print(f"[ERREUR SERIE] Lecture échouée ({cmd_name}) : {e}")
            return None

        if not line:
            print(f"[WARN] Pas de réponse à la commande {cmd_name}")
            return None

        if not line.replace('-', '').isdigit():
            print(f"[WARN] Réponse non numérique ({cmd_name}): {line!r}")
            return None

        return int(line)

    # ---------- Commandes série ----------

    def get_step(self):
        """Demande la position absolue en pas à l’Arduino."""
        if not self.connected:
            return False
        try:
            self.ser.write(b's')
        except serial.SerialException:
            print("[ERREUR SERIE] Impossible d'envoyer 's'")
            return False

        value = self._readline_int('s')
        if value is not None:
            self.step = value
            return self.step
        return False

    def get_home(self):
        """Renvoie la valeur du home locale (pas de lecture série ici)."""
        return self.home

    def set_home(self):
        self.home = self.step
        return self.home

    def get_track_flag(self):
        """Lit le flag de suivi depuis l’Arduino."""
        if not self.connected:
            return False
        try:
            self.ser.write(b'k')
        except serial.SerialException:
            print("[ERREUR SERIE] Impossible d'envoyer 'k'")
            return False

        value = self._readline_int('k')
        if value is None:
            return False

        self.track_flag = (value == 1)
        return self.track_flag

    def set_track_flag(self, state):
        """Modifie le flag de suivi."""
        if not self.connected:
            return False
        cmd = b'k1' if state else b'k0'
        try:
            self.ser.write(cmd)
        except serial.SerialException:
            print(f"[ERREUR SERIE] Impossible d'envoyer {cmd}")
            return False

        value = self._readline_int('k')
        if value is None:
            return False
        self.track_flag = (value == 1)
        return self.track_flag

    def get_outputs(self):
        if not self.connected:
            return 0
        try:
            self.ser.write(b'p')
        except serial.SerialException:
            print("[ERREUR SERIE] Impossible d'envoyer 'p'")
            return 0

        value = self._readline_int('p')
        return value if value is not None else 0

    def get_azimuth(self):
        """Retourne l’azimut calculé à partir du step actuel."""
        if not self.connected:
            return self.azimuth
        self.get_step()
        time.sleep(0.05)
        self.get_home()
        self.azimuth = self.step2azimuth(self.step, self.home, self.ref_azimuth)
        return self.azimuth

    # ---------- Commandes directes coupole ----------

    def turn_left(self):  self._send_simple(b'l')
    def turn_right(self): self._send_simple(b'r')
    def open(self):       self._send_simple(b'o')
    def close(self):      self._send_simple(b'c')
    def light_white(self): self._send_simple(b'w')
    def light_red(self):   self._send_simple(b'n')
    def light_off(self):   self._send_simple(b'b')
    def stop(self):        self._send_simple(b'x')

    def _send_simple(self, cmd: bytes):
        if not self.connected:
            return
        try:
            self.ser.write(cmd)
        except serial.SerialException:
            print(f"[ERREUR SERIE] Impossible d'envoyer {cmd}")

    def goto(self, azimuth):
        """Commande la coupole vers un azimut donné (en degrés)."""
        if not self.connected:
            return False

        self.get_home()
        time.sleep(0.05)
        target = int(self.azimuth2step(azimuth, self.home, self.ref_azimuth))
        cmd = f"t{target}\n".encode()

        try:
            self.ser.write(cmd)
        except serial.SerialException:
            print("[ERREUR SERIE] Impossible d'envoyer goto()")
            return False

        value = self._readline_int(f"t{target}")
        return value if value is not None else False

    # ---------- Conversion pas ↔ azimut ----------

    @staticmethod
    def step2azimuth(step, home, ref_azimuth):
        return ((step - home) / STEPS_PER_TURN * 360 + ref_azimuth) % 360

    @staticmethod
    def azimuth2step(azimuth, home, ref_azimuth):
        return ((azimuth - ref_azimuth) / 360 * STEPS_PER_TURN + home) % STEPS_PER_TURN
