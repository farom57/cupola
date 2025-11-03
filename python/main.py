# This is a sample Python script.
import threading
import urllib
from tkinter.constants import DISABLED, ACTIVE, NORMAL

# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import serial_com
from serial_com import Cupola
from pwi4_client import PWI4
from geometry import to_deg, to_rad, mod, compute_azimuth
import numpy as np
import time
from tkinter import Tk, Button, Label, LabelFrame, Checkbutton, IntVar, Spinbox, StringVar

mount_origin = np.array([0, 100, 0])
dome_radius = 3200  # the center of the dome is 0,0,0
opening_width = 1000
scope_offset = [0, 300, -300, 600]  # offset between the mount origin and the scope: positive to the east when the mount is pointing the south
scope_diameter = [0, 450, 200, 100]
scopes = [1,2]  # scopes enabled for tracking
port = 'COM6'


def worker():
    global s, pwi4, c, mount_connected, ra, dec, target_az, tol, keep_alive, tracking, outputs, current_az
    last_goto_time = 0  # ✅ évite d’envoyer goto trop souvent
    MIN_GOTO_INTERVAL = 3.0  # secondes entre deux ordres

    while keep_alive:
        try:
            # --- Lecture coupole ---
            if not c.connected:
                c.connect(port)
            if c.connected:
                current_az = c.get_azimuth()
                outputs = c.get_outputs()

            # --- Lecture monture ---
            try:
                s = pwi4.status()
                mount_connected = s.mount.is_connected
                if mount_connected:
                    ra = s.mount.ra_apparent_hours
                    dec = s.mount.dec_apparent_degs
                    target_az, tol = target_azimuth(pwi4)
                else:
                    mount_connected = False
            except Exception as e:
                mount_connected = False
                print(f"[PWI4] Erreur : {e}")

            # --- Asservissement ---
            if c.connected and mount_connected:
                tracking = c.get_track_flag()
                if tracking:
                    err = (current_az - target_az + 180) % 360 - 180
                    if abs(err) > tol and (time.time() - last_goto_time) > MIN_GOTO_INTERVAL:
                        print(f"[Coupole] Correction azimut ({current_az:.1f}° → {target_az:.1f}°)")
                        c.goto(target_az)
                        last_goto_time = time.time()
            else:
                tracking = None

            time.sleep(1)

        except Exception as e:
            print(f"[THREAD] Erreur inattendue : {e}")
            time.sleep(1)


def recenter():
    global s, pwi4, c, mount_connected, ra, dec, target_az, tol, keep_alive, tracking, outputs, current_az
    try:
        s = pwi4.status()
        if not s.mount.is_connected:
            mount_connected = False
        else:
            mount_connected = True
            ra = s.mount.ra_apparent_hours
            dec = s.mount.dec_apparent_degs
            target_az, tol = target_azimuth(pwi4)
    except Exception:
        mount_connected = False
        
    if c.connected and mount_connected:
        print("recentre")
        c.goto(target_az)



# return azimuth and tolerance
def target_azimuth(pwi4):
    try:
        s = pwi4.status()
    except Exception as e:
        return 0, 180
    if not s.mount.is_connected:
        return 0, 180
    ha = to_rad((s.site.lmst_hours - s.mount.ra_apparent_hours) * 15)
    de = to_rad(s.mount.dec_apparent_degs)
    lat = to_rad(s.site.latitude_degs)
    az_opt, tolerance, az_scope, el_scope = compute_azimuth(ha, de, lat, mount_origin, dome_radius, opening_width,
                                                            [scope_diameter[i] for i in scopes],
                                                            [scope_offset[i] for i in scopes])

    return to_deg(az_opt), to_deg(tolerance)


class Ui(Tk):
    def __init__(self):
        super().__init__()

        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=0)
        self.grid_rowconfigure(2, weight=0)
        self.grid_rowconfigure(3, weight=0)
        self.grid_rowconfigure(4, weight=0)
        self.grid_columnconfigure(0, weight=1, uniform="same_group")
        self.grid_columnconfigure(1, weight=1, uniform="same_group")
        self.grid_columnconfigure(2, weight=1, uniform="same_group")

        default_button_style = {
            "bg": "#333333", "fg": "red", "highlightthickness": 1,
            "font": ("Arial", 14, "bold")
        }
        default_label_style = {
            "bg": "#111111", "fg": "#AA5555", "highlightthickness": 0,
            "font": ("Arial", 14, "bold")
        }
        default_label_grid = {"padx": 10, "pady": 10, "sticky": "w"}
        default_button_grid = {"padx": 10, "pady": 10, "sticky": "nsew"}

        self.button_up = Button(self, text="↑", **default_button_style, command=c.open)
        self.button_up.grid(column=1, row=0, **default_button_grid)
        self.button_left = Button(self, text="←", **default_button_style, command=c.turn_left)
        self.button_left.grid(column=0, row=1, **default_button_grid)
        self.button_center = Button(self, text="X", **default_button_style, command=c.stop)
        self.button_center.grid(column=1, row=1, **default_button_grid)
        self.button_right = Button(self, text="→", **default_button_style, command=c.turn_right)
        self.button_right.grid(column=2, row=1, **default_button_grid)
        self.button_down = Button(self, text="↓", **default_button_style, command=c.close)
        self.button_down.grid(column=1, row=2, **default_button_grid)
        self.button_track = Button(self, text="Suivi", **default_button_style, command=self.toggle_track)
        self.button_track.grid(column=0, row=2, **default_button_grid)
        self.button_recenter = Button(self, text="Centrer", **default_button_style, command=recenter)
        self.button_recenter.grid(column=2, row=2, **default_button_grid)

        frame_mount = LabelFrame(self, text="Monture", **default_label_style)
        frame_mount.grid(column=0, row=3, columnspan=3, **default_button_grid)
        self.connected_mount = Label(frame_mount, text="Connectée", **default_label_style)
        self.connected_mount.pack(anchor="w")
        self.radec_mount = Label(frame_mount, text="Ra: xx.xxh Dec: +xx.x°", **default_label_style)
        self.radec_mount.pack(anchor="w")

        frame_instrum = LabelFrame(self, text="Instrument", **default_label_style)
        frame_instrum.grid(column=0, row=4, columnspan=3, **default_button_grid)
        self.instrum_1 = IntVar(value=1)
        instrum_1_btn = Checkbutton(frame_instrum, text="T400", onvalue=1, offvalue=0, variable=self.instrum_1, command=self.update_scope,
                                    **default_label_style)
        instrum_1_btn.pack(anchor="w")
        self.instrum_2 = IntVar(value=1)
        instrum_2_btn = Checkbutton(frame_instrum, text="APO140", onvalue=1, offvalue=0, variable=self.instrum_2, command=self.update_scope,
                                    **default_label_style)
        instrum_2_btn.pack(anchor="w")
        self.instrum_3 = IntVar()
        instrum_3_btn = Checkbutton(frame_instrum, text="Petite lunette", onvalue=1, offvalue=0, variable=self.instrum_3, command=self.update_scope,
                                    **default_label_style)
        instrum_3_btn.pack(anchor="w")
        self.instrum_az = Label(frame_instrum, text="Az: xxx.x° tol:yyy.x°", anchor="w", **default_label_style)
        self.instrum_az.pack(anchor="w")

        frame_cupola = LabelFrame(self, text="Coupole", **default_label_style)
        frame_cupola.grid(column=0, row=5, columnspan=3, **default_button_grid)
        self.connected_cupola = Label(frame_cupola, text="Connectée", **default_label_style)
        self.connected_cupola.pack(anchor="w")
        self.az_cupola = Label(frame_cupola, text="Az: xxx.x°", **default_label_style)
        self.az_cupola.pack(anchor="w")
        self.step_cupola = Label(frame_cupola, text="Step: xxx Home: xxx", **default_label_style)
        self.step_cupola.pack(anchor="w")
        ref_label = Label(frame_cupola, text="Reference:", **default_label_style)
        ref_label.pack(anchor="w")
        self.ref_value = StringVar()
        self.ref_cupola = Spinbox(frame_cupola, justify="center", from_=0, to=360, increment=0.5,
                                  textvariable=self.ref_value, command=self.update_ref, **default_button_style)
        self.ref_cupola.pack(anchor="w")
        self.ref_value.set(str(ref_azimuth))
        self.button_set_home = Button(frame_cupola, text="Calibrer", **default_button_style, command=self.set_home)
        self.button_set_home.pack(anchor="w")

        self.configure(bg="#111111", padx=10, pady=10)
        # self.geometry("400x400")

        self.title("Controle coupole")

        # --- Bouton Quitter ---
        self.button_quit = Button(self, text="Quitter", **default_button_style, command=self.on_close)
        self.button_quit.grid(column=1, row=6, padx=10, pady=20, sticky="nsew")

        # Associe la fermeture fenêtre (croix) à la même méthode
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def update_ui(self):
        self.step_cupola.config(text=f"Step: {c.step} Home: {c.home}")
        if c.connected:
            self.connected_cupola.config(text=f"Connectée", fg="green")
        else:
            self.connected_cupola.config(text=f"Deconnectée", fg="red")
        self.az_cupola.config(text=f"Az: {c.azimuth:5.1f}°")
        self.instrum_az.config(text=f"Az: {target_az:5.1f}° tol: {tol:5.1f}° ")
        self.radec_mount.config(text=f"Ra: {ra:5.1f}h Dec: {dec:5.1f}°")
        self.after(1000, window.update_ui)
        if mount_connected:
            self.connected_mount.config(text=f"Connectée", fg="green")
        else:
            self.connected_mount.config(text=f"Deconnectée", fg="red")
        if tracking is None:
            self.button_track.config(state=DISABLED, fg="grey", bg="#333333")
        elif tracking:
            self.button_track.config(state=NORMAL, fg="black", bg="green")
        else:
            self.button_track.config(state=NORMAL, fg="red", bg="#333333")

    def update_ref(self):
        c.ref_azimuth = float(self.ref_cupola.get())

    def update_scope(self):
        global scopes
        scopes = []
        if self.instrum_1.get() == 1:
            scopes.append(1)
        if self.instrum_2.get() == 1:
            scopes.append(2)
        if self.instrum_3.get() == 1:
            scopes.append(3)
        if len(scopes) == 0:
            scopes = [0]


    def toggle_track(self):
        global tracking
        if tracking is None:
            if c.connected:
                c.set_track_flag(False)
        elif not tracking:
            if c.connected and mount_connected:
                c.set_track_flag(True)
        else:
            if c.connected:
                c.set_track_flag(False)

    def set_home(self):
        c.set_track_flag(False)
        c.set_home()

    def on_close(self):
        """Arrêt propre du programme : thread, série, fenêtre."""
        global keep_alive
        print("[UI] Fermeture demandée...")
        keep_alive = False

        # Ferme la connexion série proprement
        try:
            if c.connected:
                print("[UI] Déconnexion de la coupole...")
                c.disconnect()
        except Exception as e:
            print(f"[UI] Erreur lors de la déconnexion : {e}")

        # Ferme la fenêtre
        self.destroy()
        print("[UI] Fenêtre fermée proprement.")



ref_azimuth = 177.
target_az = 180.0
tol = 0.0
ra = 0.0
dec = 0.0
current_az = 0
outputs = 0
tracking = None
pwi4 = PWI4()
c = Cupola(ref_azimuth)

mount_connected = False

try:
    s = pwi4.status()
except Exception:
    pass
t = threading.Thread(target=worker)

window = Ui()

keep_alive = True
t = threading.Thread(target=worker, daemon=True)  # ✅ ajout du daemon
t.start()
window.after(1000, window.update_ui)
window.mainloop()
keep_alive = False
t.join(timeout=2.0)  # ✅ arrêt propre

