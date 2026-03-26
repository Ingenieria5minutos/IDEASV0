import os
import csv
import time
import threading
import subprocess
import statistics
from datetime import datetime
from collections import deque

import numpy as np
import customtkinter as ctk
from tkinter import messagebox

import serial
import serial.tools.list_ports

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    from scipy.signal import butter, lfilter
    SCIPY_OK = True
except Exception:
    SCIPY_OK = False


# =========================================================
# CONFIGURACION GENERAL
# =========================================================
BAUDIOS = 115200
CARPETA_SALIDA = "datos"
NOMBRE_BASE = "ideas_v0"

RPM_MAX_VALIDA = 3000.0

# -------------------------
# Parámetros trigger modo A
# -------------------------
UMBRAL_CORRIENTE_mA = 20.0
PRETRIGGER_MUESTRAS_A = 10
MUESTRAS_CONFIRMACION_A = 2

# -------------------------
# Parámetros trigger modo B
# -------------------------
UMBRAL_RPM = 5.0
PRETRIGGER_MUESTRAS_B = 10
MUESTRAS_CONFIRMACION_B = 2

# -------------------------
# Parámetros práctica 2
# -------------------------
RPM_EQ_P2 = 1251.0
CAIDA_PERT_P2 = 0.05
BANDA_RETORNO_P2 = 0.06
MUESTRAS_CONFIRMACION_PERT_P2 = 2
MUESTRAS_CONFIRMACION_RETORNO_P2 = 4
MAX_DURACION_P2_SEG = 40.0
NUM_PERTURBACIONES_OBJETIVO = 3

ENCABEZADO_MODO_A = ["t_ms", "rpm", "current_mA"]
ENCABEZADO_MODO_B = ["t_ms", "rpm"]
ENCABEZADOS_VALIDOS = [ENCABEZADO_MODO_A, ENCABEZADO_MODO_B]

PRACTICAS = {
    "1": {
        "nombre": "Monitoreo de corriente y velocidad",
        "modo": "A"
    },
    "2": {
        "nombre": "Efecto de perturbaciones mecánicas",
        "modo": "A"
    },
    "3": {
        "nombre": "Filtrado y repetibilidad de señales",
        "modo": "A"
    },
    "4": {
        "nombre": "Evaluación de tasa de muestreo",
        "modo": "B"
    }
}

TS_B_OPCIONES = [5, 10, 20, 50, 100]

# Refresco de gráfica
REFRESH_MS = 150
MAX_PLOT_POINTS = 1200


# =========================================================
# FUNCIONES SERIAL
# =========================================================
def listar_puertos():
    puertos = serial.tools.list_ports.comports()
    return [p.device for p in puertos]


def buscar_encabezado_valido(ser, timeout_seg=10):
    t0 = time.time()

    while time.time() - t0 < timeout_seg:
        linea = ser.readline().decode("utf-8", errors="ignore").strip()

        if not linea:
            continue

        columnas = [c.strip() for c in linea.split(",")]

        if columnas in ENCABEZADOS_VALIDOS:
            return columnas

    return None


def enviar_configuracion(ser, modo, ts_b_ms):
    if modo == "A":
        comando = "A\n"
    elif modo == "B":
        comando = f"B,{ts_b_ms}\n"
    else:
        raise ValueError("Modo inválido")

    ser.write(comando.encode("utf-8"))
    ser.flush()
    return comando.strip()


# =========================================================
# APP
# =========================================================
class IDEASApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        # -------------------------------------------------
        # Apariencia general
        # -------------------------------------------------
        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")

        self.title("IDEAS V0 - Adquisición de datos")
        self.geometry("1460x1040")
        self.minsize(1240, 800)

        self.configure(fg_color="#F3F6FB")

        # -------------------------------------------------
        # Estado interno
        # -------------------------------------------------
        self.ser = None
        self.hilo_adquisicion = None
        self.stop_event = threading.Event()
        self.en_ejecucion = False
        self.last_filter_signature = None

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.carpeta_datos = os.path.join(self.script_dir, CARPETA_SALIDA)
        os.makedirs(self.carpeta_datos, exist_ok=True)

        # Buffers para gráficas
        self.plot_lock = threading.Lock()
        self.plot_t = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_rpm = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_current = deque(maxlen=MAX_PLOT_POINTS)
        self.current_plot_mode = "A"
        self.trigger_detectado_plot = False

        # -------------------------------------------------
        # Variables
        # -------------------------------------------------
        self.var_com = ctk.StringVar(value="")
        self.var_practica = ctk.StringVar(value="1 - Monitoreo de corriente y velocidad")
        self.var_ts_b = ctk.StringVar(value="10")
        self.var_duracion = ctk.StringVar(value="5")
        self.var_estado = ctk.StringVar(value="Aplicación lista")
        self.var_archivo = ctk.StringVar(value="Aún no se ha generado archivo")

        # Variables filtros práctica 3
        self.var_filtro = ctk.StringVar(value="Ninguno")
        self.var_senal_filtro = ctk.StringVar(value="Ambas")
        self.var_ventana = ctk.StringVar(value="5")
        self.var_alpha = ctk.StringVar(value="0.25")
        self.var_fc = ctk.StringVar(value="2.0")
        self.var_orden = ctk.StringVar(value="2")

        # -------------------------------------------------
        # Fuentes
        # -------------------------------------------------
        self.font_title = ctk.CTkFont(family="Segoe UI", size=28, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Segoe UI", size=13)
        self.font_section = ctk.CTkFont(family="Segoe UI", size=18, weight="bold")
        self.font_label = ctk.CTkFont(family="Segoe UI", size=14, weight="bold")
        self.font_text = ctk.CTkFont(family="Segoe UI", size=13)
        self.font_button = ctk.CTkFont(family="Segoe UI", size=16, weight="bold")
        self.font_log = ("Consolas", 11)

        self._crear_interfaz()
        self.actualizar_puertos()
        self.actualizar_visibilidad_controles()
        self._configurar_graficas_por_practica()
        self.actualizar_grafica()

        self.log("Aplicación lista.")
        self.log("Selecciona puerto COM, práctica y luego presiona 'Iniciar adquisición'.")
        if not SCIPY_OK:
            self.log("[INFO] SciPy no está disponible. Butterworth quedará deshabilitado en la práctica 3.")

    # =====================================================
    # UI
    # =====================================================
    def _crear_interfaz(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # -------------------------------------------------
        # Header
        # -------------------------------------------------
        header = ctk.CTkFrame(self, corner_radius=0, fg_color="#EAF0F8", height=86)
        header.grid(row=0, column=0, sticky="ew")
        header.grid_columnconfigure(1, weight=1)

        logo = ctk.CTkFrame(header, width=44, height=44, corner_radius=12, fg_color="#2E7DFF")
        logo.grid(row=0, column=0, padx=(24, 14), pady=20, sticky="w")
        logo.grid_propagate(False)

        logo_label = ctk.CTkLabel(
            logo,
            text="I",
            font=ctk.CTkFont(family="Segoe UI", size=22, weight="bold"),
            text_color="white"
        )
        logo_label.place(relx=0.5, rely=0.5, anchor="center")

        title_box = ctk.CTkFrame(header, fg_color="transparent")
        title_box.grid(row=0, column=1, sticky="w", pady=16)

        ctk.CTkLabel(
            title_box,
            text="IDEAS V0 - Adquisición de datos",
            font=self.font_title,
            text_color="#1E2A3A"
        ).pack(anchor="w")

        ctk.CTkLabel(
            title_box,
            text="Interfaz moderna para configuración, captura, visualización y registro experimental",
            font=self.font_subtitle,
            text_color="#5A6B7B"
        ).pack(anchor="w", pady=(2, 0))

        # -------------------------------------------------
        # Contenedor principal
        # -------------------------------------------------
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.grid(row=1, column=0, sticky="nsew", padx=22, pady=18)
        main.grid_columnconfigure(0, weight=3)
        main.grid_columnconfigure(1, weight=2)
        main.grid_rowconfigure(2, weight=1)

        # -------------------------------------------------
        # Tarjeta configuración
        # -------------------------------------------------
        self.card_config = self._card(main, "Configuración")
        self.card_config.grid(row=0, column=0, sticky="ew", padx=(0, 14), pady=(0, 14))
        self.card_config.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(
            self.card_config,
            text="Puerto COM",
            font=self.font_label,
            text_color="#223042"
        ).grid(row=1, column=0, sticky="w", padx=18, pady=(6, 10))

        self.combo_com = ctk.CTkComboBox(
            self.card_config,
            variable=self.var_com,
            values=[],
            width=180,
            height=38,
            corner_radius=10,
            font=self.font_text,
            dropdown_font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            button_color="#DCE8FF",
            button_hover_color="#C9DCFF",
            text_color="#223042"
        )
        self.combo_com.grid(row=1, column=1, sticky="w", padx=(0, 10), pady=(6, 10))

        self.btn_actualizar = ctk.CTkButton(
            self.card_config,
            text="Actualizar puertos",
            command=self.actualizar_puertos,
            width=170,
            height=38,
            corner_radius=10,
            fg_color="#3B82F6",
            hover_color="#2563EB",
            font=self.font_text
        )
        self.btn_actualizar.grid(row=1, column=2, sticky="w", padx=(0, 18), pady=(6, 10))

        ctk.CTkLabel(
            self.card_config,
            text="Práctica",
            font=self.font_label,
            text_color="#223042"
        ).grid(row=2, column=0, sticky="w", padx=18, pady=10)

        practicas_vals = [f"{k} - {v['nombre']}" for k, v in PRACTICAS.items()]
        self.combo_practica = ctk.CTkComboBox(
            self.card_config,
            variable=self.var_practica,
            values=practicas_vals,
            width=520,
            height=38,
            corner_radius=10,
            font=self.font_text,
            dropdown_font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            button_color="#DCE8FF",
            button_hover_color="#C9DCFF",
            text_color="#223042",
            command=lambda _value: self.on_practica_change()
        )
        self.combo_practica.grid(row=2, column=1, columnspan=2, sticky="w", padx=(0, 18), pady=10)

        self.lbl_ts = ctk.CTkLabel(
            self.card_config,
            text="Tasa de muestreo (ms)",
            font=self.font_label,
            text_color="#223042"
        )
        self.combo_ts = ctk.CTkComboBox(
            self.card_config,
            variable=self.var_ts_b,
            values=[str(x) for x in TS_B_OPCIONES],
            width=120,
            height=38,
            corner_radius=10,
            font=self.font_text,
            dropdown_font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            button_color="#DCE8FF",
            button_hover_color="#C9DCFF",
            text_color="#223042"
        )

        # -------------------------------------------------
        # Panel filtros práctica 3
        # -------------------------------------------------
        self.filtro_frame = ctk.CTkFrame(
            self.card_config,
            fg_color="#F8FAFD",
            corner_radius=12,
            border_width=1,
            border_color="#E2E8F0"
        )
        self.filtro_frame.grid_columnconfigure(1, weight=1)
        self.filtro_frame.grid_columnconfigure(3, weight=1)

        self.lbl_filtro_titulo = ctk.CTkLabel(
            self.filtro_frame,
            text="Filtrado en tiempo real (Práctica 3)",
            font=ctk.CTkFont(family="Segoe UI", size=15, weight="bold"),
            text_color="#1F2D3D"
        )
        self.lbl_filtro_titulo.grid(row=0, column=0, columnspan=4, sticky="w", padx=14, pady=(12, 8))

        ctk.CTkLabel(
            self.filtro_frame,
            text="Filtro",
            font=self.font_label,
            text_color="#223042"
        ).grid(row=1, column=0, sticky="w", padx=14, pady=6)

        self.combo_filtro = ctk.CTkComboBox(
            self.filtro_frame,
            variable=self.var_filtro,
            values=["Ninguno", "Media móvil", "Mediana móvil", "EMA", "Butterworth"],
            width=180,
            height=36,
            corner_radius=10,
            font=self.font_text,
            dropdown_font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            button_color="#DCE8FF",
            button_hover_color="#C9DCFF",
            text_color="#223042",
            command=lambda _value: self.on_filtro_change()
        )
        self.combo_filtro.grid(row=1, column=1, sticky="w", padx=(0, 10), pady=6)

        ctk.CTkLabel(
            self.filtro_frame,
            text="Aplicar a",
            font=self.font_label,
            text_color="#223042"
        ).grid(row=1, column=2, sticky="w", padx=(8, 0), pady=6)

        self.combo_senal_filtro = ctk.CTkComboBox(
            self.filtro_frame,
            variable=self.var_senal_filtro,
            values=["RPM", "Corriente", "Ambas"],
            width=140,
            height=36,
            corner_radius=10,
            font=self.font_text,
            dropdown_font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            button_color="#DCE8FF",
            button_hover_color="#C9DCFF",
            text_color="#223042"
        )
        self.combo_senal_filtro.grid(row=1, column=3, sticky="w", padx=(8, 14), pady=6)

        self.lbl_param1 = ctk.CTkLabel(
            self.filtro_frame,
            text="Ventana",
            font=self.font_label,
            text_color="#223042"
        )
        self.entry_param1 = ctk.CTkEntry(
            self.filtro_frame,
            textvariable=self.var_ventana,
            width=120,
            height=36,
            corner_radius=10,
            font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            text_color="#223042"
        )

        self.lbl_param2 = ctk.CTkLabel(
            self.filtro_frame,
            text="Alpha",
            font=self.font_label,
            text_color="#223042"
        )
        self.entry_param2 = ctk.CTkEntry(
            self.filtro_frame,
            textvariable=self.var_alpha,
            width=120,
            height=36,
            corner_radius=10,
            font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            text_color="#223042"
        )

        self.lbl_param3 = ctk.CTkLabel(
            self.filtro_frame,
            text="fc (Hz)",
            font=self.font_label,
            text_color="#223042"
        )
        self.entry_param3 = ctk.CTkEntry(
            self.filtro_frame,
            textvariable=self.var_fc,
            width=120,
            height=36,
            corner_radius=10,
            font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            text_color="#223042"
        )

        self.lbl_param4 = ctk.CTkLabel(
            self.filtro_frame,
            text="Orden",
            font=self.font_label,
            text_color="#223042"
        )
        self.entry_param4 = ctk.CTkEntry(
            self.filtro_frame,
            textvariable=self.var_orden,
            width=120,
            height=36,
            corner_radius=10,
            font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            text_color="#223042"
        )

        # Duración
        ctk.CTkLabel(
            self.card_config,
            text="Duración post-trigger (s)",
            font=self.font_label,
            text_color="#223042"
        ).grid(row=6, column=0, sticky="w", padx=18, pady=(10, 18))

        self.entry_duracion = ctk.CTkEntry(
            self.card_config,
            textvariable=self.var_duracion,
            width=130,
            height=38,
            corner_radius=10,
            font=self.font_text,
            fg_color="white",
            border_color="#D5DDE8",
            text_color="#223042"
        )
        self.entry_duracion.grid(row=6, column=1, sticky="w", pady=(10, 18))

        # -------------------------------------------------
        # Panel lateral acciones
        # -------------------------------------------------
        self.card_acciones = self._card(main, "Acciones", width=300)
        self.card_acciones.grid(row=0, column=1, rowspan=2, sticky="ns", pady=(0, 14))
        self.card_acciones.grid_propagate(False)

        self.btn_iniciar = ctk.CTkButton(
            self.card_acciones,
            text="▶  Iniciar adquisición",
            command=self.iniciar_adquisicion,
            height=52,
            corner_radius=12,
            fg_color="#22A06B",
            hover_color="#1A8A5C",
            font=self.font_button,
            text_color="white"
        )
        self.btn_iniciar.pack(fill="x", padx=18, pady=(18, 12))

        self.btn_detener = ctk.CTkButton(
            self.card_acciones,
            text="■  Detener",
            command=self.detener_adquisicion,
            height=48,
            corner_radius=12,
            fg_color="#E5EAF2",
            hover_color="#D8E0EB",
            font=self.font_button,
            text_color="#5A6676",
            state="disabled"
        )
        self.btn_detener.pack(fill="x", padx=18, pady=8)

        self.btn_carpeta = ctk.CTkButton(
            self.card_acciones,
            text="📁  Abrir carpeta",
            command=self.abrir_carpeta,
            height=48,
            corner_radius=12,
            fg_color="#2E7DFF",
            hover_color="#2466D3",
            font=self.font_button,
            text_color="white"
        )
        self.btn_carpeta.pack(fill="x", padx=18, pady=8)

        self.btn_limpiar_log = ctk.CTkButton(
            self.card_acciones,
            text="🧹  Limpiar log",
            command=self.limpiar_log,
            height=44,
            corner_radius=12,
            fg_color="#E5EAF2",
            hover_color="#D8E0EB",
            font=self.font_text,
            text_color="#223042"
        )
        self.btn_limpiar_log.pack(fill="x", padx=18, pady=8)

        self.badge_info = ctk.CTkFrame(self.card_acciones, fg_color="#F6F8FC", corner_radius=12)
        self.badge_info.pack(fill="x", padx=18, pady=(18, 12))

        ctk.CTkLabel(
            self.badge_info,
            text="Modo visual",
            font=ctk.CTkFont(family="Segoe UI", size=12, weight="bold"),
            text_color="#6A7787"
        ).pack(anchor="w", padx=12, pady=(10, 2))

        self.lbl_modo_visual = ctk.CTkLabel(
            self.badge_info,
            text="Listo para adquisición",
            font=self.font_text,
            text_color="#223042"
        )
        self.lbl_modo_visual.pack(anchor="w", padx=12, pady=(0, 10))

        # -------------------------------------------------
        # Tarjeta estado
        # -------------------------------------------------
        self.card_estado = self._card(main, "Estado")
        self.card_estado.grid(row=1, column=0, sticky="ew", padx=(0, 14), pady=(0, 14))
        self.card_estado.grid_columnconfigure(1, weight=1)

        self.indicador_estado = ctk.CTkLabel(
            self.card_estado,
            text="●",
            font=ctk.CTkFont(size=26, weight="bold"),
            text_color="#D18B00"
        )
        self.indicador_estado.grid(row=1, column=0, sticky="nw", padx=(18, 10), pady=(8, 4))

        self.lbl_estado = ctk.CTkLabel(
            self.card_estado,
            textvariable=self.var_estado,
            font=ctk.CTkFont(family="Segoe UI", size=22, weight="bold"),
            text_color="#B36B00"
        )
        self.lbl_estado.grid(row=1, column=1, sticky="w", pady=(10, 2), padx=(0, 18))

        ctk.CTkLabel(
            self.card_estado,
            text="Último archivo generado",
            font=ctk.CTkFont(family="Segoe UI", size=12, weight="bold"),
            text_color="#6A7787"
        ).grid(row=2, column=1, sticky="w", padx=(0, 18), pady=(6, 2))

        self.lbl_archivo = ctk.CTkLabel(
            self.card_estado,
            textvariable=self.var_archivo,
            font=self.font_text,
            text_color="#223042",
            wraplength=720,
            justify="left"
        )
        self.lbl_archivo.grid(row=3, column=1, sticky="w", padx=(0, 18), pady=(0, 16))

        # -------------------------------------------------
        # Tarjeta gráfica
        # -------------------------------------------------
        self.card_plot = self._card(main, "Visualización en tiempo real")
        self.card_plot.grid(row=2, column=0, sticky="nsew", padx=(0, 14), pady=(0, 0))
        self.card_plot.grid_rowconfigure(1, weight=1)
        self.card_plot.grid_columnconfigure(0, weight=1)

        self.plot_container = ctk.CTkFrame(
            self.card_plot,
            fg_color="white",
            corner_radius=12
        )
        self.plot_container.grid(row=1, column=0, sticky="nsew", padx=18, pady=(8, 18))
        self.plot_container.grid_rowconfigure(0, weight=1)
        self.plot_container.grid_columnconfigure(0, weight=1)

        self.fig = Figure(figsize=(8, 5), dpi=100)
        self.fig.patch.set_facecolor("white")

        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_container)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=0, column=0, sticky="nsew")

        self.line_rpm_raw, = self.ax1.plot([], [], linewidth=2, label="RPM real")
        self.line_rpm_filt, = self.ax1.plot([], [], linewidth=2, linestyle="--", label="RPM filtrada")
        self.line_current_raw, = self.ax2.plot([], [], linewidth=2, label="Corriente real")
        self.line_current_filt, = self.ax2.plot([], [], linewidth=2, linestyle="--", label="Corriente filtrada")

        # -------------------------------------------------
        # Tarjeta log
        # -------------------------------------------------
        self.card_log = self._card(main, "Consola / Log")
        self.card_log.grid(row=2, column=1, sticky="nsew")
        self.card_log.grid_rowconfigure(1, weight=1)
        self.card_log.grid_columnconfigure(0, weight=1)

        self.log_frame_inner = ctk.CTkFrame(
            self.card_log,
            fg_color="#0F1722",
            corner_radius=12,
            border_width=1,
            border_color="#1F2B3A"
        )
        self.log_frame_inner.grid(row=1, column=0, sticky="nsew", padx=18, pady=(8, 18))
        self.log_frame_inner.grid_rowconfigure(0, weight=1)
        self.log_frame_inner.grid_columnconfigure(0, weight=1)

        self.txt_log = ctk.CTkTextbox(
            self.log_frame_inner,
            wrap="word",
            font=self.font_log,
            fg_color="#0F1722",
            text_color="#D8E4F0",
            border_width=0,
            corner_radius=8
        )
        self.txt_log.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

    def _card(self, parent, titulo, width=None):
        card = ctk.CTkFrame(
            parent,
            fg_color="white",
            corner_radius=18,
            border_width=1,
            border_color="#E2E8F0",
            width=width if width else 200
        )
        if width:
            card.grid_propagate(False)

        header = ctk.CTkFrame(card, fg_color="transparent")
        header.grid(row=0, column=0, columnspan=4, sticky="ew", padx=18, pady=(16, 4))

        ctk.CTkLabel(
            header,
            text=titulo,
            font=self.font_section,
            text_color="#1F2D3D"
        ).pack(anchor="w")

        return card

    # =====================================================
    # UTILIDADES UI
    # =====================================================
    def log(self, mensaje):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.after(0, self._append_log, f"[{timestamp}] {mensaje}\n")

    def _append_log(self, texto):
        self.txt_log.insert("end", texto)
        self.txt_log.see("end")

    def limpiar_log(self):
        self.txt_log.delete("1.0", "end")
        self.log("Log limpiado.")

    def set_estado(self, texto, tipo="warn"):
        def _update():
            self.var_estado.set(texto)

            if tipo == "ok":
                color = "#1F9D60"
            elif tipo == "error":
                color = "#D64545"
            else:
                color = "#C27A00"

            self.lbl_estado.configure(text_color=color)
            self.indicador_estado.configure(text_color=color)
            self.lbl_modo_visual.configure(text=texto)

        self.after(0, _update)

    def actualizar_botones(self, en_ejecucion):
        def _update():
            if en_ejecucion:
                self.btn_iniciar.configure(state="disabled", fg_color="#94A3B8")
                self.btn_detener.configure(state="normal", fg_color="#EF4444", hover_color="#DC2626", text_color="white")
            else:
                self.btn_iniciar.configure(state="normal", fg_color="#22A06B", hover_color="#1A8A5C")
                self.btn_detener.configure(state="disabled", fg_color="#E5EAF2", text_color="#5A6676")
        self.after(0, _update)

    # =====================================================
    # FILTROS PRACTICA 3
    # =====================================================
    def on_filtro_change(self):
        self.actualizar_parametros_filtro_visibles()
        self.last_filter_signature = None

    def obtener_config_filtro(self):
        filtro = self.var_filtro.get().strip()
        senal = self.var_senal_filtro.get().strip()

        cfg = {
            "filtro": filtro,
            "senal": senal,
            "ventana": 5,
            "alpha": 0.25,
            "fc": 2.0,
            "orden": 2,
        }

        try:
            cfg["ventana"] = max(1, int(self.var_ventana.get()))
        except ValueError:
            cfg["ventana"] = 5

        try:
            cfg["alpha"] = float(self.var_alpha.get())
        except ValueError:
            cfg["alpha"] = 0.25

        try:
            cfg["fc"] = float(self.var_fc.get())
        except ValueError:
            cfg["fc"] = 2.0

        try:
            cfg["orden"] = max(1, int(self.var_orden.get()))
        except ValueError:
            cfg["orden"] = 2

        return cfg

    def firma_filtro(self):
        cfg = self.obtener_config_filtro()
        return (
            cfg["filtro"],
            cfg["senal"],
            cfg["ventana"],
            round(cfg["alpha"], 6),
            round(cfg["fc"], 6),
            cfg["orden"]
        )

    def actualizar_parametros_filtro_visibles(self):
        for w in [self.lbl_param1, self.entry_param1, self.lbl_param2, self.entry_param2,
                  self.lbl_param3, self.entry_param3, self.lbl_param4, self.entry_param4]:
            w.grid_forget()

        filtro = self.var_filtro.get().strip()

        if filtro in ["Media móvil", "Mediana móvil"]:
            self.lbl_param1.configure(text="Ventana")
            self.lbl_param1.grid(row=2, column=0, sticky="w", padx=14, pady=(8, 10))
            self.entry_param1.grid(row=2, column=1, sticky="w", padx=(0, 10), pady=(8, 10))

        elif filtro == "EMA":
            self.lbl_param2.configure(text="Alpha (0-1)")
            self.lbl_param2.grid(row=2, column=0, sticky="w", padx=14, pady=(8, 10))
            self.entry_param2.grid(row=2, column=1, sticky="w", padx=(0, 10), pady=(8, 10))

        elif filtro == "Butterworth":
            self.lbl_param3.configure(text="fc (Hz)")
            self.lbl_param4.configure(text="Orden")
            self.lbl_param3.grid(row=2, column=0, sticky="w", padx=14, pady=(8, 10))
            self.entry_param3.grid(row=2, column=1, sticky="w", padx=(0, 10), pady=(8, 10))
            self.lbl_param4.grid(row=2, column=2, sticky="w", padx=(8, 0), pady=(8, 10))
            self.entry_param4.grid(row=2, column=3, sticky="w", padx=(8, 14), pady=(8, 10))

    def media_movil(self, x, ventana):
        if len(x) == 0:
            return np.array([])
        ventana = max(1, int(ventana))
        if ventana == 1:
            return np.array(x, dtype=float)

        kernel = np.ones(ventana, dtype=float) / ventana
        y = np.convolve(np.asarray(x, dtype=float), kernel, mode="same")
        return y

    def mediana_movil(self, x, ventana):
        x = np.asarray(x, dtype=float)
        n = len(x)
        if n == 0:
            return np.array([])
        ventana = max(1, int(ventana))
        if ventana == 1:
            return x.copy()
        if ventana % 2 == 0:
            ventana += 1

        half = ventana // 2
        y = np.empty(n, dtype=float)

        for i in range(n):
            a = max(0, i - half)
            b = min(n, i + half + 1)
            y[i] = np.median(x[a:b])

        return y

    def ema(self, x, alpha):
        x = np.asarray(x, dtype=float)
        n = len(x)
        if n == 0:
            return np.array([])
        alpha = float(alpha)
        alpha = min(max(alpha, 0.001), 1.0)

        y = np.empty(n, dtype=float)
        y[0] = x[0]
        for i in range(1, n):
            y[i] = alpha * x[i] + (1.0 - alpha) * y[i - 1]
        return y

    def estimar_fs(self, t_seg):
        if len(t_seg) < 3:
            return None
        t = np.asarray(t_seg, dtype=float)
        dt = np.diff(t)
        dt = dt[dt > 0]
        if len(dt) == 0:
            return None
        dt_med = float(np.median(dt))
        if dt_med <= 0:
            return None
        return 1.0 / dt_med

    def butterworth_lp(self, x, t_seg, fc_hz, orden):
        x = np.asarray(x, dtype=float)
        if len(x) < 5:
            return x.copy()
        if not SCIPY_OK:
            return x.copy()

        fs = self.estimar_fs(t_seg)
        if fs is None or fs <= 0:
            return x.copy()

        nyq = 0.5 * fs
        if nyq <= 0:
            return x.copy()

        fc_hz = max(0.001, float(fc_hz))
        wn = fc_hz / nyq
        wn = min(max(wn, 1e-5), 0.999)

        try:
            b, a = butter(int(max(1, orden)), wn, btype="low")
            y = lfilter(b, a, x)
            return y
        except Exception:
            return x.copy()

    def aplicar_filtro_array(self, x, t_seg):
        cfg = self.obtener_config_filtro()
        filtro = cfg["filtro"]

        if filtro == "Ninguno":
            return np.asarray(x, dtype=float)
        elif filtro == "Media móvil":
            return self.media_movil(x, cfg["ventana"])
        elif filtro == "Mediana móvil":
            return self.mediana_movil(x, cfg["ventana"])
        elif filtro == "EMA":
            return self.ema(x, cfg["alpha"])
        elif filtro == "Butterworth":
            return self.butterworth_lp(x, t_seg, cfg["fc"], cfg["orden"])

        return np.asarray(x, dtype=float)

    def filtrar_series_practica_3(self, t, rpm, current):
        cfg = self.obtener_config_filtro()
        aplicar_a = cfg["senal"]

        rpm = np.asarray(rpm, dtype=float)
        current = np.asarray(current, dtype=float)

        rpm_f = rpm.copy()
        current_f = current.copy()

        if aplicar_a in ["RPM", "Ambas"]:
            rpm_f = self.aplicar_filtro_array(rpm, t)

        if aplicar_a in ["Corriente", "Ambas"]:
            current_f = self.aplicar_filtro_array(current, t)

        return rpm_f, current_f

    # =====================================================
    # GRAFICAS
    # =====================================================
    def limpiar_buffers_grafica(self):
        with self.plot_lock:
            self.plot_t.clear()
            self.plot_rpm.clear()
            self.plot_current.clear()
            self.trigger_detectado_plot = False

    def agregar_muestra_grafica(self, t_ms, rpm, current_mA=None):
        with self.plot_lock:
            self.plot_t.append(t_ms / 1000.0)
            self.plot_rpm.append(rpm)
            if current_mA is not None:
                self.plot_current.append(current_mA)
            else:
                self.plot_current.append(float("nan"))

    def _configurar_graficas_por_practica(self):
        practica = self.obtener_numero_practica()

        self.fig.clf()

        if practica in ["1", "2"]:
            self.current_plot_mode = "A"
            self.ax1 = self.fig.add_subplot(211)
            self.ax2 = self.fig.add_subplot(212)

            self.line_rpm_raw, = self.ax1.plot([], [], linewidth=2, label="RPM")
            self.line_current_raw, = self.ax2.plot([], [], linewidth=2, label="Corriente")
            self.line_rpm_filt = None
            self.line_current_filt = None

            self.ax1.set_title("Velocidad")
            self.ax1.set_ylabel("RPM")
            self.ax1.grid(True, alpha=0.3)
            self.ax1.legend(loc="upper right")

            self.ax2.set_title("Corriente")
            self.ax2.set_xlabel("Tiempo (s)")
            self.ax2.set_ylabel("mA")
            self.ax2.grid(True, alpha=0.3)
            self.ax2.legend(loc="upper right")

        elif practica == "3":
            self.current_plot_mode = "A3"
            self.ax1 = self.fig.add_subplot(211)
            self.ax2 = self.fig.add_subplot(212)

            self.line_rpm_raw, = self.ax1.plot([], [], linewidth=2, label="RPM real")
            self.line_rpm_filt, = self.ax1.plot([], [], linewidth=2, linestyle="--", label="RPM filtrada")
            self.line_current_raw, = self.ax2.plot([], [], linewidth=2, label="Corriente real")
            self.line_current_filt, = self.ax2.plot([], [], linewidth=2, linestyle="--", label="Corriente filtrada")

            self.ax1.set_title("Velocidad")
            self.ax1.set_ylabel("RPM")
            self.ax1.grid(True, alpha=0.3)
            self.ax1.legend(loc="upper right")

            self.ax2.set_title("Corriente")
            self.ax2.set_xlabel("Tiempo (s)")
            self.ax2.set_ylabel("mA")
            self.ax2.grid(True, alpha=0.3)
            self.ax2.legend(loc="upper right")

        else:
            self.current_plot_mode = "B"
            self.ax1 = self.fig.add_subplot(111)
            self.ax2 = None

            self.line_rpm_raw, = self.ax1.plot([], [], linewidth=2, label="RPM")
            self.line_rpm_filt = None
            self.line_current_raw = None
            self.line_current_filt = None

            self.ax1.set_title("Velocidad")
            self.ax1.set_xlabel("Tiempo (s)")
            self.ax1.set_ylabel("RPM")
            self.ax1.grid(True, alpha=0.3)
            self.ax1.legend(loc="upper right")

        self.fig.tight_layout()
        self.canvas.draw_idle()

    def actualizar_grafica(self):
        try:
            with self.plot_lock:
                t = list(self.plot_t)
                rpm = list(self.plot_rpm)
                current = list(self.plot_current)
                modo = self.current_plot_mode

            if len(t) > 0:
                if modo == "A":
                    self.line_rpm_raw.set_data(t, rpm)
                    self.ax1.relim()
                    self.ax1.autoscale_view()

                    self.line_current_raw.set_data(t, current)
                    self.ax2.relim()
                    self.ax2.autoscale_view()

                elif modo == "A3":
                    rpm_f, current_f = self.filtrar_series_practica_3(t, rpm, current)

                    self.line_rpm_raw.set_data(t, rpm)
                    self.line_rpm_filt.set_data(t, rpm_f)
                    self.ax1.relim()
                    self.ax1.autoscale_view()

                    self.line_current_raw.set_data(t, current)
                    self.line_current_filt.set_data(t, current_f)
                    self.ax2.relim()
                    self.ax2.autoscale_view()

                    sig = self.firma_filtro()
                    if sig != self.last_filter_signature:
                        cfg = self.obtener_config_filtro()
                        if cfg["filtro"] == "Ninguno":
                            self.log("[P3] Filtro activo: Ninguno")
                        elif cfg["filtro"] in ["Media móvil", "Mediana móvil"]:
                            self.log(f"[P3] Filtro activo: {cfg['filtro']} | señal={cfg['senal']} | ventana={cfg['ventana']}")
                        elif cfg["filtro"] == "EMA":
                            self.log(f"[P3] Filtro activo: EMA | señal={cfg['senal']} | alpha={cfg['alpha']:.3f}")
                        elif cfg["filtro"] == "Butterworth":
                            self.log(f"[P3] Filtro activo: Butterworth | señal={cfg['senal']} | fc={cfg['fc']:.3f} Hz | orden={cfg['orden']}")
                        self.last_filter_signature = sig

                elif modo == "B":
                    self.line_rpm_raw.set_data(t, rpm)
                    self.ax1.relim()
                    self.ax1.autoscale_view()

            self.canvas.draw_idle()

        except Exception as e:
            self.log(f"[WARN] Error actualizando gráfica: {e}")

        self.after(REFRESH_MS, self.actualizar_grafica)

    # =====================================================
    # EVENTOS
    # =====================================================
    def actualizar_puertos(self):
        puertos = listar_puertos()

        if not puertos:
            puertos = [""]

        self.combo_com.configure(values=puertos)

        if self.var_com.get() not in puertos:
            self.var_com.set(puertos[0])

        if puertos and puertos[0] != "":
            self.log(f"Puertos detectados: {', '.join([p for p in puertos if p])}")
        else:
            self.log("No se detectaron puertos COM.")

    def on_practica_change(self):
        self.actualizar_visibilidad_controles()
        self.limpiar_buffers_grafica()
        self._configurar_graficas_por_practica()
        self.last_filter_signature = None

    def obtener_numero_practica(self):
        texto = self.var_practica.get().strip()
        return texto.split(" - ")[0]

    def actualizar_visibilidad_controles(self):
        practica = self.obtener_numero_practica()

        self.lbl_ts.grid_forget()
        self.combo_ts.grid_forget()
        self.filtro_frame.grid_forget()

        if practica == "4":
            self.lbl_ts.grid(row=3, column=0, sticky="w", padx=18, pady=10)
            self.combo_ts.grid(row=3, column=1, sticky="w", pady=10)

        if practica == "3":
            self.filtro_frame.grid(row=4, column=0, columnspan=3, sticky="ew", padx=18, pady=(4, 8))
            self.actualizar_parametros_filtro_visibles()

    def abrir_carpeta(self):
        try:
            os.makedirs(self.carpeta_datos, exist_ok=True)
            if os.name == "nt":
                os.startfile(self.carpeta_datos)
            else:
                subprocess.Popen(["xdg-open", self.carpeta_datos])
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo abrir la carpeta:\n{e}")

    def _guardar_metadata_txt(self, archivo_csv, cfg, extra=None):
        try:
            ruta_txt = os.path.splitext(archivo_csv)[0] + "_meta.txt"
            ahora = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            lineas = [
                "IDEAS V0 - Metadatos del ensayo",
                "=" * 50,
                f"Fecha y hora: {ahora}",
                f"Archivo CSV: {archivo_csv}",
                f"Práctica: {cfg.get('practica', '')} - {PRACTICAS[cfg.get('practica', '')]['nombre']}",
                f"Puerto: {cfg.get('puerto', '')}",
                f"Modo: {cfg.get('modo', '')}",
                f"Duración post-trigger (s): {cfg.get('duracion', '')}",
            ]

            if cfg.get("modo") == "B":
                lineas.append(f"Tasa de muestreo solicitada (ms): {cfg.get('ts_b_ms', '')}")

            if cfg.get("practica") == "2":
                lineas.extend([
                    f"Referencia fija P2 (RPM): {RPM_EQ_P2}",
                    f"Caída perturbación (%): {CAIDA_PERT_P2 * 100:.2f}",
                    f"Banda retorno (%): {BANDA_RETORNO_P2 * 100:.2f}",
                    f"Confirmación perturbación (muestras): {MUESTRAS_CONFIRMACION_PERT_P2}",
                    f"Confirmación retorno (muestras): {MUESTRAS_CONFIRMACION_RETORNO_P2}",
                    f"Máx duración P2 (s): {MAX_DURACION_P2_SEG}",
                    f"Perturbaciones objetivo: {NUM_PERTURBACIONES_OBJETIVO}",
                ])

            if cfg.get("practica") == "3":
                filtro_cfg = self.obtener_config_filtro()
                lineas.extend([
                    f"Filtro: {filtro_cfg['filtro']}",
                    f"Aplicar a: {filtro_cfg['senal']}",
                    f"Ventana: {filtro_cfg['ventana']}",
                    f"Alpha: {filtro_cfg['alpha']}",
                    f"fc (Hz): {filtro_cfg['fc']}",
                    f"Orden: {filtro_cfg['orden']}",
                    "Nota: el CSV guardado corresponde a la señal cruda adquirida.",
                ])

            if extra:
                lineas.append("")
                lineas.append("Resultados adicionales")
                lineas.append("-" * 30)
                for k, v in extra.items():
                    lineas.append(f"{k}: {v}")

            with open(ruta_txt, "w", encoding="utf-8") as f:
                f.write("\n".join(lineas))

            self.log(f"Metadatos guardados: {ruta_txt}")

        except Exception as e:
            self.log(f"[WARN] No se pudieron guardar metadatos: {e}")

    def _calcular_estadisticas_muestreo(self, filas, idx_t):
        try:
            t_ms_vals = []
            for fila in filas:
                try:
                    t_ms_vals.append(float(fila[idx_t]))
                except Exception:
                    pass

            if len(t_ms_vals) < 3:
                return None

            dt_ms = []
            for i in range(1, len(t_ms_vals)):
                d = t_ms_vals[i] - t_ms_vals[i - 1]
                if d > 0:
                    dt_ms.append(d)

            if len(dt_ms) < 2:
                return None

            dt_prom = sum(dt_ms) / len(dt_ms)
            dt_med = statistics.median(dt_ms)
            fs_prom = 1000.0 / dt_prom if dt_prom > 0 else 0.0
            fs_med = 1000.0 / dt_med if dt_med > 0 else 0.0

            return {
                "muestras": len(t_ms_vals),
                "dt_promedio_ms": round(dt_prom, 3),
                "dt_mediana_ms": round(dt_med, 3),
                "fs_promedio_hz": round(fs_prom, 3),
                "fs_mediana_hz": round(fs_med, 3),
            }

        except Exception as e:
            self.log(f"[WARN] Error calculando muestreo real: {e}")
            return None

    def validar_configuracion(self):
        puerto = self.var_com.get().strip()
        if not puerto:
            messagebox.showwarning("Validación", "Selecciona un puerto COM.")
            return None

        try:
            duracion = float(self.var_duracion.get())
            if duracion <= 0:
                raise ValueError
        except ValueError:
            messagebox.showwarning("Validación", "La duración debe ser un número mayor a 0.")
            return None

        practica = self.obtener_numero_practica()
        modo = PRACTICAS[practica]["modo"]

        ts_b_ms = 10
        if modo == "B":
            try:
                ts_b_ms = int(self.var_ts_b.get())
            except ValueError:
                messagebox.showwarning("Validación", "Selecciona una tasa de muestreo válida.")
                return None

        if practica == "3":
            filtro = self.var_filtro.get().strip()

            if filtro in ["Media móvil", "Mediana móvil"]:
                try:
                    ventana = int(self.var_ventana.get())
                    if ventana < 1:
                        raise ValueError
                except ValueError:
                    messagebox.showwarning("Validación", "La ventana del filtro debe ser un entero >= 1.")
                    return None

            elif filtro == "EMA":
                try:
                    alpha = float(self.var_alpha.get())
                    if not (0 < alpha <= 1):
                        raise ValueError
                except ValueError:
                    messagebox.showwarning("Validación", "Alpha debe estar en el intervalo (0, 1].")
                    return None

            elif filtro == "Butterworth":
                if not SCIPY_OK:
                    messagebox.showwarning("Validación", "Butterworth requiere SciPy. Instálalo con: pip install scipy")
                    return None
                try:
                    fc = float(self.var_fc.get())
                    orden = int(self.var_orden.get())
                    if fc <= 0 or orden < 1:
                        raise ValueError
                except ValueError:
                    messagebox.showwarning("Validación", "Para Butterworth, fc > 0 y orden >= 1.")
                    return None

        return {
            "puerto": puerto,
            "duracion": duracion,
            "practica": practica,
            "modo": modo,
            "ts_b_ms": ts_b_ms
        }

    # =====================================================
    # CONTROL ADQUISICION
    # =====================================================
    def iniciar_adquisicion(self):
        if self.en_ejecucion:
            return

        cfg = self.validar_configuracion()
        if cfg is None:
            return

        self.limpiar_buffers_grafica()
        self._configurar_graficas_por_practica()
        self.last_filter_signature = None

        self.stop_event.clear()
        self.en_ejecucion = True
        self.actualizar_botones(True)

        self.set_estado("Iniciando adquisición...", "warn")
        self.log("=" * 70)
        self.log(f"Práctica seleccionada: {cfg['practica']} - {PRACTICAS[cfg['practica']]['nombre']}")
        self.log(f"Puerto: {cfg['puerto']}")
        self.log(f"Modo: {cfg['modo']}")
        self.log(f"Duración: {cfg['duracion']} s")

        if cfg["modo"] == "B":
            self.log(f"Tasa de muestreo solicitada: {cfg['ts_b_ms']} ms")

        if cfg["practica"] == "2":
            self.log("Lógica especial activa: detección automática de 3 perturbaciones y retorno al equilibrio fijo.")
            self.log(f"Referencia fija P2: {RPM_EQ_P2:.1f} RPM")
            self.log(f"Caída para perturbación: {CAIDA_PERT_P2*100:.1f}%")
            self.log(f"Banda de retorno: ±{BANDA_RETORNO_P2*100:.1f}%")

        if cfg["practica"] == "3":
            filtro_cfg = self.obtener_config_filtro()
            self.log("Práctica 3: visualización en vivo de señal real vs señal filtrada.")
            if filtro_cfg["filtro"] == "Ninguno":
                self.log("Filtro seleccionado: Ninguno")
            elif filtro_cfg["filtro"] in ["Media móvil", "Mediana móvil"]:
                self.log(
                    f"Filtro seleccionado: {filtro_cfg['filtro']} | señal={filtro_cfg['senal']} | "
                    f"ventana={filtro_cfg['ventana']}"
                )
            elif filtro_cfg["filtro"] == "EMA":
                self.log(
                    f"Filtro seleccionado: EMA | señal={filtro_cfg['senal']} | "
                    f"alpha={filtro_cfg['alpha']:.3f}"
                )
            elif filtro_cfg["filtro"] == "Butterworth":
                self.log(
                    f"Filtro seleccionado: Butterworth | señal={filtro_cfg['senal']} | "
                    f"fc={filtro_cfg['fc']:.3f} Hz | orden={filtro_cfg['orden']}"
                )
            self.log("Nota: el CSV guardado corresponde a la señal cruda adquirida.")

        if cfg["practica"] == "4":
            self.log("Práctica 4: evaluación de tasa de muestreo.")
            self.log("Visualización activa: solo velocidad (RPM).")
            self.log(f"TS configurado: {cfg['ts_b_ms']} ms")

        self.hilo_adquisicion = threading.Thread(
            target=self._adquisicion_worker,
            args=(cfg,),
            daemon=True
        )
        self.hilo_adquisicion.start()

    def detener_adquisicion(self):
        if self.en_ejecucion:
            self.stop_event.set()
            self.log("Se solicitó detener la adquisición...")
            self.set_estado("Deteniendo...", "warn")

    def finalizar_interfaz(self):
        self.en_ejecucion = False
        self.actualizar_botones(False)

    # =====================================================
    # WORKER
    # =====================================================
    def _adquisicion_worker(self, cfg):
        archivo_salida = None
        ser = None

        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            archivo_salida = os.path.join(
                self.carpeta_datos,
                f"{NOMBRE_BASE}_P{cfg['practica']}_{timestamp}.csv"
            )

            self.log(f"Abriendo puerto {cfg['puerto']} a {BAUDIOS} baudios...")
            ser = serial.Serial(cfg["puerto"], BAUDIOS, timeout=1)
            time.sleep(2)
            ser.reset_input_buffer()

            comando = enviar_configuracion(ser, cfg["modo"], cfg["ts_b_ms"])
            self.log(f"Configuración enviada: {comando}")

            self.set_estado("Buscando encabezado válido...", "warn")
            self.log("Buscando encabezado válido...")
            columnas = buscar_encabezado_valido(ser, timeout_seg=10)

            if columnas is None:
                raise RuntimeError(
                    "No se encontró un encabezado válido. "
                    "Verifica Arduino, cable, baud rate y que el monitor serial esté cerrado."
                )

            self.log(f"Encabezado detectado: {columnas}")

            if cfg["modo"] == "A" and columnas != ENCABEZADO_MODO_A:
                raise RuntimeError(f"Se esperaba Modo A {ENCABEZADO_MODO_A}, pero Arduino envió {columnas}")

            if cfg["modo"] == "B" and columnas != ENCABEZADO_MODO_B:
                raise RuntimeError(f"Se esperaba Modo B {ENCABEZADO_MODO_B}, pero Arduino envió {columnas}")

            extra_metadata = None

            if cfg["practica"] == "2":
                self._capturar_practica_2(ser, columnas, archivo_salida)
            elif cfg["modo"] == "A":
                self._capturar_modo_a(ser, columnas, archivo_salida, cfg["duracion"])
            elif cfg["modo"] == "B":
                extra_metadata = self._capturar_modo_b(ser, columnas, archivo_salida, cfg["duracion"], cfg["ts_b_ms"])

            self._guardar_metadata_txt(archivo_salida, cfg, extra=extra_metadata)

            self.after(0, lambda: self.var_archivo.set(archivo_salida))
            self.set_estado("Adquisición terminada correctamente", "ok")
            self.log(f"Archivo guardado: {archivo_salida}")

        except Exception as e:
            self.set_estado("Error en adquisición", "error")
            self.log(f"[ERROR] {e}")
            self.after(0, lambda: messagebox.showerror("Error", str(e)))

        finally:
            try:
                if ser is not None and ser.is_open:
                    ser.close()
            except Exception:
                pass

            self.after(0, self.finalizar_interfaz)

    # =====================================================
    # CAPTURA PRÁCTICA 1 Y 3 (MODO A estándar)
    # =====================================================
    def _capturar_modo_a(self, ser, columnas, archivo_salida, duracion_seg):
        idx_t = columnas.index("t_ms")
        idx_rpm = columnas.index("rpm")
        idx_corr = columnas.index("current_mA")

        self.set_estado("Esperando trigger por corriente...", "warn")
        self.log("Modo A: tiempo, rpm, corriente")
        self.log(f"Trigger por corriente > {UMBRAL_CORRIENTE_mA} mA")
        self.log("Esperando conexión/activación del motor...")

        buffer_previo = deque(maxlen=PRETRIGGER_MUESTRAS_A)
        consecutivas = 0
        trigger_detectado = False
        datos_guardar = []
        t_inicio_trigger = None

        while not self.stop_event.is_set():
            linea = ser.readline().decode("utf-8", errors="ignore").strip()

            if not linea:
                continue

            partes = [x.strip() for x in linea.split(",")]
            if len(partes) != len(columnas):
                continue

            try:
                t_ms = float(partes[idx_t])
                rpm = float(partes[idx_rpm])
                corriente = float(partes[idx_corr])
            except ValueError:
                continue

            if abs(rpm) > RPM_MAX_VALIDA:
                continue

            self.agregar_muestra_grafica(t_ms, rpm, corriente)

            fila = partes[:]
            buffer_previo.append(fila)

            if not trigger_detectado:
                if abs(corriente) > UMBRAL_CORRIENTE_mA:
                    consecutivas += 1
                else:
                    consecutivas = 0

                if consecutivas >= MUESTRAS_CONFIRMACION_A:
                    trigger_detectado = True
                    self.trigger_detectado_plot = True
                    t_inicio_trigger = time.time()
                    self.set_estado("Trigger detectado - grabando...", "ok")
                    self.log("Conexión detectada. Grabando datos...")

                    datos_guardar.extend(list(buffer_previo))
            else:
                datos_guardar.append(fila)

                if len(datos_guardar) % 20 == 0:
                    self.log(f"Muestras acumuladas: {len(datos_guardar)}")

                if time.time() - t_inicio_trigger >= duracion_seg:
                    break

        if self.stop_event.is_set():
            self.log("Captura detenida manualmente.")

        self._guardar_csv_simple(archivo_salida, columnas, datos_guardar)

        self.log("Adquisición terminada.")
        self.log(f"Muestras pretrigger usadas: {min(len(buffer_previo), PRETRIGGER_MUESTRAS_A)}")
        self.log(f"Muestras totales guardadas: {len(datos_guardar)}")

    # =====================================================
    # CAPTURA PRÁCTICA 2 (Perturbaciones con referencia fija)
    # =====================================================
    def _capturar_practica_2(self, ser, columnas, archivo_salida):
        idx_t = columnas.index("t_ms")
        idx_rpm = columnas.index("rpm")
        idx_corr = columnas.index("current_mA")

        self.log("Práctica 2: detección automática de perturbaciones mecánicas.")
        self.log("Se contará una perturbación cuando la rpm caiga lo suficiente y luego regrese al equilibrio.")
        self.log(f"Objetivo: {NUM_PERTURBACIONES_OBJETIVO} perturbaciones válidas.")

        datos_guardar = []

        perturbacion_actual = 1
        en_perturbacion = False

        contador_caida = 0
        contador_retorno = 0

        t_inicio_ensayo = time.time()

        umbral_caida = RPM_EQ_P2 * (1.0 - CAIDA_PERT_P2)

        self.set_estado("Esperando perturbación 1", "warn")
        self.log(f"Referencia fija: {RPM_EQ_P2:.1f} RPM")
        self.log(f"Umbral de caída: {umbral_caida:.1f} RPM")

        while not self.stop_event.is_set():
            if time.time() - t_inicio_ensayo >= MAX_DURACION_P2_SEG:
                self.log(f"Tiempo máximo alcanzado ({MAX_DURACION_P2_SEG} s). Finalizando práctica 2.")
                break

            linea = ser.readline().decode("utf-8", errors="ignore").strip()
            if not linea:
                continue

            partes = [x.strip() for x in linea.split(",")]
            if len(partes) != len(columnas):
                continue

            try:
                t_ms = float(partes[idx_t])
                rpm = float(partes[idx_rpm])
                corriente = float(partes[idx_corr])
            except ValueError:
                continue

            if abs(rpm) > RPM_MAX_VALIDA:
                continue

            self.agregar_muestra_grafica(t_ms, rpm, corriente)
            datos_guardar.append(partes[:])

            if not en_perturbacion:
                self.set_estado(f"Esperando perturbación {perturbacion_actual}", "warn")

                if rpm < umbral_caida:
                    contador_caida += 1
                else:
                    contador_caida = 0

                if contador_caida >= MUESTRAS_CONFIRMACION_PERT_P2:
                    en_perturbacion = True
                    contador_caida = 0
                    contador_retorno = 0
                    self.set_estado(f"Perturbación {perturbacion_actual} en proceso", "ok")
                    self.log(
                        f"Perturbación {perturbacion_actual} detectada "
                        f"(rpm={rpm:.1f}, eq={RPM_EQ_P2:.1f}, umbral_caida={umbral_caida:.1f})"
                    )

            else:
                error_rel = abs(rpm - RPM_EQ_P2) / max(RPM_EQ_P2, 1e-9)

                if error_rel <= BANDA_RETORNO_P2:
                    contador_retorno += 1
                else:
                    contador_retorno = 0

                if contador_retorno >= MUESTRAS_CONFIRMACION_RETORNO_P2:
                    self.log(
                        f"Recuperación de perturbación {perturbacion_actual} confirmada "
                        f"(rpm={rpm:.1f}, eq={RPM_EQ_P2:.1f})"
                    )
                    contador_retorno = 0
                    en_perturbacion = False

                    if perturbacion_actual >= NUM_PERTURBACIONES_OBJETIVO:
                        self.set_estado("Tercera perturbación completada - regresó al equilibrio", "ok")
                        self.log("Se completaron 3 perturbaciones válidas. Finalizando adquisición.")
                        break
                    else:
                        perturbacion_actual += 1
                        self.set_estado(f"Esperando perturbación {perturbacion_actual}", "warn")
                        self.log(f"Ahora esperando perturbación {perturbacion_actual}...")

        if self.stop_event.is_set():
            self.log("Captura detenida manualmente.")

        self._guardar_csv_simple(archivo_salida, columnas, datos_guardar)

        self.log("Adquisición de práctica 2 terminada.")
        self.log(f"Muestras totales guardadas: {len(datos_guardar)}")

    # =====================================================
    # CAPTURA MODO B - PRÁCTICA 4
    # =====================================================
    def _capturar_modo_b(self, ser, columnas, archivo_salida, duracion_seg, ts_b_ms):
        idx_t = columnas.index("t_ms")
        idx_rpm = columnas.index("rpm")

        self.set_estado("Esperando trigger por RPM...", "warn")
        self.log("Modo B: tiempo, rpm")
        self.log(f"TS solicitado: {ts_b_ms} ms")
        self.log(f"Trigger por rpm > {UMBRAL_RPM}")
        self.log("Esperando giro del motor...")

        buffer_previo = deque(maxlen=PRETRIGGER_MUESTRAS_B)
        consecutivas = 0
        trigger_detectado = False
        datos_guardar = []
        t_inicio_trigger = None

        while not self.stop_event.is_set():
            linea = ser.readline().decode("utf-8", errors="ignore").strip()

            if not linea:
                continue

            partes = [x.strip() for x in linea.split(",")]
            if len(partes) != len(columnas):
                continue

            try:
                t_ms = float(partes[idx_t])
                rpm = float(partes[idx_rpm])
            except ValueError:
                continue

            if abs(rpm) > RPM_MAX_VALIDA:
                continue

            self.agregar_muestra_grafica(t_ms, rpm, None)

            fila = partes[:]
            buffer_previo.append(fila)

            if not trigger_detectado:
                if abs(rpm) > UMBRAL_RPM:
                    consecutivas += 1
                else:
                    consecutivas = 0

                if consecutivas >= MUESTRAS_CONFIRMACION_B:
                    trigger_detectado = True
                    self.trigger_detectado_plot = True
                    t_inicio_trigger = time.time()
                    self.set_estado("Trigger detectado - grabando...", "ok")
                    self.log("Giro detectado. Grabando datos...")

                    datos_guardar.extend(list(buffer_previo))
            else:
                datos_guardar.append(fila)

                if len(datos_guardar) % 20 == 0:
                    self.log(f"Muestras acumuladas: {len(datos_guardar)}")

                if time.time() - t_inicio_trigger >= duracion_seg:
                    break

        if self.stop_event.is_set():
            self.log("Captura detenida manualmente.")

        self._guardar_csv_simple(archivo_salida, columnas, datos_guardar)

        self.log("Adquisición terminada.")
        self.log(f"Muestras pretrigger usadas: {min(len(buffer_previo), PRETRIGGER_MUESTRAS_B)}")
        self.log(f"Muestras totales guardadas: {len(datos_guardar)}")

        stats = self._calcular_estadisticas_muestreo(datos_guardar, idx_t)

        if stats is not None:
            self.log(f"[P4] dt promedio real: {stats['dt_promedio_ms']} ms")
            self.log(f"[P4] dt mediana real: {stats['dt_mediana_ms']} ms")
            self.log(f"[P4] fs promedio real: {stats['fs_promedio_hz']} Hz")
            self.log(f"[P4] fs mediana real: {stats['fs_mediana_hz']} Hz")

            try:
                ts_solicitado = float(ts_b_ms)
                error_abs = abs(stats["dt_promedio_ms"] - ts_solicitado)
                self.log(f"[P4] Error absoluto respecto a TS solicitado: {round(error_abs, 3)} ms")
                stats["ts_solicitado_ms"] = ts_solicitado
                stats["error_abs_ts_ms"] = round(error_abs, 3)
            except Exception:
                pass

        return stats

    # =====================================================
    # GUARDADO
    # =====================================================
    @staticmethod
    def _guardar_csv_simple(archivo_salida, columnas, filas):
        with open(archivo_salida, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(columnas)
            for fila in filas:
                writer.writerow(fila)


# =========================================================
# MAIN
# =========================================================
if __name__ == "__main__":
    app = IDEASApp()
    app.mainloop()