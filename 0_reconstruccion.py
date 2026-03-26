import time
import math
from pathlib import Path

import serial
import pandas as pd
import matplotlib.pyplot as plt


# =========================================================
# CONFIGURACION DEL USUARIO
# =========================================================

PORT = "COM5"               # <-- CAMBIA ESTO SI HACE FALTA
BAUDRATE = 250000
DURATION_S = 8.0            # captura total; deja margen para reset/calibracion
TIMEOUT = 1.0

OUTPUT_DIR = Path("capturas_n20")
OUTPUT_DIR.mkdir(exist_ok=True)

# ---- Hardware INA240 ----
VREF = 5.0
ADC_MAX = 1023
R_SHUNT = 0.1               # ohm
GAIN = 20.0                 # INA240A1 = 20, A2 = 50, A3 = 100, A4 = 200

# ---- Encoder y caja ----
PPR_MOTOR = 7
GEAR_RATIO = 10             # caja 1:50
PPR_OUTPUT = PPR_MOTOR * GEAR_RATIO

# ---- Reconstruccion de velocidad ----
VEL_WINDOW_SAMPLES = 20     # 10 muestras a 1 kHz = 10 ms
MIN_DT_S = 1e-6

# ---- Corriente ----
CURRENT_DEADBAND_MA = 0.0   # por ejemplo 5.0 si quieres truncar ruido pequeño

# ---- Depuracion ----
PRINT_RX = True             # True para ver lo que llega por serial
WAIT_AFTER_OPEN_S = 5.0     # muy importante por reset + calibracion Arduino


# =========================================================
# CAPTURA SERIAL
# =========================================================

def capturar_serial(port: str, baudrate: int, duration_s: float, timeout: float = 1.0):
    """
    Captura datos del Arduino.
    Espera lineas con formato:
      t_us,adc,totalPulses

    Ignora lineas que empiezan con '#'.
    Intenta detectar adcZero en una linea tipo:
      # adcZero=512
    """
    ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    # Al abrir el puerto, el Arduino suele resetearse.
    # Aqui le damos tiempo a que haga setup + calibracion.
    time.sleep(WAIT_AFTER_OPEN_S)

    # Vaciar buffer por si quedaron lineas viejas/incompletas
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    print(f"[INFO] Puerto abierto: {port} @ {baudrate}")
    print(f"[INFO] Espera inicial tras abrir puerto: {WAIT_AFTER_OPEN_S:.2f} s")
    print(f"[INFO] Capturando durante {duration_s:.2f} s...")

    rows = []
    adc_zero = None

    t0 = time.time()
    while time.time() - t0 < duration_s:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        if PRINT_RX:
            print(f"[RX] {line}")

        # Lineas de comentario del Arduino
        if line.startswith("#"):
            if "adcZero=" in line:
                try:
                    adc_zero = int(line.split("adcZero=")[1].strip())
                    print(f"[INFO] adcZero detectado desde Arduino: {adc_zero}")
                except Exception:
                    pass
            continue

        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 3:
            continue

        try:
            t_us = int(parts[0])
            adc = int(parts[1])
            total_pulses = int(parts[2])
            rows.append((t_us, adc, total_pulses))
        except ValueError:
            continue

    ser.close()
    print(f"[INFO] Captura terminada. Muestras validas: {len(rows)}")

    if len(rows) == 0:
        raise RuntimeError(
            "No se capturaron datos validos del puerto serial. "
            "Revisa baudrate, puerto COM, sketch cargado y tiempo de calibracion."
        )

    df = pd.DataFrame(rows, columns=["t_us", "adc", "total_pulses"])
    return df, adc_zero


# =========================================================
# RECONSTRUCCION
# =========================================================

def reconstruir_senales(df: pd.DataFrame, adc_zero: int | None):
    """
    A partir de:
      t_us, adc, total_pulses

    reconstruye:
      t_s
      corriente_mA
      rpm_motor_raw
      rpm_output_raw
      rpm_motor_filt
      rpm_output_filt
      omega_motor_rad_s
      omega_output_rad_s
    """
    out = df.copy()

    # ---- Tiempo ----
    out["t_s"] = (out["t_us"] - out["t_us"].iloc[0]) / 1e6
    out["dt_s"] = out["t_us"].diff() / 1e6
    dt_med = out["dt_s"].median()
    if pd.isna(dt_med) or dt_med <= 0:
        dt_med = 0.001
    out["dt_s"] = out["dt_s"].fillna(dt_med)
    out.loc[out["dt_s"] < MIN_DT_S, "dt_s"] = MIN_DT_S

    # ---- Corriente ----
    if adc_zero is None:
        adc_zero = int(out["adc"].iloc[: min(200, len(out))].median())
        print(f"[WARN] No vino adcZero desde Arduino. Usando estimado: {adc_zero}")

    out["adc_zero"] = adc_zero
    out["v_out"] = out["adc"] * VREF / ADC_MAX
    out["v_zero"] = adc_zero * VREF / ADC_MAX
    out["delta_v"] = out["v_out"] - out["v_zero"]

    out["current_A"] = (out["delta_v"] / GAIN) / R_SHUNT
    out["current_mA"] = out["current_A"] * 1000.0

    if CURRENT_DEADBAND_MA > 0:
        mask = out["current_mA"].abs() < CURRENT_DEADBAND_MA
        out.loc[mask, "current_mA"] = 0.0

    # Si quieres forzar a no negativos, descomenta:
    # out["current_mA"] = out["current_mA"].clip(lower=0.0)

    # ---- Pulsos y velocidad ----
    out["dp"] = out["total_pulses"].diff().fillna(0)

    # crudo muestra a muestra
    out["rps_motor_raw"] = (out["dp"] / PPR_MOTOR) / out["dt_s"]
    out["rpm_motor_raw"] = out["rps_motor_raw"] * 60.0

    out["rps_output_raw"] = (out["dp"] / PPR_OUTPUT) / out["dt_s"]
    out["rpm_output_raw"] = out["rps_output_raw"] * 60.0

    # filtrado por ventana
    n = VEL_WINDOW_SAMPLES
    if len(out) > n:
        out["dp_win"] = out["total_pulses"].diff(periods=n)
        out["dt_win_s"] = out["t_s"].diff(periods=n)

        out["rps_motor_filt"] = (out["dp_win"] / PPR_MOTOR) / out["dt_win_s"]
        out["rpm_motor_filt"] = out["rps_motor_filt"] * 60.0

        out["rps_output_filt"] = (out["dp_win"] / PPR_OUTPUT) / out["dt_win_s"]
        out["rpm_output_filt"] = out["rps_output_filt"] * 60.0
    else:
        out["rpm_motor_filt"] = out["rpm_motor_raw"]
        out["rpm_output_filt"] = out["rpm_output_raw"]

    out["rpm_motor_filt"] = out["rpm_motor_filt"].bfill().fillna(0.0)
    out["rpm_output_filt"] = out["rpm_output_filt"].bfill().fillna(0.0)

    out["omega_motor_rad_s"] = out["rpm_motor_filt"] * (2.0 * math.pi / 60.0)
    out["omega_output_rad_s"] = out["rpm_output_filt"] * (2.0 * math.pi / 60.0)

    return out, adc_zero


# =========================================================
# REPORTE
# =========================================================

def imprimir_reporte(df: pd.DataFrame, adc_zero: int):
    fs_mean = 1.0 / df["dt_s"].mean()
    fs_med = 1.0 / df["dt_s"].median()

    print("\n" + "=" * 64)
    print("REPORTE DE RECONSTRUCCION")
    print("=" * 64)
    print(f"Muestras:                   {len(df)}")
    print(f"Duracion total:             {df['t_s'].iloc[-1]:.6f} s")
    print(f"fs promedio:                {fs_mean:.2f} Hz")
    print(f"fs mediana:                 {fs_med:.2f} Hz")
    print(f"adcZero usado:              {adc_zero}")
    print(f"VREF:                       {VREF}")
    print(f"R_SHUNT:                    {R_SHUNT} ohm")
    print(f"GAIN INA240:                {GAIN}")
    print(f"PPR motor:                  {PPR_MOTOR}")
    print(f"Relacion de caja:           1:{GEAR_RATIO}")
    print(f"PPR salida equivalente:     {PPR_OUTPUT}")
    print(f"Ventana vel filtrada:       {VEL_WINDOW_SAMPLES} muestras")
    print("-" * 64)
    print(f"Corriente min/max [mA]:     {df['current_mA'].min():.2f} / {df['current_mA'].max():.2f}")
    print(f"RPM motor max (filtrada):   {df['rpm_motor_filt'].max():.2f}")
    print(f"RPM salida max (filtrada):  {df['rpm_output_filt'].max():.2f}")
    print(f"Omega salida max [rad/s]:   {df['omega_output_rad_s'].max():.2f}")
    print("=" * 64)

    print("\nQUE AJUSTAR SI ALGO NO CUADRA:")
    print("1) Si la corriente sale mal escalada -> revisa GAIN y R_SHUNT.")
    print("2) Si la corriente no arranca cerca de cero -> revisa adcZero.")
    print("3) Si la velocidad sale muy ruidosa -> sube VEL_WINDOW_SAMPLES.")
    print("4) Si la velocidad sale mal escalada -> revisa GEAR_RATIO.")
    print("5) Si fs promedio es muy baja -> revisa baudrate, sketch y carga serial.")


# =========================================================
# GRAFICAS
# =========================================================

def graficar(df: pd.DataFrame, save_path: Path):
    plt.figure(figsize=(10, 4))
    plt.plot(df["t_s"], df["current_mA"])
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Corriente [mA]")
    plt.title("Corriente reconstruida")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(save_path / "corriente.png", dpi=150)
    plt.show()

    plt.figure(figsize=(10, 4))
    plt.plot(df["t_s"], df["rpm_motor_raw"], alpha=0.35, label="RPM motor raw")
    plt.plot(df["t_s"], df["rpm_motor_filt"], label="RPM motor filtrada")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("RPM motor")
    plt.title("Velocidad del motor")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path / "rpm_motor.png", dpi=150)
    plt.show()

    plt.figure(figsize=(10, 4))
    plt.plot(df["t_s"], df["rpm_output_raw"], alpha=0.35, label="RPM salida raw")
    plt.plot(df["t_s"], df["rpm_output_filt"], label="RPM salida filtrada")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("RPM salida")
    plt.title("Velocidad del eje de salida")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path / "rpm_salida.png", dpi=150)
    plt.show()


# =========================================================
# PRUEBA MINIMA DE LECTURA
# =========================================================

def prueba_lectura_rapida(port: str, baudrate: int, timeout: float = 1.0, n_lines: int = 20):
    """
    Funcion opcional para verificar rapido si realmente llegan lineas.
    """
    print("[INFO] Iniciando prueba rapida de lectura...")
    ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
    time.sleep(WAIT_AFTER_OPEN_S)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    for i in range(n_lines):
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        print(f"[TEST {i+1:02d}] {repr(line)}")

    ser.close()
    print("[INFO] Fin de prueba rapida.")


# =========================================================
# MAIN
# =========================================================

def main():
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    raw_csv = OUTPUT_DIR / f"captura_raw_{timestamp}.csv"
    rec_csv = OUTPUT_DIR / f"captura_reconstruida_{timestamp}.csv"

    # Si quieres probar lectura rapida primero, descomenta:
    # prueba_lectura_rapida(PORT, BAUDRATE, TIMEOUT, n_lines=30)
    # return

    df_raw, adc_zero = capturar_serial(
        port=PORT,
        baudrate=BAUDRATE,
        duration_s=DURATION_S,
        timeout=TIMEOUT
    )
    df_raw.to_csv(raw_csv, index=False)
    print(f"[INFO] CSV crudo guardado en: {raw_csv}")

    df_rec, adc_zero = reconstruir_senales(df_raw, adc_zero=adc_zero)
    df_rec.to_csv(rec_csv, index=False)
    print(f"[INFO] CSV reconstruido guardado en: {rec_csv}")

    imprimir_reporte(df_rec, adc_zero)
    graficar(df_rec, OUTPUT_DIR)


if __name__ == "__main__":
    main()