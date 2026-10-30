"""
Teleoperacion interactiva standalone para el robot humanoide Unitree G1 (Windows/Host).
Permite comandar presets (wave, squat, bow, tpose, stand) y articulaciones individuales
en vivo dentro del visor 3D de MuJoCo sin necesidad de ROS 2.

Ejecutar con:
    python run_teleop_standalone.py
"""
import time
import math
import threading
import numpy as np
import mujoco
import mujoco.viewer

ACTUATOR_NAMES = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'
]

# Estado global compartido con el hilo de entrada de consola
CURRENT_PRESET = "stand"
CUSTOM_CTRL = np.zeros(len(ACTUATOR_NAMES), dtype=float)
RUNNING = True
LOCK = threading.Lock()

def user_input_thread():
    global CURRENT_PRESET, RUNNING, CUSTOM_CTRL
    print("\n========================================================")
    print("   Consola de Teleoperacion Interactiva - Unitree G1   ")
    print("========================================================")
    print("Comandos disponibles en tiempo real:")
    print("  wave   -> Saludo fluido con mano derecha alzada")
    print("  squat  -> Sentadilla manteniendo centro de masa")
    print("  bow    -> Reverencia con la cintura")
    print("  tpose  -> Postura en T (brazos horizontales)")
    print("  stand  -> Postura de pie de reposo")
    print("  list   -> Listar las 29 articulaciones y sus valores")
    print("  set <indice> <radianes> -> Comandar una articulacion especifica")
    print("  q      -> Salir\n")

    while RUNNING:
        try:
            line = input("Unitree-G1 > ").strip()
            if not line:
                continue
            if line.lower() == 'q':
                RUNNING = False
                break
            elif line.lower() == 'list':
                with LOCK:
                    print("\nArticulaciones disponibles:")
                    for i, name in enumerate(ACTUATOR_NAMES):
                        print(f" [{i:2d}] {name:28s}: {CUSTOM_CTRL[i]:.2f} rad")
                    print()
            elif line.lower().startswith('set '):
                parts = line.split()
                if len(parts) == 3:
                    try:
                        idx = int(parts[1])
                        val = float(parts[2])
                        if 0 <= idx < len(ACTUATOR_NAMES):
                            with LOCK:
                                CURRENT_PRESET = "custom"
                                CUSTOM_CTRL[idx] = val
                            print(f">> Articulacion [{idx}] {ACTUATOR_NAMES[idx]} = {val:.2f} rad")
                        else:
                            print(f"Indice fuera de rango (0 a {len(ACTUATOR_NAMES)-1})")
                    except ValueError:
                        print("Uso: set <indice_entero> <valor_float>")
            elif line.lower() in ['wave', 'squat', 'bow', 'tpose', 'stand']:
                with LOCK:
                    CURRENT_PRESET = line.lower()
                print(f">> Modo activo: {CURRENT_PRESET.upper()}")
            else:
                print(f"Comando no reconocido. Presets: wave, squat, bow, tpose, stand, list, set <idx> <rad>, q")
        except (EOFError, KeyboardInterrupt):
            RUNNING = False
            break

def main():
    global RUNNING, CUSTOM_CTRL
    model_path = "models/unitree_g1/scene.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Inicializar en el keyframe oficial de postura de pie ('stand')
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        default_ctrl = np.copy(model.key(0).ctrl)
    else:
        default_ctrl = np.zeros(model.nu)

    with LOCK:
        CUSTOM_CTRL = np.copy(default_ctrl)

    # Iniciar hilo de lectura de consola
    input_t = threading.Thread(target=user_input_thread, daemon=True)
    input_t.start()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 2.1
            viewer.cam.elevation = -8
            viewer.cam.azimuth = 145
            viewer.cam.lookat[:] = [0.0, -0.1, 0.95]

            step_count = 0
            while viewer.is_running() and RUNNING:
                step_start = time.time()
                t = data.time

                with LOCK:
                    preset = CURRENT_PRESET
                    ctrl_copy = np.copy(CUSTOM_CTRL)

                # Generar consignas segun preset
                target_ctrl = np.copy(default_ctrl)

                if preset == "wave":
                    # Saludo fluido
                    wave_freq = 4.5
                    target_ctrl[22] = -1.4                       # right_shoulder_pitch
                    target_ctrl[23] = -1.3                       # right_shoulder_roll
                    target_ctrl[24] = 0.0                        # right_shoulder_yaw
                    target_ctrl[25] = 0.6 + 0.35 * math.sin(wave_freq * t)  # right_elbow
                    target_ctrl[26] = -0.8                       # right_wrist_roll
                    target_ctrl[27] = 0.45 * math.sin(wave_freq * t)        # right_wrist_pitch

                elif preset == "squat":
                    flex = 0.18 * (1.0 - math.cos(2.0 * t))
                    target_ctrl[0] -= flex
                    target_ctrl[3] += 2.0 * flex
                    target_ctrl[4] -= flex
                    target_ctrl[6] -= flex
                    target_ctrl[9] += 2.0 * flex
                    target_ctrl[10] -= flex

                elif preset == "bow":
                    target_ctrl[14] = 0.25 * (1.0 - math.cos(1.5 * t))

                elif preset == "tpose":
                    target_ctrl[16] = 1.57   # left_shoulder_roll
                    target_ctrl[23] = -1.57  # right_shoulder_roll
                    target_ctrl[18] = 0.0    # left_elbow
                    target_ctrl[25] = 0.0    # right_elbow

                elif preset == "stand":
                    target_ctrl = np.copy(default_ctrl)

                elif preset == "custom":
                    target_ctrl = ctrl_copy

                # Filtro de suavizado exponencial hacia el objetivo (evita movimientos bruscos)
                data.ctrl[:] = 0.92 * data.ctrl[:] + 0.08 * target_ctrl

                # Paso de fisica
                mujoco.mj_step(model, data)

                step_count += 1
                if step_count % 8 == 0:
                    viewer.sync()

                # Control de frecuencia de tiempo real
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    finally:
        RUNNING = False

if __name__ == "__main__":
    main()
