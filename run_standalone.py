"""
Standalone MuJoCo runner para el robot humanoide Unitree G1 (UADE).
Permite verificar la fisica y visualizacion 3D de inmediato sin ROS 2.

Ejecutar con:
    python run_standalone.py
"""
import time
import math
import numpy as np
import mujoco
import mujoco.viewer

def main():
    model_path = "models/unitree_g1/scene.xml"
    print("============================================================")
    print(f" Cargando modelo del Humanoide Unitree G1: {model_path}")
    print("============================================================")
    
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print(f" - Coordenadas generalizadas (nq): {model.nq}")
    print(f" - Grados de libertad (nv): {model.nv}")
    print(f" - Actuadores / Motores (nu): {model.nu}")
    print(f" - Cuerpos / Eslabones: {model.nbody}")

    # Inicializar en el keyframe oficial de postura de pie ('stand')
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        default_ctrl = np.copy(model.key(0).ctrl)
        data.ctrl[:] = default_ctrl
        print(f" - Inicializado en keyframe: '{model.key(0).name}' (Altura: {data.qpos[2]:.2f}m)")
    else:
        default_ctrl = np.zeros(model.nu)

    # Identificar indices de actuadores del brazo derecho
    # 22: right_shoulder_pitch, 23: right_shoulder_roll, 24: right_shoulder_yaw
    # 25: right_elbow, 26: right_wrist_roll, 27: right_wrist_pitch, 28: right_wrist_yaw
    actuator_names = [model.actuator(i).name for i in range(model.nu)]
    try:
        r_sh_p_idx = actuator_names.index('right_shoulder_pitch_joint')
        r_sh_r_idx = actuator_names.index('right_shoulder_roll_joint')
        r_sh_y_idx = actuator_names.index('right_shoulder_yaw_joint')
        r_el_idx = actuator_names.index('right_elbow_joint')
        r_w_r_idx = actuator_names.index('right_wrist_roll_joint')
        r_w_p_idx = actuator_names.index('right_wrist_pitch_joint')
        r_w_y_idx = actuator_names.index('right_wrist_yaw_joint')
    except ValueError:
        r_sh_p_idx, r_sh_r_idx, r_sh_y_idx = 22, 23, 24
        r_el_idx, r_w_r_idx, r_w_p_idx, r_w_y_idx = 25, 26, 27, 28

    print("\nIniciando visor interactivo 3D...")
    print("Controles del visor de MuJoCo:")
    print(" - Click izquierdo + arrastrar: Rotar camara")
    print(" - Click derecho + arrastrar: Desplazar camara (pan)")
    print(" - Rueda del raton: Zoom")
    print(" - Doble click en un eslabon + click derecho: Aplicar fuerzas interactivas con el cursor")
    print(" - Espacio: Pausar / reanudar simulacion")
    print("Presiona Ctrl+C en la consola o cierra la ventana para salir.\n")

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Configurar camara inicial mirando al humanoide de frente a la altura del pecho
            viewer.cam.distance = 2.1
            viewer.cam.elevation = -8
            viewer.cam.azimuth = 145
            viewer.cam.lookat[:] = [0.0, -0.1, 0.95]

            step_count = 0
            while viewer.is_running():
                step_start = time.time()
                t = data.time

                # Mantener postura base de pie en piernas, cintura y brazo izquierdo
                current_ctrl = np.copy(default_ctrl)

                # Transicion suave de elevacion del brazo durante los primeros 1.5 segundos
                alpha = min(1.0, t / 1.5)

                # Postura cinematica de saludo (Waving):
                # 1. Hombro elevado hacia arriba y adelante: pitch = -1.4 rad
                # 2. Hombro separado lateralmente: roll = -1.3 rad
                # 3. Codo flexionado apuntando el antebrazo y mano hacia el cielo: ~0.6 rad
                # 4. Palma orientada hacia el frente: wrist_roll = -0.8 rad
                target_pitch = -1.4
                target_roll = -1.3
                target_yaw = 0.0
                target_w_roll = -0.8

                # Oscilacion de vaiven del antebrazo y la mano a los lados de la cabeza
                wave_freq = 4.5  # Hz / rad
                forearm_wave = 0.35 * math.sin(wave_freq * t)
                hand_flap = 0.45 * math.sin(wave_freq * t)

                target_elbow = 0.6 + forearm_wave
                target_w_pitch = hand_flap

                # Interpolar suavemente desde la postura de reposo
                current_ctrl[r_sh_p_idx] = (1.0 - alpha) * default_ctrl[r_sh_p_idx] + alpha * target_pitch
                current_ctrl[r_sh_r_idx] = (1.0 - alpha) * default_ctrl[r_sh_r_idx] + alpha * target_roll
                current_ctrl[r_sh_y_idx] = (1.0 - alpha) * default_ctrl[r_sh_y_idx] + alpha * target_yaw
                current_ctrl[r_el_idx] = (1.0 - alpha) * default_ctrl[r_el_idx] + alpha * target_elbow
                current_ctrl[r_w_r_idx] = (1.0 - alpha) * default_ctrl[r_w_r_idx] + alpha * target_w_roll
                current_ctrl[r_w_p_idx] = (1.0 - alpha) * default_ctrl[r_w_p_idx] + alpha * target_w_pitch

                # Asignar a actuadores de MuJoCo
                data.ctrl[:] = current_ctrl

                # Paso de fisica
                mujoco.mj_step(model, data)

                # Sincronizar visor a ~60 Hz
                step_count += 1
                if step_count % 8 == 0:
                    viewer.sync()

                # Control de frecuencia de tiempo real
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    except Exception as e:
        print(f"Visor grafico no disponible o modo headless: {e}")
        print("Ejecutando test en modo headless por 1000 pasos de simulacion...")
        for _ in range(1000):
            t = data.time
            data.ctrl[:] = default_ctrl
            data.ctrl[r_sh_p_idx] = -1.4
            data.ctrl[r_sh_r_idx] = -1.3
            data.ctrl[r_el_idx] = 0.6 + 0.3 * math.sin(4.5 * t)
            data.ctrl[r_w_p_idx] = 0.4 * math.sin(4.5 * t)
            mujoco.mj_step(model, data)
        print("Test headless completado exitosamente.")
        print(f"Tiempo simulado: {data.time:.2f}s | Altura final del robot: {data.qpos[2]:.3f} m")

if __name__ == "__main__":
    main()
