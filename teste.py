# DEU CERTO


import traceback
import time

# Bloco de proteção
try:
    import pybullet as p
    import pybullet_data
    import math
    import random
    import numpy as np
    import json
    import paho.mqtt.client as mqtt
    import os

    # =========================
    # CONFIGURAÇÕES
    # =========================
    MQTT_BROKER = "broker.hivemq.com"
    MQTT_PORT = 1883
    MQTT_TOPIC = "robot/telemetria"

    ROBOT_SCALE = 1.8 
    BASE_SPEED = 7.0        
    MAX_FORCE = 4000
    TURN_GAIN = 5.0         
    SMOOTH_FACTOR = 0.4     

    SENSOR_MAX_DIST = 8.0  
    SENSOR_DELAY = 1        

    DIST_REACTION = 2.2     
    DIST_CRITICAL = 0.9     
    DIST_GOAL_STOP = 1.5    

    WATCHDOG_TIME = 2.0
    RESCUE_FRAMES = 40

    # Mudei a seed para gerar um mapa novo e mais aberto
    random.seed(555)       
    np.random.seed(555)

    # =========================
    # SETUP
    # =========================
    print("=== INICIANDO SIMULAÇÃO (CENÁRIO NAVEGÁVEL) ===")

    client = None
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "HuskyNav")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
    except Exception: pass

    def safe_publish(payload):
        if client: 
            try: client.publish(MQTT_TOPIC, json.dumps(payload))
            except: pass

    def clamp(x, a, b): return max(a, min(b, x))

    if p.isConnected(): p.disconnect()
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(cameraDistance=22, cameraYaw=0, cameraPitch=-89, cameraTargetPosition=[0, 0, 0])

    def create_obs(pos, size, color, angle_deg=0):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
        orn = p.getQuaternionFromEuler([0, 0, math.radians(angle_deg)])
        return p.createMultiBody(0, col, vis, pos, orn)

    WALL_C = [0.2, 0.2, 0.2, 1]
    OBST_C = [0.6, 0.2, 0.2, 1] 
    GOAL_C = [0.0, 1.0, 0.0, 0.7] 

    plane = p.loadURDF("plane.urdf")
    p.changeDynamics(plane, -1, lateralFriction=0.8)
    create_obs([-13, 0, 1], [0.5, 19, 1], WALL_C) 
    create_obs([13, 0, 1],  [0.5, 19, 1], WALL_C) 
    create_obs([0, 19, 1],  [13.5, 0.5, 1], WALL_C) 
    create_obs([0, -13, 1], [13.5, 0.5, 1], WALL_C) 

    # --- GERAÇÃO OTIMIZADA (MAIS ESPAÇO) ---
    print("Gerando obstáculos com corredores...")
    placed_blocks = []
    attempts = 0
    start_area = [0, -10]
    goal_area = [0, 14]

    def is_area_clear(x, y, margin):
        # Protege corredor central inicial e final
        if math.hypot(x - start_area[0], y - start_area[1]) < margin + 3.0: return False
        if math.hypot(x - goal_area[0], y - goal_area[1]) < margin + 3.0: return False
        return True

    # Reduzi para 22 obstáculos (menos densidade)
    while len(placed_blocks) < 22 and attempts < 5000:
        attempts += 1
        
        # Blocos menores (Máx 1.2m em vez de 3.0m)
        sx, sy = random.uniform(0.8, 1.2), random.uniform(0.8, 1.2)
            
        rx, ry = random.uniform(-9, 9), random.uniform(-8, 15)
        
        if not is_area_clear(rx, ry, max(sx, sy)): continue
        
        too_close = False
        for (ex, ey, esx, esy) in placed_blocks:
            # Aumentei a margem mínima entre blocos para 2.5m
            # O robô tem ~1.0m de largura, então sobra 1.5m de folga
            dist_centers = math.hypot(rx-ex, ry-ey)
            min_gap = (max(sx, sy) + max(esx, esy)) + 2.5 
            if dist_centers < min_gap:
                too_close = True; break
        if too_close: continue

        create_obs([rx, ry, 1.0], [sx, sy, 1.0], OBST_C, random.uniform(-45, 45))
        placed_blocks.append((rx, ry, sx, sy))

    # Robô
    GOAL_POS = goal_area
    p.createMultiBody(0, p.createCollisionShape(p.GEOM_CYLINDER, radius=1.0, height=0.1),
                      p.createVisualShape(p.GEOM_CYLINDER, radius=1.0, length=0.1, rgbaColor=GOAL_C),
                      [GOAL_POS[0], GOAL_POS[1], 0.05])

    start_pos = [start_area[0], start_area[1], 0.6]
    husky_urdf = os.path.join(pybullet_data.getDataPath(), "husky/husky.urdf")
    robot = p.loadURDF(husky_urdf, start_pos, p.getQuaternionFromEuler([0, 0, 1.57]), globalScaling=ROBOT_SCALE)

    for i in range(p.getNumJoints(robot)):
        p.changeDynamics(robot, i, mass=0.5*ROBOT_SCALE, lateralFriction=1.0) 
        p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, force=0)
    p.changeDynamics(robot, -1, mass=5.0*ROBOT_SCALE) 

    left_joints, right_joints = [2, 4], [3, 5]
    motor_l_buf, motor_r_buf = [0.0]*1, [0.0]*1

    last_pos = start_pos[:]
    last_time = time.time()
    last_log = time.time()
    rescue_cnt = 0
    prev_vl, prev_vr, smooth_turn = 0.0, 0.0, 0.0
    metric_dist, metric_col = 0.0, 0
    metric_dist_no_crash = 0.0
    col_cd = 0
    mission_complete = False
    avoid_side_lock = 0 
    avoid_timer = 0

    def get_yaw(orn): return p.getEulerFromQuaternion(orn)[2]
    def noisy(d): return max(0.1, d + np.random.normal(0, 0.02))

    # =========================
    # LOOP
    # =========================
    print("Simulação rodando...")
    while True:
        now = time.time()
        pos, orn = p.getBasePositionAndOrientation(robot)
        current_yaw = get_yaw(orn) 

        d_step = math.hypot(pos[0]-last_pos[0], pos[1]-last_pos[1])
        metric_dist += d_step
        metric_dist_no_crash += d_step
        last_pos = pos[:]

        if col_cd > 0: col_cd -= 1
        if len(p.getContactPoints(robot)) > 4 and col_cd == 0:
                for c in p.getContactPoints(robot):
                    if c[2] != plane: 
                        metric_col += 1; col_cd = 50; metric_dist_no_crash = 0.0
                        break

        # Sensores Wide
        rot = p.getMatrixFromQuaternion(orn)
        fwd = [rot[0], rot[3], rot[6]]
        rgt = [rot[1], rot[4], rot[7]]
        
        offset_lat, offset_fwd, height = 0.7, 0.8, 0.4
        nose_x, nose_y = pos[0] + fwd[0]*offset_fwd, pos[1] + fwd[1]*offset_fwd
        
        origins = [
            [nose_x - rgt[0]*offset_lat, nose_y - rgt[1]*offset_lat, pos[2]+height], 
            [nose_x,                     nose_y,                     pos[2]+height], 
            [nose_x + rgt[0]*offset_lat, nose_y + rgt[1]*offset_lat, pos[2]+height]  
        ]
        targets = [[o[0]+fwd[0]*SENSOR_MAX_DIST, o[1]+fwd[1]*SENSOR_MAX_DIST, o[2]] for o in origins]

        readings = []
        if int(metric_dist*10) % 3 == 0: p.removeAllUserDebugItems()

        res = p.rayTestBatch([origins[0], origins[1], origins[2]], targets)
        for i, r in enumerate(res):
            val = noisy(r[2] * SENSOR_MAX_DIST)
            readings.append(val)
            if int(metric_dist*10) % 3 == 0:
                clr = [1,0,0] if val < DIST_CRITICAL else ([1,1,0] if val < DIST_REACTION else [0,1,0])
                p.addUserDebugLine(origins[i], [origins[i][0]+fwd[0]*val, origins[i][1]+fwd[1]*val, origins[i][2]], clr)

        d_R, d_C, d_L = readings 
        min_d = min(readings)

        # Lógica
        dx, dy = GOAL_POS[0]-pos[0], GOAL_POS[1]-pos[1]
        dist_to_goal = math.hypot(dx, dy)
        goal_heading = math.atan2(dy, dx)
        goal_err = math.atan2(math.sin(goal_heading - current_yaw), math.cos(goal_heading - current_yaw))

        if dist_to_goal < DIST_GOAL_STOP or mission_complete:
            mission_complete = True
            status = "CHEGOU!"
            vl, vr = 0, 0
            p.addUserDebugText("CHEGOU!", [pos[0], pos[1], pos[2]+1.5], [0,1,0], 0.3)
        
        else:
            if now - last_time > WATCHDOG_TIME:
                if math.hypot(pos[0]-start_pos[0], pos[1]-start_pos[1]) > 1.0: 
                    if math.hypot(pos[0]-last_pos[0], pos[1]-last_pos[1]) < 0.05:
                        rescue_cnt = RESCUE_FRAMES
                last_time = now

            if rescue_cnt > 0:
                status = "RESGATE"
                if rescue_cnt > 15: vl, vr = -7.0, -7.0 
                else: vl, vr = -7.0, 7.0 
                rescue_cnt -= 1
                smooth_turn = 0; avoid_side_lock = 0
            
            else:
                # Beco
                if d_L < 1.0 and d_C < 1.0 and d_R < 1.0:
                    status = "BECO"
                    raw_turn = -TURN_GAIN * 1.5; target_speed = -1.0; avoid_side_lock = 0

                # Desvio
                elif min_d < DIST_REACTION:
                    status = "DESVIANDO"
                    if avoid_timer > 0:
                        side_sign = avoid_side_lock
                        avoid_timer -= 1
                    else:
                        score_L = d_L + (d_C * 0.5)
                        score_R = d_R + (d_C * 0.5)
                        if score_L > score_R: side_sign = 1.0; avoid_side_lock = 1.0
                        else: side_sign = -1.0; avoid_side_lock = -1.0
                        avoid_timer = 20 

                    prox = clamp((DIST_REACTION - min_d) / (DIST_REACTION - DIST_CRITICAL), 0.0, 1.0)
                    raw_turn = side_sign * TURN_GAIN * prox
                    target_speed = BASE_SPEED * (1.0 - prox * 0.7)

                # Livre
                else:
                    status = "LIVRE"
                    raw_turn = goal_err * 2.2
                    target_speed = BASE_SPEED
                    avoid_timer = 0 

                smooth_turn = smooth_turn * SMOOTH_FACTOR + raw_turn * (1 - SMOOTH_FACTOR)
                vl = target_speed - smooth_turn
                vr = target_speed + smooth_turn

        vl = clamp(vl, -15, 15); vr = clamp(vr, -15, 15)
        motor_l_buf.append(vl); motor_r_buf.append(vr)
        cmd_l, cmd_r = motor_l_buf.pop(0), motor_r_buf.pop(0)

        for j in left_joints: p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=cmd_l, force=MAX_FORCE)
        for j in right_joints: p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=cmd_r, force=MAX_FORCE)

        if now - last_log > 0.15: 
            reaction = (SENSOR_DELAY + 1) * (1./240.)
            p1, p2 = np.array(start_area), np.array(goal_area)
            if np.linalg.norm(p2-p1) > 0:
                lat_err = np.abs(np.cross(p2-p1, p1-np.array([pos[0],pos[1]]))) / np.linalg.norm(p2-p1)
            else: lat_err = 0

            safe_publish({
                "status": status, "colisoes": metric_col, 
                "dist_sem_impacto": round(metric_dist_no_crash, 2),
                "tempo_reacao_s": round(reaction, 3), "erro_lat_medio": round(lat_err, 2),
                "sensores": [round(x,1) for x in readings], "motor": [round(cmd_l,1), round(cmd_r,1)]
            })
            last_log = now
            if not mission_complete:
                p.addUserDebugText(f"Goal: {dist_to_goal:.1f}m", [pos[0], pos[1], pos[2]+1.0], [0,0,0], 0.2)
                p.addUserDebugLine([pos[0],pos[1],0.5], [GOAL_POS[0],GOAL_POS[1],0.5], [0,0,1], 1.0, 0.1)

        p.stepSimulation()
        time.sleep(1.0/240.0)

except KeyboardInterrupt:
    print("Parada solicitada.")

except Exception as e:
    print("\nERRO FATAL:", e)
    traceback.print_exc()

finally:
    print("Fim.")
    input(">>> ENTER para fechar <<<")
    if client: 
        try: client.loop_stop(); client.disconnect()
        except: pass
    if p.isConnected(): p.disconnect()