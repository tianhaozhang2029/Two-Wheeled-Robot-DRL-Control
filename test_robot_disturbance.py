# ====================================================================
# Classic PID - Ultimate Disturbance Video Demo Script
# ====================================================================
import pybullet as p
import pybullet_data
import time
import os
import math

# Force disconnect to prevent GUI lockups from previous runs
try: p.disconnect()
except: pass

# --- 🎬 戏剧效果控制台 (Director's Console - MATCHED TO V41) ---
KICK_FORCE_1 = 600      # Step 600: Frontal Push
KICK_FORCE_2 = 200      # Step 1400: Lateral Kick
KICK_HEIGHT = 0.15    
target_v = 0.24         # Cruising target speed (m/s)
target_yaw = 0.0        # [CRUCIAL FIX] Changed to 0.0 to match V41 exactly
# -------------------------------------------------------------

# 1. Environment Initialization
physicsClient = p.connect(p.GUI, options="--width=560 --height=700")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
TIME_STEP = 1/240
p.setTimeStep(TIME_STEP)

# Visual optimizations and initial camera angle
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.2])

# 2. Load Model (Compatible with different execution paths)
p.loadURDF("plane.urdf")
try:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "balance_bot.urdf")
    robotId = p.loadURDF(urdf_path, [0, 0, 0.1])
except NameError:
    robotId = p.loadURDF("balance_bot.urdf", [0, 0, 0.1])

# 3. Joint Initialization
wheel_indices = [0, 1]
for j in range(p.getNumJoints(robotId)):
    p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, force=0)

# --- 4. Optimized Cascaded PID Parameters ---
# A. Outer Velocity Loop
Kp_vel = 0.2     
Ki_vel = 0.005   
Kd_vel = 0.1     

# B. Inner Upright (Pitch) Loop
Kp_pitch = 40.0
Kd_pitch = 5.0

# C. Steering (Yaw) Loop
Kp_yaw = 1.0
Kd_yaw = 0.1

# State Variables
integral_vel = 0
last_v_error = 0
filtered_v = 0   
alpha = 0.1      
step_count = 0

# 5. Simulation Loop
try:
    print("\n" + "="*65)
    print(f">>> 🎬 PID DISTURBANCE TEST | Target Vel: {target_v:.2f} m/s | Target Yaw: {target_yaw:.2f} rad")
    print("="*65 + "\n")
    
    while True:
        # --- A. Acquire State and Velocity Filtering ---
        pos, orn = p.getBasePositionAndOrientation(robotId)
        euler = p.getEulerFromQuaternion(orn)
        current_pitch = euler[1]
        current_yaw = euler[2]

        linear_vel, angular_vel = p.getBaseVelocity(robotId)
        
        # Use forward velocity in the body frame
        raw_v_forward = linear_vel[0] * math.cos(current_yaw) + linear_vel[1] * math.sin(current_yaw)
        
        # Low-pass filter
        filtered_v = alpha * raw_v_forward + (1 - alpha) * filtered_v
        
        pitch_rate = angular_vel[1]
        yaw_rate = angular_vel[2]

        # --- B. Outer Velocity Loop Calculation (Cascaded Velocity Control) ---
        v_error = target_v - filtered_v
        integral_vel += v_error
        # Integral anti-windup clamping
        integral_vel = max(min(integral_vel, 10.0), -10.0)
        
        v_derivative = v_error - last_v_error
        last_v_error = v_error

        # Output desired pitch angle
        desired_pitch = (Kp_vel * v_error + Ki_vel * integral_vel + Kd_vel * v_derivative)
        # Strictly limit pitch angle to prevent flipping while chasing speed
        desired_pitch = max(min(desired_pitch, 0.2), -0.2) 

        # --- C. Inner Upright Loop and Steering Loop ---
        pitch_error = current_pitch - desired_pitch
        base_torque = (Kp_pitch * pitch_error) + (Kd_pitch * pitch_rate)
        
        # Calculate yaw error using the shortest path to prevent multi-turn windup
        yaw_error = math.atan2(math.sin(current_yaw - target_yaw), math.cos(current_yaw - target_yaw))
        steer_torque = (Kp_yaw * yaw_error) + (Kd_yaw * yaw_rate)

        # --- D. Execution ---
        torque_l = base_torque + steer_torque
        torque_r = base_torque - steer_torque
        
        # Motor physical limits clipping
        torque_l = max(min(torque_l, 25.0), -25.0)
        torque_r = max(min(torque_r, 25.0), -25.0)

        p.setJointMotorControlArray(
            bodyUniqueId=robotId,
            jointIndices=wheel_indices,
            controlMode=p.TORQUE_CONTROL,
            forces=[torque_l, torque_r]
        )

        p.stepSimulation()
        
        # --- E. Visual Tracking & Disturbance Injection ---
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5, 
            cameraYaw=45, 
            cameraPitch=-20, # Matched to V41 camera angle
            cameraTargetPosition=[pos[0], pos[1], 0.2]
        )
        
        # ==========================================
        # 💥 External Disturbance Injection (MATCHED TO V41)
        # ==========================================
        if step_count == 400:
            p.applyExternalForce(objectUniqueId=robotId, linkIndex=-1,
                                 forceObj=[-KICK_FORCE_1, 0, 0], posObj=[0, 0, KICK_HEIGHT],
                                 flags=p.WORLD_FRAME)
            print("\n" + "⚠️"*15)
            print(f"💥 BOOM! [Step 400] FRONTAL PUSH ({KICK_FORCE_1}N @ Z={KICK_HEIGHT}m) INJECTED!")
            print("⚠️"*15 + "\n")
            
        elif step_count == 800:
            p.applyExternalForce(objectUniqueId=robotId, linkIndex=-1,
                                 forceObj=[0, KICK_FORCE_2, 0], posObj=[0, 0, KICK_HEIGHT],
                                 flags=p.WORLD_FRAME)
            print("\n" + "⚠️"*15)
            print(f"💥 BOOM! [Step 800] LATERAL KICK ({KICK_FORCE_2}N @ Z={KICK_HEIGHT}m) INJECTED!")
            print("⚠️"*15 + "\n")

        if step_count % 120 == 0:
            print_yaw_err_rad = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
            print_yaw_err_deg = math.degrees(print_yaw_err_rad)
            print(f"Step {step_count:4d} | Speed: {filtered_v:>5.2f} / {target_v:>5.2f} | Yaw Error: {print_yaw_err_deg:>6.1f}° | Status: ⚙️ Classic PID Control")
            
        step_count += 1
        time.sleep(TIME_STEP)
        
        # Safety break if the robot falls over completely
        if abs(current_pitch) > 1.0:
            print("\n[FATAL] Robot has fallen over! PID failed to recover from disturbance.")
            time.sleep(1.5) 
            break

except KeyboardInterrupt:
    pass
finally:
    print("\nCleaning up physics engine...")
    try: p.disconnect()
    except: pass
    print("Environment closed.")