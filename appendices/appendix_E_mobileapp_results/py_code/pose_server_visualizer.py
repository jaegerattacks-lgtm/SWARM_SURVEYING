import socket
import json
import csv
import time
import math
import matplotlib.pyplot as plt
from datetime import datetime
import sys

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
TIMESTAMP = int(time.time())
FILENAME = f"robot_data_{TIMESTAMP}.csv"
IMAGE_NAME = f"robot_graphs_{TIMESTAMP}.png"

# 🎯 SENSITIVITY SETTINGS
MOVEMENT_THRESHOLD = 0.1 # 0.02 == 2cm

# --- SETUP SOCKET ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) 

print(f"✅ Server Started on Port {UDP_PORT}")
print(f"✅ Data will be saved to: {FILENAME}")
print(f"✅ Graphs will be saved to: {IMAGE_NAME}")
print("👉 Move your phone. The plots will update live.")
print("🔴 To STOP: Close the Plot Window (Click X) or Press Ctrl+C repeatedly.")

# --- SETUP PLOTS (2x2 Grid) ---
plt.ion() 
fig, axs = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle("Live ARKit Robot Telemetry", fontsize=16)

ax_path = axs[0, 0]   # Path (X vs Z)
ax_z = axs[0, 1]      # Z vs Time
ax_y = axs[1, 0]      # Height (Y vs Time)
ax_x = axs[1, 1]      # Drift (X vs Time)

# Lists
times = []
x_vals = []
y_vals = []
z_vals = []

# --- EXIT HANDLING ---
is_running = True

def on_close(event):
    global is_running
    print("\n🛑 Window Closed. Stopping...")
    is_running = False

fig.canvas.mpl_connect('close_event', on_close)

# --- STATE VARIABLES ---
start_time = None
last_saved_pos = None 

try:
    with open(FILENAME, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Pos_X", "Pos_Y", "Pos_Z", "Rot_X", "Rot_Y", "Rot_Z", "Rot_W", "Obstacle_Dist"])
        
        while is_running:
            # Double check if window exists
            if not plt.fignum_exists(fig.number):
                is_running = False
                break

            try:
                try:
                    data, addr = sock.recvfrom(1024)
                except BlockingIOError:
                    plt.pause(0.01) 
                    continue
                
                pose = json.loads(data.decode('utf-8'))
                
                ts = pose['timestamp']
                px, py, pz = pose['position']
                qx, qy, qz, qw = pose['orientation']
                obs_dist = pose.get('obstacle_dist', 0)

                if start_time is None:
                    start_time = ts

                rel_time = ts - start_time
                
                # Check Threshold
                should_save = False
                if last_saved_pos is None:
                    should_save = True
                else:
                    lx, ly, lz = last_saved_pos
                    dist = math.sqrt((px - lx)**2 + (py - ly)**2 + (pz - lz)**2)
                    if dist > MOVEMENT_THRESHOLD:
                        should_save = True
                
                # Save & Plot
                if should_save:
                    writer.writerow([ts, px, py, pz, qx, qy, qz, qw, obs_dist])
                    file.flush()
                    
                    times.append(rel_time)
                    x_vals.append(px)
                    y_vals.append(py)
                    z_vals.append(pz)
                    last_saved_pos = [px, py, pz]

                    # Plotting
                    ax_path.clear()
                    ax_z.clear()
                    ax_y.clear()
                    ax_x.clear()

                    # 1. Path
                    ax_path.plot(x_vals, z_vals, 'b-', label='Path')
                    if x_vals:
                        ax_path.plot(x_vals[0], z_vals[0], 'go', label='Start')
                        ax_path.plot(x_vals[-1], z_vals[-1], 'ro', label='Cur')
                    ax_path.set_title("Top Down (X vs Z)")
                    ax_path.set_xlabel("X (Side)")
                    ax_path.set_ylabel("Z (Forward)")
                    ax_path.grid(True)
                    ax_path.legend()
                    ax_path.axis('equal')

                    # 2. Z vs Time
                    ax_z.plot(times, z_vals, 'm-')
                    ax_z.set_title("Forward Motion (Z vs Time)")
                    ax_z.grid(True)

                    # 3. Y vs Time
                    ax_y.plot(times, y_vals, 'g-')
                    ax_y.set_title("Height (Y vs Time)")
                    ax_y.grid(True)

                    # 4. X vs Time
                    ax_x.plot(times, x_vals, 'r-')
                    ax_x.set_title("Lateral Drift (X vs Time)")
                    ax_x.grid(True)

                    plt.pause(0.001) 

            except json.JSONDecodeError:
                pass
            except KeyboardInterrupt:
                print("\n🛑 Ctrl+C Detected!")
                is_running = False

except KeyboardInterrupt:
    print("\n🛑 Ctrl+C Detected (Outer)!")
finally:
    print(f"\n✅ Loop Finished. CSV saved to {FILENAME}")
    
    # --- AUTO-SAVE IMAGE LOGIC ---
    try:
        print(f"💾 Saving final graph to {IMAGE_NAME}...")
        fig.savefig(IMAGE_NAME)
        print("✅ Graph Saved Successfully!")
    except Exception as e:
        print(f"❌ Error saving graph: {e}")

    # Keep window open for inspection
    if plt.fignum_exists(fig.number):
        print("👀 Close the window manually to exit script completely.")
        plt.ioff()
        plt.show()