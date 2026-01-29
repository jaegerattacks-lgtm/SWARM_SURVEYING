import socket
import json
import csv
import time
import math
import matplotlib.pyplot as plt
from datetime import datetime
import sys
import threading

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
TIMESTAMP = int(time.time())
FILENAME = f"robot_data_{TIMESTAMP}.csv"
IMAGE_NAME = f"robot_graphs_{TIMESTAMP}.png"

# 🎯 SENSITIVITY SETTINGS
MOVEMENT_THRESHOLD = 0.1  # 0.1 meters (10cm)
OBSTACLE_WARNING_DIST = 0.5 # 0.5 meters (50cm)

# --- SETUP SOCKET ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) 

print(f"✅ Server Started on Port {UDP_PORT}")
print(f"✅ Data will be saved to: {FILENAME}")
print("👉 Move your phone. The plots will update live.")
print("⌨️  COMMANDS: Type 's' + Enter to START (Reset Origin). Type 'q' + Enter to QUIT.")

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

# --- STATE VARIABLES ---
is_running = True
phone_address = None # We store the phone IP here once connected
start_time = None
last_saved_pos = None 

# --- INPUT THREAD (Listens for 's' and 'q') ---
def input_listener():
    global is_running, phone_address
    while is_running:
        try:
            # This waits for user input in the terminal
            cmd = input()
            
            if cmd.lower() == 's':
                if phone_address:
                    print("🚀 Sending START command to phone...")
                    sock.sendto("START".encode('utf-8'), phone_address)
                else:
                    print("⚠️ Wait for phone to connect first!")
            
            elif cmd.lower() == 'q':
                print("🛑 Sending STOP command...")
                if phone_address:
                    sock.sendto("STOP".encode('utf-8'), phone_address)
                is_running = False
                
        except EOFError:
            break

# Start the input thread
threading.Thread(target=input_listener, daemon=True).start()

# --- WINDOW CLOSE HANDLING ---
def on_close(event):
    global is_running
    print("\n🛑 Window Closed. Stopping...")
    is_running = False

fig.canvas.mpl_connect('close_event', on_close)

try:
    with open(FILENAME, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Pos_X", "Pos_Y", "Pos_Z", "Rot_X", "Rot_Y", "Rot_Z", "Rot_W", "Obstacle_Dist"])
        
        while is_running:
            if not plt.fignum_exists(fig.number):
                is_running = False
                break

            try:
                # 1. Receive Data
                try:
                    data, addr = sock.recvfrom(1024)
                    
                    # Store Phone Address for sending commands back
                    if phone_address is None:
                        phone_address = addr
                        print(f"📱 Phone Connected: {addr}")
                        
                except BlockingIOError:
                    plt.pause(0.01) 
                    continue
                
                pose = json.loads(data.decode('utf-8'))
                
                ts = pose['timestamp']
                px, py, pz = pose['position']
                qx, qy, qz, qw = pose['orientation']
                obs_dist = pose.get('obstacle_dist', 10.0) # Default to far if missing

                if start_time is None:
                    start_time = ts

                rel_time = ts - start_time
                
                # 2. Object Detection Logic
                has_object = False
                if obs_dist < OBSTACLE_WARNING_DIST and obs_dist > 0:
                    has_object = True
                    print(f"⚠️ OBJECT DETECTED: {obs_dist:.2f}m")

                # 3. Check Threshold
                should_save = False
                if last_saved_pos is None:
                    should_save = True
                else:
                    lx, ly, lz = last_saved_pos
                    dist = math.sqrt((px - lx)**2 + (py - ly)**2 + (pz - lz)**2)
                    if dist > MOVEMENT_THRESHOLD:
                        should_save = True
                
                # 4. Save & Plot
                if should_save:
                    writer.writerow([ts, px, py, pz, qx, qy, qz, qw, obs_dist])
                    file.flush()
                    
                    times.append(rel_time)
                    x_vals.append(px)
                    y_vals.append(py)
                    z_vals.append(pz)
                    last_saved_pos = [px, py, pz]

                    # Clear Plots
                    ax_path.clear()
                    ax_z.clear()
                    ax_y.clear()
                    ax_x.clear()

                    # Plot 1: Path
                    ax_path.plot(x_vals, z_vals, 'b-', label='Path')
                    if x_vals:
                        ax_path.plot(x_vals[0], z_vals[0], 'go', label='Start')
                        ax_path.plot(x_vals[-1], z_vals[-1], 'ro', label='Cur')
                    
                    # Show OBSTACLE Warning on Plot
                    if has_object:
                        ax_path.text(0.5, 0.9, f"⚠️ OBSTACLE: {obs_dist:.2f}m", 
                                     transform=ax_path.transAxes, color='red', 
                                     fontsize=12, fontweight='bold', ha='center')

                    ax_path.set_title("Top Down (X vs Z)")
                    ax_path.set_xlabel("X (Side)")
                    ax_path.set_ylabel("Z (Forward)")
                    ax_path.grid(True)
                    ax_path.axis('equal')

                    # Plot 2: Z vs Time
                    ax_z.plot(times, z_vals, 'm-')
                    ax_z.set_title("Forward Motion (Z vs Time)")
                    ax_z.grid(True)

                    # Plot 3: Y vs Time
                    ax_y.plot(times, y_vals, 'g-')
                    ax_y.set_title("Height (Y vs Time)")
                    ax_y.grid(True)

                    # Plot 4: X vs Time
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
    
    try:
        print(f"💾 Saving final graph to {IMAGE_NAME}...")
        fig.savefig(IMAGE_NAME)
        print("✅ Graph Saved Successfully!")
    except Exception as e:
        print(f"❌ Error saving graph: {e}")

    if plt.fignum_exists(fig.number):
        print("👀 Close the window manually to exit script completely.")
        plt.ioff()
        plt.show()