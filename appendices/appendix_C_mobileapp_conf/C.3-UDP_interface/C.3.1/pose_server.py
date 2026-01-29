import socket
import json
import csv
import time
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # Required for 3D

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
FILENAME = f"pose_path_3d_{int(time.time())}.csv"

# 🎯 SENSITIVITY SETTINGS
# 0.05 = 5cm. Increase to 0.1 (10cm) if it's too jittery.
MOVEMENT_THRESHOLD = 1 

# --- SETUP SOCKET ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)   

print(f"✅ 3D Server Started on Port {UDP_PORT}")
print(f"✅ Saving data to: {FILENAME}")
print("👉 Move your phone. The plot will update.")
print("🔴 Press Ctrl+C to STOP and inspect the final graph.")

# --- SETUP 3D PLOT ---
plt.ion() # Interactive mode on
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Lists to hold the ENTIRE history
x_vals, y_vals, z_vals = [], [], []

# --- STATE VARIABLES ---
last_saved_pos = None 

try:
    with open(FILENAME, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Pos_X", "Pos_Y", "Pos_Z", "Rot_X", "Rot_Y", "Rot_Z", "Rot_W"])
        
        while True:
            try:
                # 1. Receive Data
                data, addr = sock.recvfrom(1024)
                pose = json.loads(data.decode('utf-8'))
                
                ts = pose['timestamp']
                px, py, pz = pose['position']
                qx, qy, qz, qw = pose['orientation']
                
                # 2. Check Threshold
                should_save = False
                if last_saved_pos is None:
                    should_save = True
                else:
                    lx, ly, lz = last_saved_pos
                    # 3D Distance Formula
                    dist = math.sqrt((px - lx)**2 + (py - ly)**2 + (pz - lz)**2)
                    if dist > MOVEMENT_THRESHOLD:
                        should_save = True
                
                # 3. Save & Plot
                if should_save:
                    writer.writerow([ts, px, py, pz, qx, qy, qz, qw])
                    file.flush()
                     
                    # Store data for plotting
                    x_vals.append(px)
                    y_vals.append(pz) # Map ARKit Z (Depth) to Plot Y
                    z_vals.append(py) # Map ARKit Y (Height) to Plot Z (Up)
                    
                    # --- 3D PLOTTING LOGIC ---
                    ax.clear() # Refresh the frame
                    
                    # Plot the path (Blue line)
                    ax.plot(x_vals, y_vals, z_vals, linewidth=2, label='Path')
                    
                    # Plot the Start (Green) and Current Head (Red)
                    if len(x_vals) > 0:
                        ax.scatter([x_vals[0]], [y_vals[0]], [z_vals[0]], c='green', s=50, label='Start')
                        ax.scatter([x_vals[-1]], [y_vals[-1]], [z_vals[-1]], c='red', s=100, label='Current')

                    # Labels (mapped to how we plotted them)
                    ax.set_xlabel("X (Left/Right)")
                    ax.set_ylabel("Z (Forward/Back)")
                    ax.set_zlabel("Y (Height)")
                    ax.set_title(f"Live 3D Tracking - Points: {len(x_vals)}")
                    
                    last_saved_pos = [px, py, pz]
                    
                    # Pause to let the plot render
                    plt.pause(0.001) 

            except BlockingIOError:
                plt.pause(0.01) 
                continue
            except json.JSONDecodeError:
                print("Skipping bad packet")

except KeyboardInterrupt:
    print("\n🛑 Recording Stopped.")
    print(f"✅ Data saved to {FILENAME}")
    print("👀 The window is now open for inspection. Close it to exit.")
    
    # --- FINAL VIEW MODE ---
    plt.ioff() # Turn off interactive mode
    plt.show() # Keep the window open until you close it