import socket
import json
import csv
import time
import threading
import queue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
TIMESTAMP = int(time.time())
FILE_NAV = f"robot_telemetry_{TIMESTAMP}.csv"
FILE_SURVEY = f"robot_survey_{TIMESTAMP}.csv"
OBSTACLE_WARNING_DIST = 0.5 # Meters

# Queue for thread safety
data_queue = queue.Queue()
is_running = True
phone_address = None

# --- NETWORK THREAD ---
def udp_listener():
    global is_running, phone_address
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)
    
    print(f"✅ Server Started on Port {UDP_PORT}")
    print(f"📂 Telemetry: {FILE_NAV}")
    print(f"📂 Survey:    {FILE_SURVEY}")
    print("-" * 40)
    
    while is_running:
        try:
            data, addr = sock.recvfrom(4096)
            if phone_address is None:
                phone_address = addr
                print(f"📱 Phone Connected: {addr}")
            
            data_queue.put(data)
            
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Socket Error: {e}")
            break
    sock.close()

net_thread = threading.Thread(target=udp_listener, daemon=True)
net_thread.start()

# --- PLOT SETUP ---
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle("Real-Time Robot Telemetry", fontsize=16)

# 1. Path Plot (Top Left) - KEEPS FULL HISTORY
line_path, = axs[0, 0].plot([], [], 'b-', label='Path')
point_robot, = axs[0, 0].plot([], [], 'ro', label='Robot')
axs[0, 0].set_title("Top Down (X vs Z)")
axs[0, 0].grid(True); axs[0, 0].axis('equal')

# 2. Forward Motion (Top Right) - SCROLLING (Limit 500)
line_z, = axs[0, 1].plot([], [], 'm-')
axs[0, 1].set_title("Z (Forward) vs Time")
axs[0, 1].grid(True)

# 3. Height (Bottom Left) - SCROLLING (Limit 500)
line_y, = axs[1, 0].plot([], [], 'g-')
axs[1, 0].set_title("Y (Height) vs Time")
axs[1, 0].grid(True)

# 4. Drift (Bottom Right) - SCROLLING (Limit 500)
line_x, = axs[1, 1].plot([], [], 'r-')
axs[1, 1].set_title("X (Lateral Drift) vs Time")
axs[1, 1].grid(True)

# Data Storage
# LIMITED lists (for time graphs)
times, x_vals, y_vals, z_vals = [], [], [], []
# FULL lists (for the map)
full_path_x, full_path_z = [], []

survey_x, survey_z = [], [] 
start_time = None

f_nav = open(FILE_NAV, 'w', newline='')
f_survey = open(FILE_SURVEY, 'w', newline='')
writer_nav = csv.writer(f_nav)
writer_survey = csv.writer(f_survey)

writer_nav.writerow(["Timestamp", "Pos_X", "Pos_Y", "Pos_Z", "Rot_X", "Rot_Y", "Rot_Z", "Rot_W", "Obstacle_Dist"])
writer_survey.writerow(["Timestamp", "Pos_X", "Pos_Y", "Pos_Z", "Label", "Note"])

# --- UPDATE FUNCTION ---
def update(frame):
    global start_time
    
    while not data_queue.empty():
        try:
            raw_data = data_queue.get_nowait()
            packet = json.loads(raw_data.decode('utf-8'))
            packet_type = packet.get('type', 'nav')
            
            ts = packet['timestamp']
            px, py, pz = packet['position']
            
            if start_time is None: start_time = ts
            rel_time = ts - start_time
            
            if packet_type == 'nav':
                qx, qy, qz, qw = packet['orientation']
                obs_dist = packet.get('obstacle_dist', 10.0)
                
                # Terminal Output
                if obs_dist < OBSTACLE_WARNING_DIST:
                    sys.stdout.write(f"\r\033[91m⚠️  OBSTACLE: {obs_dist:.2f}m  \033[0m")
                else:
                    sys.stdout.write(f"\r\033[92m✅ DISTANCE: {obs_dist:.2f}m   \033[0m")
                sys.stdout.flush()

                writer_nav.writerow([ts, px, py, pz, qx, qy, qz, qw, obs_dist])
                
                # 1. Store in FULL path (No Limit)
                full_path_x.append(px)
                full_path_z.append(pz)

                # 2. Store in SCROLLING charts (Limit 500)
                times.append(rel_time)
                x_vals.append(px)
                y_vals.append(py)
                z_vals.append(pz)
                
                if len(times) > 500:
                    times.pop(0); x_vals.pop(0); y_vals.pop(0); z_vals.pop(0)

            elif packet_type == 'survey':
                label = packet.get('label', 'Point')
                note = packet.get('note', '')
                writer_survey.writerow([ts, px, py, pz, label, note])
                f_survey.flush()
                
                sys.stdout.write(f"\n📍 SURVEY POINT: {label} at [{px:.2f}, {pz:.2f}]\n")
                
                survey_x.append(px)
                survey_z.append(pz)
                axs[0, 0].plot(px, pz, 'r*', markersize=10) 

        except json.JSONDecodeError:
            continue
    
    if times:
        # Update Full Path Map
        line_path.set_data(full_path_x, full_path_z)
        point_robot.set_data([full_path_x[-1]], [full_path_z[-1]])
        
        # Update Scrolling Time Graphs
        line_z.set_data(times, z_vals)
        line_y.set_data(times, y_vals) 
        line_x.set_data(times, x_vals)
        
        # Auto-scale axes
        for ax in axs.flat:
            ax.relim()
            ax.autoscale_view()

    return line_path, point_robot, line_z, line_y, line_x

try:
    ani = FuncAnimation(fig, update, interval=100, blit=False)
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    is_running = False
    f_nav.close()
    f_survey.close()
    print("\n✅ Closed.")