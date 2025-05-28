import matplotlib.pyplot as plt
import json
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from typing import Dict, List
import glob
import os

def load_route_data(lap_number: int = None) -> Dict:
    if lap_number is not None:
        filename = f"route_data_lap_{lap_number}.json"
        try:
            with open(filename, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"Tur {lap_number} verisi bulunamadı!")
            return None
    else:
        route_files = glob.glob("route_data_lap_*.json")
        if not route_files:
            print("Hiç tur verisi bulunamadı!")
            return None
            
        all_data = {}
        for file in route_files:
            try:
                with open(file, 'r') as f:
                    data = json.load(f)
                    lap_num = data['lap_number']
                    all_data[lap_num] = data
            except Exception as e:
                print(f"{file} dosyası okunurken hata: {e}")
        return all_data

def load_checkpoint_data() -> Dict:
    try:
        with open("checkpoints.json", 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print("checkpoints.json dosyası bulunamadı!")
        return None
    except Exception as e:
        print(f"Checkpoint verileri okunurken hata: {e}")
        return None

def visualize_route(lap_number: int = None):
    data = load_route_data(lap_number)
    if not data:
        return
        
    if lap_number is not None:
        visualize_single_lap(data)
    else:
        visualize_all_laps(data)

def visualize_single_lap(data: Dict):
    fig = plt.figure(figsize=(15, 10))
    
    ax1 = fig.add_subplot(221, projection='3d')
    points = data['points']
    x_coords = [p['position'][0] for p in points]
    y_coords = [p['position'][1] for p in points]
    z_coords = [p['position'][2] for p in points]
    
    checkpoint_points = [p for p in points if p['is_checkpoint']]
    cp_x = [p['position'][0] for p in checkpoint_points]
    cp_y = [p['position'][1] for p in checkpoint_points]
    cp_z = [p['position'][2] for p in checkpoint_points]
    
    ax1.plot(x_coords, y_coords, z_coords, 'b-', alpha=0.5, label='Route')
 
    ax1.scatter(cp_x, cp_y, cp_z, c='r', marker='o', s=100, label='Checkpoints')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title(f'3D Trajectory - Lap {data["lap_number"]}')
    ax1.legend()
    
    ax2 = fig.add_subplot(222)
    times = [p['timestamp'] for p in points]
    velocities = [np.sqrt(sum(v**2 for v in p['velocity'])) for p in points]
    ax2.plot(times, velocities, 'g-', label='Speed')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Speed (m/s)')
    ax2.set_title('Speed Profile')
    ax2.grid(True)
    ax2.legend()
    
    ax3 = fig.add_subplot(223)
    roll = [p['orientation'][0] for p in points]
    pitch = [p['orientation'][1] for p in points]
    yaw = [p['orientation'][2] for p in points]
    ax3.plot(times, roll, 'r-', label='Roll')
    ax3.plot(times, pitch, 'g-', label='Pitch')
    ax3.plot(times, yaw, 'b-', label='Yaw')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (rad)')
    ax3.set_title('Orientation Angles')
    ax3.grid(True)
    ax3.legend()
    
    ax4 = fig.add_subplot(224)
    left = [p['lidar_readings']['left'] for p in points]
    right = [p['lidar_readings']['right'] for p in points]
    ax4.plot(times, left, 'b-', label='Left')
    ax4.plot(times, right, 'y-', label='Right')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Distance (m)')
    ax4.set_title('Lidar Readings')
    ax4.grid(True)
    ax4.legend()

    stats = data.get('statistics', {})
    if stats:
        stats_text = f"Tur {data['lap_number']} İstatistikleri:\n"
        stats_text += f"Toplam Mesafe: {stats.get('total_distance', 0):.2f}m\n"
        stats_text += f"Toplam Süre: {stats.get('total_duration', 0):.2f}s\n"
        stats_text += f"Ortalama Hız: {stats.get('average_speed', 0):.2f}m/s"
        plt.figtext(0.02, 0.02, stats_text, fontsize=8, bbox=dict(facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    plt.show()

def visualize_all_laps(all_data: Dict):
    fig = plt.figure(figsize=(15, 10))
    
    ax1 = fig.add_subplot(221, projection='3d')
    colors = plt.cm.rainbow(np.linspace(0, 1, len(all_data)))
    
    for (lap_num, data), color in zip(all_data.items(), colors):
        points = data['points']
        x_coords = [p['position'][0] for p in points]
        y_coords = [p['position'][1] for p in points]
        z_coords = [p['position'][2] for p in points]
        ax1.plot(x_coords, y_coords, z_coords, color=color, alpha=0.5, label=f'Lap {lap_num}')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectories - All Laps')
    ax1.legend()
    
    ax2 = fig.add_subplot(222)
    for (lap_num, data), color in zip(all_data.items(), colors):
        points = data['points']
        times = [p['timestamp'] for p in points]
        velocities = [np.sqrt(sum(v**2 for v in p['velocity'])) for p in points]
        ax2.plot(times, velocities, color=color, label=f'Lap {lap_num}')
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Speed (m/s)')
    ax2.set_title('Speed Comparison')
    ax2.grid(True)
    ax2.legend()
    
    ax3 = fig.add_subplot(223)
    lap_numbers = []
    durations = []
    distances = []
    
    for lap_num, data in sorted(all_data.items(), key=lambda x: int(x[0])):
        points = data['points']
        if points:
            duration = points[-1]['timestamp'] - points[0]['timestamp']
            
            total_distance = 0
            for i in range(1, len(points)):
                p1 = points[i-1]['position']
                p2 = points[i]['position']
                distance = np.sqrt(sum((p2[j] - p1[j])**2 for j in range(3)))
                total_distance += distance
            
            lap_numbers.append(int(lap_num))
            durations.append(duration)
            distances.append(total_distance)
    
    ax3.bar(lap_numbers, durations, color='skyblue')
    ax3.set_xlabel('Lap Number')
    ax3.set_ylabel('Duration (s)')
    ax3.set_title('Lap Durations')
    ax3.grid(True)
    
    for i, v in enumerate(durations):
        ax3.text(lap_numbers[i], v, f'{v:.1f}s', ha='center', va='bottom')
    
    ax4 = fig.add_subplot(224)
    ax4.bar(lap_numbers, distances, color='lightgreen')
    ax4.set_xlabel('Lap Number')
    ax4.set_ylabel('Distance (m)')
    ax4.set_title('Lap Distances')
    ax4.grid(True)
    
    for i, v in enumerate(distances):
        ax4.text(lap_numbers[i], v, f'{v:.1f}m', ha='center', va='bottom')
    
    plt.tight_layout()
    plt.show()

def analyze_performance(all_data: Dict):
    print("\n=== Performans Analizi ===")
    
    for lap_num, data in sorted(all_data.items(), key=lambda x: int(x[0])):
        points = data['points']
        if not points:
            continue
            
        print(f"\nTur {lap_num} Analizi:")
        
        duration = points[-1]['timestamp'] - points[0]['timestamp']
        total_distance = 0
        speeds = []
        accelerations = []
        
        segments = []
        for i in range(1, len(points)):
            p1 = points[i-1]
            p2 = points[i]
            
            distance = np.sqrt(sum((p2['position'][j] - p1['position'][j])**2 for j in range(3)))
            total_distance += distance
            
            time_diff = p2['timestamp'] - p1['timestamp']
            if time_diff > 0:
                speed = distance / time_diff
                speeds.append(speed)
                
                if len(speeds) > 1:
                    acc = (speeds[-1] - speeds[-2]) / time_diff
                    accelerations.append(acc)
            
            segments.append({
                'distance': distance,
                'duration': time_diff,
                'speed': speed if time_diff > 0 else 0,
                'is_checkpoint': p2['is_checkpoint']
            })
        
        avg_speed = np.mean(speeds) if speeds else 0
        max_speed = max(speeds) if speeds else 0
        min_speed = min(speeds) if speeds else 0
        avg_acc = np.mean(accelerations) if accelerations else 0
        
        slow_segments = [s for s in segments if s['speed'] < avg_speed * 0.7]
        slow_points = []
        for i, s in enumerate(segments):
            if s['speed'] < avg_speed * 0.7:
                slow_points.append({
                    'index': i,
                    'speed': s['speed'],
                    'position': points[i+1]['position'],
                    'is_checkpoint': s['is_checkpoint']
                })
        
        print(f"Toplam Süre: {duration:.2f} saniye")
        print(f"Toplam Mesafe: {total_distance:.2f} metre")
        print(f"Ortalama Hız: {avg_speed:.2f} m/s")
        print(f"Maksimum Hız: {max_speed:.2f} m/s")
        print(f"Minimum Hız: {min_speed:.2f} m/s")
        print(f"Ortalama İvmelenme: {avg_acc:.2f} m/s²")
        
        print("\nOptimizasyon Önerileri:")
        
        if slow_points:
            print("\nYavaş Noktalar:")
            for point in slow_points:
                if point['is_checkpoint']:
                    print(f"- Checkpoint {points[point['index']+1]['checkpoint_id']} yakınında yavaşlama")
                    print(f"  Hız: {point['speed']:.2f} m/s (Ortalama: {avg_speed:.2f} m/s)")
                else:
                    print(f"- Konum ({point['position'][0]:.1f}, {point['position'][1]:.1f}) yakınında yavaşlama")
                    print(f"  Hız: {point['speed']:.2f} m/s (Ortalama: {avg_speed:.2f} m/s)")
        
        checkpoint_segments = [s for s in segments if s['is_checkpoint']]
        if checkpoint_segments:
            avg_checkpoint_time = np.mean([s['duration'] for s in checkpoint_segments])
            print(f"\nCheckpoint Geçişleri:")
            print(f"- Ortalama checkpoint geçiş süresi: {avg_checkpoint_time:.2f} saniye")
            if avg_checkpoint_time > 0.5:  # 0.5 saniyeden uzun geçişler
                print("- Checkpoint geçiş süreleri optimize edilebilir")
        
        speed_variations = np.std(speeds) if speeds else 0
        if speed_variations > avg_speed * 0.3:  # Hız değişimi ortalamanın %30'undan fazlaysa
            print("\nHız Optimizasyonu:")
            print("- Hız değişimleri çok fazla, daha sabit hız önerilir")
            print(f"- Hız standart sapması: {speed_variations:.2f} m/s")
        
        print("\n" + "="*50)

def visualize_checkpoints():
    data = load_checkpoint_data()
    if not data:
        return
        
    fig = plt.figure(figsize=(15, 10))
    
    ax1 = fig.add_subplot(221, projection='3d')
    
    checkpoints = data['checkpoints']
    
    cp_ids = []
    x_coords = []
    y_coords = []
    z_coords = []
    
    for cp_id, cp_data in checkpoints.items():
        cp_ids.append(int(cp_id))
        x_coords.append(cp_data['position']['x'])
        y_coords.append(cp_data['position']['y'])
        z_coords.append(cp_data['position']['z'])
    
    ax1.scatter(x_coords, y_coords, z_coords, c='red', marker='o', s=100, label='Checkpoints')
    
    for i, cp_id in enumerate(cp_ids):
        ax1.text(x_coords[i], y_coords[i], z_coords[i], f'  CP{cp_id}', fontsize=8)
    
    for cp_id, cp_data in checkpoints.items():
        cp_x = cp_data['position']['x']
        cp_y = cp_data['position']['y']
        cp_z = cp_data['position']['z']
        
        for connection in cp_data['connections']:
            target_id = str(connection['to_checkpoint_id'])
            if target_id in checkpoints:
                target_cp = checkpoints[target_id]
                target_x = target_cp['position']['x']
                target_y = target_cp['position']['y']
                target_z = target_cp['position']['z']
                
                ax1.plot([cp_x, target_x], [cp_y, target_y], [cp_z, target_z], 
                        'b-', alpha=0.5, linewidth=1)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Checkpoint Network (3D)')
    ax1.legend()
    
    ax2 = fig.add_subplot(222)
    distances = []
    connections_count = []
    
    for cp_id, cp_data in checkpoints.items():
        connections = cp_data['connections']
        connections_count.append(len(connections))
        if connections:
            avg_distance = np.mean([conn['distance'] for conn in connections])
            distances.append(avg_distance)
        else:
            distances.append(0)
    
    bars = ax2.bar(cp_ids, distances, color='skyblue', alpha=0.7)
    ax2.set_xlabel('Checkpoint ID')
    ax2.set_ylabel('Average Connection Distance (m)')
    ax2.set_title('Checkpoint Connection Distances')
    ax2.grid(True, alpha=0.3)
    
    for bar, distance in zip(bars, distances):
        if distance > 0:
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{distance:.1f}', ha='center', va='bottom', fontsize=8)
    
    ax3 = fig.add_subplot(223)
    
    left_readings = []
    right_readings = []
    cp_labels = []
    
    for cp_id, cp_data in checkpoints.items():
        if cp_data['passage_history']:
            last_passage = cp_data['passage_history'][-1]
            left_readings.append(last_passage['lidar_readings']['left'])
            right_readings.append(last_passage['lidar_readings']['right'])
            cp_labels.append(f'CP{cp_id}')
    
    x_pos = np.arange(len(cp_labels))
    width = 0.35
    
    ax3.bar(x_pos - width/2, left_readings, width, label='Left Lidar', color='blue', alpha=0.7)
    ax3.bar(x_pos + width/2, right_readings, width, label='Right Lidar', color='yellow', alpha=0.7)
    
    ax3.set_xlabel('Checkpoints')
    ax3.set_ylabel('Lidar Distance (m)')
    ax3.set_title('Lidar Readings at Checkpoints')
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(cp_labels, rotation=45)
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    ax4 = fig.add_subplot(224)
    
    approach_sides = {'left': 0, 'right': 0}
    side_checkpoints = {'left': [], 'right': []}
    
    for cp_id, cp_data in checkpoints.items():
        if cp_data['passage_history']:
            last_passage = cp_data['passage_history'][-1]
            side = last_passage['approach_side']
            approach_sides[side] += 1
            side_checkpoints[side].append(cp_id)
    
    labels = list(approach_sides.keys())
    sizes = list(approach_sides.values())
    colors = ['lightblue', 'lightcoral']
    
    if sum(sizes) > 0:
        wedges, texts, autotexts = ax4.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax4.set_title('Checkpoint Approach Sides Distribution')
        
        info_text = ""
        for side, cp_list in side_checkpoints.items():
            if cp_list:
                info_text += f"{side.capitalize()}: CP{', CP'.join(cp_list)}\n"
        
        ax4.text(1.2, 0, info_text, fontsize=8, verticalalignment='center',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
    
    plt.tight_layout()
    plt.show()

def visualize_checkpoint_performance():

    checkpoint_data = load_checkpoint_data()
    route_data = load_route_data()
    
    if not checkpoint_data or not route_data:
        print("Veri yüklenemedi!")
        return
    
    fig = plt.figure(figsize=(15, 10))
    
    ax1 = fig.add_subplot(221)
    
    checkpoint_times = {}
    for lap_num, lap_data in route_data.items():
        if 'checkpoint_times' in lap_data:
            for cp_id, time_spent in lap_data['checkpoint_times'].items():
                if cp_id not in checkpoint_times:
                    checkpoint_times[cp_id] = []
                checkpoint_times[cp_id].append(time_spent)
    
    avg_times = {}
    for cp_id, times in checkpoint_times.items():
        avg_times[cp_id] = np.mean(times)
    
    if avg_times:
        cp_ids = list(avg_times.keys())
        times = list(avg_times.values())
        
        bars = ax1.bar(cp_ids, times, color='lightgreen', alpha=0.7)
        ax1.set_xlabel('Checkpoint ID')
        ax1.set_ylabel('Average Time Spent (s)')
        ax1.set_title('Time Spent at Each Checkpoint')
        ax1.grid(True, alpha=0.3)
        
        for bar, time_val in zip(bars, times):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{time_val:.2f}s', ha='center', va='bottom', fontsize=8)
    
    ax2 = fig.add_subplot(222)
    
    checkpoint_velocities = {}
    checkpoints = checkpoint_data['checkpoints']
    
    for cp_id, cp_data in checkpoints.items():
        if cp_data['passage_history']:
            velocities = []
            for passage in cp_data['passage_history']:
                velocity = passage['velocity']
                # Calculate speed magnitude
                speed = np.sqrt(velocity['vx']**2 + velocity['vy']**2 + velocity['vz']**2)
                velocities.append(speed)
            checkpoint_velocities[int(cp_id)] = np.mean(velocities)
    
    if checkpoint_velocities:
        cp_ids = list(checkpoint_velocities.keys())
        velocities = list(checkpoint_velocities.values())
        
        bars = ax2.bar(cp_ids, velocities, color='orange', alpha=0.7)
        ax2.set_xlabel('Checkpoint ID')
        ax2.set_ylabel('Average Velocity (m/s)')
        ax2.set_title('Average Velocity at Checkpoints')
        ax2.grid(True, alpha=0.3)
        
        for bar, vel in zip(bars, velocities):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                    f'{vel:.2f}', ha='center', va='bottom', fontsize=8)
    
    ax3 = fig.add_subplot(223)
    
    checkpoint_stability = {}
    for cp_id, cp_data in checkpoints.items():
        if cp_data['passage_history']:
            stabilities = []
            for passage in cp_data['passage_history']:
                orientation = passage['orientation']
                stability = abs(orientation['roll']) + abs(orientation['pitch'])
                stabilities.append(stability)
            checkpoint_stability[int(cp_id)] = np.mean(stabilities)
    
    if checkpoint_stability:
        cp_ids = list(checkpoint_stability.keys())
        stabilities = list(checkpoint_stability.values())
        
        bars = ax3.bar(cp_ids, stabilities, color='red', alpha=0.7)
        ax3.set_xlabel('Checkpoint ID')
        ax3.set_ylabel('Instability (|roll| + |pitch|)')
        ax3.set_title('Stability at Checkpoints (Lower is Better)')
        ax3.grid(True, alpha=0.3)
        
        for bar, stab in zip(bars, stabilities):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                    f'{stab:.3f}', ha='center', va='bottom', fontsize=8)
    
    ax4 = fig.add_subplot(224)
    
    checkpoint_balance = {}
    for cp_id, cp_data in checkpoints.items():
        if cp_data['passage_history']:
            balances = []
            for passage in cp_data['passage_history']:
                lidar = passage['lidar_readings']
                left = lidar['left']
                right = lidar['right']
                balance = (right - left) / max(right + left, 0.1)
                balances.append(balance)
            checkpoint_balance[int(cp_id)] = np.mean(balances)
    
    if checkpoint_balance:
        cp_ids = list(checkpoint_balance.keys())
        balances = list(checkpoint_balance.values())
        
        colors = ['red' if abs(b) > 0.3 else 'orange' if abs(b) > 0.1 else 'green' for b in balances]
        
        bars = ax4.bar(cp_ids, balances, color=colors, alpha=0.7)
        ax4.set_xlabel('Checkpoint ID')
        ax4.set_ylabel('Lidar Balance (-1: Left, +1: Right)')
        ax4.set_title('Lidar Balance at Checkpoints')
        ax4.axhline(y=0, color='black', linestyle='-', alpha=0.5)
        ax4.axhline(y=0.1, color='orange', linestyle='--', alpha=0.5, label='±10% threshold')
        ax4.axhline(y=-0.1, color='orange', linestyle='--', alpha=0.5)
        ax4.axhline(y=0.3, color='red', linestyle='--', alpha=0.5, label='±30% threshold')
        ax4.axhline(y=-0.3, color='red', linestyle='--', alpha=0.5)
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        
        for bar, bal in zip(bars, balances):
            ax4.text(bar.get_x() + bar.get_width()/2, 
                    bar.get_height() + (0.01 if bal >= 0 else -0.03),
                    f'{bal:.2f}', ha='center', va='bottom' if bal >= 0 else 'top', fontsize=8)
    
    plt.tight_layout()
    plt.show()

def checkpoint_menu():
    while True:
        print("\n" + "="*50)
        print("CHECKPOINT VISUALIZATION MENU")
        print("="*50)
        print("1. Checkpoint Network (Positions & Connections)")
        print("2. Checkpoint Performance Analysis")  
        print("3. Route vs Checkpoints Comparison")
        print("4. Checkpoint Statistics Summary")
        print("5. Return to Main Menu")
        print("="*50)
        
        choice = input("Select an option (1-5): ").strip()
        
        if choice == '1':
            print("Displaying checkpoint network...")
            visualize_checkpoints()
        elif choice == '2':
            print("Analyzing checkpoint performance...")
            visualize_checkpoint_performance()
        elif choice == '3':
            print("Comparing routes with checkpoints...")
            visualize_route_vs_checkpoints()
        elif choice == '4':
            print("Showing checkpoint statistics...")
            show_checkpoint_statistics()
        elif choice == '5':
            print("Returning to main menu...")
            break
        else:
            print("Invalid choice! Please select 1-5.")

def visualize_route_vs_checkpoints():
    route_data = load_route_data()
    checkpoint_data = load_checkpoint_data()
    
    if not route_data or not checkpoint_data:
        print("Veri yüklenemedi!")
        return
    
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    checkpoints = checkpoint_data['checkpoints']
    cp_x = [cp['position']['x'] for cp in checkpoints.values()]
    cp_y = [cp['position']['y'] for cp in checkpoints.values()]
    cp_z = [cp['position']['z'] for cp in checkpoints.values()]
    cp_ids = [int(cp_id) for cp_id in checkpoints.keys()]
    
    ax.scatter(cp_x, cp_y, cp_z, c='red', marker='o', s=200, label='Checkpoints', alpha=0.8)
    
    for i, cp_id in enumerate(cp_ids):
        ax.text(cp_x[i], cp_y[i], cp_z[i], f'  CP{cp_id}', fontsize=10)
    
    colors = plt.cm.rainbow(np.linspace(0, 1, len(route_data)))
    
    for (lap_num, lap_data), color in zip(route_data.items(), colors):
        points = lap_data['points']
        x_coords = [p['position'][0] for p in points]
        y_coords = [p['position'][1] for p in points]
        z_coords = [p['position'][2] for p in points]
        
        ax.plot(x_coords, y_coords, z_coords, color=color, alpha=0.6, 
               linewidth=2, label=f'Lap {lap_num} Route')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Routes vs Checkpoints (3D View)')
    ax.legend()
    
    plt.tight_layout()
    plt.show()

def show_checkpoint_statistics():
    checkpoint_data = load_checkpoint_data()
    if not checkpoint_data:
        return
    
    checkpoints = checkpoint_data['checkpoints']
    
    print("\n" + "="*60)
    print("CHECKPOINT STATISTICS SUMMARY")
    print("="*60)
    
    total_checkpoints = len(checkpoints)
    total_connections = sum(len(cp['connections']) for cp in checkpoints.values())
    avg_connections = total_connections / total_checkpoints if total_checkpoints > 0 else 0
    
    print(f"Total Checkpoints: {total_checkpoints}")
    print(f"Total Connections: {total_connections}")
    print(f"Average Connections per Checkpoint: {avg_connections:.2f}")
    
    x_coords = [cp['position']['x'] for cp in checkpoints.values()]
    y_coords = [cp['position']['y'] for cp in checkpoints.values()]
    z_coords = [cp['position']['z'] for cp in checkpoints.values()]
    
    print(f"\nPosition Ranges:")
    print(f"X: {min(x_coords):.2f} to {max(x_coords):.2f} ({max(x_coords)-min(x_coords):.2f}m range)")
    print(f"Y: {min(y_coords):.2f} to {max(y_coords):.2f} ({max(y_coords)-min(y_coords):.2f}m range)")
    print(f"Z: {min(z_coords):.2f} to {max(z_coords):.2f} ({max(z_coords)-min(z_coords):.2f}m range)")
    
    distances = []
    for cp in checkpoints.values():
        for conn in cp['connections']:
            distances.append(conn['distance'])
    
    if distances:
        print(f"\nConnection Distance Statistics:")
        print(f"Minimum Distance: {min(distances):.2f}m")
        print(f"Maximum Distance: {max(distances):.2f}m")
        print(f"Average Distance: {np.mean(distances):.2f}m")
        print(f"Standard Deviation: {np.std(distances):.2f}m")
    
    connection_counts = {cp_id: len(cp['connections']) for cp_id, cp in checkpoints.items()}
    most_connected = max(connection_counts, key=connection_counts.get)
    least_connected = min(connection_counts, key=connection_counts.get)
    
    print(f"\nMost Connected Checkpoint: CP{most_connected} ({connection_counts[most_connected]} connections)")
    print(f"Least Connected Checkpoint: CP{least_connected} ({connection_counts[least_connected]} connections)")
    
    approach_sides = {'left': 0, 'right': 0}
    for cp in checkpoints.values():
        if cp['passage_history']:
            side = cp['passage_history'][-1]['approach_side']
            approach_sides[side] += 1
    
    total_approaches = sum(approach_sides.values())
    if total_approaches > 0:
        print(f"\nApproach Side Distribution:")
        for side, count in approach_sides.items():
            percentage = (count / total_approaches) * 100
            print(f"{side.capitalize()}: {count} ({percentage:.1f}%)")
    
    print("="*60)

def main_menu():
    while True:
        print("\n" + "="*50)
        print("ROUTE & CHECKPOINT VISUALIZER")
        print("="*50)
        print("1. Visualize Routes")
        print("2. Checkpoint Analysis")
        print("3. Performance Analysis")
        print("4. Exit")
        print("="*50)
        
        choice = input("Select an option (1-4): ").strip()
        
        if choice == '1':
            lap_choice = input("Enter lap number (or press Enter for all laps): ").strip()
            if lap_choice:
                try:
                    lap_num = int(lap_choice)
                    visualize_route(lap_num)
                except ValueError:
                    print("Invalid lap number!")
            else:
                visualize_route()
        elif choice == '2':
            checkpoint_menu()
        elif choice == '3':
            data = load_route_data()
            if data:
                analyze_performance(data)
            else:
                print("No route data found!")
        elif choice == '4':
            print("Exiting...")
            break
        else:
            print("Invalid choice! Please select 1-4.")

if __name__ == '__main__':
    main_menu()
    
