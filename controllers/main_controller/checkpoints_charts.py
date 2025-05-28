import matplotlib.pyplot as plt
import json
from mpl_toolkits.mplot3d import Axes3D

def visualize_checkpoints():
    """Visualize checkpoint data from checkpoints.json file"""
    try:
        with open("checkpoints.json", 'r') as f:
            data = json.load(f)
            checkpoints = data['checkpoints']
        
        if not checkpoints:
            print("No checkpoint data found!")
            return
        
        # Create figure with subplots
        fig = plt.figure(figsize=(15, 10))
        
        # 3D Trajectory Plot
        ax1 = fig.add_subplot(221, projection='3d')
        x_coords = []
        y_coords = []
        z_coords = []
        
        # Sort checkpoints by ID to ensure correct order
        sorted_checkpoints = sorted(checkpoints.items(), key=lambda x: int(x[0]))
        
        for cp_id, cp in sorted_checkpoints:
            x_coords.append(cp['position']['x'])
            y_coords.append(cp['position']['y'])
            z_coords.append(cp['position']['z'])
        
        ax1.plot(x_coords, y_coords, z_coords, 'b-', label='Trajectory')
        ax1.scatter(x_coords, y_coords, z_coords, c='r', marker='o', label='Checkpoints')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Trajectory')
        ax1.legend()
        
        # Altitude Plot
        ax2 = fig.add_subplot(222)
        ax2.plot(z_coords, 'g-', label='Altitude')
        ax2.set_xlabel('Checkpoint Number')
        ax2.set_ylabel('Altitude (m)')
        ax2.set_title('Altitude Profile')
        ax2.grid(True)
        ax2.legend()
        
        # Orientation Plot
        ax3 = fig.add_subplot(223)
        roll = []
        pitch = []
        yaw = []
        
        for cp_id, cp in sorted_checkpoints:
            if cp['passage_history']:
                last_passage = cp['passage_history'][-1]
                roll.append(last_passage['orientation']['roll'])
                pitch.append(last_passage['orientation']['pitch'])
                yaw.append(last_passage['orientation']['yaw'])
        
        ax3.plot(roll, 'r-', label='Roll')
        ax3.plot(pitch, 'g-', label='Pitch')
        ax3.plot(yaw, 'b-', label='Yaw')
        ax3.set_xlabel('Checkpoint Number')
        ax3.set_ylabel('Angle (rad)')
        ax3.set_title('Orientation Angles')
        ax3.grid(True)
        ax3.legend()
        
        # Velocity Plot
        ax4 = fig.add_subplot(224)
        v_x = []
        v_y = []
        v_z = []
        
        for cp_id, cp in sorted_checkpoints:
            if cp['passage_history']:
                last_passage = cp['passage_history'][-1]
                v_x.append(last_passage['velocity']['vx'])  # Changed from v_x to vx
                v_y.append(last_passage['velocity']['vy'])  # Changed from v_y to vy
                v_z.append(last_passage['velocity']['vz'])  # Changed from v_z to vz
        
        ax4.plot(v_x, 'r-', label='Vx')
        ax4.plot(v_y, 'g-', label='Vy')
        ax4.plot(v_z, 'b-', label='Vz')
        ax4.set_xlabel('Checkpoint Number')
        ax4.set_ylabel('Velocity (m/s)')
        ax4.set_title('Velocity Components')
        ax4.grid(True)
        ax4.legend()
        
        # Add metadata information
        metadata = data.get('metadata', {})
        if metadata:
            last_update = metadata.get('last_update', 'N/A')
            last_checkpoint_id = metadata.get('last_checkpoint_id', 'N/A')
            plt.figtext(0.02, 0.02, f'Last Update: {last_update}\nLast Checkpoint ID: {last_checkpoint_id}', 
                       fontsize=8, bbox=dict(facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
    except FileNotFoundError:
        print("No checkpoint data file found!")
    except Exception as e:
        print(f"Error visualizing checkpoints: {e}") 