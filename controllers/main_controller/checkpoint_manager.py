import json
import os
from datetime import datetime
from math import sqrt

class CheckpointManager:
    def __init__(self, json_file_path="checkpoints.json"):
        self.json_file_path = json_file_path
        self.checkpoints = {}
        self.metadata = {
            "last_checkpoint_id": 0,
            "last_update": datetime.now().isoformat()
        }
        self.load_data()

    def load_data(self):
        if os.path.exists(self.json_file_path):
            try:
                with open(self.json_file_path, 'r') as f:
                    data = json.load(f)
                    self.checkpoints = data.get('checkpoints', {})
                    self.metadata = data.get('metadata', self.metadata)
            except json.JSONDecodeError:
                print("Error: Invalid JSON file. Starting with empty data.")
                self.checkpoints = {}
            except Exception as e:
                print(f"Error loading data: {e}")
                self.checkpoints = {}

    def save_data(self):
        try:
            data = {
                'checkpoints': self.checkpoints,
                'metadata': {
                    'last_checkpoint_id': self.metadata['last_checkpoint_id'],
                    'last_update': datetime.now().isoformat()
                }
            }
            with open(self.json_file_path, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        except Exception as e:
            print(f"Error saving data: {e}")
            return False

    def add_checkpoint(self, position, passage_data, orientation=None):
        new_id = str(self.metadata['last_checkpoint_id'] + 1)
        
        checkpoint_data = {
            'id': int(new_id),
            'position': {
                'x': position['x'],
                'y': position['y'],
                'z': position['z']
            },
            'connections': [],
            'passage_history': [{
                'timestamp': datetime.now().isoformat(),
                'approach_side': passage_data['passage_info']['side'],
                'lidar_readings': {
                    'left': passage_data['passage_info']['left_lidar'],
                    'right': passage_data['passage_info']['right_lidar']
                },
                'orientation': {
                    'roll': orientation[0] if orientation else passage_data['orientation']['roll'],
                    'pitch': orientation[1] if orientation else passage_data['orientation']['pitch'],
                    'yaw': orientation[2] if orientation else passage_data['orientation']['yaw']
                },
                'velocity': {
                    'vx': passage_data['velocity']['v_x'],
                    'vy': passage_data['velocity']['v_y'],
                    'vz': passage_data['velocity']['v_z']
                }
            }]
        }

        self.checkpoints[new_id] = checkpoint_data
        self.metadata['last_checkpoint_id'] = int(new_id)
        
        self._update_connections(new_id)
        
        self.save_data()
        
        return new_id

    def _update_connections(self, new_checkpoint_id):
        new_checkpoint = self.checkpoints[new_checkpoint_id]
        new_pos = new_checkpoint['position']

        for checkpoint_id, checkpoint in self.checkpoints.items():
            if checkpoint_id == new_checkpoint_id:
                continue
            distance = sqrt(
                (new_pos['x'] - checkpoint['position']['x'])**2 +
                (new_pos['y'] - checkpoint['position']['y'])**2 +
                (new_pos['z'] - checkpoint['position']['z'])**2
            )


            if distance <= 3.0:
                new_checkpoint['connections'].append({
                    'to_checkpoint_id': int(checkpoint_id),
                    'distance': distance,
                    'difficulty': 1.0,
                    'average_speed': 0.0,
                    'best_approach_angle': 0.0
                })

                checkpoint['connections'].append({
                    'to_checkpoint_id': int(new_checkpoint_id),
                    'distance': distance,
                    'difficulty': 1.0,
                    'average_speed': 0.0,
                    'best_approach_angle': 0.0
                })

    def get_checkpoint(self, checkpoint_id):
        return self.checkpoints.get(str(checkpoint_id))

    def get_all_checkpoints(self):
        return self.checkpoints

    def update_checkpoint(self, checkpoint_id, new_data):
        if str(checkpoint_id) in self.checkpoints:
            self.checkpoints[str(checkpoint_id)].update(new_data)
            self.save_data()
            return True
        return False

    def delete_checkpoint(self, checkpoint_id):
        if str(checkpoint_id) in self.checkpoints:
            for checkpoint in self.checkpoints.values():
                checkpoint['connections'] = [
                    conn for conn in checkpoint['connections']
                    if conn['to_checkpoint_id'] != int(checkpoint_id)
                ]
            
            del self.checkpoints[str(checkpoint_id)]
            self.save_data()
            return True
        return False 