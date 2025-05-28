from math import sqrt
from typing import List, Dict, Optional, Tuple
import numpy as np

class CheckpointNode:
    def __init__(self, checkpoint_id: int, position: Tuple[float, float, float], 
                 orientation: Tuple[float, float, float],
                 passage_info: Optional[Dict] = None):
        self.id = checkpoint_id
        self.position = position  
        self.orientation = orientation 
        self.passage_info = passage_info or {} 
        self.connections: List[CheckpointNode] = []
        self.transition_type = "straight" 
        self.approach_angle = 0.0  
        self.exit_angle = 0.0  

        self.potential_field = {
            'x_range': 0.3, 
            'y_range': 0.3,  
            'z_range': 0.2, 
            'lidar_left': 0.7,  
            'lidar_right': 0.3, 
            'is_in_field': False,  
            'lidar_left': 0.0,
            'lidar_right': 0.0
        }
        self.is_checkpoint = True 
        self.optimal_speed = 0.3 
        self.stabilization_time = 0.2 
        self.exit_speed = 0.3  

    def calculate_transition_type(self, next_node: 'CheckpointNode') -> str:
        angle_diff = abs(self.orientation[2] - next_node.orientation[2])
        
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
            
        if abs(angle_diff) < 0.1: 
            return "straight"
        elif angle_diff > 0:
            return "turn_left"
        else:
            return "turn_right"

    def calculate_approach_angle(self, next_node: 'CheckpointNode') -> float:
        dx = next_node.position[0] - self.position[0]
        dy = next_node.position[1] - self.position[1]
        return np.arctan2(dy, dx)

    def calculate_exit_angle(self, next_node: 'CheckpointNode') -> float:
        return next_node.calculate_approach_angle(self)



def create_checkpoint_connections(checkpoints: List[CheckpointNode], max_distance: float = 10.0) -> None:
    for checkpoint in checkpoints:
        checkpoint.connections = []
    
    for i in range(len(checkpoints) - 1):
        current = checkpoints[i]
        next_cp = checkpoints[i + 1]
        
        transition_type = current.calculate_transition_type(next_cp)
        approach_angle = current.calculate_approach_angle(next_cp)
        exit_angle = current.calculate_exit_angle(next_cp)
        
        current.transition_type = transition_type
        current.approach_angle = approach_angle
        current.exit_angle = exit_angle
        
        current.connections = [next_cp]  
        print(f"Checkpoint {current.id} -> {next_cp.id} bağlantısı oluşturuldu")


def calculate_median_height(checkpoints: List[CheckpointNode]) -> float:
    heights = [cp.position[2] for cp in checkpoints]
    heights.sort()
    n = len(heights)
    if n % 2 == 0:
        return (heights[n//2 - 1] + heights[n//2]) / 2
    return heights[n//2]

def is_in_potential_field(drone_pos: Tuple[float, float, float], 
                         checkpoint: CheckpointNode) -> bool:
    x, y, z = drone_pos
    cp_x, cp_y, cp_z = checkpoint.position
    
    x_in_range = abs(x - cp_x) <= checkpoint.potential_field['x_range']
    y_in_range = abs(y - cp_y) <= checkpoint.potential_field['y_range']
    z_in_range = abs(z - cp_z) <= checkpoint.potential_field['z_range']
    
    return x_in_range and y_in_range and z_in_range

def calculate_optimal_intermediate_point(current: CheckpointNode, next_cp: CheckpointNode, 
                                       next_next_cp: CheckpointNode = None) -> Tuple[float, float, float]:
    base_x, base_y, base_z = next_cp.position

    if next_next_cp:
        dx = next_next_cp.position[0] - next_cp.position[0]
        dy = next_next_cp.position[1] - next_cp.position[1]
        angle = np.arctan2(dy, dx)
        
        dx2 = next_cp.position[0] - current.position[0]
        dy2 = next_cp.position[1] - current.position[1]
        angle2 = np.arctan2(dy2, dx2)
        
        angle_diff = angle - angle2
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        if abs(angle_diff) > 0.1:
            offset = 0.2  
            if angle_diff > 0:  
                base_x += offset * np.cos(angle + np.pi/2)
                base_y += offset * np.sin(angle + np.pi/2)
            else:  
                base_x += offset * np.cos(angle - np.pi/2)
                base_y += offset * np.sin(angle - np.pi/2)
    
    return (base_x, base_y, base_z)

def a_star_pathfinding(start_checkpoint: CheckpointNode, 
                      goal_checkpoint: CheckpointNode,
                      all_checkpoints: List[CheckpointNode]) -> Optional[List[CheckpointNode]]:
    sorted_checkpoints = sorted(all_checkpoints, key=lambda x: x.id)
    
    start_idx = next(i for i, cp in enumerate(sorted_checkpoints) if cp.id == start_checkpoint.id)
    goal_idx = next(i for i, cp in enumerate(sorted_checkpoints) if cp.id == goal_checkpoint.id)
    
    path = []
    current_idx = start_idx
    
    while current_idx <= goal_idx:
        current = sorted_checkpoints[current_idx]
        path.append(current)
        
        if current_idx + 1 <= goal_idx:
            next_cp = sorted_checkpoints[current_idx + 1]
            next_next_cp = sorted_checkpoints[current_idx + 2] if current_idx + 2 <= goal_idx else None
            
            optimal_pos = calculate_optimal_intermediate_point(current, next_cp, next_next_cp)
            
            if optimal_pos != next_cp.position:
                intermediate = CheckpointNode(
                    next_cp.id,
                    optimal_pos,
                    next_cp.orientation,
                    next_cp.passage_info
                )
                intermediate.potential_field = next_cp.potential_field.copy()
                path.append(intermediate)
                print(f"Checkpoint {current.id} -> {next_cp.id} arası optimal ara nokta: "
                      f"X={optimal_pos[0]:.2f}, Y={optimal_pos[1]:.2f}, Z={optimal_pos[2]:.2f}")
        
        current_idx += 1
    
    return path


def find_path_between_checkpoints(start_id: int, goal_id: int, 
                                checkpoints: List[CheckpointNode]) -> Optional[List[CheckpointNode]]:
    start_checkpoint = next((cp for cp in checkpoints if cp.id == start_id), None)
    goal_checkpoint = next((cp for cp in checkpoints if cp.id == goal_id), None)
    
    if not start_checkpoint or not goal_checkpoint:
        print(f"Checkpoint'ler bulunamadı: {start_id} -> {goal_id}")
        return None
    
    create_checkpoint_connections(checkpoints)
    
    path = a_star_pathfinding(start_checkpoint, goal_checkpoint, checkpoints)
    
    if not path:
        print("Yol bulunamadı!")
        return None
    
    print(f"\nBulunan yol: {' -> '.join(str(cp.id) for cp in path)}")
    return path 