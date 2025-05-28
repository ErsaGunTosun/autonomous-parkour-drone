from dataclasses import dataclass
from typing import List, Dict, Tuple
import json
import time
from datetime import datetime

@dataclass
class RoutePoint:
    timestamp: float
    position: Tuple[float, float, float] 
    orientation: Tuple[float, float, float] 
    velocity: Tuple[float, float, float]  
    lidar_readings: Dict[str, float] 
    checkpoint_id: int = None  
    is_checkpoint: bool = False  

    def to_dict(self) -> Dict:
        return {
            'timestamp': self.timestamp,
            'position': self.position,
            'orientation': self.orientation,
            'velocity': self.velocity,
            'lidar_readings': self.lidar_readings,
            'checkpoint_id': self.checkpoint_id,
            'is_checkpoint': self.is_checkpoint
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'RoutePoint':
        return cls(
            timestamp=data['timestamp'],
            position=data['position'],
            orientation=data['orientation'],
            velocity=data['velocity'],
            lidar_readings=data['lidar_readings'],
            checkpoint_id=data.get('checkpoint_id'),
            is_checkpoint=data.get('is_checkpoint', False)
        )

@dataclass
class RouteSegment:
    start_point: RoutePoint
    end_point: RoutePoint
    duration: float 
    distance: float  
    avg_speed: float  

class RouteRecorder:
    def __init__(self):
        self.current_route: List[RoutePoint] = []
        self.route_segments: List[RouteSegment] = []
        self.current_lap: int = 0
        self.lap_start_time: float = 0
        self.checkpoint_times: Dict[int, float] = {} 
        self.is_recording: bool = False
        self.last_record_time: float = 0
        self.min_record_interval: float = 0.1  
        self.significant_change_threshold: float = 0.05  
        self.last_position: Tuple[float, float, float] = None
        
    def start_recording(self, lap_number: int):
        self.current_route = []
        self.route_segments = []
        self.current_lap = lap_number
        self.lap_start_time = 0  
        self.checkpoint_times = {}
        self.is_recording = True
        self.last_record_time = 0
        self.last_position = None
        print(f"Tur {lap_number} kaydı başlatıldı")
        
    def stop_recording(self):
        if not self.is_recording:
            return
            
        self.is_recording = False
        self.save_route_data()
        
        print(f"Tur {self.current_lap} kaydı tamamlandı.")
        
    def record_point(self, drone) -> RoutePoint:
        if not self.is_recording:
            return None
            
        current_time = drone.robot.getTime()
        
        if self.lap_start_time == 0:
            self.lap_start_time = current_time
            
        position = drone.gps.getValues()
        orientation = drone.imu.getRollPitchYaw()
        velocity = drone.gyro.getValues()
        
        lidar_readings = {
            "front": drone.front_lidar.getValue() / 1000,
            "back": drone.back_lidar.getValue() / 1000,
            "left": drone.left_lidar.getValue() / 1000,
            "right": drone.right_lidar.getValue() / 1000
        }
        
        checkpoint_id = None
        is_checkpoint = False
        if hasattr(drone, 'current_checkpoint'):
            checkpoint_id = drone.current_checkpoint
            is_checkpoint = True
            self.checkpoint_times[checkpoint_id] = current_time - self.lap_start_time
        

        should_record = False
        
        if current_time - self.last_record_time >= self.min_record_interval:
            should_record = True
            
        if self.last_position is not None:
            distance = self._calculate_distance(position, self.last_position)
            if distance >= self.significant_change_threshold:
                should_record = True
                
        if is_checkpoint:
            should_record = True
            
        if not should_record:
            return None
            
        point = RoutePoint(
            timestamp=current_time - self.lap_start_time,
            position=tuple(position),
            orientation=tuple(orientation),
            velocity=tuple(velocity),
            lidar_readings=lidar_readings,
            checkpoint_id=checkpoint_id,
            is_checkpoint=is_checkpoint
        )
        
        self.current_route.append(point)
        self.last_record_time = current_time
        self.last_position = position
        
        if len(self.current_route) > 1:
            prev_point = self.current_route[-2]
            segment = self._create_segment(prev_point, point)
            self.route_segments.append(segment)
        
        return point
    
    def _create_segment(self, start: RoutePoint, end: RoutePoint) -> RouteSegment:
        duration = end.timestamp - start.timestamp
        distance = self._calculate_distance(start.position, end.position)
        avg_speed = distance / duration if duration > 0 else 0
        
        return RouteSegment(
            start_point=start,
            end_point=end,
            duration=duration,
            distance=distance,
            avg_speed=avg_speed
        )
    
    def _calculate_distance(self, pos1: Tuple[float, float, float], 
                          pos2: Tuple[float, float, float]) -> float:
        return ((pos1[0] - pos2[0])**2 + 
                (pos1[1] - pos2[1])**2 + 
                (pos1[2] - pos2[2])**2)**0.5
    
    def save_route_data(self):
        if not self.current_route:
            return
            
        route_data = {
            "lap_number": self.current_lap,
            "start_time": datetime.fromtimestamp(self.lap_start_time).isoformat(),
            "checkpoint_times": self.checkpoint_times,
            "points": [p.to_dict() for p in self.current_route],
            "segments": [
                {
                    "start_time": s.start_point.timestamp,
                    "end_time": s.end_point.timestamp,
                    "duration": s.duration,
                    "distance": s.distance,
                    "avg_speed": s.avg_speed
                }
                for s in self.route_segments
            ]
        }
        
        filename = f"route_data_lap_{self.current_lap}.json"
        with open(filename, 'w') as f:
            json.dump(route_data, f, indent=2)
            
        print(f"Rota verileri {filename} dosyasına kaydedildi")
    
    def get_lap_statistics(self) -> Dict:
        if not self.current_route:
            return {}
            
        total_distance = sum(s.distance for s in self.route_segments)
        total_duration = self.current_route[-1].timestamp
        avg_speed = total_distance / total_duration if total_duration > 0 else 0
        
        return {
            "lap_number": self.current_lap,
            "total_distance": total_distance,
            "total_duration": total_duration,
            "average_speed": avg_speed,
            "checkpoint_times": self.checkpoint_times,
            "number_of_points": len(self.current_route),
            "number_of_segments": len(self.route_segments)
        } 