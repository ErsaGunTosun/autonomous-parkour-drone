from math import sqrt

class PassagePointFilter:
    def __init__(self):
        self.last_point = None
        self.MIN_DISTANCE = 2.0  
        self.best_point = None
        self.points_in_region = []
        self.current_region_id = 0
        
    def add_point(self, point_data):

        current_pos = (point_data['position']['x'], point_data['position']['y'])
        
        if self.last_point is None:
            self.last_point = current_pos
            self.best_point = point_data
            self.points_in_region = [point_data]
            return point_data  
            
        distance = sqrt(
            (current_pos[0] - self.last_point[0])**2 +
            (current_pos[1] - self.last_point[1])**2
        )
      
        if distance < self.MIN_DISTANCE:
            self.points_in_region.append(point_data)
            
            if self.is_better_point(point_data, self.best_point):
                self.best_point = point_data
                return point_data  
                
            return None
        else:
            self.last_point = current_pos
            self.best_point = point_data
            self.points_in_region = [point_data]
            self.current_region_id += 1
            
            return point_data  
    
    def is_better_point(self, new_point, current_best):
        if current_best is None:
            return True
            
        # Compare lidar balance
        new_lidar_diff = abs(new_point['passage_info']['left_lidar'] - new_point['passage_info']['right_lidar'])
        best_lidar_diff = abs(current_best['passage_info']['left_lidar'] - current_best['passage_info']['right_lidar'])
        
        # Compare stability
        new_stability = abs(new_point['orientation']['roll']) + abs(new_point['orientation']['pitch'])
        best_stability = abs(current_best['orientation']['roll']) + abs(current_best['orientation']['pitch'])
        
        return (new_lidar_diff < best_lidar_diff) and (new_stability < best_stability)
