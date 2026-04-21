from learning.meta_controller import RiskMetaController
from planning.path_finder import a_star_search

class AutonomousAgent:
    def __init__(self, name, start, goal, color, path_color, sensing_radius=2):
        self.name = name
        self.current_pos = start
        self.goal = goal
        self.color = color
        self.path_color = path_color
        self.sensing_radius = sensing_radius
        
        self.path = []
        self.step = 0
        self.status = "Normal"
        
        self.rl_agent = RiskMetaController(epsilon=0.2)
        self.current_weight = 10
        self.current_state = "LOW_RISK"

    def plan_initial_path(self, grid, shared_belief_map):
        self.path = a_star_search(grid, self.current_pos, self.goal, shared_belief_map, self.current_weight) or []

    def check_v2v_alert(self, shared_belief_map, assessor, grid):
        if not self.path: return
        check_range = min(5, len(self.path))
        for i in range(check_range):
            px, py = self.path[i]
            if assessor.is_risky(shared_belief_map[px, py]):
                self.path = a_star_search(grid, self.current_pos, self.goal, shared_belief_map, self.current_weight) or []
                self.status = "V2V Replanning"
                break

    def sense_environment(self, world_grid, moving_obstacles, shared_belief_map, assessor):
        cx, cy = self.current_pos
        need_replan = False
        
        for dx in range(-self.sensing_radius, self.sensing_radius + 1):
            for dy in range(-self.sensing_radius, self.sensing_radius + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < world_grid.shape[0] and 0 <= ny < world_grid.shape[1]:
                    # Phát hiện vật cản tĩnh hoặc động
                    if world_grid[nx, ny] == 1 or (nx, ny) in moving_obstacles:
                        new_belief = assessor.update_belief(shared_belief_map[nx, ny], 1)
                        shared_belief_map[nx, ny] = new_belief
                        if assessor.is_risky(new_belief) and (nx, ny) in self.path:
                            need_replan = True
                            
        if need_replan:
            self.path = a_star_search(world_grid, self.current_pos, self.goal, shared_belief_map, self.current_weight)
            if not self.path:
                self.status = "EMERGENCY_BRAKE"
            else:
                self.status = "Replanning"