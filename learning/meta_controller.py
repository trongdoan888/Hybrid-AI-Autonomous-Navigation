import numpy as np
import random

class RiskMetaController:
    def __init__(self, epsilon=0.2, alpha=0.1, gamma=0.9):
        # Các mức trọng số w mà AI có thể chọn
        # 1: Liều lĩnh, 10: Cân bằng, 25: Cực kỳ an toàn
        self.actions = [1, 10, 25] 
        
        # Q-Table: Lưu trữ điểm số của từng hành động ứng với từng trạng thái
        self.q_table = {} 
        
        self.epsilon = epsilon  # Tỉ lệ khám phá (Thử nghiệm w mới)
        self.alpha = alpha      # Learning rate (Tốc độ học)
        self.gamma = gamma      # Discount factor (Tầm nhìn xa)

    def _get_state(self, current_pos, belief_map):
        """
        Trạng thái (State) là tổng rủi ro ở 8 ô xung quanh xe.
        Chúng ta gom nhóm rủi ro thành 3 mức: 0 (Thấp), 1 (Vừa), 2 (Cao) để Q-Table không bị quá lớn.
        """
        cx, cy = current_pos
        local_risk = 0
        
        # Quét khu vực 3x3 xung quanh xe
        for i in range(-1, 2):
            for j in range(-1, 2):
                nx, ny = cx + i, cy + j
                if 0 <= nx < belief_map.shape[0] and 0 <= ny < belief_map.shape[1]:
                    local_risk += belief_map[nx, ny]
                    
        # Phân loại trạng thái
        if local_risk < 0.5: return "LOW_RISK"
        elif local_risk < 2.0: return "MEDIUM_RISK"
        else: return "HIGH_RISK"

    def choose_weight(self, current_pos, belief_map):
        state = self._get_state(current_pos, belief_map)
        
        # Khởi tạo state trong Q-Table nếu chưa có
        if state not in self.q_table:
            self.q_table[state] = {action: 0.0 for action in self.actions}

        # Epsilon-Greedy: Đôi khi thử một trọng số ngẫu nhiên để học cái mới
        if random.uniform(0, 1) < self.epsilon:
            chosen_w = random.choice(self.actions)
        else:
            # Chọn trọng số có điểm Q cao nhất cho trạng thái hiện tại
            chosen_w = max(self.q_table[state], key=self.q_table[state].get)
            
        return state, chosen_w

    def learn(self, state, action, reward, next_state):
        """Cập nhật Q-Table dựa trên phần thưởng nhận được"""
        if next_state not in self.q_table:
            self.q_table[next_state] = {a: 0.0 for a in self.actions}
            
        # Công thức Bellman (Cốt lõi của Q-Learning)
        old_q = self.q_table[state][action]
        next_max_q = max(self.q_table[next_state].values())
        
        new_q = old_q + self.alpha * (reward + self.gamma * next_max_q - old_q)
        self.q_table[state][action] = new_q