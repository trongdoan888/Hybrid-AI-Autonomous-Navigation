import heapq
import math

def heuristic(a, b, belief_map=None, weight=15):
    """
    Hàm Heuristic nâng cao: Manhattan Distance + Risk Penalty
    """
    # 1. Tính khoảng cách Manhattan cơ bản
    dist = abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    # 2. Nâng cấp: Cộng thêm trọng số rủi ro từ bản đồ niềm tin (Bayesian)
    risk_penalty = 0
    if belief_map is not None:
        # Lấy xác suất rủi ro tại ô (a[0], a[1])
        # Xác suất càng cao, chi phí ước lượng càng lớn
        risk_val = belief_map[a[0], a[1]]
        
        # SỬA ĐỔI: Nhân với trọng số động (weight) do AI quyết định thay vì fix cứng số 15
        risk_penalty = risk_val * weight 
        
    return dist + risk_penalty

def a_star_search(grid, start, goal, belief_map=None, current_weight=15):
    # NÂNG CẤP 1: Khai báo 8 hướng di chuyển
    # (dx, dy, cost)
    neighbors = [
        (0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),             # 4 hướng thẳng
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414) # 4 hướng chéo
    ]
    
    close_set = set()
    came_from = {}
    g_score = {start: 0}
    
    # SỬA ĐỔI: Truyền current_weight vào hàm heuristic
    f_score = {start: heuristic(start, goal, belief_map, current_weight)}
    
    o_heap = []
    heapq.heappush(o_heap, (f_score[start], start))
    
    while o_heap:
        current = heapq.heappop(o_heap)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        close_set.add(current)
        
        for i, j, cost in neighbors: # Lấy thêm biến cost
            neighbor = current[0] + i, current[1] + j
            
            # Kiểm tra biên
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1: 
                    continue 
                
                # NÂNG CẤP 2: Chống đi xuyên góc (Anti-corner-cutting)
                # Nếu đi chéo, cả 2 ô liền kề tạo nên góc chéo đó phải trống
                if i != 0 and j != 0:
                    if grid[current[0]+i][current[1]] == 1 or grid[current[0]][current[1]+j] == 1:
                        continue
            else: 
                continue

            # NÂNG CẤP 3: Cộng chi phí thực tế (1 hoặc 1.414) thay vì luôn cộng 1
            tentative_g_score = g_score[current] + cost
            
            if neighbor in close_set and tentative_g_score >= g_score.get(neighbor, 0):
                continue
                
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                
                # SỬA ĐỔI: Truyền current_weight vào hàm heuristic cho các bước lặp
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal, belief_map, current_weight)
                
                if neighbor not in [n[1] for n in o_heap]:
                    heapq.heappush(o_heap, (f_score[neighbor], neighbor))
                
    return False

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    return path[::-1]