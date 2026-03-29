import heapq

def heuristic(a, b, belief_map=None):
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
        # Hệ số 15 giúp xe cực kỳ ưu tiên các khu vực 'sạch' (xác suất thấp)
        risk_penalty = risk_val * 15 
        
    return dist + risk_penalty

def a_star_search(grid, start, goal, belief_map=None):
    """
    Thuật toán A* nhận thêm bản đồ niềm tin để tìm đường đi an toàn nhất
    """
    neighbors = [(0,1), (0,-1), (1,0), (-1,0)] # 4 hướng di chuyển
    close_set = set()
    came_from = {}
    
    g_score = {start: 0}
    # Sử dụng hàm heuristic có tính đến rủi ro
    f_score = {start: heuristic(start, goal, belief_map)}
    
    o_heap = []
    heapq.heappush(o_heap, (f_score[start], start))
    
    while o_heap:
        # Lấy ô có f_score thấp nhất (kết hợp giữa đường ngắn và rủi ro thấp)
        current = heapq.heappop(o_heap)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        close_set.add(current)
        
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            
            # Kiểm tra biên
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                # Kiểm tra vật cản cứng (đã xác nhận)
                if grid[neighbor[0]][neighbor[1]] == 1: 
                    continue 
            else: 
                continue

            # Tính toán g_score: mỗi bước đi mặc định tốn phí là 1
            tentative_g_score = g_score[current] + 1
            
            if neighbor in close_set and tentative_g_score >= g_score.get(neighbor, 0):
                continue
                
            # Nếu tìm thấy đường đi tốt hơn hoặc ô này chưa được thăm
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                # Nâng cấp: f(n) = g(n) + h(n) (trong đó h(n) đã bao gồm Risk Penalty)
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal, belief_map)
                
                # Tránh trùng lặp trong heap (tối ưu hiệu năng)
                if neighbor not in [i[1] for i in o_heap]:
                    heapq.heappush(o_heap, (f_score[neighbor], neighbor))
                
    return False

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    return path[::-1]