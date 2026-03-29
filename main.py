import pygame
import numpy as np
import time
import random
from environment.grid_world import GridWorld
from planning.path_finder import a_star_search
from perception.bayesian_filter import BayesianRiskAssessor
from utils.helpers import save_logs

# --- Cấu hình giao diện ---
WIDTH, HEIGHT = 600, 600
GRID_SIZE = 10
CELL_SIZE = WIDTH // GRID_SIZE

# Màu sắc
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)      # Vật cản cứng (Grid = 1)
GRAY_LIGHT = (230, 230, 230) # Lưới mặc định
GRAY_RISK = (180, 180, 180)  # Ô có rủi ro (Belief > 0.4)
RED   = (255, 0, 0)    # Xe
GREEN = (0, 255, 0)    # Đích
BLUE  = (0, 100, 255)  # Lộ trình
GRID_LINE = (210, 210, 210)

def draw_grid(screen, world, path, current_pos, goal, belief_map):
    screen.fill(WHITE)
    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            rect = pygame.Rect(y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            
            # Vẽ dựa trên trạng thái vật cản và bản đồ niềm tin
            risk = belief_map[x, y]
            if world.grid[x, y] == 1:
                pygame.draw.rect(screen, BLACK, rect)
            elif risk > 0.4: # Ô đang bị nghi ngờ (màu xám trung bình)
                pygame.draw.rect(screen, GRAY_RISK, rect)
            
            pygame.draw.rect(screen, GRID_LINE, rect, 1)

    # Vẽ lộ trình dự kiến
    if path:
        for p in path:
            center = (p[1] * CELL_SIZE + CELL_SIZE // 2, p[0] * CELL_SIZE + CELL_SIZE // 2)
            pygame.draw.circle(screen, BLUE, center, CELL_SIZE // 6)

    # Vẽ đích và xe
    pygame.draw.rect(screen, GREEN, pygame.Rect(goal[1] * CELL_SIZE, goal[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
    car_center = (current_pos[1] * CELL_SIZE + CELL_SIZE // 2, current_pos[0] * CELL_SIZE + CELL_SIZE // 2)
    pygame.draw.circle(screen, RED, car_center, CELL_SIZE // 3)
    
    pygame.display.flip()

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Hybrid AI: Dynamic Obstacles & Risk-Aware")

    world = GridWorld(GRID_SIZE, GRID_SIZE)
    assessor = BayesianRiskAssessor(threshold=0.7)
    
    start, goal = (0, 0), (9, 9)
    current_pos = start
    
    # Tạo 10 vật cản tĩnh ngẫu nhiên lúc khởi tạo để mỗi lần chạy mỗi khác
    for _ in range(10):
        rx, ry = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (rx, ry) != start and (rx, ry) != goal:
            world.set_obstacle(rx, ry)

    belief_map = np.full((GRID_SIZE, GRID_SIZE), 0.1)
    path = a_star_search(world.grid, start, goal, belief_map) or []

    logs = []
    running = True
    step = 0

    print("🚀 Bắt đầu mô phỏng. Quan sát các ô màu xám (rủi ro) và đen (vật cản mới)...")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                if logs: save_logs(logs)
                running = False

        if current_pos != goal and running:
            status = "Normal"
            
            # --- NÂNG CẤP: TẠO VẬT CẢN ĐỘNG TRÊN LỘ TRÌNH (SENSE) ---
            # Xác suất 40% xuất hiện vật cản ở các ô sắp đi tới
            if random.random() < 0.4 and len(path) > 2:
                # Chọn ngẫu nhiên 1 trong 3 ô kế tiếp trên đường đi để giả lập vật cản xuất hiện
                idx = random.randint(0, 2)
                rx, ry = path[idx]
                
                # Cập nhật Bayesian
                new_belief = assessor.update_belief(belief_map[rx, ry], 1)
                belief_map[rx, ry] = new_belief
                
                if assessor.is_risky(new_belief):
                    print(f"⚠️ Phát hiện rủi ro cao tại ({rx}, {ry})! Tính lại đường...")
                    world.set_obstacle(rx, ry)
                    path = a_star_search(world.grid, current_pos, goal, belief_map) or []
                    status = "Replanning"

            # --- BƯỚC 2: ACT ---
            if path:
                logs.append({
                    "Step": step,
                    "Position": f"({current_pos[0]},{current_pos[1]})",
                    "Risk": round(belief_map[current_pos], 2),
                    "Status": status
                })
                current_pos = path.pop(0)
                step += 1
            
            # Cập nhật màn hình (nhớ truyền thêm belief_map)
            draw_grid(screen, world, path, current_pos, goal, belief_map)
            time.sleep(0.4) 
            
        elif current_pos == goal:
            print("🏁 Chúc mừng Trọng, xe đã về đích!")
            if logs: save_logs(logs)
            time.sleep(2)
            running = False

    pygame.quit()

if __name__ == "__main__":
    main()