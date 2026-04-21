import pygame
import numpy as np
import time
import random
from environment.grid_world import GridWorld
from planning.kinematics import smooth_path_chaikin
from planning.path_finder import a_star_search
from perception.bayesian_filter import BayesianRiskAssessor
from utils.helpers import save_logs
from entities.agent import AutonomousAgent

# --- CẤU HÌNH GIAO DIỆN MỚI ---
WIDTH = 700 # Mở rộng sa bàn cho dễ nhìn
UI_HEIGHT = 160 # Vùng UI gọn gàng
HEIGHT = WIDTH + UI_HEIGHT 
GRID_SIZE = 15
CELL_SIZE = WIDTH // GRID_SIZE

# Biến điều khiển
SIMULATION_DELAY = 0.5 # Mặc định 0.5s/bước (Chậm và dễ nhìn)
IS_PAUSED = True       # BẮT ĐẦU Ở TRẠNG THÁI TẠM DỪNG

# Bảng màu Modern Dark Theme
WHITE = (255, 255, 255)
BLACK = (20, 20, 25)
GRAY_RISK = (180, 180, 190)
GRID_LINE = (230, 230, 235)

UI_BG = (30, 32, 40)          # Nền bảng điều khiển (Dark Slate)
BTN_DEFAULT = (50, 55, 70)    # Nút bấm thường
BTN_HOVER = (70, 75, 95)      # Nút bấm khi hover chuột
BTN_PLAY = (46, 204, 113)     # Màu Xanh lá (Tiếp tục)
BTN_PAUSE = (241, 196, 15)    # Màu Vàng (Tạm dừng)
BTN_DANGER = (231, 76, 60)    # Màu Đỏ (Reset)

CAR_1_COLOR = (255, 50, 50)     
PATH_1_COLOR = (255, 170, 170) 
CAR_2_COLOR = (50, 100, 255)     
PATH_2_COLOR = (150, 180, 255) 
GREEN = (46, 204, 113)           

# Biến toàn cục hệ thống
world = None
shared_belief_map = None
agents = []
logs = []
is_finished = False 

# ==========================================
# CÁC LỚP UI COMPONENTS (ĐƯỢC LÀM ĐẸP)
# ==========================================
class Button:
    def __init__(self, x, y, w, h, text, default_color, hover_color=BTN_HOVER):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.default_color = default_color
        self.current_color = default_color
        self.hover_color = hover_color
        self.is_hovered = False

    def draw(self, screen, font):
        # Đổ bóng nhẹ
        shadow_rect = self.rect.copy()
        shadow_rect.y += 3
        pygame.draw.rect(screen, (15, 15, 20), shadow_rect, border_radius=8)
        
        # Vẽ nút chính
        color = self.hover_color if self.is_hovered else self.current_color
        pygame.draw.rect(screen, color, self.rect, border_radius=8)
        
        text_surf = font.render(self.text, True, WHITE)
        text_rect = text_surf.get_rect(center=self.rect.center)
        screen.blit(text_surf, text_rect)

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.is_hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.is_hovered:
                return True
        return False

class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, start_val, label):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = start_val
        self.label = label
        self.grabbed = False
        self.handle_w = 18

    def draw(self, screen, font):
        # Nhãn
        label_surf = font.render(f"{self.label}: {round(self.val, 2)}", True, (200, 210, 220))
        screen.blit(label_surf, (self.rect.x, self.rect.y - 30))
        
        # Rãnh trượt
        pygame.draw.rect(screen, (20, 22, 30), self.rect, border_radius=5)
        
        # Con chạy
        pos_x = self.rect.x + (self.val - self.min_val) / (self.max_val - self.min_val) * self.rect.w
        handle_rect = pygame.Rect(pos_x - self.handle_w//2, self.rect.y - 6, self.handle_w, self.rect.h + 12)
        
        h_color = (100, 150, 255) if self.grabbed else (180, 190, 210)
        pygame.draw.rect(screen, h_color, handle_rect, border_radius=6)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos_x = self.rect.x + (self.val - self.min_val) / (self.max_val - self.min_val) * self.rect.w
            handle_rect = pygame.Rect(pos_x - self.handle_w//2, self.rect.y - 6, self.handle_w, self.rect.h + 12)
            if handle_rect.collidepoint(event.pos) or self.rect.collidepoint(event.pos):
                self.grabbed = True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.grabbed = False
        elif event.type == pygame.MOUSEMOTION:
            if self.grabbed:
                mouse_x = max(self.rect.x, min(event.pos[0], self.rect.x + self.rect.w))
                ratio = (mouse_x - self.rect.x) / self.rect.w
                self.val = self.min_val + ratio * (self.max_val - self.min_val)

# ==========================================
# LOGIC HỆ THỐNG
# ==========================================
def reset_simulation(new_size):
    global GRID_SIZE, CELL_SIZE, world, shared_belief_map, agents, logs, is_finished, IS_PAUSED
    GRID_SIZE = int(new_size)
    CELL_SIZE = WIDTH // GRID_SIZE
    world = GridWorld(GRID_SIZE, GRID_SIZE)
    shared_belief_map = np.full((GRID_SIZE, GRID_SIZE), 0.1)
    logs = []
    is_finished = False
    IS_PAUSED = True # Tự động Pause khi Reset
    
    agent1 = AutonomousAgent("Xe Đỏ", (0, 0), (GRID_SIZE-1, GRID_SIZE-1), CAR_1_COLOR, PATH_1_COLOR, sensing_radius=2)
    agent2 = AutonomousAgent("Xe Xanh", (GRID_SIZE-1, 0), (0, GRID_SIZE-1), CAR_2_COLOR, PATH_2_COLOR, sensing_radius=2)
    
    agents = [agent1, agent2]
    for agent in agents:
        agent.plan_initial_path(world.grid, shared_belief_map)

def main():
    global SIMULATION_DELAY, GRID_SIZE, logs, is_finished, IS_PAUSED
    pygame.init()
    font_main = pygame.font.SysFont('Segoe UI', 16, bold=True)
    font_alert = pygame.font.SysFont('Segoe UI', 22, bold=True)
    
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Hybrid AI Navigation V3.1 - Interactive Dashboard")

    assessor = BayesianRiskAssessor(threshold=0.7)
    reset_simulation(GRID_SIZE)
    moving_obstacles = [(GRID_SIZE//2, GRID_SIZE//2)]
    
    # Khởi tạo UI (Sắp xếp lại cho gọn gàng)
    btn_play = Button(30, WIDTH + 25, 130, 45, "▶ TIẾP TỤC", BTN_PLAY)
    btn_reset = Button(180, WIDTH + 25, 120, 45, "🔄 RESET", BTN_DANGER)
    btn_random = Button(320, WIDTH + 25, 140, 45, "🎲 5 VẬT CẢN", BTN_DEFAULT)
    
    slider_speed = Slider(30, WIDTH + 120, 250, 8, 0.05, 2.0, SIMULATION_DELAY, "Độ trễ (Giây/Bước)")
    slider_size = Slider(320, WIDTH + 120, 250, 8, 10, 25, GRID_SIZE, "Kích thước Sa Bàn")
    
    is_mouse_pressed = False
    running = True

    while running:
        # Cập nhật màu nút Play/Pause
        if IS_PAUSED:
            btn_play.text = "▶ TIẾP TỤC"
            btn_play.current_color = BTN_PLAY
        else:
            btn_play.text = "⏸ TẠM DỪNG"
            btn_play.current_color = BTN_PAUSE

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                if logs: save_logs(logs)
                running = False
            
            # Xử lý Nút bấm
            if btn_play.handle_event(event):
                if not is_finished: IS_PAUSED = not IS_PAUSED
            
            if btn_reset.handle_event(event):
                reset_simulation(int(slider_size.val))
                
            if btn_random.handle_event(event):
                for _ in range(5):
                    rx, ry = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
                    if world.grid[rx, ry] == 0 and (rx, ry) not in [a.current_pos for a in agents]:
                        world.set_obstacle(rx, ry)
                        shared_belief_map[rx, ry] = 1.0
                for agent in agents:
                    agent.path = a_star_search(world.grid, agent.current_pos, agent.goal, shared_belief_map, agent.current_weight) or []

            # Xử lý Slider
            slider_speed.handle_event(event)
            slider_size.handle_event(event)
            SIMULATION_DELAY = slider_speed.val 

            # Xử lý chuột vẽ vật cản (Chỉ cho phép vẽ khi Pause để tránh lỗi xung đột)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                is_mouse_pressed = True
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                is_mouse_pressed = False

        if is_mouse_pressed and IS_PAUSED:
            mx, my = pygame.mouse.get_pos()
            if my < WIDTH: 
                gx, gy = my // CELL_SIZE, mx // CELL_SIZE
                if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE and world.grid[gx, gy] == 0:
                    world.set_obstacle(gx, gy)
                    shared_belief_map[gx, gy] = 1.0
                    for agent in agents:
                        if (gx, gy) in agent.path:
                            agent.path = a_star_search(world.grid, agent.current_pos, agent.goal, shared_belief_map, agent.current_weight) or []
                            if not agent.path: agent.status = "EMERGENCY_BRAKE"

        # --- CHỈ CHẠY LOGIC AI KHI KHÔNG TẠM DỪNG ---
        has_emergency = False
        
        if not IS_PAUSED and not is_finished:
            # Di chuyển vật cản động
            new_moving = []
            for mx, my in moving_obstacles:
                if world.grid[mx, my] == 0: shared_belief_map[mx, my] = 0.1
                dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]
                random.shuffle(dirs)
                moved = False
                for dx, dy in dirs:
                    nx, ny = mx + dx, my + dy
                    if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and world.grid[nx, ny] == 0:
                        new_moving.append((nx, ny))
                        moved = True
                        break
                if not moved: new_moving.append((mx, my))
            moving_obstacles = new_moving

            # Điều khiển xe
            all_reached = True
            for agent in agents:
                if agent.current_pos != agent.goal:
                    all_reached = False
                    if agent.status == "EMERGENCY_BRAKE":
                        has_emergency = True
                        continue
                    
                    old_state = agent.current_state
                    agent.current_state, agent.current_weight = agent.rl_agent.choose_weight(agent.current_pos, shared_belief_map)
                    
                    agent.check_v2v_alert(shared_belief_map, assessor, world.grid)
                    agent.sense_environment(world.grid, moving_obstacles, shared_belief_map, assessor)

                    if agent.path:
                        agent.current_pos = agent.path.pop(0)
                        agent.step += 1
                        logs.append({"Step": agent.step, "Agent": agent.name, "Pos": f"({agent.current_pos[0]},{agent.current_pos[1]})", "Risk": round(float(shared_belief_map[agent.current_pos]), 2), "RL_Weight": agent.current_weight, "Status": agent.status, "GridSize": GRID_SIZE})

            if all_reached:
                print("🏁 Đã về đích!")
                if logs: save_logs(logs)
                is_finished = True
                IS_PAUSED = True

        # ==========================
        # VẼ GIAO DIỆN
        # ==========================
        screen.fill(WHITE)
        
        # 1. Vẽ lưới sa bàn
        for x in range(GRID_SIZE):
            for y in range(GRID_SIZE):
                rect = pygame.Rect(y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                if world.grid[x, y] == 1: pygame.draw.rect(screen, BLACK, rect)
                elif shared_belief_map[x, y] > 0.4: pygame.draw.rect(screen, GRAY_RISK, rect)
                pygame.draw.rect(screen, GRID_LINE, rect, 1)

        for mx, my in moving_obstacles:
            pygame.draw.rect(screen, (243, 156, 18), (my * CELL_SIZE, mx * CELL_SIZE, CELL_SIZE, CELL_SIZE), border_radius=4)

        for agent in agents:
            pygame.draw.rect(screen, GREEN, (agent.goal[1] * CELL_SIZE, agent.goal[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE), border_radius=4)
            if agent.path and len(agent.path) > 1:
                pts = [(p[1] * CELL_SIZE + CELL_SIZE//2, p[0] * CELL_SIZE + CELL_SIZE//2) for p in smooth_path_chaikin(agent.path)]
                pygame.draw.lines(screen, agent.path_color, False, pts, 4)
            pygame.draw.circle(screen, agent.color, (agent.current_pos[1]*CELL_SIZE+CELL_SIZE//2, agent.current_pos[0]*CELL_SIZE+CELL_SIZE//2), CELL_SIZE//3 + 2)

        # 2. Vẽ Control Panel
        pygame.draw.rect(screen, UI_BG, (0, WIDTH, WIDTH, UI_HEIGHT))
        pygame.draw.line(screen, (50, 52, 60), (0, WIDTH), (WIDTH, WIDTH), 3)
        
        btn_play.draw(screen, font_main)
        btn_reset.draw(screen, font_main)
        btn_random.draw(screen, font_main)
        slider_speed.draw(screen, font_main)
        slider_size.draw(screen, font_main)

        # Trạng thái hệ thống
        if IS_PAUSED and not is_finished:
            status_text = font_alert.render("⏸ ĐANG TẠM DỪNG - HÃY VẼ VẬT CẢN", True, (241, 196, 15))
            screen.blit(status_text, (480, WIDTH + 35))
        if has_emergency:
            err_text = font_alert.render("🚨 KẸT ĐƯỜNG!", True, (231, 76, 60))
            screen.blit(err_text, (480, WIDTH + 35))
        elif is_finished:
            win_text = font_alert.render("🏁 HOÀN THÀNH!", True, (46, 204, 113))
            screen.blit(win_text, (480, WIDTH + 35))

        pygame.display.flip()
        
        if not IS_PAUSED and not is_finished:
            time.sleep(SIMULATION_DELAY)

    pygame.quit()

if __name__ == "__main__":
    main()