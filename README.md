# 🧠 Hybrid AI Autonomous Navigation (Python)

Dự án mô phỏng agent điều hướng trong môi trường lưới có vật cản tĩnh và động, kết hợp:
- Lập kế hoạch đường đi A* (Planning)
- Ước lượng xác suất chướng ngại Bayesian (Perception)
- Cập nhật môi trường Grid World (Environment)
- Hiển thị GUI realtime bằng Pygame
- Xuất log dữ liệu thí nghiệm

---

## 📁 Cấu trúc thư mục
- `main.py`: Entry point, vòng lặp Sense-Plan-Act
- `environment/grid_world.py`: mô hình môi trường dạng lưới và chướng ngại
- `planning/path_finder.py`: thuật toán A*, hàm heuristic tích hợp rủi ro
- `perception/bayesian_filter.py`: Bayes risk update + ngưỡng rủi ro
- `utils/helpers.py`: lưu log, tính toán phụ
- `simulation_logs.csv`: (sản phẩm) log đường đi

---

## 🧩 Mô tả chức năng từng file

### `main.py`
- Khởi tạo môi trường `GridWorld` 10x10
- Tạo ngẫu nhiên 10 vật cản tĩnh
- Khởi tạo `belief_map` toàn bộ ô = 0.1
- Tạo `BayesianRiskAssessor(threshold=0.7)`
- Gọi `a_star_search(grid, start, goal, belief_map)` để xây đường ban đầu

Vòng lặp chính (Sense-Think-Act):
1. Dynamic Obstacle Sensing (Local Sensing Range): thăm dò 3 ô đầu kế tiếp trên đường đi, 40% khả năng xuất hiện chướng ngại tạm thời.
   - Đây gọi là `Local Sensing Range` (tầm nhìn cục bộ), mô phỏng LiDAR/Camera thực tế.
   - Agent không có “con mắt thần”, chỉ biết trạng thái vật cản khi đã tiến gần (tương đương 3 bước).
2. Cập nhật xác suất rủi ro bằng `update_belief` (sensor reading = 1)
3. Nếu `is_risky` -> chuyển ô thành chướng ngại thật và `replan` bằng A*
4. Di chuyển một bước theo đường, cập nhật GUI và lưu log
5. Khi tới đích, lưu `simulation_logs.csv`

---

### Heuristic details (cực kỳ quan trọng)
- `h(n)` = Manhattan(a,b) + `risk_penalty`
- `risk_penalty = belief_map[a] * w`
- Tham số `w=15` là weight điều chỉnh hành vi:
  - `w = 0`: xe chỉ tối ưu quãng đường, có thể đi qua ô có rủi ro cao (hành vi liều lĩnh)
  - `w > 10`: xe ưu tiên an toàn, tránh ô rủi ro dù đường dài hơn
- Kết hợp này là điểm “Hybrid AI”: A* (tìm đường tất định) + Bayesian belief (suy luận xác suất) để ra quyết định an toàn trong môi trường bất định.

### `environment/grid_world.py`
- `GridWorld(width, height)`: đối tượng luồng môi trường
  - `grid` numpy array, 0=trống, 1=vật cản
- `set_obstacle(x,y)`: đặt vật cản tại ô
- `is_blocked(x,y)`: kiểm tra ô chặn

### `planning/path_finder.py`
- `heuristic(a, b, belief_map=None)`: Manhattan + rủi ro
  - `risk_penalty = belief_map[a] * 15`
- `a_star_search(grid, start, goal, belief_map=None)`: A* 4-hướng
  - Kiểm tra biên, vật cản cứng
  - g_score, f_score + heapq
  - Path reconstruction
- `reconstruct_path(came_from, current)`: chuyển came_from -> list

### `perception/bayesian_filter.py`
- `BayesianRiskAssessor(threshold=0.7)`
  - `p_hit=0.9`, `p_miss=0.2`
- `update_belief(prior, sensor_reading)`:
  - sensor=1: xác suất vật cản khi cảm biến báo có
  - sensor=0: xác suất vật cản khi cảm biến báo không
- `is_risky(probability)`: kiểm tra > threshold

---

## 🔬 Chi tiết thuật toán (Algorithm Details)

### A* Search (ở `planning/path_finder.py`)
A* là thuật toán tìm đường tối ưu trên đồ thị trạng thái với hàm chi phí dự đoán `f(n)=g(n)+h(n)`.
- `g(n)`: chi phí đường đi từ start đến node hiện tại (mỗi bước +1)
- `h(n)`: heuristic ước lượng chi phí từ node đến goal
  - Ở đây dùng Manhattan distance: `|x1-x2| + |y1-y2|`
  - mở rộng bằng Risk Penalty: `belief_map[x,y]*15` để ưu tiên đường nguy cơ thấp
- Cơ chế:
  1. Khởi tạo open set là priority queue (`o_heap`) chứa start
  2. Lặp cho đến khi open set rỗng:
     - lấy current node có `f` thấp nhất
     - nếu current==goal -> reconstruct path
     - đánh dấu current vào closed set
     - với mỗi neighbor xung quanh (4 hướng):
       - bỏ qua nếu ngoài biên hoặc là obstacle cứng (grid==1)
       - tính `tentative_g = g[current] + 1`
       - nếu neighbor chưa thăm hoặc `tentative_g < g[neighbor]`
         - cập nhật came_from, g_score, f_score
         - push vào open set nếu chưa có
  3. Nếu không tìm được goal -> trả về False
- Ưu điểm: A* đảm bảo đường ngắn nhất khi heuristic admissible. Risk penalty tăng độ an toàn bằng cộng chi phí cho đường nguy hiểm.

### Bayesian Update (ở `perception/bayesian_filter.py`)
Mô tả logic Bayes để cập nhật niềm tin rủi ro (belief) dựa trên cảm biến nhiễu:
- Biến prior = P(obstacle)
- sensor_reading có 2 giá trị:
  - 1: cảm biến báo có chướng ngại
  - 0: cảm biến báo không chướng ngại
- Khi sensor=1:
  - numerator = p_hit * prior
  - denominator = p_hit*prior + p_miss*(1-prior)
- Khi sensor=0:
  - numerator = (1-p_hit)*prior
  - denominator = (1-p_hit)*prior + (1-p_miss)*(1-prior)
- posterior = numerator / (denominator + 1e-9)
- `is_risky`: nếu posterior > threshold (0.7) -> coi như nguy cơ cao và chuyển thành chướng ngại thật trong `GridWorld`

### Vòng lặp Sense-Think-Act (ở `main.py`)
1. Sense: giả lập độ xác suất xuất hiện vật cản động trên route với 40% tỉ lệ
2. Think: cập nhật `belief_map` bằng `update_belief`, kiểm tra `is_risky`
3. Act: nếu nguy cơ cao => `world.set_obstacle(rx, ry)` và `path = a_star_search(...)`
4. Move: đi 1 bước, draw grid với màu sắc hiển thị:
   - Đen: chướng ngại thật
   - Xám: ô xác suất nguy cơ > 0.4
   - Xanh dương: route dự kiến
   - Đỏ: xe, Xanh lá: đích

### Lưu log và phân tích
- Mỗi step lưu `Step`, `Position`, `Risk`, `Status` vào danh sách
- Ghi vào `simulation_logs.csv` bằng `utils/helpers.save_logs`
- Có thể dùng file CSV để phân tích tập hợp: tổng bước, số lần Replanning, thời gian...

### `utils/helpers.py`
- `calculate_distance(path)` trả `len(path)` (số bước). Dùng được cho so sánh nữa.
- `save_logs(...)` ghi file CSV đầy enough để phục vụ nghiên cứu.

---

### 1 số mở rộng thuật toán đề xuất
- Thêm tri-state cho sensor (0: chắc chắn trống, 1: chắc chắn chặn, 0.5: không xác định)
- Giảm rủi ro động bằng Particle Filter hoặc POMDP
- Ước lượng chi phí qua MDP có reward/penalty dành cho mục tiêu an toàn

### 2 nguồn tham khảo
- 
---

## ⚙️ Công nghệ & thư viện
- Python 3.x
- NumPy: thao tác lưới, ma trận
- pygame: hiển thị tùy chọn GUI và animation
- heapq: priority queue cho A*
- csv: lưu logs

---

## ▶️ Cách chạy
1. Cài đặt yêu cầu (nếu chưa cài):
```bash
python -m pip install numpy pygame
```
2. Chạy:
```bash
python main.py
```
3. Quan sát: cửa sổ Pygame hiện lưới, ô đen = chướng ngại cứng,
ô xám = ô rủi ro, vòng xanh = đường A*
4. Kết thúc -> tạo `simulation_logs.csv`

---

## 🛠️ Tinh chỉnh nhanh
- Thay `GRID_SIZE`, `CELL_SIZE`, `start`, `goal` trong `main.py`
- Thay `n = 10` chướng ngại ban đầu
- Thay `threshold` trong `BayesianRiskAssessor`
- Thay hệ số rủi ro `15` trong `heuristic`

---

## 📌 Gợi ý mở rộng
- Thêm di chuyển 8 hướng
- Tích hợp map không gian thật bằng file JSON/CSV
- Đa agent và tránh tắc đường
- Thay Bayesian filter bằng Particle Filter hoặc POMDP

