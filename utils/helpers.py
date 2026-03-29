import csv
import os

def save_logs(log_data, filename="simulation_logs.csv"):
    """
    Lưu dữ liệu hành trình vào file CSV.
    log_data: Danh sách các dict chứa thông tin từng bước.
    """
    keys = log_data[0].keys() if log_data else []
    
    # Kiểm tra nếu file đã tồn tại thì ghi đè, nếu chưa thì tạo mới
    with open(filename, 'w', newline='', encoding='utf-8') as output_file:
        dict_writer = csv.DictWriter(output_file, fieldnames=keys)
        dict_writer.writeheader()
        dict_writer.writerows(log_data)
    
    print(f"📊 Đã xuất dữ liệu nghiên cứu vào file: {filename}")

def calculate_distance(path):
    """Tính tổng chiều dài quãng đường (số ô di chuyển)"""
    return len(path)