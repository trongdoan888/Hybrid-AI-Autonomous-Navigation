def smooth_path_chaikin(path, iterations=3):
    """
    Thuật toán Chaikin làm mượt quỹ đạo từ các điểm nút lưới nguyên (A*)
    thành đường cong liên tục bằng cách liên tục 'gọt' các góc nhọn.
    """
    if not path or len(path) <= 2:
        return path
        
    smooth_path = path
    for _ in range(iterations):
        new_path = [smooth_path[0]] # Luôn giữ điểm đầu
        
        for i in range(len(smooth_path) - 1):
            p0 = smooth_path[i]
            p1 = smooth_path[i+1]
            
            # Tìm điểm 1/4 và 3/4 của đoạn thẳng nối 2 điểm
            q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
            r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
            
            new_path.extend([q, r])
            
        new_path.append(smooth_path[-1]) # Luôn giữ điểm cuối
        smooth_path = new_path
        
    return smooth_path