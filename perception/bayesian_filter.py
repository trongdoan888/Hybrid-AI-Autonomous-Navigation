import numpy as np

class BayesianRiskAssessor:
    def __init__(self, threshold=0.7):
        self.threshold = threshold  # Ngưỡng xác suất để coi là vật cản thực sự
        # Độ chính xác của cảm biến (Sensor Model)
        self.p_hit = 0.9    # P(Cảm biến báo 1 | Thực sự là 1)
        self.p_miss = 0.2   # P(Cảm biến báo 1 | Thực sự là 0) - Báo nhiễu

    def update_belief(self, prior, sensor_reading):
        """
        Cập nhật xác suất có vật cản dựa trên công thức Bayes
        prior: Xác suất cũ (0.0 -> 1.0)
        sensor_reading: 1 (có vật) hoặc 0 (trống) từ cảm biến
        """
        if sensor_reading == 1:
            # P(Vật cản | Cảm biến báo 1)
            numerator = self.p_hit * prior
            denominator = self.p_hit * prior + self.p_miss * (1 - prior)
        else:
            # P(Vật cản | Cảm biến báo 0)
            numerator = (1 - self.p_hit) * prior
            denominator = (1 - self.p_hit) * prior + (1 - self.p_miss) * (1 - prior)
        
        return numerator / (denominator + 1e-9) # Tránh chia cho 0

    def is_risky(self, probability):
        return probability > self.threshold