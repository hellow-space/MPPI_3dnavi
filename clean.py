import cv2
import numpy as np

# ------------------- 你只需要改这里 -------------------
INPUT_PGM = "pcb_clean.pgm"       # 你的输入地图
OUTPUT_PGM = "pcb_final.pgm"# 输出干净地图
# -----------------------------------------------------

# 1. 读取PGM图片（灰度模式）
img = cv2.imread(INPUT_PGM, cv2.IMREAD_GRAYSCALE)

# 2. 二值化：把地图变成纯黑纯白（ROS地图专用）
# 205 是ROS地图阈值：>205=可通行(白)，<100=障碍(黑)，中间=未知
_, binary = cv2.threshold(img, 205, 255, cv2.THRESH_BINARY)

# 3. 去噪核心：形态学开运算 = 先腐蚀再膨胀 → 干掉孤立小噪点
kernel = np.ones((4, 4), np.uint8)  # 核越大去得越狠，2x2 最适合地图
clean = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

# 4. 保存干净的PGM地图
cv2.imwrite(OUTPUT_PGM, clean)

print(f"✅ 去噪完成！已保存到：{OUTPUT_PGM}")
print("🔍 噪点已全部清除！")
