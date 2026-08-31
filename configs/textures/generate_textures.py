import cv2
import numpy as np
import os

output_dir = "/home/purab/Purab/Projects/ROS/configs/textures"
os.makedirs(output_dir, exist_ok=True)

# 1. Ground Texture (512 x 512) - High-contrast Checkerboard + High-Frequency Grid & Noise (~25 KB)
size = 512
ground_img = np.zeros((size, size, 3), dtype=np.uint8)

# Base checkerboard (32x32 grid cells)
num_squares = 32
sq_len = size // num_squares
for i in range(num_squares):
    for j in range(num_squares):
        if (i + j) % 2 == 0:
            ground_img[i*sq_len:(i+1)*sq_len, j*sq_len:(j+1)*sq_len] = [230, 230, 230]
        else:
            ground_img[i*sq_len:(i+1)*sq_len, j*sq_len:(j+1)*sq_len] = [30, 30, 30]

# Add high-frequency grid lines every 8 pixels
for k in range(0, size, 8):
    ground_img[k:k+1, :] = [10, 10, 10]
    ground_img[:, k:k+1] = [10, 10, 10]

# Add random noise for micro-features
noise = np.random.randint(-35, 35, (size, size, 3), dtype=np.int16)
ground_img = np.clip(ground_img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

# Add feature dots
np.random.seed(42)
for _ in range(800):
    cx = np.random.randint(0, size)
    cy = np.random.randint(0, size)
    r = np.random.randint(2, 6)
    color = (int(np.random.randint(0, 255)), int(np.random.randint(0, 255)), int(np.random.randint(0, 255)))
    cv2.circle(ground_img, (cx, cy), r, color, -1)

ground_path = os.path.join(output_dir, "ground_texture.png")
cv2.imwrite(ground_path, ground_img)
print(f"Saved lightweight ground texture to {ground_path} ({os.path.getsize(ground_path)/1024:.1f} KB)")

# 2. Object / Box Texture (256 x 256) - Geometric Patterns & High-Contrast Blocks (~10 KB)
box_size = 256
box_img = np.full((box_size, box_size, 3), 240, dtype=np.uint8)

block_sz = 32
for r in range(0, box_size, block_sz):
    for c in range(0, box_size, block_sz):
        sub = np.random.choice([0, 255], size=(block_sz, block_sz), p=[0.5, 0.5]).astype(np.uint8)
        sub_3ch = cv2.merge([sub, sub, sub])
        box_img[r:r+block_sz, c:c+block_sz] = sub_3ch

cv2.rectangle(box_img, (0, 0), (box_size-1, box_size-1), (0, 0, 255), 6)
cv2.line(box_img, (0, 0), (box_size, box_size), (255, 0, 0), 4)
cv2.line(box_img, (0, box_size), (box_size, 0), (0, 255, 0), 4)

box_path = os.path.join(output_dir, "box_texture.png")
cv2.imwrite(box_path, box_img)
print(f"Saved lightweight box texture to {box_path} ({os.path.getsize(box_path)/1024:.1f} KB)")
