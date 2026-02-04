import pandas as pd
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt

# 读取数据
csv_file = "/home/dw/robot/image/proc_laser11/20260129_144406_points_smooth.csv"
try:
    df = pd.read_csv(csv_file)
except FileNotFoundError:
    print(f"错误：找不到文件 {csv_file}")
    exit()

# 定义要标记的点
targets = [
    {"id": 1, "x": 201},
    {"id": 2, "x": 218},
    {"id": 1, "x": 226},
    {"id": 2, "x": 185},
    {"id": 1, "x": 307},
    {"id": 2, "x": 307},
]

fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# 绘制原始数据
for laser_id, g in df.groupby("laser_id"):
    ax.scatter(
        g["x_pixel"],
        g["distance_cm"],
        g["y_pixel"],
        s=10,
        alpha=0.3,
        label=f"Laser {laser_id}"
    )

print("--- 正在标记目标点 ---")
for t in targets:
    tid = t["id"]
    tx = t["x"]
    laser_data = df[df["laser_id"] == tid]

    if not laser_data.empty:
        idx = (laser_data["x_pixel"] - tx).abs().idxmin()
        match = df.loc[idx]
        mx, md, my = match["x_pixel"], match["distance_cm"], match["y_pixel"]

        # 画红星标记
        ax.scatter(mx, md, my, c='red', s=150, marker='*', edgecolors='black', zorder=10)
        print(f"已标记 ID:{tid} X:{tx} -> 匹配: X={mx:.1f}, Dist={md:.2f}")

# 图形设置
ax.invert_zaxis()
ax.set_xlim(0, 640)
ax.set_zlim(0, 480)
ax.view_init(elev=90, azim=0)

ax.set_xlabel("x_pixel")
ax.set_ylabel("distance (cm)")
ax.set_zlabel("y_pixel")
ax.set_title("Laser Reconstruction - Saved Image")
ax.legend()

# 保存图片
output_file = "reconstruction_result.png"
plt.tight_layout()
plt.savefig(output_file, dpi=150)
print(f"--- 运行成功！结果已保存为: {output_file} ---")