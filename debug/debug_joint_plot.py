import numpy as np
import matplotlib.pyplot as plt

# 데이터 읽기
data = np.loadtxt("jh_controller_position.txt")

# 각 데이터 열로 분리
time = data[:, 0]
q = data[:, 1:8]
q_desired = data[:, 8:15]
qdot = data[:, 15:22]
qdot_desired = data[:, 22:29]

time = time - time[0]

# 플로팅 설정
fig, axes = plt.subplots(7, 2, figsize=(14, 20))

# 각 joint에 대해 플로팅
for i in range(7):
    # 좌측 열: q와 q_desired
    axes[i, 0].plot(time, q[:, i], 'r-', label=f'q[{i+1}]', linewidth=1)
    axes[i, 0].plot(time, q_desired[:, i], 'k--', label=f'q_desired[{i+1}]', linewidth=2)
    axes[i, 0].set_xlabel("Time")
    axes[i, 0].set_ylabel("Position")
    axes[i, 0].legend()
    axes[i, 0].set_title(f"q[{i+1}] and q_desired[{i+1}] over Time")
    
    # 우측 열: qdot와 qdot_desired
    axes[i, 1].plot(time, qdot[:, i], 'r-', label=f'qdot[{i+1}]', linewidth=1)
    axes[i, 1].plot(time, qdot_desired[:, i], 'k--', label=f'qdot_desired[{i+1}]', linewidth=2)
    axes[i, 1].set_xlabel("Time")
    axes[i, 1].set_ylabel("Velocity")
    axes[i, 1].legend()
    axes[i, 1].set_title(f"qdot[{i+1}] and qdot_desired[{i+1}] over Time")

plt.tight_layout()
plt.show()
