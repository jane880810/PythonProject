import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 方式1：設定字體
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# 方式2：直接在繪圖時指定字體
# font = fm.FontProperties(fname='/usr/share/fonts/truetype/wqy/wqy-microhei.ttc')

# 測試繪圖
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot([1, 2, 3, 4], [1, 4, 9, 16], 'o-', label='數據1')
ax.plot([1, 2, 3, 4], [1, 2, 4, 8], 's-', label='數據2')

ax.set_title('機器手臂運動軌跡', fontsize=16)
ax.set_xlabel('時間 (秒)', fontsize=12)
ax.set_ylabel('位置 (mm)', fontsize=12)
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()