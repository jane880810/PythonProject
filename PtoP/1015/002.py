'''
輸入A
輸入B
輸入弧度
顯示3D曲線圖
顯示圓心
顯示1000點座標
'''
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

try:
    import plotly.graph_objects as go

    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False
    print("⚠️  Plotly 未安裝，將使用 matplotlib 進行交互式顯示")

# ✅ 中文字型設定
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False


def compute_arc_with_auto_center(A, B, radius_scale=2.0, num_points=100):
    """計算圓弧軌跡"""
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)
    AB = B - A
    M = (A + B) / 2  # AB 中點

    # 若 AB 平行 Z 軸，改用 X 軸為法向參考
    z_ref = np.array([0, 0, 1]) if not np.allclose(np.cross(AB, [0, 0, 1]), 0) else np.array([1, 0, 0])

    # 求出圓所在平面的方向向量
    dir_vec = np.cross(AB, z_ref)
    dir_vec /= np.linalg.norm(dir_vec)

    # 計算圓心 C
    half_len = np.linalg.norm(AB) / 2
    h = half_len * radius_scale
    C = M + dir_vec * h

    # 建立圓弧軌跡點
    v1 = A - C
    v2 = B - C
    normal = np.cross(v1, v2)
    normal /= np.linalg.norm(normal)

    v1 /= np.linalg.norm(v1)
    v2_proj = v2 - np.dot(v2, v1) * v1
    v2_proj /= np.linalg.norm(v2_proj)

    angle = np.arccos(np.clip(np.dot(v1, v2 / np.linalg.norm(v2)), -1, 1))
    theta = np.linspace(0, angle, num_points)

    # 向量化計算圓弧點
    radius = np.linalg.norm(A - C)
    arc = C + radius * (np.outer(np.cos(theta), v1) + np.outer(np.sin(theta), v2_proj))

    return arc, A, B, C


def get_point_input(point_name):
    """獲取點座標的輸入函數"""
    while True:
        try:
            print(f"\n請輸入點 {point_name} 的座標：")
            x = float(input(f"  {point_name}_x = "))
            y = float(input(f"  {point_name}_y = "))
            z = float(input(f"  {point_name}_z = "))
            return np.array([x, y, z])
        except ValueError:
            print("❌ 輸入格式錯誤，請輸入數字！")


def get_radius_scale():
    """獲取圓弧彎曲程度"""
    while True:
        try:
            radius_scale = float(input("\n請輸入圓弧彎曲程度 (建議: 1.0-3.0，預設: 2.0): ") or "2.0")
            if radius_scale > 0:
                return radius_scale
            print("❌ 請輸入正數！")
        except ValueError:
            print("❌ 輸入格式錯誤，請輸入數字！")


def calculate_arc_info(arc_points):
    """計算圓弧資訊"""
    distances = np.linalg.norm(np.diff(arc_points, axis=0), axis=1)
    arc_length = np.sum(distances)
    avg_spacing = arc_length / (len(arc_points) - 1)
    return arc_length, avg_spacing


def create_plotly_view(arc_points, A, B, C, radius_scale):
    """使用 Plotly 創建交互式3D視窗"""
    hover_text = [f"點 {i}<br>X: {x:.3f}<br>Y: {y:.3f}<br>Z: {z:.3f}"
                  for i, (x, y, z) in enumerate(arc_points)]

    fig = go.Figure()

    # 添加圓弧軌跡點
    fig.add_trace(go.Scatter3d(
        x=arc_points[:, 0], y=arc_points[:, 1], z=arc_points[:, 2],
        mode='markers+lines',
        marker=dict(size=3, color='blue', opacity=0.8),
        line=dict(color='blue', width=4),
        name='圓弧軌跡 (1000點)',
        hovertext=hover_text,
        hoverinfo='text'
    ))

    # 添加關鍵點
    points = [
        (A, 'red', f'起點 A ({A[0]}, {A[1]}, {A[2]})'),
        (B, 'green', f'終點 B ({B[0]}, {B[1]}, {B[2]})'),
        (C, 'blue', f'圓心 C ({C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f})')
    ]

    for point, color, name in points:
        fig.add_trace(go.Scatter3d(
            x=[point[0]], y=[point[1]], z=[point[2]],
            mode='markers',
            marker=dict(size=12, color=color),
            name=name,
            hovertext=name,
            hoverinfo='text'
        ))

    # 添加連接線
    lines = [
        ([A[0], B[0]], [A[1], B[1]], [A[2], B[2]], 'black', 'dash', 'A→B 直線'),
        ([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], 'red', 'dot', 'A→C 半徑'),
        ([B[0], C[0]], [B[1], C[1]], [B[2], C[2]], 'green', 'dot', 'B→C 半徑')
    ]

    for x, y, z, color, dash, name in lines:
        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            line=dict(color=color, width=2 if dash == 'dot' else 3, dash=dash),
            name=name,
            showlegend=True
        ))

    # 設置佈局
    fig.update_layout(
        title=f"交互式3D圓弧軌跡視窗 (1000點) - 彎曲程度: {radius_scale}",
        scene=dict(
            xaxis_title="X 軸",
            yaxis_title="Y 軸",
            zaxis_title="Z 軸",
            aspectmode='cube'
        ),
        width=1000,
        height=800
    )

    fig.show()


def create_matplotlib_view(arc_points, A, B, C, radius_scale):
    """使用 Matplotlib 創建交互式3D視窗"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 繪製圓弧軌跡
    ax.plot(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
            color='blue', linewidth=3, label='圓弧軌跡')
    scatter = ax.scatter(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
                         s=15, color='blue', alpha=0.7, picker=True)

    # 添加主要點
    points = [
        (A, 'red', f'起點 A {A}'),
        (B, 'green', f'終點 B {B}'),
        (C, 'blue', f'圓心 C ({C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f})')
    ]

    for point, color, label in points:
        ax.scatter(*point, color=color, s=150, edgecolors='black', linewidth=2, label=label)

    # 添加連接線
    lines = [
        ([A[0], B[0]], [A[1], B[1]], [A[2], B[2]], 'k--', 'A→B 直線'),
        ([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], 'r:', 'A→C 半徑'),
        ([B[0], C[0]], [B[1], C[1]], [B[2], C[2]], 'g:', 'B→C 半徑')
    ]

    for x, y, z, style, label in lines:
        ax.plot(x, y, z, style, linewidth=2, label=label)

    # 創建註釋
    annot = ax.text(0, 0, 0, '', fontsize=10,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8),
                    visible=False)

    def on_hover(event):
        if event.inaxes == ax:
            cont, ind = scatter.contains(event)
            if cont:
                point_idx = ind['ind'][0]
                x, y, z = arc_points[point_idx]
                annot.set_text(f'點 {point_idx}\nX: {x:.3f}\nY: {y:.3f}\nZ: {z:.3f}')
                annot.set_position((x, y))
                annot.set_visible(True)
            else:
                annot.set_visible(False)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('motion_notify_event', on_hover)

    # 設置軸
    ax.set_xlabel('X 軸', fontsize=12)
    ax.set_ylabel('Y 軸', fontsize=12)
    ax.set_zlabel('Z 軸', fontsize=12)
    ax.set_title(f'交互式3D圓弧軌跡視窗 (1000點)\n滑鼠懸停查看點座標 - 彎曲程度: {radius_scale}',
                 fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show()


def create_interactive_3d_view(arc_points, A, B, C, radius_scale):
    """創建交互式3D視窗"""
    if PLOTLY_AVAILABLE:
        print("🎯 使用 Plotly 創建交互式3D視窗...")
        create_plotly_view(arc_points, A, B, C, radius_scale)
    else:
        print("🎯 使用 Matplotlib 創建交互式3D視窗...")
        create_matplotlib_view(arc_points, A, B, C, radius_scale)


def create_standard_3d_view(arc_points, A, B, C, radius_scale):
    """繪製標準3D視圖"""
    print("\n📊 生成標準3D視圖...")
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 圓弧軌跡
    ax.plot(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
            label='圓弧軌跡', color='blue', linewidth=3)
    ax.scatter(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
               s=2, color='blue', alpha=0.5)

    # A → B 直線
    ax.plot([A[0], B[0]], [A[1], B[1]], [A[2], B[2]],
            'k--', label='A→B（直線）', linewidth=2)

    # A → C、B → C 半徑線
    ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]],
            'r:', label='A→C（半徑）', linewidth=2)
    ax.plot([B[0], C[0]], [B[1], C[1]], [B[2], C[2]],
            'g:', label='B→C（半徑）', linewidth=2)

    # 標註點位
    ax.scatter(*A, color='red', s=100, edgecolors='black', linewidth=1)
    ax.scatter(*B, color='green', s=100, edgecolors='black', linewidth=1)
    ax.scatter(*C, color='blue', s=100, edgecolors='black', linewidth=1)

    # 文字標籤
    ax.text(*A, f'A({A[0]}, {A[1]}, {A[2]})', fontsize=12, fontweight='bold')
    ax.text(*B, f'B({B[0]}, {B[1]}, {B[2]})', fontsize=12, fontweight='bold')
    ax.text(*C, f'C(圓心)\n({C[0]:.1f}, {C[1]:.1f}, {C[2]:.1f})', fontsize=10, fontweight='bold')

    # 軸設定
    ax.set_xlabel('X 軸', fontsize=12)
    ax.set_ylabel('Y 軸', fontsize=12)
    ax.set_zlabel('Z 軸', fontsize=12)
    ax.set_title(f"機械手臂末端從 A 到 B 繞 C 為圓心的圓弧運動\n(彎曲程度: {radius_scale}, 高精度1000點分割)",
                 fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show()


def main():
    print("=" * 50)
    print("🎯 3D 圓弧軌跡生成器")
    print("=" * 50)

    # 獲取使用者輸入
    A = get_point_input("A")
    B = get_point_input("B")
    radius_scale = get_radius_scale()

    # 計算圓弧軌跡
    print(f"\n✅ 計算中...")
    print(f"   起點 A: {A}")
    print(f"   終點 B: {B}")
    print(f"   彎曲程度: {radius_scale}")

    arc_points, A, B, C = compute_arc_with_auto_center(A, B, radius_scale=radius_scale, num_points=1000)

    print(f"   圓心 C: [{C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f}]")
    print(f"   軌跡點數: {len(arc_points)} (高精度分割)")

    # 計算圓弧資訊
    arc_length, avg_spacing = calculate_arc_info(arc_points)
    print(f"   圓弧長度: {arc_length:.2f}")
    print(f"   平均點間距: {avg_spacing:.3f}")

    # 繪製標準視圖
    create_standard_3d_view(arc_points, A, B, C, radius_scale)

    print(f"\n✅ 高精度圓弧軌跡生成完成！(1000點分割)")

    # 創建交互式3D視窗
    print(f"\n🎮 正在創建交互式3D視窗...")
    create_interactive_3d_view(arc_points, A, B, C, radius_scale)


if __name__ == "__main__":
    main()