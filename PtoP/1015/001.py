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
import matplotlib

try:
    import plotly.graph_objects as go
    import plotly.express as px

    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False
    print("⚠️  Plotly 未安裝，將使用 matplotlib 進行交互式顯示")

# ✅ 中文字型設定
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False


def compute_arc_with_auto_center(A, B, radius_scale=2.0, num_points=100):
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)
    AB = B - A
    M = (A + B) / 2  # AB 中點

    # 若 AB 平行 Z 軸，改用 X 軸為法向參考
    z_ref = np.array([0, 0, 1])
    if np.allclose(np.cross(AB, z_ref), 0):
        z_ref = np.array([1, 0, 0])

    # 求出圓所在平面的方向向量
    dir_vec = np.cross(AB, z_ref)
    dir_vec = dir_vec / np.linalg.norm(dir_vec)

    # 計算圓心 C
    half_len = np.linalg.norm(AB) / 2
    h = half_len * radius_scale
    C = M + dir_vec * h

    # 建立圓弧軌跡點
    v1 = A - C
    v2 = B - C
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)

    v1 = v1 / np.linalg.norm(v1)
    v2_proj = v2 - np.dot(v2, v1) * v1
    v2_proj = v2_proj / np.linalg.norm(v2_proj)

    angle = np.arccos(np.clip(np.dot(v1, v2 / np.linalg.norm(v2)), -1, 1))
    theta = np.linspace(0, angle, num_points)
    arc = [C + (np.cos(t) * v1 + np.sin(t) * v2_proj) * np.linalg.norm(A - C) for t in theta]

    return np.array(arc), A, B, C


def get_point_input(point_name):
    """獲取點座標的輸入函數"""
    while True:
        try:
            print(f"\n請輸入點 {point_name} 的座標：")
            x = float(input(f"  {point_name}_x = "))
            y = float(input(f"  {point_name}_y = "))
            z = float(input(f"  {point_name}_z = "))
            return [x, y, z]
        except ValueError:
            print("❌ 輸入格式錯誤，請輸入數字！")


def create_interactive_3d_view(arc_points, A, B, C, radius_scale):
    """創建交互式3D視窗顯示1000點並支援滑鼠懸停顯示座標"""

    if PLOTLY_AVAILABLE:
        # 使用 Plotly 創建高度交互式的3D圖
        print("🎯 使用 Plotly 創建交互式3D視窗...")

        # 創建圓弧軌跡點的懸停文字
        hover_text = [f"點 {i}<br>X: {x:.3f}<br>Y: {y:.3f}<br>Z: {z:.3f}"
                      for i, (x, y, z) in enumerate(arc_points)]

        fig = go.Figure()

        # 添加圓弧軌跡點
        fig.add_trace(go.Scatter3d(
            x=arc_points[:, 0],
            y=arc_points[:, 1],
            z=arc_points[:, 2],
            mode='markers+lines',
            marker=dict(size=3, color='blue', opacity=0.8),
            line=dict(color='blue', width=4),
            name='圓弧軌跡 (1000點)',
            hovertext=hover_text,
            hoverinfo='text'
        ))

        # 添加 A、B、C 點
        fig.add_trace(go.Scatter3d(
            x=[A[0]], y=[A[1]], z=[A[2]],
            mode='markers',
            marker=dict(size=12, color='red'),
            name=f'起點 A ({A[0]}, {A[1]}, {A[2]})',
            hovertext=f"起點 A<br>X: {A[0]}<br>Y: {A[1]}<br>Z: {A[2]}",
            hoverinfo='text'
        ))

        fig.add_trace(go.Scatter3d(
            x=[B[0]], y=[B[1]], z=[B[2]],
            mode='markers',
            marker=dict(size=12, color='green'),
            name=f'終點 B ({B[0]}, {B[1]}, {B[2]})',
            hovertext=f"終點 B<br>X: {B[0]}<br>Y: {B[1]}<br>Z: {B[2]}",
            hoverinfo='text'
        ))

        fig.add_trace(go.Scatter3d(
            x=[C[0]], y=[C[1]], z=[C[2]],
            mode='markers',
            marker=dict(size=12, color='blue'),
            name=f'圓心 C ({C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f})',
            hovertext=f"圓心 C<br>X: {C[0]:.2f}<br>Y: {C[1]:.2f}<br>Z: {C[2]:.2f}",
            hoverinfo='text'
        ))

        # 添加連接線
        fig.add_trace(go.Scatter3d(
            x=[A[0], B[0]], y=[A[1], B[1]], z=[A[2], B[2]],
            mode='lines',
            line=dict(color='black', width=3, dash='dash'),
            name='A→B 直線',
            showlegend=True
        ))

        fig.add_trace(go.Scatter3d(
            x=[A[0], C[0]], y=[A[1], C[1]], z=[A[2], C[2]],
            mode='lines',
            line=dict(color='red', width=2, dash='dot'),
            name='A→C 半徑',
            showlegend=True
        ))

        fig.add_trace(go.Scatter3d(
            x=[B[0], C[0]], y=[B[1], C[1]], z=[B[2], C[2]],
            mode='lines',
            line=dict(color='green', width=2, dash='dot'),
            name='B→C 半徑',
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

    else:
        # 使用 matplotlib 創建帶註釋的3D圖
        print("🎯 使用 Matplotlib 創建交互式3D視窗...")

        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        # 繪製圓弧軌跡
        line = ax.plot(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
                       color='blue', linewidth=3, label='圓弧軌跡')[0]
        scatter = ax.scatter(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
                             s=15, color='blue', alpha=0.7, picker=True)

        # 添加主要點
        ax.scatter(*A, color='red', s=150, edgecolors='black', linewidth=2, label=f'起點 A {A}')
        ax.scatter(*B, color='green', s=150, edgecolors='black', linewidth=2, label=f'終點 B {B}')
        ax.scatter(*C, color='blue', s=150, edgecolors='black', linewidth=2,
                   label=f'圓心 C ({C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f})')

        # 添加連接線
        ax.plot([A[0], B[0]], [A[1], B[1]], [A[2], B[2]], 'k--', linewidth=2, label='A→B 直線')
        ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], 'r:', linewidth=2, label='A→C 半徑')
        ax.plot([B[0], C[0]], [B[1], C[1]], [B[2], C[2]], 'g:', linewidth=2, label='B→C 半徑')

        # 創建註釋
        annot = ax.text(0, 0, 0, '', fontsize=10,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))

        def on_hover(event):
            if event.inaxes == ax:
                # 檢查是否點擊到散點
                cont, ind = scatter.contains(event)
                if cont:
                    # 獲取最近的點
                    point_idx = ind['ind'][0]
                    x, y, z = arc_points[point_idx]

                    # 更新註釋
                    annot.set_text(f'點 {point_idx}\nX: {x:.3f}\nY: {y:.3f}\nZ: {z:.3f}')
                    annot.set_position((x, y))
                    annot.set_visible(True)
                    fig.canvas.draw_idle()
                else:
                    annot.set_visible(False)
                    fig.canvas.draw_idle()

        # 綁定滑鼠事件
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
    print("=" * 50)
    print("🎯 3D 圓弧軌跡生成器")
    print("=" * 50)

    # 🔧 獲取使用者輸入的座標
    A = get_point_input("A")
    B = get_point_input("B")

    # 詢問圓弧參數
    while True:
        try:
            radius_scale = float(input(f"\n請輸入圓弧彎曲程度 (建議: 1.0-3.0，預設: 2.0): ") or "2.0")
            if radius_scale > 0:
                break
            else:
                print("❌ 請輸入正數！")
        except ValueError:
            print("❌ 輸入格式錯誤，請輸入數字！")

    print(f"\n✅ 計算中...")
    print(f"   起點 A: {A}")
    print(f"   終點 B: {B}")
    print(f"   彎曲程度: {radius_scale}")

    # 計算圓弧軌跡（分割成1000個點）
    arc_points, A, B, C = compute_arc_with_auto_center(A, B, radius_scale=radius_scale, num_points=1000)

    print(f"   圓心 C: [{C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f}]")
    print(f"   軌跡點數: {len(arc_points)} (高精度分割)")

    # 計算圓弧長度和點間距
    distances = np.sqrt(np.sum(np.diff(arc_points, axis=0) ** 2, axis=1))
    arc_length = np.sum(distances)
    avg_spacing = arc_length / (len(arc_points) - 1)
    print(f"   圓弧長度: {arc_length:.2f}")
    print(f"   平均點間距: {avg_spacing:.3f}")


def main():
    print("=" * 50)
    print("🎯 3D 圓弧軌跡生成器")
    print("=" * 50)

    # 🔧 獲取使用者輸入的座標
    A = get_point_input("A")
    B = get_point_input("B")

    # 詢問圓弧參數
    while True:
        try:
            radius_scale = float(input(f"\n請輸入圓弧彎曲程度 (建議: 1.0-3.0，預設: 2.0): ") or "2.0")
            if radius_scale > 0:
                break
            else:
                print("❌ 請輸入正數！")
        except ValueError:
            print("❌ 輸入格式錯誤，請輸入數字！")

    print(f"\n✅ 計算中...")
    print(f"   起點 A: {A}")
    print(f"   終點 B: {B}")
    print(f"   彎曲程度: {radius_scale}")

    # 計算圓弧軌跡（分割成1000個點）
    arc_points, A, B, C = compute_arc_with_auto_center(A, B, radius_scale=radius_scale, num_points=1000)

    print(f"   圓心 C: [{C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f}]")
    print(f"   軌跡點數: {len(arc_points)} (高精度分割)")

    # 計算圓弧長度和點間距
    distances = np.sqrt(np.sum(np.diff(arc_points, axis=0) ** 2, axis=1))
    arc_length = np.sum(distances)
    avg_spacing = arc_length / (len(arc_points) - 1)
    print(f"   圓弧長度: {arc_length:.2f}")
    print(f"   平均點間距: {avg_spacing:.3f}")

    # 🧭 繪製標準視圖
    print(f"\n📊 生成標準3D視圖...")
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 圓弧軌跡（加粗藍線 + 細小藍點）
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
    ax.text(A[0], A[1], A[2], f'A({A[0]}, {A[1]}, {A[2]})',
            fontsize=12, fontweight='bold')
    ax.text(B[0], B[1], B[2], f'B({B[0]}, {B[1]}, {B[2]})',
            fontsize=12, fontweight='bold')
    ax.text(C[0], C[1], C[2], f'C(圓心)\n({C[0]:.1f}, {C[1]:.1f}, {C[2]:.1f})',
            fontsize=10, fontweight='bold')

    # 軸設定
    ax.set_xlabel('X 軸', fontsize=12)
    ax.set_ylabel('Y 軸', fontsize=12)
    ax.set_zlabel('Z 軸', fontsize=12)

    title = f"機械手臂末端從 A 到 B 繞 C 為圓心的圓弧運動\n(彎曲程度: {radius_scale}, 高精度1000點分割)"
    plt.title(title, fontsize=14, fontweight='bold')

    ax.legend(fontsize=11)

    # 設置相等的軸比例
    ax.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show()

    print(f"\n✅ 高精度圓弧軌跡生成完成！(1000點分割)")

    # 🎮 創建交互式3D視窗
    print(f"\n🎮 正在創建交互式3D視窗...")
    create_interactive_3d_view(arc_points, A, B, C, radius_scale)


if __name__ == "__main__":
    main()