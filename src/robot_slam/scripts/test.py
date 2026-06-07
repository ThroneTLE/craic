
"""
grid_visual_debug.py

可视化调试版：
1. 从 ROS /scan 话题读取真实雷达数据并转为点云
2. 显示九宫格
3. 显示12条边检测区域
4. 显示判断结果
5. 显示挡板分布
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

import rospy
from sensor_msgs.msg import LaserScan

GRID = 0.39

EDGES = {
    "31_32":((-0.195,0.39),0),
    "32_33":((0.195,0.39),0),
    "40_41":((-0.195,0.0),0),
    "41_42":((0.195,0.0),0),
    "49_50":((-0.195,-0.39),0),
    "50_51":((0.195,-0.39),0),

    "31_40":((-0.39,0.195),90),
    "40_49":((-0.39,-0.195),90),
    "32_41":((0.0,0.195),90),
    "41_50":((0.0,-0.195),90),
    "33_42":((0.39,0.195),90),
    "42_51":((0.39,-0.195),90),
}

RECT_LENGTH = 0.45
RECT_WIDTH = 0.10
BLOCK_LEN = 0.06


def point_in_rot_rect(px, py, cx, cy, ang_deg):
    ang = math.radians(-ang_deg)

    dx = px - cx
    dy = py - cy

    rx = dx * math.cos(ang) - dy * math.sin(ang)
    ry = dx * math.sin(ang) + dy * math.cos(ang)

    return (
        abs(rx) <= RECT_LENGTH/2 and
        abs(ry) <= RECT_WIDTH/2
    )


def cluster_length(points):
    """计算点集的最大跨度（任意两点间最大距离），对稀疏点更鲁棒"""
    if len(points) < 2:
        return 0

    pts = np.array(points)
    dist = np.linalg.norm(pts[:, None] - pts[None, :], axis=2)
    return float(np.max(dist))


def detect_edges(cloud):

    result = {}
    detail = {}

    for name,(center,ang) in EDGES.items():

        inside=[]

        for x,y in cloud:

            if point_in_rot_rect(
                x,y,
                center[0],
                center[1],
                ang
            ):
                inside.append((x,y))

        length = cluster_length(inside)

        result[name] = length > BLOCK_LEN
        detail[name] = (len(inside), length)

    # 打印每条边的调试信息
    print("\n====== 边检测详情 ======")
    for name, (center, ang) in EDGES.items():
        n_pts, clen = detail[name]
        blocked = result[name]
        print(f"{name}  center=({center[0]:+.3f},{center[1]:+.3f})  ang={ang:>3}°  "
              f"点数={n_pts:>4}  簇长度={clen:.3f}m  -> {'BLOCKED' if blocked else 'FREE'}")
    print("=" * 40)

    return result


def draw(edge_result, cloud):

    fig,ax = plt.subplots(figsize=(10,10))

    cloud=np.array(cloud)

    if len(cloud):
        ax.scatter(
            cloud[:,0],
            cloud[:,1],
            s=8,
            label="LiDAR"
        )

    xs=[-0.585,-0.195,0.195,0.585]
    ys=[-0.585,-0.195,0.195,0.585]

    for x in xs:
        ax.plot([x,x],[-0.585,0.585])

    for y in ys:
        ax.plot([-0.585,0.585],[y,y])

    labels={
        31:(-0.39,0.39),
        32:(0,0.39),
        33:(0.39,0.39),
        40:(-0.39,0),
        41:(0,0),
        42:(0.39,0),
        49:(-0.39,-0.39),
        50:(0,-0.39),
        51:(0.39,-0.39)
    }

    for k,(x,y) in labels.items():
        ax.text(x,y,str(k),fontsize=12)

    for name,(center,ang) in EDGES.items():

        blocked=edge_result[name]

        color='red' if blocked else 'green'

        rect=Rectangle(
            (-RECT_LENGTH/2,-RECT_WIDTH/2),
            RECT_LENGTH,
            RECT_WIDTH,
            fill=False,
            linewidth=2,
            edgecolor=color
        )

        t=(Affine2D()
            .rotate_deg(ang)
            .translate(center[0],center[1])
            + ax.transData)

        rect.set_transform(t)

        ax.add_patch(rect)

        ax.text(
            center[0],
            center[1],
            name,
            fontsize=8
        )

    print("\n====== 挡板检测结果 ======")

    for k,v in edge_result.items():
        print(f"{k}: {'BLOCKED' if v else 'FREE'}")

    ax.set_title("Grid Edge Detection Debug View")
    ax.set_aspect("equal")
    ax.grid(True)

    plt.show()


def get_cloud():
    """从 ROS /scan 话题读取一帧雷达数据，转为笛卡尔点云"""
    scan = rospy.wait_for_message("/scan", LaserScan, timeout=5.0)

    pts = []
    for i, r in enumerate(scan.ranges):
        if scan.range_min < r < scan.range_max:
            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            pts.append((x, y))

    rospy.loginfo(f"LaserScan -> {len(pts)} points")
    return pts


if __name__ == "__main__":
    rospy.init_node("grid_edge_detector", anonymous=True)

    cloud = get_cloud()

    result = detect_edges(cloud)

    draw(result, cloud)
