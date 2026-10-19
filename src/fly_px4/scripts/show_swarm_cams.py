#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将六路 /vision/annotated_N 以 OpenCV 网格窗口显示。
屏幕尺寸、行列和顶部留白可由私有参数覆盖；按键或 Ctrl+C 关闭全部窗口。"""
import os
import sys
import signal
import threading

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# 参数 & 常量
N_DRONES = 6
TOPIC_FMT = "/vision/annotated_{i}"        # i = 1..6
WINDOW_NAME_FMT = "Drone {i} ({ns})"       # 显示的窗口标题
DRONE_NS = [f"typhoon_h480_{i}" for i in range(N_DRONES)]  # 0..5

# 部分 GUI 后端需在 imshow 后再次定位窗口；缓存最新帧避免闪烁。
bridge = CvBridge()
lock = threading.Lock()
# 缓存: idx (0..5) -> np.ndarray (最新收到的 BGR 图像)
latest_frames: dict = {}
# 窗口是否已首次完成 moveWindow  (idx -> bool)
window_placed: dict = {}
# 窗口像素位置缓存 (idx -> (x, y, w, h))
window_rects: dict = {}

stop_event = threading.Event()


def grid_geometry(screen_w, screen_h, rows, cols, top_margin):
    """
    计算 2x3 网格每格的 (x, y, w, h)。
    返回列表: [dict(x,y,w,h), ...] 顺序: 行主序 (0..cols*rows-1)。
    """
    cell_w = screen_w // cols
    cell_h = (screen_h - top_margin) // rows
    rects = []
    for r in range(rows):
        for c in range(cols):
            rects.append(dict(
                x=c * cell_w,
                y=top_margin + r * cell_h,
                w=cell_w,
                h=cell_h,
            ))
    return rects


def make_callback(idx):
    """为第 idx 架无人机构造 Image 订阅回调 (闭包)。"""
    win_name = WINDOW_NAME_FMT.format(i=idx + 1, ns=DRONE_NS[idx])

    def cb(msg):
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, f"[{win_name}] cv_bridge 解码失败: {e}")
            return
        with lock:
            latest_frames[idx] = img

    return cb


def on_sigint(signum, frame):
    """Ctrl+C: 通知渲染循环退出, 从而让 cv2.destroyAllWindows() 被调用。"""
    stop_event.set()


def main():
    rospy.init_node("show_swarm_cams", anonymous=False, disable_signals=True)
    signal.signal(signal.SIGINT, on_sigint)

    screen_w   = rospy.get_param("~screen_w",   1920)
    screen_h   = rospy.get_param("~screen_h",   1080)
    grid_rows  = rospy.get_param("~grid_rows",  2)
    grid_cols  = rospy.get_param("~grid_cols",  3)
    top_margin = rospy.get_param("~top_margin", 30)

    if grid_rows * grid_cols < N_DRONES:
        rospy.logwarn(f"网格 {grid_rows}x{grid_cols}={grid_rows*grid_cols} < 6 架无人机, "
                      f"自动放大到 2x3")
        grid_rows, grid_cols = 2, 3

    rects = grid_geometry(screen_w, screen_h, grid_rows, grid_cols, top_margin)

    # 1. 创建 6 个窗口 & 摆好初始位置 (行主序: 0..5 正好是 cam0..cam5)
    win_names = []
    for idx in range(N_DRONES):
        name = WINDOW_NAME_FMT.format(i=idx + 1, ns=DRONE_NS[idx])
        win_names.append(name)
        # WINDOW_NORMAL 允许用户之后再自己拖大小, 但第一次我们固定它。
        cv2.namedWindow(name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        r = rects[idx]
        window_rects[idx] = r
        cv2.resizeWindow(name, r["w"], r["h"])
        cv2.moveWindow(name, r["x"], r["y"])
        window_placed[idx] = False
        rospy.loginfo(f"[{name}] 窗口位置 ({r['x']},{r['y']})  大小 {r['w']}x{r['h']}")

    # 2. 启动 6 个订阅器
    subs = []
    for idx in range(N_DRONES):
        topic = TOPIC_FMT.format(i=idx + 1)
        subs.append(rospy.Subscriber(topic, Image, make_callback(idx),
                                     queue_size=1, buff_size=2 ** 22))
    rospy.loginfo(f"已订阅 6 个话题: {TOPIC_FMT.format(i=1)} ~ {TOPIC_FMT.format(i=6)}")

    # OpenCV 渲染保留在主线程，waitKey 同时处理窗口事件。
    no_frame_count = [0] * N_DRONES  # 用于只打印一次"等首帧"日志
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and not stop_event.is_set():
        any_shown = False
        for idx in range(N_DRONES):
            with lock:
                img = latest_frames.get(idx)
            if img is None:
                if no_frame_count[idx] == 0:
                    no_frame_count[idx] = 1
                    rospy.logdebug(f"[{win_names[idx]}] 等待话题 "
                                   f"{TOPIC_FMT.format(i=idx+1)} 的首帧 ...")
                continue
            any_shown = True
            cv2.imshow(win_names[idx], img)
            # 某些后端首次 moveWindow 不生效, 第一帧显示完之后再 move 一次
            if not window_placed[idx]:
                r = window_rects[idx]
                cv2.moveWindow(win_names[idx], r["x"], r["y"])
                window_placed[idx] = True

        # 每 30ms 轮询一次按键 + 让窗口重绘; 'q' 或 ESC 退出
        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            rospy.loginfo("收到 q / ESC, 退出显示。")
            break

        if not any_shown:
            # 没画面时避免忙等
            rate.sleep()

    # 4. 清理
    for s in subs:
        s.unregister()
    for name in win_names:
        try:
            cv2.destroyWindow(name)
        except Exception:
            pass
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    rospy.loginfo("show_swarm_cams 正常退出。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rospy.logerr(f"show_swarm_cams 异常退出: {exc}")
        # 保证所有窗口销毁, 否则屏幕上可能残留下卡死的窗口
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        sys.exit(1)
