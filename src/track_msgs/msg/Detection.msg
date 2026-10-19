# track_msgs/Detection.msg
# 一次 YOLO 检测 + 它的世界系定位估计。
#
# 由 fly_px4/scripts/swarm_vision.py 产生, 由 fly_px4/scripts/swarm_fusion.py 消费。
# 这个包故意只放消息、不依赖任何东西: racer_ws 是 catkin_make 编的、catkin_ws 是
# catkin build 编的, 让 exploration_manager 依赖本包会把两个工作空间的构建顺序绑死。
# 所以面向 RACER 的接口一律只用 nav_msgs/std_msgs, 本包只在 catkin_ws 内部用。

# stamp = **成像时刻**(彩色图的 header.stamp), 不是发布时刻。
# 融合端要靠它把估计推到当前时刻, 消掉 100~200ms 的流水线延迟。
# frame_id 恒为 "world"(与 RACER 占用图同帧)。
std_msgs/Header header

uint8   drone_id        # 1..6 (对应 typhoon_h480_{drone_id-1})
uint8   cls_id          # 0 green, 1 blue, 2 brown, 3 white, 4 red  (== YOLO model.names)
float32 conf            # YOLO 置信度

# ---- 原始像素观测 (融合端做二次判断/复算用) ----
float32 u               # 检测框中心列
float32 v_top           # 框上沿行
float32 v_bot           # 框下沿行
float32 box_w           # 框宽(像素)

# ---- 世界系定位估计 ----
# 估计的是 actor **模型原点** 的 (x,y), 即 z=1.0191 那个点 ——
# 这正是裁判 get_model_state('actor_N') 返回、并用来和我们的消息比对的那个点。
float32 x
float32 y

# 各向异性不确定度: 沿视线方向的测距误差通常是横向的 2~3 倍。
# 融合端用 bearing 把它们旋转成 2x2 的 R; 用各向同性 R 会把 KF 带偏。
float32 sigma_along     # 沿视线 1-sigma (m)
float32 sigma_cross     # 垂直视线 1-sigma (m)
float32 bearing         # 视线在世界系的方位角 (rad)
float32 slant           # 相机到目标的斜距 (m)

# 哪些估计量参与了本次融合 (位掩码), 便于事后分析哪条链路在起作用:
#   1 = DEPTH      深度图@躯干中位数
#   2 = MONO_FOOT  框底 ∩ z=0.159  (脚部平面)
#   4 = MONO_HEAD  框顶 ∩ z=1.879  (头顶平面)
#   8 = JOINT      脚/头两条射线联合解出 (距离, 相机高度);
#                  只用于离群否决和相机高度自标定, 不进加权平均
# 注: 刻意没有"框中心 ∩ 原点高度"这一项 —— 透视投影非线性, 框上下沿的像素中点
#     并不是人体几何中心的投影, 实测会引入 +0.02m 量级的**系统性**测距偏差。
uint8   method

bool    truncated       # 框贴到图像边缘(脚或头被截), 此时 MONO_FOOT/PIXHEIGHT 不可信
float32 pipeline_lag    # 发布时刻 - 成像时刻 (s), 监控用

# Independent image-centre feedback for yaw. Never replace this bearing with
# the depth surface bearing used by the localisation estimate above.
# Valid only with the actual camera orientation bracketed at header.stamp.
bool    image_bearing_valid
float32 image_bearing   # world azimuth of the bounding-box centre ray (rad)
float32 image_error_x   # bounding-box centre u - CameraInfo cx (pixels)
