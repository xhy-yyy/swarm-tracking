# track_msgs/TargetState.msg
# 一个行人目标的融合状态。由 swarm_fusion.py 发布, 供监视器/rviz/调试使用。
# 注意: 这是我们**自己的估计和自己的影子计数**, 不含任何裁判反馈或 Gazebo 真值。

uint8   tid             # 内部航迹槽位 0..5
string  cls             # green|blue|brown|white|red
# 对应哪个裁判话题: green|blue|brown|white|red1|red2, 未分配时为空串。
# 两个红衣必须占不同的槽(发同一个话题会互相打断 15 秒计数);
# 但红衣的身份互换是无害的 —— 裁判的两个红回调都遍历 [4,5] 按位置匹配,
# 只要两条估计是对两个真红人的双射就行。
string  slot

float32 x
float32 y
float32 vx
float32 vy
float32 pos_std         # sqrt(Pxx+Pyy), 发布门限之一 (<0.35 才发)
float32 last_meas_age   # 距最后一次有效量测的秒数

# 影子计数: 我们自己连续发布了多久 (裁判要求同一目标连续 15s、每帧误差<1m、间隔<1s)。
# 在"发布间隔>0.9s / KF 重init / 门控失败 / 航迹重关联"时清零。
float32 lock_seconds

# 0 UNSEEN  从未见过
# 1 TRACKED 有新鲜量测
# 2 COASTING 靠预测续着(最多 publish_hold 秒)
# 3 ELIMINATED 判定已被裁判消除
# 4 DECOY   判定为不可得分的诱饵(如 casual_female 静态人体模型)
uint8   status

int8    assigned_drone  # 1..6, 0 = 无人负责
