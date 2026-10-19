# track_msgs/TargetArray.msg
# 全部 6 个目标槽位的当前状态 + 任务级汇总。
std_msgs/Header header
TargetState[] targets
uint8 n_eliminated      # 已判定消除的数量 (影子计数推断, 非裁判反馈)
uint8 n_tracked         # 当前有新鲜量测的数量
float32 mission_seconds # 从本节点启动算起的秒数
