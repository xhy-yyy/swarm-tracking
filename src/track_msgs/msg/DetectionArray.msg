# track_msgs/DetectionArray.msg
# 一个推理批次的全部检测(可能来自多架飞机)。
# header.stamp = 本批次最早的成像时刻; 每条 Detection 自带精确的成像时刻。
std_msgs/Header header
Detection[] dets
