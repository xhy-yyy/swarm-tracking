# 检测模型说明

当前运行模型是从原工作空间复制的 `best_11s_new.pt`，随独立权重资源包提供。
文件字节数与 SHA-256 保存在 `config/assets.json`。

- 推理接口：Ultralytics `YOLO`。
- 类别顺序：`0 green, 1 blue, 2 brown, 3 white, 4 red`。
- 两个红色行人由融合节点按航迹区分，不是两个检测类别。
- 运行节点：`src/fly_px4/scripts/swarm_vision.py`。
- 默认设备：`cuda:0`；默认批量上限为六路。
- 路径配置：`SWARM_WEIGHTS` 环境变量或节点 `~model_path` 参数。

当前工作空间中未找到这个 checkpoint 的完整训练配置、数据集许可记录、
训练日志或独立测试集指标，因此本文件不填造 mAP、准确率或跨场景性能。
根目录 BSD-3-Clause 许可证不自动适用于权重和训练数据。发布者应在对外上传权重前
补充训练来源及可分发范围；源码也支持用户提供自己的同类别模型。

旧 `robocupV5.pt` 不属于当前默认链路，未加入此次发布资源。
