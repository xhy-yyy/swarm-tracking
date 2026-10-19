# 7 张固定地图的六机巡航

已准备 7 套预设 `route1..route7`，使用同一个 `fly_px4_1` 控制程序。比赛时由操作者观察马路布局后手动选航线，官方地图可以改名。`base0、base3、base4、base5、base6、base15、base16` 只是准备阶段的参考图和内部配置代号，不要求官方文件使用这些名称。

## 启动

比赛时按原方式启动仿真，观察马路后在另一个终端选择预设，例如 `route2`：

```bash
# 终端 1：原六机启动命令
roslaunch px4 multi_vehicle_6.launch

# 终端 2：根据看到的马路布局选择预设
bash $SWARM_WS/src/fly_px4/scripts/start_swarm.sh route2
```

比赛当天先把裁判公布的六机起点填入 [start_positions.yaml](../../config/start_positions.yaml)，再选择第二条命令的 `routeN`。七套航线统一放在 `config/patrol_maps/baseN.yaml`，`route1` 加载 `base0.yaml`，公共速度/跟踪参数继承 `config/patrol_routes.yaml`。启动脚本的比赛流程仅加载人工选择的航线和起点 YAML，不读取仿真 world 文件、官方地图名称或 Gazebo 真值，也不要求本机保留参考 world 文件。

```bash
# 原 base0 仍可不传参数
bash $SWARM_WS/src/fly_px4/scripts/start_swarm.sh

# 查看按马路选择航线的对照
bash $SWARM_WS/src/fly_px4/scripts/start_swarm.sh --list-routes

# 单独启动飞控时也能选择地图；规划和视觉应已启动
# map:=base15 是内部预设代号，对应 route6，与官方文件名无关
roslaunch fly_px4 swarm_fly_px4_1.launch map:=base15

# 赛前本地离线检查，需要准备阶段的参考 world；比赛不运行此选项
bash $SWARM_WS/src/fly_px4/scripts/start_swarm.sh route7 --check-only
```

原 `baseN` 参数仍兼容，例如 `start_swarm.sh base3` 与 `start_swarm.sh route2` 加载相同航线。仿真的 world 加载仍由原 `multi_vehicle_6.launch` 负责；它按比赛环境提供的场景启动，巡航脚本不选择或识别仿真地图。此前新增的启动前 world 比对已经移除。

## 只看马路怎样选择

先对齐地图方向：东侧有两条相距很近的南北路，西侧两条南北路的间距较大。将东侧放在右边，图上方为北。下面的“中区/东区”按南北路之间的街区宽度判断，“北区/南区”按主街上下的街区高度判断；不需要读取世界坐标。

| 启动参数 | 中区与东区宽度 | 北区与南区高度 | 内部参考配置 |
|---|---|---|---|
| route1 | 近似等宽 | 近似等高 | base0 |
| route2 | 东区明显宽，约中区的 1.8 倍 | 近似等高 | base3 |
| route3 | 中区明显宽，约东区的 1.5 倍 | 近似等高 | base4 |
| route4 | 近似等宽 | 北区较矮、南区较高 | base5 |
| route5 | 近似等宽 | 北区较高、南区较矮 | base6 |
| route6 | 中区明显宽，约东区的 1.8 倍 | 北区较高、南区较矮 | base15 |
| route7 | 中区明显宽，约东区的 1.8 倍 | 北区较矮、南区较高 | base16 |

[只含马路的七套布局对照图](road_choices.png)：每张图使用相同方向和比例，红色为主街、蓝色为中间南北路。先按北/南区高度分为三组，再比较中/东区宽度，就能选择对应预设。官方文件名不参与选择。

## 赛前编译

本次控制程序已编译。以后只改 YAML 无需编译；改 C++ 后本工作空间使用：

```bash
cd $SWARM_WS
catkin build fly_px4 --no-deps --make-args fly_px4_1 -j2
# 修改 EGO 规划状态机后，还需编译它的节点
catkin build ego_planner --no-deps --make-args ego_planner_node -j2
```

## 巡航到点后直接切换下一航点

沿用旧提交 `15652b4` 的直接切点方式。水平距巡航航点小于 `arrive_radius=1.5m` 时立即发布下一航点，不锁住当前位置等待机头对准。EGO 规划水平运动，机头在移动中按现有 `patrol_yaw_rate_max` 限速跟随航向。

七套原地图及三套比赛地图的六机共用该切点方式，包括闭环首尾切换。一次性进场、跟踪抢占、归航时的转向与避障继续沿用现有逻辑。速度、航点、跟踪高度和起飞延时保持现有配置；已移除不再使用的 `patrol_yaw_align_tolerance_deg`。

`test/smoke_patrol_waypoint_handoff.py` 用独立 ROS master 和模拟 MAVROS/EGO 输入运行实际控制节点，检查十套地图在机头偏离下一段 90 度时仍立即切点、移动指令继续转发、进场只走一次、两圈闭合以及跟踪抢占和归航后恢复巡航。测试不启动 Gazebo 或 PX4。

## 跟踪高度

跟踪目标高度以 [config/patrol_routes.yaml](../../config/patrol_routes.yaml) 的 `patrol.defaults.track_z` 为准，当前配置为 3.24m；`patrol.routes.drone_N.track_z` 可单机覆盖，地图预设也沿用这个优先级。相机零姿态安装位置下移 4cm 后，跟踪高度由 3.20m 提高到 3.24m，使零姿态光心高度仍为 3.04m；雷达扫描面和机体高度也相应提高 4cm。主控不再把值钳到 2.7m 或巡航高度范围，也没有编译进程序的默认跟踪高度。缺失、非数值、非有限数值或非正数会拒绝启动。

高度在节点启动时读取。以后修改 YAML 后重启 `swarm_fly_px4_1.launch` 即可应用，无需重新编译。跟踪高度改变后，相机光心和二维雷达扫描面仍需要落在配置的感知薄片内；`track_z` 设定的是目标高度，实际高度仍取决于飞控响应。

## 比赛当天填写起点

统一编辑 [config/start_positions.yaml](../../config/start_positions.yaml)。`drone_1..drone_6` 对应 `typhoon_h480_0..typhoon_h480_5`，填写与马路航点相同地图坐标系中的水平 `x/y`，单位米。下方示例列出原来起点，实际配置以文件当前内容为准；可以单独调整任意一架飞机，也可以六机整体平移。

```yaml
drone_1: {x: -14.0, y: -3.0}
drone_2: {x: -14.0, y:  0.0}
drone_3: {x: -14.0, y:  3.0}
drone_4: {x: -17.0, y: -3.0}
drone_5: {x: -17.0, y:  0.0}
drone_6: {x: -17.0, y:  3.0}
```

两套巡航 launch 自动把同一份 YAML 交给各机飞控和里程计桥接。桥接把本地估计位置加上该机起点得到地图位置；飞控把 EGO 的地图指令减去该机起点得到本地指令。已有水平尺度修正继续生效。七套马路航点的位置保持不变，巡航、归航和视觉跟踪使用一致的坐标。

改完文件，停止并重新运行 `start_swarm.sh routeN` 即可，无需编译。如果分别启动两个巡航 launch，必须把 `swarm_relay_ego.launch` 和 `swarm_fly_px4_1.launch` 一起重启。脚本会在启动任何节点之前检查六机配置并打印起点，缺失坐标、字符串、NaN 或无穷大都会停止启动。

该 YAML 不加载进 `robocup_xhyyy.launch` 或 `multi_vehicle_6.launch`，也不改变仿真中的飞机位置。若使用前者的平移后起点，请手动把 YAML 的 1～3 号机 `x` 改为 `0.0`、4～6 号机 `x` 改为 `-3.0`，`y` 保持不变。这里沿用原方案的地面高度与坐标轴方向，只配置水平起点。

`test/smoke_start_positions.py` 在独立 ROS master 上使用模拟 MAVROS 输入和 EGO 指令，运行实际六机飞控与里程计桥接，验证整体平移 14m、单独修改 6 号机起点、相机位姿和飞控指令的双向坐标转换、马路目标保持不变，以及非法起点拒绝启动。测试不启动 Gazebo、PX4 或真实飞机。

需要使用另一份起点文件时，两套巡航 launch 都支持 `start_positions_file:=/绝对路径/starts.yaml`。赛前离线检查也读取当前起点 YAML，检查新起点到进场点的路段。当前示意图和报告按本次起点 YAML 绘制，再次修改起点后需重新核对进场段。

## 4 号机的起飞进场

`base0.world` 和 `base2.world` 在 `(-3,-3.7)` 都有 `lamp_post_191`。起点为 `(-3,-3)` 时，灯杆中心仅距飞机中心 0.7m。日志显示此次选择的是 `base0` 预设，原首航点 `(-16.5,-43.5)`；起飞后直飞该点的线段距灯杆碰撞表面只有约 0.14m，还会穿过房区。

`base0` 的 4 号机现先经过 `(-6,-0.8)` 和 `(-16.5,-1.5)`，在第二点接入逆时针闭环：`(-16.5,-1.5) → (-43.5,-1.5) → (-43.5,-43.5) → (-16.5,-43.5) → 首点`。仍垂直离地到 2m，然后向西北边爬升边进场；一次性进场段不重复，第二点同时是闭环首点，之后按闭环正常巡航和归航。起点 YAML 仍填写实际起点，不用移动坐标来绕障碍。

两张参考地图的离线碰撞检查显示：新首段从距灯杆约 0.63m 的实际起点向外远离，后续两段及进场切角的中心净空至少约 2.18m。起点本身仍偏近；这项检查与模拟 MAVROS 验证不能代替 Gazebo 实飞。

其余六个预设的 4 号机也使用两个一次性进场点，按主街位置衔接原南西闭环：

| 预设 | 首个进场点 | 西侧路口进场点 |
|---|---|---|
| base0 / base3 / base4 | (-6,-0.8) | (-16.5,-1.5) |
| base5 / base16 | (-6,6.2) | (-16.5,6.2) |
| base6 / base15 | (-6,-9.2) | (-16.5,-11.3) |

以当前 4 号机起点 `(-3,-3)` 检查，所有预设接入首个进场点后的两段及切角净空至少约 2.18m。`base5/base6/base15/base16` 的该起点不在主街上，首次接入会经过道路外的空地，不能描述为全程马路净空。`base16` 的该起点还距 `house_2_125` 的碰撞投影仅约 0.036m；新增进场点向外远离房屋，但原地离地到 2m 的阶段仍需核实实际间隙，不能据此认定起飞安全。上述检查不会改变实际起点或两份仿真 launch。

按用户说明，实际起飞点本身有障碍物的地图比赛不会使用，这些情况排除实飞使用；有新的无障碍起点时，在起点 YAML 填写真实坐标并重新核对进场段。

`base2.world` 尚无专用六机巡航预设，且中间南北路为斜路，与 `route2` / `base3` 不同。此处只修正当前使用的 `base0` 预设的 4 号机起飞进场，不能据此认为整套 `base0` 航线适合 `base2`。

## 原始起点与分工

| ID | Gazebo 名称 | 世界坐标起点 | 巡航区域 |
|---|---|---|---|
| 1 | typhoon_h480_0 | (-14,-3) | 南中 SC |
| 2 | typhoon_h480_1 | (-14,0) | 北中 NC |
| 3 | typhoon_h480_2 | (-14,3) | 北东 NE |
| 4 | typhoon_h480_3 | (-17,-3) | 南西 SW |
| 5 | typhoon_h480_4 | (-17,0) | 南东 SE |
| 6 | typhoon_h480_5 | (-17,3) | 北西 NW |

每机仍绕所属区域飞四点闭环，巡航目标高度统一 4.2m（飞控本地 ENU 高度），跟踪高度读取 YAML 的 `track_z`，公共巡航速度 12m/s、进场速度 8m/s。七套地图均沿用原 base0 起飞方式（`climb_in_place=false`）：六机独立解锁、原起点垂直爬到 2m 后，就由 PX4 边爬升边飞向本机首个进场点或巡航点。达到 `min_expl_z=4.1m` 且 `start_delay` 到期才交给 EGO；延迟从开始 CLIMB 计时，与爬升重叠，不阻止 2m 以上的水平移动。保留现有 `patrol_collision_check=true`。巡航位置指令范围同步为 `[4.1,4.9]m`，软限 5.7m、硬限 6.0m；EGO 和二维雷达点云的规划平面统一为 4.5m。

七套地图的 1～6 号机 `start_delay` 已全部设为 `0.0` 秒。各机达到 `min_expl_z=4.1m` 后立即允许 EGO 接管，没有额外的固定进场等待。启动脚本保持连续启动方式；起飞仍先垂直离地到 2m，再按既有进场点边升边飞。

新增六图的 1/2/3/5/6 号机首个 `entry_waypoints` 仍按原六机格点布置：目标相对原起点东移 0.5m，主街北移的 base5/base16 北移 9m，主街南移的 base6/base15 南移 9m。4 号机改为上述进场段。六机首个目标之间至少相隔 3m；更改起点后，进场点仍保持原来的地图坐标。实际间距仍取决于各机控制响应和起飞时机。

后续进场点由 EGO 规划：3/5 号机沿 `x=-15` 附近进入主街，再东西向进场；6 号机进入西侧车道再西行。`entry_waypoints` 在首圈之前只走一次，闭环结束直接回到 `waypoints[0]`，跟踪结束也只选择闭环归航，不重走进场段。七个预设的 4 号机均从西侧路口按“西 → 南 → 东 → 北”逆时针巡航，航点坐标和上述主街进场段保持不变；公共 YAML 和内置 base0 兜底闭环使用相同顺序。

## 首航点停住的修复

2026-10-05 的 base5 日志中，3/6 号机首目标为 `y=12`，实际停在 `y≈9.7`，仍距目标约 2.3m。EGO 根据局部轨迹的计时结束切入 `WAIT_TARGET`，而 `traj_server` 继续发布带新时间戳的终点悬停指令；主控收到新鲜指令就不重发目标，直到 25s 冻结脱困才恢复规划。

EGO 现在在轨迹计时结束后检查 MAVROS 估计里程计与目标的距离。超过 `fsm/goal_reached_thresh=1.0m` 时保留当前目标，从实时里程计重新规划；实际到达该范围才进入等待状态。二维规划按实际水平距离判断，到点判断不会把巡航平面 4.5m 与 YAML 跟踪高度的差值误认为未到点；三维规划仍检查完整三维距离。主控仍按原来的 1.5m 到点半径切换巡航点。本修复不读取 Gazebo 真值，沿用原来的避障设置。

`test/smoke_ego_goal_completion.py` 在独立 ROS master 上运行实际 EGO 和轨迹发布器，注入模拟里程计，验证轨迹结束但未到点时继续规划、到点后停止规划、接着完成下一个航点，以及 YAML 跟踪高度下仍按水平位置完成二维目标。`smoke_patrol_maps.py` 同时检查跟踪指令采用 YAML 的值、单机覆盖、非法配置拒绝启动，以及跟踪释放后回到巡航目标高度 4.2m。测试不启动 Gazebo 或 PX4。

## 地图与净空

以下几何分析仅在赛前针对已有参考文件完成，用来准备固定航点；比赛运行时不读这些文件。坐标来自各 `.world` 的道路、碰撞模型及保存的 `<state>`。使用保存的 link 世界位姿，避免把它再次叠加到 model 位姿。加油站的模型原点与碰撞几何中心并不重合；离线解析还应用了 Collada 场景变换、英寸单位与 SDF 缩放。

| 地图 | 中间南北路 x | 主街 y | 最小闭环静态净空 | 最慢圈时粗估 |
|---|---:|---:|---:|---:|
| base0 | 45 | 0 | 2.13m | 24.1s |
| base3 | 30 | 0 | 1.83m | 26.6s |
| base4 | 60 | 0 | 1.83m | 25.2s |
| base5 | 45 | 10 | 1.64m | 25.4s |
| base6 | 45 | -10 | 1.71m | 25.7s |
| base15 | 65 | -10 | 1.71m | 27.5s |
| base16 | 65 | 10 | 1.64m | 27.1s |

`base3` 北中东侧取 `x=29`、北东西侧取 `x=32`，避开 `(27,3.7)` 附近灯杆。`base4` 南中东侧取 `x=60`、南东西侧取 `x=63`，避开 `(58,-3.7)` 附近灯杆。`base5/base16` 南、北主街车道取 `y=6.2/11.8`，避开移到 `y≈8/14` 的灯杆。`base6/base15` 取 `y=-11.3/-8.3`。东侧外圈沿原方案继续取 `x=113`。

所有闭环几何之间至少相隔 3m。这个数值描述固定路线之间的距离；EGO 实际轨迹、跟踪绕行、进场路线之间的间隔需要由规划与仿真验证。

离线检查包含：闭环、首尾闭合段、到点半径导致的切角、起点到进场点再到闭环首点、配置起点净空、巡航高度与六机闭环间距。巡航碰撞高度带为 `[3.85,5.15]m`，进场带为 `[1.35,5.15]m`，要求航线中心距保守碰撞投影至少 1.5m。闭环保持原净空结果；进场必须按实际起点重查。当前图和 validation.json 按现有起点 YAML 更新，完整检查会标出起点偏近及进场离开道路的情况，不能将接入道路后的净空通过理解为全部起飞段通过。

## 文件、图与检查

- 公共参数：[patrol_routes.yaml](../../config/patrol_routes.yaml)
- 六机起点：[start_positions.yaml](../../config/start_positions.yaml)
- 七套航线：[base0](../../config/patrol_maps/base0.yaml)、[base3](../../config/patrol_maps/base3.yaml)、[base4](../../config/patrol_maps/base4.yaml)、[base5](../../config/patrol_maps/base5.yaml)、[base6](../../config/patrol_maps/base6.yaml)、[base15](../../config/patrol_maps/base15.yaml)、[base16](../../config/patrol_maps/base16.yaml)
- [7 图总览](overview.png)：实线为闭环、虚线为一次性进场、箭头为巡航方向、叉号为固定起点、灰色为碰撞投影、黑点为灯杆。
- [比赛选航线用的马路布局对照](road_choices.png)
- 单图：[base0](base0.png)、[base3](base3.png)、[base4](base4.png)、[base5](base5.png)、[base6](base6.png)、[base15](base15.png)、[base16](base16.png)
- [离线检查结果](validation.json)

```bash
cd $SWARM_WS/src/fly_px4
python3 scripts/check_patrol_maps.py --all
python3 scripts/check_patrol_maps.py --map base4
python3 scripts/check_patrol_maps.py --all --plot-dir docs/patrol_maps --report docs/patrol_maps/validation.json
python3 scripts/plot_patrol_road_choices.py
source $SWARM_WS/devel/setup.bash
python3 test/smoke_patrol_maps.py
python3 test/smoke_patrol_waypoint_handoff.py
python3 test/smoke_ego_goal_completion.py
python3 test/test_start_positions.py
python3 test/smoke_start_positions.py
```

检查依赖 `python3-numpy、python3-shapely、python3-yaml`，画图还需要 `python3-matplotlib`；本机已具备。`check_patrol_routes.py` 原调用保留，也支持新的地图选项。

已验证 C++ 编译、7 图共 42 条航线的 ROS 参数，以及独立 ROS + 模拟 MAVROS 输入下的垂直离地、2m 以上边爬边进场、EGO 接管延迟、一次性进场、进场/巡航速度切换、连续两圈闭合、跟踪释放后归航及损坏航线拒绝 base0 兜底。比赛启动流程已改为人工选预设，不核对官方地图。尚未逐图完成六机 Gazebo 实飞；静态净空和粗估圈时不代表实飞结果。
