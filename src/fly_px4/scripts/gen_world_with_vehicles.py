#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将多机模型内联到 world，在传感器线程启动前完成加载。
避免运行时 spawn_model 与 CPU ray 雷达并发修改 ODE 碰撞空间。
逐机配置 MAVLink/云台端口并保持话题命名空间；成功后将输出路径写到 stdout。"""

import argparse
import re
import sys


def read(path):
    with open(path, 'r') as f:
        return f.read()


def extract_model_block(sdf_text):
    """从 <sdf><model ...>...</model></sdf> 里抠出 model 整块."""
    start = sdf_text.find('<model')
    end = sdf_text.rfind('</model>')
    if start < 0 or end < 0:
        raise RuntimeError('SDF 里找不到 <model> 块')
    return sdf_text[start:end + len('</model>')]


def set_or_insert(block, plugin_name, tag, value):
    """把 plugin 里的 <tag> 设成 value; 没有就插在 plugin 开标签后面."""
    m = re.search(r'<plugin\s+name=[\'"]%s[\'"][^>]*>' % re.escape(plugin_name),
                  block)
    if not m:
        raise RuntimeError('SDF 里找不到 plugin %s' % plugin_name)
    p_start = m.end()
    p_end = block.find('</plugin>', p_start)
    if p_end < 0:
        raise RuntimeError('plugin %s 没有闭合标签' % plugin_name)
    body = block[p_start:p_end]
    new_el = '\n      <%s>%s</%s>' % (tag, value, tag)
    if re.search(r'<%s>.*?</%s>' % (tag, tag), body, re.S):
        body = re.sub(r'<%s>.*?</%s>' % (tag, tag),
                      '<%s>%s</%s>' % (tag, value, tag), body, count=1, flags=re.S)
    else:
        body = new_el + body
    return block[:p_start] + body + block[p_end:]


def make_vehicle(model_block, name, pose, tcp_port, gimbal_port):
    """复制一份 model 块, 换名字/位姿/端口."""
    blk = re.sub(r'<model\s+name=[\'"][^\'"]+[\'"]\s*>',
                 "<model name='%s'>" % name, model_block, count=1)
    # model 级 pose 用起点覆盖 XY，并在原模型离地高度上叠加 Z。
    # 定位第一个 link 前的 pose，允许 model 与 pose 之间存在注释。
    x, y, z = pose
    first_link = blk.find('<link')
    m = re.search(r'<pose>([^<]*)</pose>', blk[:first_link] if first_link > 0
                  else blk)
    if m:
        base = [float(v) for v in m.group(1).split()]
        new_pose = '<pose>%g %g %g %g %g %g</pose>' % (
            x + base[0], y + base[1], z + base[2], base[3], base[4], base[5])
        blk = blk[:m.start(0)] + new_pose + blk[m.end(0):]
    else:
        blk = re.sub(r'(<model[^>]*>)', r"\1\n    <pose>%g %g %g 0 0 0</pose>"
                     % (x, y, z), blk, count=1)
    blk = set_or_insert(blk, 'mavlink_interface', 'mavlink_tcp_port', tcp_port)
    blk = set_or_insert(blk, 'gimbal_controller', 'udp_gimbal_port_remote',
                        gimbal_port)
    return blk


def indent(text, pad='    '):
    return '\n'.join(pad + ln if ln.strip() else ln for ln in text.split('\n'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('base_world')
    ap.add_argument('out_world')
    ap.add_argument('sdf')
    ap.add_argument('--name-prefix', default='typhoon_h480')
    ap.add_argument('--poses', required=True,
                    help='"x,y,z;x,y,z;..." 每架一组, 数量决定飞机数')
    ap.add_argument('--tcp-base', type=int, default=4560)
    ap.add_argument('--gimbal-base', type=int, default=13030)
    a = ap.parse_args()

    poses = []
    for chunk in a.poses.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        vals = [float(v) for v in chunk.split(',')]
        if len(vals) != 3:
            raise RuntimeError('pose 要写成 x,y,z: %r' % chunk)
        poses.append(vals)

    world = read(a.base_world)
    model_block = extract_model_block(read(a.sdf))

    blocks = []
    for i, pose in enumerate(poses):
        blocks.append(make_vehicle(model_block,
                                   '%s_%d' % (a.name_prefix, i),
                                   pose,
                                   a.tcp_base + i,
                                   a.gimbal_base + i))

    injected = ('\n<!-- ==== 以下 %d 架飞机由 gen_world_with_vehicles.py 内联, '
                '不要手改 ==== -->\n' % len(blocks)) + indent(
        '\n'.join(blocks)) + '\n'

    idx = world.rfind('</world>')
    if idx < 0:
        raise RuntimeError('base world 里找不到 </world>')
    out = world[:idx] + injected + world[idx:]
    with open(a.out_world, 'w') as f:
        f.write(out)
    sys.stdout.write(a.out_world)


if __name__ == '__main__':
    main()
