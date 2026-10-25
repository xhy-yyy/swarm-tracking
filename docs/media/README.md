# 演示素材 / Demo media

素材由项目作者提供，用于展示原开发环境中的六机视觉检测、目标跟踪与搜索场景。
这些录屏不替代当前源码版本的复测记录，验证范围见 [validation.md](../validation.md)。

The project author supplied these recordings from the original development environment.
See the [validation record](../validation.md) for the current source workspace's reproduction scope.

| 仓库文件 / Repository file | 原始素材 / Source | 处理 / Processing |
| --- | --- | --- |
| [`swarm-overview.jpg`](swarm-overview.jpg) | `微信图片_20261025201524_38_15.jpg` | 原图复制，2816 × 1280；未修改图像内容。 / Unmodified copy. |
| [`target-tracking.mp4`](target-tracking.mp4) | `125257071f368ec7e0d64a1419dd4685.mp4` | 约 20.4 秒，1152 × 720；HEVC 转 H.264，保留完整画幅、时长与 AAC 音轨。 / Full-frame H.264 conversion with original audio. |
| [`swarm-search.mp4`](swarm-search.mp4) | `7ac12c22415cf1763a559b4db823f022.mp4` | 约 19.6 秒，1152 × 720；同上。 / Same conversion. |
| [`target-tracking.gif`](target-tracking.gif) | 目标跟踪视频 / Target tracking clip | 第 3–11 秒片段循环，6 fps、400 × 400；移除两侧黑边后等比缩放与补边。 / 8-second preview with aspect ratio preserved. |
| [`swarm-search.gif`](swarm-search.gif) | 搜索俯视视频 / Overhead search clip | 同上。 / Same preview settings. |

MP4 使用 H.264、`yuv420p` 与 `faststart`，便于常见浏览器播放；GIF 用于 README 中的动态预览。
转码未插帧、加速或重排原始内容。桌面原始文件保持原样。

MP4 files use H.264, `yuv420p`, and `faststart` for browser compatibility. GIFs provide animated README previews.
No interpolation, speed changes, or scene reordering were applied. Original source files were left untouched.
