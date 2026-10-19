# 发布到 GitHub

源码仓库与大资源分别发布。根目录已初始化独立 Git 仓库，构建产物、
外部依赖 checkout、模型网格、权重和 `dist/` 不会进入提交。

## 本地检查

```bash
bash scripts/build.sh
bash scripts/test.sh
python3 scripts/check_assets.py
python3 scripts/audit_release.py
```

本次的验证范围见 `docs/validation.md`。README 已明确标注未重跑完整任务。
模型训练来源和分发范围尚未记录，见 `docs/model_card.md`；可先公开源码，
权重由使用者自行提供，或在补充来源后另发权重资源。

## 生成发布文件

```bash
python3 scripts/package_release.py
```

生成文件位于 `dist/`：

- `swarm-tracking-source-v0.1.0.tar.gz`：源码快照。
- `swarm-tracking-models-v0.1.0.tar.gz`：仿真模型网格与材质。
- `swarm-tracking-weights-v0.1.0.tar.gz`：当前检测权重。
- `SHA256SUMS`：压缩包校验值。

打包脚本同步更新 `config/release_assets.json`，因此应在打包后再提交源码。
同一版本必须使用配套资源；更换资源后重新打包并提交清单。

## 首次推送

在 GitHub 创建空仓库后，从本项目根目录执行，仓库地址已配置为 `xhy-yyy/swarm-tracking`：

```bash
git add .
git diff --cached --stat
git commit -m "Prepare initial public release"
git remote -v
# origin: https://github.com/xhy-yyy/swarm-tracking.git
git push -u origin main
git tag v0.1.0
git push origin v0.1.0
```

随后创建对应的 GitHub Release，把所需资源包与 `SHA256SUMS` 作为附件上传。
目标仓库为 https://github.com/xhy-yyy/swarm-tracking 。资源附件在源码推送后通过 GitHub Release 分发。

原创部分的 BSD-3-Clause 许可按 xhy 署名。EGO 与其他来源保留各自许可，详见
`THIRD_PARTY_NOTICES.md`，不要用根目录许可证覆盖这些组件。
