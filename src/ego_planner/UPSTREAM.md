# Vendored planner snapshot

This is the project's modified EGO-Swarm planner. It is kept as source because
replacing it with an unmodified upstream checkout changes mission behavior.

- Upstream project: https://github.com/ZJU-FAST-Lab/ego-planner-swarm
- Historical upstream commit: not present in the imported workspace metadata.
- Local source snapshot: `5fc179fcdca65c60b611f8783b8c351cf2871193`.
- License: GPL-3.0; individual files and utility packages retain their own notices.

Local changes include planar mapping/trajectory handling and mission-facing
replanning behavior. During publication preparation, package dependency
declarations and generated-message target ordering were corrected. No planner
algorithm rewrite was performed. Original source copyright headers remain.
