# Contributing

Use the pinned ROS Noetic environment and run `bash scripts/build.sh` followed by
`bash scripts/test.sh`. Include the map, start-position configuration, relevant
topic observations and reproduction steps when reporting a flight issue.

Keep model weights, bags, logs, simulator checkouts and build output out of Git.
Do not change algorithms, coordinate conventions and launch defaults together
without explaining and testing the behavior change. Preserve upstream source
notices and keep updates to vendored EGO code documented in its `UPSTREAM.md`.
