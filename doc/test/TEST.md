# TEST.md

Smoke tests: **2 tests** total.

Build-time smoke specs run inside each Dockerfile `-test` stage. Specs live
under `test/bats/smoke/{shared,devel-test,runtime-test}/`; `shared/` runs on
every `-test` stage, the per-stage folders hold stage-specific assertions.
The figure above is the total `@test` count across all stage folders, kept
in sync with `grep -cE '^@test'` (base's `sync-doc-counts.sh` source of
truth) -- regenerate it when you add or remove specs.

## Smoke specs

### test/bats/smoke/shared/urdf2usd_env.bats (2)

| Test | Description |
|------|-------------|
| `entrypoint.sh is installed and executable` | Entrypoint present + executable |
| `bash is available on PATH` | bash resolvable on PATH |
