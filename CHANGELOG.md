# Changelog — research prototype

This file indexes historical experiments. The new documentation commit **does not implement new planning algorithms**.

| Version marker | Starting commit | Scope |
|---|---|---|
| initial | `d2f6bf8` | 2D+T two-rover D* Lite, Graph Cut, dynamic replanning |
| step-scaled | `16f166c` | Full horizon from max shortest route steps × 2; no fixed priority experiment |
| connected bridge | `077ae45` | Directed Start–Goal reachability and dynamic cut slices |
| four-rover bridge | `44c693e` | Four-label graph partition and passage order experiments |
| state/margin | `a0ace83` | Dynamic potential and physical safety margins |
| reachability/margin | `d5d280b` | Reachability feedback and synchronized path verification |
| restriction-only supervisor | `148b250` | 60s lookahead / 10s update, supervisor resource gates, onboard PP |
| contingency | `5b41a87` | Local detour, bridge closure, safe hold/resume, 10-seed comparison |

## 2026-10-16 Documentation and audit archival update

- No previous commits, algorithm source files, or tests were altered.
- Preserve historical README under `docs/archive/`.
- Add research framing, preliminary technical report, phase catalog, repository audit and test logs.
- Recover 4 historic generated outputs from archived ZIP (without recomputation).
- Add Python 3.13 GitHub Actions regression test workflow; the workflow can run only after push.

See [repository audit](docs/audits/REPOSITORY_AUDIT_20261016.md).
