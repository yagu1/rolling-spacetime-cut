# 2026-10-16 Repository Reproducibility Check

**Target:** `yagu1/rolling-spacetime-cut`, baseline `main` = `5b41a870084576685e2c665db87b0362133e58f6`.

## Scope and change policy

This archival update adds research documentation, an experiment index, two test logs, a workflow, and **four previously generated artifacts** recovered from older ZIP files. It does **not** modify existing research algorithms or regression tests.

## Original audit

- GitHub baseline: eight main-branch commits through `5b41a870`; GitHub tree not truncated.
- Past last ZIP and baseline GitHub files: `107 / 107` files verified to match in the original audit.
- Full suite on Linux/Python 3.13.5: **61 passed in 40.82 s**.
- Saved evidence: [pytest_full_before_docs_20261016.log](pytest_full_before_docs_20261016.log).
- Historical audit: [REPOSITORY_AUDIT_20261016.md](REPOSITORY_AUDIT_20261016.md).

## Recovered artifacts (exact bytes from `rolling_spacetime_cut_state_margin.zip`)

| Path | Bytes | SHA-256 |
|---|---:|---|
| `output/connected_cut/report.json` | 607401 | `ac0d4e291c996212146963380897c6724db2dcb3e46af11524eafedd1d12d380` |
| `output/state_margin/report.json` | 181867 | `accf9a931bfd0d8b53783ed39fb2fb0b974828454b37d6bf0758200535024c9c` |
| `output/state_margin/cut_comparison.png` | 62478 | `47e07caff456ef7ca45610e6182f7ac3f719f8e3ac9be1840496777d4beb2aab` |
| `output/state_margin/margin_comparison.png` | 69577 | `b2dccdaf231394e0812162a1f85618988d576c86ff663c025ef086217101f705` |

The recovered results are **historical outputs**, not a new experiment run, and may depend on the precise historical code/environment. They are tracked as archival artifacts even though other generated output can be rebuilt. The previous `.gitignore` contains `output/connected_cut/`: the recovered JSON was added explicitly and remains tracked.

## Post-documentation regression checks

Run from repository root:

```bash
python -m pip install -r requirements.txt
python -m pytest -q tests
```

The full output from this archival update is saved in [pytest_full_after_docs_20261016.log](pytest_full_after_docs_20261016.log). **After archive-only changes: 61 passed / 0 failed in 42.00 s on Linux/Python 3.13.5.** Read the log rather than relying on this note if later edits have altered the state.

The GitHub Actions workflow [python-tests.yml](../../.github/workflows/python-tests.yml) was added for `push`, `pull_request`, and manual dispatch with Python 3.13. **It has not been verified to run on GitHub until this bundle is pushed.**

## Experimental commands

```bash
python experiments/hierarchical_restriction.py --lookahead 60 --update 10 --budget 32
python experiments/render_hierarchical_restriction.py
python experiments/compare_budget_restriction.py

python experiments/contingency_restriction.py --budget 16 --seed 17
python experiments/run_contingency_batch.py --seeds 10 --budget 16
python experiments/render_contingency_restriction.py
```

Outputs are recorded in `output/hierarchical_restriction/` and `output/contingency_restriction/`. The archived pilot data are not a statistical generalization over maps, robotics dynamics or independently randomized hazards.

## Known limitations

- Synthetic grid (1m / 1s), synchronous restrictions, idealized collision shield and point-to-point interpolated robot motion. No real robot safety certification.
- A fixed-S/G TA, no RP-to-TA reassignment, no general local Graph Cut in current supervisor, no MCTS.
- Archived images and reports are useful for audit/review but do not establish theoretical completeness, optimality, safety or publication acceptance.
