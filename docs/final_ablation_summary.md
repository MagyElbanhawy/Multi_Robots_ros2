# Final Ablation Study Summary

Best method: **full_emrmf**.

| Method | Mean RMSE (m) | Std RMSE (m) | 95% CI (m) | Fusion time (ms) | Improvement vs baseline |
| --- | ---: | ---: | ---: | ---: | ---: |
| baseline_graph_slam | 0.3592 | 0.0720 | 0.0210 | 6.90 | 0.0% |
| decentralized_only | 0.2626 | 0.0549 | 0.0160 | 10.80 | 26.9% |
| trust_only | 0.1792 | 0.0266 | 0.0078 | 8.79 | 50.1% |
| full_emrmf | 0.1491 | 0.0280 | 0.0082 | 16.07 | 58.5% |

Each row aggregates 5 repeated runs across 9 packet-loss, delay, and noise scenarios.
