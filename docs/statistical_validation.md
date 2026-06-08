# Statistical Validation

| Method | Runs | Samples/run | Mean RMSE (m) | Std Dev (m) | 95% CI (m) | Improvement vs baseline |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Baseline Graph SLAM | 5 | 200 | 0.359 | 0.072 | +/-0.021 | 0.0% |
| Decentralized Only | 5 | 200 | 0.263 | 0.055 | +/-0.016 | 26.9% |
| Trust Only | 5 | 200 | 0.179 | 0.027 | +/-0.008 | 50.1% |
| Full EMRMF | 5 | 200 | 0.149 | 0.028 | +/-0.008 | 58.5% |

Paired Wilcoxon signed-rank test comparing Baseline Graph SLAM and Full EMRMF across matched repeated configurations:

- Matched pairs: 1620
- W = 0
- p < 0.001
