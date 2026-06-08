# Gazebo EMRMF Scalability Summary

| Robot count | Mean pose RMSE | Std | 95% CI | Alignment RMSE | Fusion time (ms) | Mean theta | Accepted constraints | Success rate |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 0.0126 | 0.0004 | 0.0003 | 0.0532 | 14.90 | 0.190 | 219 | 0.912 |
| 3 | 0.0234 | 0.0006 | 0.0005 | 0.0644 | 17.03 | 0.167 | 325 | 0.904 |
| 4 | 0.0351 | 0.0002 | 0.0002 | 0.0769 | 19.24 | 0.149 | 420 | 0.874 |
| 5 | 0.0466 | 0.0003 | 0.0002 | 0.0886 | 21.39 | 0.132 | 510 | 0.850 |

Each row aggregates 5 trials for the specified robot count.
