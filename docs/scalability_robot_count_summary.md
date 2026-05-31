# Gazebo EMRMF Scalability Summary

| Robot count | Mean pose RMSE | Std | 95% CI | Alignment RMSE | Fusion time (ms) | Mean theta | Accepted constraints | Success rate |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 0.0334 | 0.0004 | 0.0004 | 0.0542 | 14.90 | 0.086 | 219 | 0.912 |
| 3 | 0.0447 | 0.0007 | 0.0006 | 0.0655 | 17.03 | 0.075 | 325 | 0.902 |
| 4 | 0.0562 | 0.0003 | 0.0003 | 0.0781 | 19.24 | 0.066 | 414 | 0.862 |
| 5 | 0.0674 | 0.0003 | 0.0003 | 0.0898 | 21.38 | 0.058 | 483 | 0.805 |

Each row aggregates 5 trials for the specified robot count.
