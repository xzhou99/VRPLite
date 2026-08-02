# HSD single-OD prototype - experiment summary

Certificate target: relative full-space gap <= 0.5% (same for every method). All methods warm-start from their own previous-scenario state. Reference flows: exact UE over the full enumerated path space.

## Scenario A_normal (q=100, cap overrides none)

```
method                     iters  RMP   SP peakV vars   rel gap  flow RMSE      ms  structural actions
------------------------------------------------------------------------------------------------------
HSD (ours)                     1    1    1     2    2   0.4185%    11.7979     0.6  -
Frank-Wolfe                    5    0    6    13   13   0.3302%     6.6603     1.1  -
Gradient projection            5    0    6     4    4   0.1095%     6.1788     0.5  -
Flat SD (paths)                4    4    5     4    4   0.0000%     0.0000     3.6  -
Flat SD (aggregated init)      1    1    1     2    2   0.4185%    11.7979     0.3  -
```

## Scenario B1_incident (q=100, cap overrides {3: 15.0})

```
method                     iters  RMP   SP peakV vars   rel gap  flow RMSE      ms  structural actions
------------------------------------------------------------------------------------------------------
HSD (ours)                     2    2    2     3    3   0.4357%    11.3312     1.6  promote:1
Frank-Wolfe                    5    0    5    13   13   0.4211%     6.9302     1.0  -
Gradient projection            5    0    5     4    4   0.1794%     2.0209     0.4  -
Flat SD (paths)                1    1    1     4    4   0.0000%     0.0000     1.7  -
Flat SD (aggregated init)      2    2    2     3    3   0.4357%    11.3312     1.8  -
```

## Scenario B2_surge (q=140, cap overrides {3: 15.0})

```
method                     iters  RMP   SP peakV vars   rel gap  flow RMSE      ms  structural actions
------------------------------------------------------------------------------------------------------
HSD (ours)                     3    3    3     5    5   0.0000%     0.0000     6.8  generate:1, promote:1
Frank-Wolfe                    6    0    6    13   13   0.3822%     2.0661     1.3  -
Gradient projection            2    0    2     5    5   0.4538%     4.1304     0.3  -
Flat SD (paths)                2    2    2     5    5   0.0000%     0.0000     6.2  -
Flat SD (aggregated init)      3    3    3     5    5   0.0000%     0.0000     5.4  -
```

## Scenario C_recovery (q=100, cap overrides none)

```
method                     iters  RMP   SP peakV vars   rel gap  flow RMSE      ms  structural actions
------------------------------------------------------------------------------------------------------
HSD (ours)                     1    2    2     5    2   0.0000%     0.0000    19.7  merge:2, drop:1
Frank-Wolfe                    6    0    6    13   13   0.4757%     2.3798     1.4  -
Gradient projection            2    0    2     5    4   0.2973%     1.3765     0.2  -
Flat SD (paths)                1    1    1     5    5   0.0000%     0.0000     6.3  -
Flat SD (aggregated init)      1    1    1     5    5   0.0000%     0.0000     7.4  -
```

## Scenario D_reshock (q=100, cap overrides {3: 15.0})

```
method                     iters  RMP   SP peakV vars   rel gap  flow RMSE      ms  structural actions
------------------------------------------------------------------------------------------------------
HSD (ours)                     2    2    2     3    3   0.0191%     0.4047     1.8  promote:1
Frank-Wolfe                    5    0    5    13   13   0.4762%     2.2066     1.1  -
Gradient projection            3    0    3     5    4   0.3896%     3.0501     0.3  -
Flat SD (paths)                1    1    1     5    5   0.0000%     0.0000     1.7  -
Flat SD (aggregated init)      1    1    1     5    5   0.0000%     0.0000     2.9  -
```

## HSD round-by-round narrative

```
[A_normal    ] round   1  gap=  0.4185%  K=2  -
[A_normal    ] round   2  gap=  0.4185%  K=2  -
[A_normal    ] round   3  gap=  0.4185%  K=2  -
[A_normal    ] round   4  gap=  0.4185%  K=2  -
[A_normal    ] round   5  gap=  0.4185%  K=2  -
[A_normal    ] round   6  gap=  0.4185%  K=2  -
[B1_incident ] round   1  gap=  5.0268%  K=3  promote p2 out of family_A [priced path hidden]
[B1_incident ] round   2  gap=  0.4357%  K=3  -
[B1_incident ] round   3  gap=  0.4357%  K=3  -
[B1_incident ] round   4  gap=  0.4357%  K=3  -
[B1_incident ] round   5  gap=  0.4357%  K=3  -
[B1_incident ] round   6  gap=  0.4357%  K=3  -
[B1_incident ] round   7  gap=  0.4357%  K=3  -
[B2_surge    ] round   1  gap=  2.4747%  K=4  generate p5 (new priced path)
[B2_surge    ] round   2  gap=  0.8723%  K=5  promote p3 out of family_B [priced path hidden]
[B2_surge    ] round   3  gap=  0.0000%  K=5  -
[B2_surge    ] round   4  gap=  0.0000%  K=5  -
[B2_surge    ] round   5  gap=  0.0000%  K=5  -
[B2_surge    ] round   6  gap=  0.0000%  K=5  -
[B2_surge    ] round   7  gap=  0.0000%  K=5  -
[B2_surge    ] round   8  gap=  0.0000%  K=5  -
[C_recovery  ] round   1  gap=  0.0000%  K=5  -
[C_recovery  ] round   2  gap=  0.0000%  K=5  -
[C_recovery  ] round   3  gap=  0.0000%  K=5  -
[C_recovery  ] round   4  gap=  0.0000%  K=4  merge {p2, p1} -> (p2+p1) (pi=0.50/0.50)
[C_recovery  ] round   5  gap=  0.0000%  K=3  merge {p3, p4} -> (p3+p4) (pi=0.70/0.30)
[C_recovery  ] round   6  gap=  0.0000%  K=2  drop p5 (lambda ~ 0)
[C_recovery  ] round   7  gap=  0.0000%  K=2  -
[C_recovery  ] round   8  gap=  0.0000%  K=2  -
[C_recovery  ] round   9  gap=  0.0000%  K=2  -
[C_recovery  ] round  10  gap=  0.0000%  K=2  -
[C_recovery  ] round  11  gap=  0.0000%  K=2  -
[C_recovery  ] round  12  gap=  0.0000%  K=2  -
[D_reshock   ] round   1  gap=  3.3342%  K=3  promote p2 out of (p2+p1) [priced path hidden]
[D_reshock   ] round   2  gap=  0.0191%  K=3  -
[D_reshock   ] round   3  gap=  0.0191%  K=3  -
[D_reshock   ] round   4  gap=  0.0191%  K=3  -
[D_reshock   ] round   5  gap=  0.0191%  K=3  -
[D_reshock   ] round   6  gap=  0.0191%  K=3  -
[D_reshock   ] round   7  gap=  0.0191%  K=3  -
```

## Final frontier

- lambda = 0.3769  `Atom[p2](p2:1.000)`
- lambda = 0.0933  `Atom[p1](p1:1.000)`
- lambda = 0.5298  `Atom[(p3+p4)](p3:0.701, p4:0.299)`
