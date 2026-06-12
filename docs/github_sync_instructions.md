# GitHub sync instructions for EMRMF experiment package

Use this checklist when `Test-Path src/emrmf_experiments` returns `False` on Windows.

## 1. Confirm that you are in the real clone

The expected repository folder used during the reviewer-release workflow was:

```powershell
cd "C:\path\to\repo\ros2 phd"
```

Do not use the similarly named folder below unless it is also a proper clone:

```powershell
C:\path\to\repo\ros2 phd
```

Check that the GitHub remote exists:

```powershell
git remote -v
```

The output should include:

```text
origin  https://github.com/<repository>/Multi_Robots_ros2.git (fetch)
origin  https://github.com/<repository>/Multi_Robots_ros2.git (push)
```

## 2. Update main from GitHub

```powershell
git checkout main
git pull origin main
```

Then check whether the experiment package exists:

```powershell
Test-Path src/emrmf_experiments
```

If this prints `False`, GitHub `main` does not yet contain the latest `emrmf_experiments` package.

## 3. Push the validation package branch

The validation package must be pushed as a branch and merged into `main` through a pull request:

```powershell
git checkout -b reviewer/add-emrmf-experiments-validation
git add src/emrmf_experiments scripts/generate_emrmf_experiment_validation.py README.md .github/workflows/ci.yml
git commit -m "Add EMRMF experiment validation package"
git push -u origin reviewer/add-emrmf-experiments-validation
```

Open a GitHub pull request from:

```text
reviewer/add-emrmf-experiments-validation
```

into:

```text
main
```

After merging the pull request, run:

```powershell
git checkout main
git pull origin main
Test-Path src/emrmf_experiments
```

The final command should print `True`.

## 4. Validate the experiment package

After `src/emrmf_experiments` is present, run:

```powershell
py -m compileall -f -q src/emrmf_core/emrmf_core src/emrmf_core/launch src/emrmf_experiments/emrmf_experiments src/emrmf_experiments/launch scripts
$env:PYTHONPATH="src/emrmf_core;src/emrmf_experiments"
pytest -q src/emrmf_core/test src/emrmf_experiments/test
```

Generate reviewer-ready validation artifacts with:

```powershell
$env:PYTHONPATH="src/emrmf_experiments"
py scripts/generate_emrmf_experiment_validation.py --output-dir docs/emrmf_experiment_logs --p-values 2,3,4 --gamma-values 0.1,0.3,0.5,1.0 --delay-values 0.0,0.5,2.0 --packet-loss-values 0.0,0.10,0.30 --sensor-noise-values 0.10,0.20,0.30 --repeated-runs 5 --tau-e 0.5
```
