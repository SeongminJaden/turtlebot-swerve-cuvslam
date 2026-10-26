# Isaac Sim과 Isaac Lab 설치·실행

이 프로젝트는 팀별 URDF/USD를 전제로 한다. M38은 실행 시연 모델이며 관절 이름과 구동 수치는 교체 대상이다.

## 확인한 환경

- Ubuntu 24.04, Python 3.12, RTX 5060 Laptop 8 GB
- NVIDIA driver 595.91.07
- Isaac Sim 6.1.0.0: `~/isaacsim6_venv`
- Isaac Lab 소스 커밋 `78b12aed1d2a56439493b8ea6d3538e6e379e2e5`: `~/IsaacLab`
- Lab Python: `~/IsaacLab/.venv/bin/python`
- 물리 백엔드: `isaaclab_physx.physics.PhysxCfg`, 즉 Isaac Sim PhysX

릴리스 이름만으로 현재 main 코드를 재현하지 않는다. 이 소스는 3.0 계열 개발 스냅샷이며 실제 커밋을 고정한다. 설치된 패키지 목록은 `evidence/2026-10-26/environment/`에 있다. 이 목록의 로컬 editable 경로는 각 PC에 맞게 바꿔야 한다.

## 신규 PC 설치 순서

```bash
nvidia-smi
python3 --version
df -h .
python3 -m venv ~/isaacsim6_venv
~/isaacsim6_venv/bin/python -m pip install --upgrade pip
~/isaacsim6_venv/bin/python -m pip install --no-cache-dir \
  'isaacsim[all,extscache]==6.1.0.0' --extra-index-url https://pypi.nvidia.com
```

다운로드·압축해제 임시 공간까지 확보한다. 기존 환경을 지우면서 공간을 만들지 않는다. 공식 uv 설치 절차를 읽고 uv를 준비한다.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
git clone https://github.com/isaac-sim/IsaacLab.git ~/IsaacLab
git -C ~/IsaacLab checkout 78b12aed1d2a56439493b8ea6d3538e6e379e2e5
cd ~/IsaacLab
uv sync --frozen
```

프로젝트 폴더로 돌아온다. 위 `uv sync`는 해당 소스의 lock을 따르는 신규 PC 설치 경로다. 이번 검증은 이미 존재하던 Lab venv와 새로 설치한 Sim venv의 조합으로 수행했고, 빈 PC에서 전체 절차를 다시 실행한 결과는 아니다. 공식 설치 설명은 [Isaac Lab 설치 문서](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/setup/installation/index.html), 세부 버전은 고정한 소스의 `pyproject.toml`·`uv.lock`을 기준으로 한다.

NVIDIA Omniverse 약관에 동의한 뒤:

```bash
export OMNI_KIT_ACCEPT_EULA=YES
PY=~/IsaacLab/.venv/bin/python
$PY scripts/lab_python.py rl/probe_physics.py --visualizer none
$PY scripts/lab_python.py rl/smoke_env.py --visualizer none
```

`PHYSICS_PROBE_COMPLETE`, `ENV_SMOKE_COMPLETE`와 실제 pose 로그를 확인한다. 이 Lab 스냅샷의 CLI는 `--visualizer none`을 사용한다. 다른 버전의 `--headless` 예제를 섞지 않는다.

`lab_python.py`는 Lab의 의존성을 우선 사용하고 별도 Sim 설치의 site-packages를 뒤에 추가한다. 설치 위치를 바꾸면 다음처럼 지정한다.

```bash
export ISAAC_SIM_SITE=/your/sim/venv/lib/python3.12/site-packages
```

## 미로와 모델 생성

```bash
$PY scripts/build_maze.py
$PY scripts/prepare_runtime_asset.py
```

미로 JSON은 제공 ZIP에서 가져왔다. mm를 m로, 바닥 XZ를 XY로 변환한다. 80개 벽·83개 지지대를 만들고, 볼트와 세부 마운트는 지름60mm 지지대로 단순화한다. 두 미로를 USD에 포함하지만 현재 학습 경로는 첫 번째 미로의8.4m 경로다.

원본 로봇 USD는 보존한다. `m38_runtime.usda`는 articulation root를 base_link로 지정하는 레이어다. 이 수정이 없으면 원본에서 팔 링크를 floating root로 선택하는 문제가 있었다.
