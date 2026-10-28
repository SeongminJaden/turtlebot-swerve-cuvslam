# 팀별 미로 강화학습 실습

## 목표와 현재 범위

팀별 URDF/USD를 Isaac Sim PhysX에서 움직이고 Isaac Lab에서 PPO 정책을 학습한다. M38 시연은 정책이 월드 XY 이동 명령을 정하고 해석적 스워브 운동학이 모터 목표를 만든다. 시뮬레이터 위치를 관측하는 알려진 지도 waypoint 과제다. 카메라만으로 미지의 미로를 해결하는 정책이나 cuVSLAM 폐루프 완주를 뜻하지 않는다.

## 학습 순서

1. `docs/ISAAC_LAB_SETUP.md`로 버전과 실행 환경을 확인한다.
2. 자기 로봇의 참조·단위·축·관절·충돌체를 검증한다.
3. 정지 상태와 고정 명령 이동을 실제 PhysX로 확인한다.
4. `MazeEnv`의 USD 경로, 관절 매핑, 구동기와 행동 변환을 자기 구조에 맞춘다.
5. 관측 shape·finite·보상 부호·reset 거리를 한 환경에서 확인한다.
6. 짧은 학습을 실행하고 체크포인트 평가를 한다.
7. 실패 궤적과 실제 관절값을 보고 한 조건씩 수정한다.
8. 같은 시작점과 평가 시간으로 전후를 비교한다.

## 실행 명령

```bash
export OMNI_KIT_ACCEPT_EULA=YES
PY=~/IsaacLab/.venv/bin/python
mkdir -p runs evidence/experiment
$PY scripts/lab_python.py rl/train_ppo.py \
  --iterations 100 --num-envs 8 --seed 7 \
  --output runs/team-experiment-01 --visualizer none \
  > evidence/experiment/train.log 2>&1
$PY scripts/lab_python.py rl/evaluate.py \
  --checkpoint runs/team-experiment-01/policy.pt \
  --output evidence/experiment/eval --steps 3000 --visualizer none \
  > evidence/experiment/eval.log 2>&1
```

재개 시 `--resume <policy.pt>`와 **새로운 출력 폴더**를 지정한다. 신규 체크포인트는 optimizer도 보존한다. 최초100회 파일은 가중치만 저장했으므로 optimizer는 초기화된다. RNG와 모든 환경 상태를 저장하는 비트 단위 재개는 구현하지 않았다.

## 학습 설계와 팁

관측12개는 목표 오차2, 월드 속도2, 방향 sin/cos2, 조향관절4, 이전 행동2다. 액션2개를 축당0.18m/s로 변환한다. diagonal 합성 속도는 더 크므로 제한을 설명할 때 축당 값임을 명시한다.

물리60Hz, 정책15Hz, rollout64step, 병렬8환경이다. 반복당512transition,100회는51,200transition이다. 평균 rollout 보상과 에피소드 return 및 성공률은 서로 다르다. PPO는 작은 Actor/Critic MLP, Adam3e-4, γ=.99, GAE λ=.95, clip=.2, epoch4, minibatch256을 사용한다.

초기 환경은 경로의 전방 목표만 학습했다. 100회 정책은160초 평가에서도 중간에 정체했다. 반대 방향 목표와 시작 위치 오차도 경험하도록 reset을 바꾸는 실험을 추가했다. 총보상 상승만으로 완료라고 판정하지 않는다.

조향 제한 ±90도 근처에서 작은 오차로 목표가 +90/-90을 반복하면 실제 이동이 느려질 수 있다. 0.06rad 범위에서 기존 조향 분기를 유지하는 처리를 추가했다. 이 변경은 작은 방향 오차를 허용하므로 실제 궤적과 속도로 확인한다.

OOM이면 환경 수와 카메라 해상도, 동시 프로세스를 확인한다. 학습 중 렌더링을 끄고 평가 영상만 기록한다. dt를 늘리면 물리와 제어가 바뀌므로 단순한 속도 최적화로 취급하지 않는다. 쉬운 직선 목표부터 미로까지 단계적으로 난도를 높이는 커리큘럼은 확장 과제다.

## 결과와 영상

`report.json`에 성공 여부·종료 이유·마지막 waypoint·목표 오차·정책 해시를 저장한다. `trajectory.csv`는 실제 root pose, `poses.npz`는 body pose와 관절값이다. 실패하면 원인 조사에 사용한다.

```bash
$PY scripts/lab_python.py scripts/render_trajectory.py \
  --recording evidence/experiment/eval --frames 36
```

이 스크립트는 기록된 실제 PhysX body pose를 Isaac Sim으로 재생 렌더링한다. 재생 단계에서 경로를 임의로 생성하지 않는다. GIF는 샘플 프레임을 빠르게 재생하므로 원래 시뮬레이션 시간은 `render/capture.json`에서 확인한다. 직접 실시간 화면 녹화와 구분해 표시한다.

## Git 실험 기록

```bash
git switch -c experiment/team-reward-01
git diff -- rl/maze_env.py
git add rl/maze_env.py docs
git commit -m "experiment: revise waypoint reset distribution"
git rev-parse HEAD
sha256sum runs/team-experiment-01/policy.pt
```

실험마다 코드 커밋, 실행 명령, 모델·미로 해시, seed, 패키지 버전, 체크포인트 해시와 평가 결과를 함께 기록한다. 한 번에 여러 보상과 구동기 값을 바꾸면 원인 추적이 어렵다. 공유한 커밋을 되돌릴 때 `git revert`를 사용한다.
