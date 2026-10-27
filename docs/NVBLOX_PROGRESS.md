# nvblox / Nav2 확장 진행 상태

2026-10-27 작업 스냅샷. 기존 두 지정 미로 경로 검증과 새 Nav2 통합 검증은 구분한다.

- nvblox 5.0 공식 바이너리 GPU 시작 및 깊이 적분 확인.
- Nav2 planner_server + NvbloxCostmapLayer 연결, 1초 주기 재계획.
- map 경로를 5 cm 간격으로 변환하여 기존 24차원 PPO 관측에 연결.
- 경로 2.5초 / nvblox slice 1초 지연 시 주행 차단.
- 첫 통합 GUI 실행: 경로 20회 갱신, 33.735초에 차체 접촉으로 중단. 완주 검증 실패.
- 후속 조정: 반경 0.315 m, inflation 0.42 m, cost scaling 5.0, NavFn Dijkstra, 전방 추종점 0.10 m. 검증 진행 중.
- 스워브 코너 속도 최적화는 미완료. 접촉 없는 경로·추종 검증 이후 속도 비교 필요.
- 전체 Python 테스트 58개 통과. 단위 검사 통과가 폐루프 주행 성공을 의미하지 않음.

## 실행

```bash
python3 scripts/setup_nvblox_runtime.py
GUI=1 USE_NVBLOX=1 DURATION=180 bash scripts/run_predictive_async.sh evidence/nvblox-run
```

기존 로컬 robot_localization 런타임과 Sim/Lab 설치가 필요하다. 최초 RTX 셰이더 컴파일은 100초 이상 소요될 수 있다.

## 후속 교안

기존 54장 교안은 유지한다. 아래 별도 교안은 작성·검증 중이며 최종 결과와 시연 자료는 아직 확정하지 않았다.

https://docs.google.com/presentation/d/10B9z7mA8zPu8sN_ghsi8B1viEOsL84xXxQIRI4-yiZE

기존 교안의 최신 네이티브 정보: teaching/output/google-slides.json
