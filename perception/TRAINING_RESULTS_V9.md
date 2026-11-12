# 인식 정확도 추가 개선 — 2026-10-30

추가 학습과 독립 시험을 완료했다. **기존 모델보다 개선됐지만 시험 mAP50 99%, 전체 객체의 위치 오차 10mm 이내 97% 목표는 달성하지 못했다.**
검증 점수와 시험 점수를 구분하며 시험 결과를 이용한 재선택은 하지 않았다.

## 같은 시험 데이터에서 비교

학습800 / 검증100은 data-v8-clean을 유지했다. 시험100장만 seed5050000–5050099로 새로 생성했다.
기존 시험과 학습·검증의 seed 중복이 없고, 데이터 검사를 통과했다.
가중치·추론 설정·소스 해시를 시험 생성 전에 고정했고, 평가 후 동일함을 확인했다.

| 독립 시험 지표 | 기존 v8 | 개선 v9 |
|---|---:|---:|
| 전체 YOLO mAP50 | 97.95% | **98.42%** |
| mAP50–95 | 91.17% | **95.85%** |
| YOLO→위치 오차 중앙값, 유효 추정만 | 2.54mm | **2.29mm** |
| 위치 오차 90백분위, 유효 추정만 | 6.62mm | **6.10mm** |
| 전체 객체 중 10mm 이내 | 93.90% (431/459) | **94.99% (436/459)** |
| 유효 위치 추정 | 451/459 | 452/459 |
| 검출+위치 추정 평균 시간/프레임 | 24.83ms | 35.54ms |

시간은 이미지 읽기와 정답 bbox 비교를 제외한 측정이며, 실시간 로봇 시스템 전체 지연이 아니다.
개선 모델의 실패23개는 검출 누락7개와 위치 오차10mm 초과16개다.
검출 mAP50은 0.47%p, 엄격한 박스 지표 mAP50–95는 4.68%p 개선됐다.
위치 중앙값은 약9.6% 감소했지만 큰 오차와 누락이 남는다.

| 개선 모델 클래스 | 시험 AP50 |
|---|---:|
| red_cylinder | 98.79% |
| red_cuboid | 97.51% |
| green_cylinder | 98.78% |
| green_cuboid | 98.71% |
| box | 99.50% |
| apriltag | 97.21% |

AprilTag AP는 영역 검출 지표이며 ID 해독률이나 바코드 문자열 인식률이 아니다.

## 선택 과정과 설정

- YOLOv8n-P2: v8에서 50epoch 추가학습, 800px, batch4, AdamW 초기학습률0.00015와 cosine 감쇠. 검증 mAP50 기준36회차 선택.
- CenterPose DLA34: v8에서 25epoch 추가학습, batch32, worker0, 초기학습률0.00002와 cosine 감쇠. 검증 손실 기준24회차 선택 후 실제 위치 지표로 비교.
- 위치 추정: bbox 최대 변의1.35/1.5/1.65배 크롭을 GPU 배치로 처리하고, 유효한 카메라 좌표계 위치의 성분별 중앙값을 사용한다. 회전·모서리는 기본1.5배 크롭 결과를 유지한다.
- 검출 confidence0.25와 위치 confidence0.25를 유지했다. 검출 confidence0.1은 검증 누락을 개선하지 못했다.
- 최종 검증: mAP50 **99.01%**, 위치 중앙값 **2.34mm**, 전체10mm 이내 **97.83%**. 이 점수를 독립 시험 성능으로 사용하지 않는다.
- 검증에서 비교한 크롭 조합·추가학습 후보는 run 디렉터리에 보존했다. 최종 시험은 모델/설정 고정 후 한 번 수행했다.
- 관련 위치 기하·무효 처리·중앙값·배치 검사 단위테스트 **10개 통과**.

훈련 초반에는 다른 시뮬레이션이 GPU 메모리 약4.4GB를 사용해 YOLO batch4를 선택했다.
관측 시점의 GPU 사용률은93–95%였지만90% 고정 사용률은 아니다.
기록한 전체1065초 구간 평균은75.9%이며, 일부 YOLO학습·위치학습·검증·데이터생성·시험·대기와 다른 프로세스 부하를 포함한다.
이 평균을 학습 구간만의 GPU 성능으로 해석하면 안 된다. epoch는 사용률을 높이기 위해 늘리지 않고 검증으로 선택했다.

## 범위와 남은 한계

모든 퍽은 박스 위에 완전히 지지되며 직육면체의 누운 자세와6접촉면, 위치·회전 무작위를 유지한다.
실제 tag36h11 ID 및16mm/48mm 규격도 유지했다.
밤부랩 박스는 갈색 단순 대체 형상이며 실제 인쇄와 치수는 확인되지 않았다.
카메라 보정값과 물체 치수를 아는 합성 환경의 평가다. 완전히 가려져 정답 bbox가 없는 물체는 평가 대상이 아니다.
검출 실패는 성공률 분모에서 제외하지 않았다. 회전·실물 집기·전체 미션 완주는 검증하지 않았다.

이번 시험은 이미 공개된 데이터이므로 이후 모델을 더 개선하면 새로운 독립 시험이 필요하다.
99%/97% 달성으로 승인하지 않으며, 현재 v9를 개선된 합성 기준 모델로 보존한다.

## 재현과 결과 파일

```bash
/home/seongmin/Projects/yolov8-ubuntu24-practice/.venv-yolo/bin/python perception/scripts/evaluate_models.py \
  --data runs/perception/data-v9-accuracy --run runs/perception/train-v9-accuracy \
  --split test --imgsz 800 --pose-views scale3_median --confidence .25
```

- 모델: `runs/perception/train-v9-accuracy/yolo/weights/best.pt`, `centerpose/best.pt`
- 최종 설정: `runs/perception/train-v9-accuracy/inference.json`
- 시험/비교: `test-evaluation.json`, `comparison.json`
- 고정 기록: `frozen-before-test.json`, `test-data-before-evaluation.json`
- 기존 모델 동일 시험: `runs/perception/baseline-v8-on-v9-test/test-evaluation.json`
- 데이터: `runs/perception/data-v9-accuracy/manifest.json`, `validation.json`
- GUI: `training_monitor.py`가 v9 결과를 표시한다. 닫아도 저장된 결과는 유지된다.

이전 독립 시험 결과는 [v8 보고서](TRAINING_RESULTS_CLEAN.md)에 보존했다.
