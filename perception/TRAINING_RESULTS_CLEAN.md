# 수정된 합성 데이터 학습 결과 — 2026-10-30

**독립 시험100장에서 전체 YOLO mAP50 97% 목표를 달성했다.**
위치 오차는 실제 YOLO 검출 박스를 입력해 평가했다.

## 데이터 오류 수정과 시험 절차

기존 data-v5의 일부 AprilTag 정답 bbox가 서로 다른 표식에 걸쳐 합쳐지는 오류를 발견했다.
따라서 이전 94.8% 결과는 성능 승인 근거에서 제외한다. 픽셀별 mesh visibility 변경을
단일 텍스처 표면으로 바꾸고, prim별 가시 픽셀 마스크에서 정답 bbox를 생성했다.
모든 태그 bbox8,918개가 실제 물리 표면의 투영 범위 안에 있는지 검사했다.

새 데이터는 **학습800 / 검증100 / 시험100장**, seed4050000부터이며 이전 데이터와 분리했다.
모든 퍽의 박스 위 완전 지지·비겹침, 누운 직육면체 포함6접촉면, 위치·회전 무작위를 유지했다.
실제 tag36h11 ID·16/48mm 규격을 반영했다. 박스는 여전히 밤부랩 상자의 갈색 단순 대체 모델이다.

체크포인트 선택에는 검증만 사용했다. 시험 전에 가중치·데이터 SHA256을 저장했고,
시험과 GPU 비교 후에도 해시가 동일함을 확인했다. 시험 결과에 맞춘 추가 모델 변경은 없다.

## YOLO 시험 결과

| 지표 | 결과 |
|---|---:|
| 전체 mAP50 | 97.93% |
| mAP50-95 | 91.50% |
| Precision | 99.24% |
| Recall | 97.36% |

| 클래스 | AP50 |
|---|---:|
| red_cylinder | 98.12% |
| red_cuboid | 99.50% |
| green_cylinder | 98.59% |
| green_cuboid | 96.74% |
| box | 99.50% |
| apriltag | 95.15% |

97% 목표는 **6클래스 전체 mAP50** 기준이다. 각 클래스가 모두97% 이상이라는 뜻은 아니다.
AprilTag AP는 표식 영역 검출 성능이며 ID 해독률이나 바코드 문자열 인식률이 아니다.

## 위치 오차

같은 ID의 반복 부착과 형상 대칭을 고려하는 모서리 손실을 적용했다.
DLA34에서 크롭 배율·초점거리·알려진 치수로 정규화한 깊이를 추가 학습하고,
예측 중심 광선과 깊이로 카메라 좌표계 중심 위치를 복원한다. 회전 후보는 PnP이다.

| 시험 조건 | 위치 오차 중앙값 | 90백분위 |
|---|---:|---:|
| 정답 bbox 입력 | 2.81mm | 7.19mm |
| 실제 YOLO bbox 입력 | 2.68mm | 6.97mm |

시험 대상 457개 중 위치 추정 유효 451개.
위 중앙값·90백분위는 유효 추정 기준이다. 검출 실패·무효 추정도 실패로 포함한
**전체 객체10mm 이내 비율은 93.65%**다.

위치 정확도 개선을 확인했지만 회전 정확도·집기 성공률은 이 결과로 승인하지 않는다.
대칭을 무시한 ordered-corner 오차는 120.0px로,
대칭 등가 자세의 정확도와 동일한 지표가 아니다. 카메라 보정값과 물체 치수를 안다는
가정이며 실제 상자 치수·인쇄, 실물, 전체 미로와 팔 제어까지 검증한 결과가 아니다.

## 학습 설정과 GPU

- YOLOv8n-P2: 640px, batch8, 20epoch. 최대60으로 시작했으나 검증이5회 연속97% 이상인 조건에서 종료.
- CenterPose: batch32, 개선 모델을 초기값으로 새 데이터에서30epoch 추가학습. 선택된 epoch는 26.
- 이전 데이터에서 대칭·깊이 head를80epoch 학습한 초기 가중치를 재사용했다.
- 관련 기하·배치·미션 단위검사22개 통과. 전체 unittest 수집에서는 별도5개 pytest 기반 모듈이
  이 학습 venv의 pytest 부재로 import되지 않았다. 전체 테스트 통과로 보고하지 않는다.

| 측정 구간 | 평균 GPU 사용률 |
|---|---:|
| YOLO 학습 실행 전체 | 90.8% |
| CenterPose 학습 실행 전체 | 81.2% |
| CenterPose 실제 미니배치 연산 비교(batch32/worker0) | 97.5% |

전체 구간에는 초기화·검증·체크포인트 저장이 포함된다. 90%로 고정된 사용률은 아니다.
추가 비교에서는 batch32/worker0이 초당 470객체로 가장 빨랐고,
batch64/worker2는 초당 423객체여서 batch32를 유지했다.
비교 중 학습된 임시 가중치는 저장하지 않았다. 이후 실행의 last 체크포인트 저장 간격은
5epoch로 설정했고, best는 검증 개선 시마다 저장한다. 시험을 통과한 가중치는 변경하지 않았다.

## 파일

- 데이터: `runs/perception/data-v8-clean`
- YOLO: `runs/perception/train-v8-clean/yolo/weights/best.pt`
- CenterPose: `runs/perception/train-v8-clean/centerpose/best.pt`
- 시험: `runs/perception/train-v8-clean/test-evaluation.json`
- 시험 전 고정 기록: `runs/perception/train-v8-clean/frozen-before-test.json`
- 데이터 품질: `runs/perception/data-v8-clean/validation.json`
- GPU: `runs/perception/train-v8-clean/gpu-summary.json`, `gpu-benchmark.json`
- 단위검사: `runs/perception/train-v8-clean/targeted-tests.log`

재실행법과 모델 형식은 [README](README.md)를 참고한다.
