# 미션 인식 학습 — 실제 표식 / 800·100·100장

최신 데이터는 `runs/perception/data-v11-domain`, 결과는 `runs/perception/train-v11-domain`이다.
카메라 위치·방향·롤, 조명 밝기·색온도·방향, 실제 필라멘트 색상을 랜덤화하고 찌그러진 박스를 포함했다. 800/100/100장, YOLO 40회(batch 8), CenterPose 40회(batch 32) 추가 학습.
같은 새 시험 100장에서 v9→v11 전체 mAP50은 47.45%→96.62%, 박스 AP50은 18.72%→99.44%였다. 변형 박스 52개 중 51개 검출(confidence .25 / IoU .50); 작은 변형별 표본이므로 실물 정확도로 해석하지 않는다.
검출+자세의 유효 추정 중앙값은 30.68→8.70mm, 전체 376개 중 10mm 이내 비율은 10.64%→52.39%다. 변형 박스 52개는 강체 자세 평가에서 제외하며 검출 평가에는 포함한다. 전체 검출 97%와 위치 10mm 이내 97% 목표는 아직 미달이다.
실물 개발 사진 한 장에서 기존에 놓친 박스를 새 모델이 검출했다(IoU 약 .92). 실물 다각도·형상 구별·위치 정확도의 포괄 검증은 완료되지 않았다. 박스 크기 205×205×70mm는 미확정이다.
재현: `evaluate_models.py --data runs/perception/data-v11-domain --run runs/perception/train-v11-domain --split test --imgsz 800 --pose-views scale3_median --confidence .25`.
결과: `comparison-summary.json`, `box-deformation-v11.json`, `real-final/diagnostic.json`, `frozen-before-test.json`. 이전 v9 결과는 [이전 보고서](TRAINING_RESULTS_V9.md)에 보존했다.
아래 기본 학습 예시는 v8 기준이며 현재 설정은 `config/perception/training.json`과 `run_domain_training.sh`에 있다.

이전 data-v5에는 일부 AprilTag 정답 박스가 여러 물체에 걸쳐 합쳐진 오류가 있었다.
이전 [결과](TRAINING_RESULTS_1000.md)의 YOLO 점수는 승인 근거에서 제외한다.
태그를 여러 픽셀 mesh로 만들고 visibility를 변경하던 방식을 단일 텍스처 표면으로
교체했고, prim 경로별 실제 가시 픽셀 마스크로 박스를 계산한다. 각 태그 bbox가
물리 표면의 투영 범위 안에 있는지 생성 시와 데이터 검사 시 모두 확인한다.

## 데이터와 물리 조건

- 학습 800 / 검증 100 / 시험 100장, 새 시드 범위 4050000부터 사용.
- 퍽은 모두 박스 윗면에 완전히 지지되며 서로 겹치지 않는다.
- 박스·퍽 위치와 yaw 무작위. 직육면체 6개 접촉면, 누움·뒤집힘 포함.
- 조명·시점·가림·퍽 없는 장면도 포함한다. 어려운 태그를 점수 때문에 제외하지 않는다.
- 밤부랩 개별 필라멘트 박스는 갈색 단순 대체 모델, 임시 치수 205×205×70mm.
  실제 인쇄·접힘·치수는 미확인이다.
- 실제 배포표 기준 tag36h11. 퍽 ID0..11: 검은 테두리16mm / 흰 여백포함20mm.
  직육면체6면, 원기둥 위·아래·곡면1곳에 같은 물품 ID를 반복한다.
- 장소 ID12..14: 48mm / 60mm, 박스 윗면 중앙. 25% 장면에는 중앙 태그가 없다.
- mission.pdf의 빨강·초록 퍽을 사용한다. 등록표의 R/G/B 이름으로 색을 고정하지 않는다.
- Code128 P2-01..04는 별도 디코더 대상이다. YOLO 태그 검출 AP는 ID/문자열 해독률이 아니다.

## 모델과 평가

YOLOv8n-P2: stride4 작은 물체 검출 단계를 추가한 6클래스 모델.
빨강·초록 × 원기둥·직육면체, 박스, AprilTag를 검출한다.
640px, batch8, mosaic0, scale0.15. 검증 mAP50이 5회 연속97% 이상이고
최소20epoch를 채우면 종료하고, 검증 mAP50이 가장 높은 가중치를 선택한다.

CenterPose: NVIDIA TAO DLA34를 사용하는 커스텀128px crop 학습기, batch32.
동일 ID가 여러 면에 반복되므로 8개 적절한 대칭 회전을 허용하는 모서리 손실을 쓴다.
기존 중심/모서리 head에, 크롭 배율·초점거리·알려진 치수로 정규화한 깊이 head를 학습한다.
위치는 예측 중심 광선과 학습 깊이로 복원하며 회전 후보는 PnP에서 구한다.
`translation_valid`와 `rotation_valid`는 구분한다. 회전 필터 통과는 정확도 승인이 아니다.
TAO CLI/Isaac ROS에 바로 넣는 체크포인트는 아니다.

`evaluate_models.py`는 정답 bbox 입력과 실제 YOLO bbox 입력을 모두 평가한다.
후자는 클래스별 최고 confidence 검출을 사용하고 검출 실패도 10mm 성공률 분모에 넣는다.
객체 치수와 카메라 내부파라미터는 알려져 있다고 가정한다. 모델 고정 SHA256을 저장한 뒤
시험100장을 평가한다. 실물 성능, 전체 미션 완주, 집기 성공률과 구분한다.

## 실행

```bash
# 새 데이터 생성: 기존 디렉터리는 덮어쓰지 않는다.
OMNI_KIT_ACCEPT_EULA=YES /home/seongmin/isaacsim6_venv/bin/python perception/scripts/generate_dataset.py --gui --output runs/perception/new-data --train 800 --val 100 --test 100 --video-frames 0 --seed-base 5050000

# 학습 및 검증만 수행. 시험 데이터로 모델을 선택하지 않는다.
bash perception/scripts/run_training.sh runs/perception/new-data runs/perception/new-run

# 최신 실행에서 사용한 CenterPose 추가학습 설정
# PERCEPTION_POSE_INIT=runs/perception/train-v6-1000/centerpose/best.pt
# PERCEPTION_POSE_EPOCHS=30 PERCEPTION_POSE_LR=.0001

# 모델을 고정한 후 새 시험 데이터 평가
/home/seongmin/Projects/yolov8-ubuntu24-practice/.venv-yolo/bin/python perception/scripts/evaluate_models.py --data runs/perception/new-data --run runs/perception/new-run --split test --imgsz 640

GDK_BACKEND=x11 /usr/bin/python3 perception/scripts/training_monitor.py --data runs/perception/data-v8-clean --run runs/perception/train-v8-clean
```

GPU 사용률은 batch와 실제 데이터 공급·검증·저장 구간에 따라 변동한다.
epoch 수는 학습량과 과적합 기준으로 결정하며 GPU 사용률을 고정하는 장치가 아니다.
마지막 검증·시험 결과, 가중치, 학습 곡선, GPU 측정은 최신 run 디렉터리에 있다.

원본 NVIDIA Apache-2.0 LICENSE와 commit 출처는 `third_party/centerpose`에 보존했다.
인스턴스 마스크의 prim 경로 매핑은 [NVIDIA Replicator API](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.12.27/source/extensions/omni.replicator.core/docs/API.html)의 instance_segmentation 계약을 사용한다.

## 실제 RealSense 확인

`./scripts/run_realsense_perception.sh`로 퍽 식별과 CenterPose를 함께 실행한다. 연결·표시·좌표·실물 확인 범위는 [실시간 실행 안내](REALSENSE_LIVE.md)를 참고한다.

## 실물 빨강 재질 보정

사용자 확인: Bambu Lab PLA Basic 마룬 레드10205. `config/perception/materials.json`에 공식 BambuStudio 색상 #9D2235와 sRGB→linear 변환값을 저장했고, 이후 생성기는 이 값을 사용한다. 확인용8프레임은 `runs/perception/maroon-color-smoke`에 있다. 기존 v9 데이터·가중치는 이전 빨강으로 학습됐으며 이번 설정 수정만으로 갱신되지 않는다. 실제 카메라의 노출·화이트밸런스와 렌더링 조명을 포함한 재학습/실물 재검증이 필요하다.

실물 초록도 사용자 확인된 Bambu Green10501(공식 #00AE42)로 생성기 재질을 수정했다. 기존 v9 가중치는 그대로다. 실시간 점검의15초/28개 샘플에서는 green_cuboid가 모두 검출됐고 confidence0.80–0.89였다. 이는 이 자세의 검출 존재 확인이며 정확도/재현율 평가가 아니다. 같은 실물 이미지의 입력해상도640/800/1280 비교에서는 초록 confidence가0.357/0.847/0.758이어서 현재800을 유지했다. 자료는 `runs/perception/realsense-live/green-check`에 있다.
