# Git 실습 기록

현재 개발 브랜치: `codex/swerve-lecture-foundation`

## 실습 A: 저장소의 출발점

```bash
git status --short
git log --oneline --decorate
git show ff09d94 --stat
```

출발점은 모델 자산과 Ubuntu 준비 문서입니다. 개발 전 상태와 개발 후 기능을 구분합니다.

## 실습 B: 제어 기능 커밋 읽기

```bash
git show 4f0f992 --stat
git show 4f0f992 -- turtlebot_swerve/kinematics.py
git show 4f0f992 -- tests/test_kinematics.py
```

코드와 검증을 같은 변경에서 읽습니다. 왜 앞바퀴는 직진 때 π/2 조향이고 뒤 오른쪽은 회전 부호가 음수인지 모델 축과 연결해 설명합니다.

## 실습 C: 재현 증거

```bash
python3 scripts/record_command.py --output evidence/student-run -- \
  python3 -m unittest discover -s tests -v
cat evidence/student-run/manifest.json
git diff --stat
git status --short
```

로그가 PASS라고만 쓰여 있으면 다른 사람이 재현하기 어렵습니다. 실행 명령, 커밋, 미커밋 변경 상태와 종료 코드를 같이 읽습니다. manifest의 SHA256으로 원본 로그를 확인합니다.

## 실습 D: 작은 변경과 리뷰

```bash
git switch -c practice/document-wheel-axis
# 코드 설명 또는 의미 있는 테스트를 수정한다.
git diff
git add turtlebot_swerve/kinematics.py tests/test_kinematics.py
git diff --cached
python3 -m unittest discover -s tests -v
git commit -m "docs: explain wheel axis convention"
```

Git 작성자 설정이 없다면 자신의 이름과 이메일을 저장소 로컬 설정에 입력합니다. 이번 자동 생성 커밋은 Codex 작성자로 기록했습니다. 기존 사용자 전역 설정은 바꾸지 않았습니다.

```bash
git config user.name "본인 이름"
git config user.email "본인 이메일"
```

## 실습 E: 원격 공유 전 확인

```bash
git remote -v
git diff --check
git status --short
git log --oneline -5
```

원격 push는 아직 수행하지 않았습니다. CAD 모델의 배포 범위와 원격 저장소 접근 권한을 확인한 뒤 공유합니다.
