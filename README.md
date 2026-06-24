# Autonomous-Driving-Robot-Course
2026 광운대 자율주행 로봇 전문가 과정
## Fallen Person Detection Model

본 프로젝트에서는 스토리지 로봇의 카메라를 이용해 바닥에 누워 있는 사람을 탐지하기 위해 YOLOv8 기반 객체 탐지 모델을 학습하였다. 탐지 대상 클래스는 `Fallen Person` 1개로 설정하였으며, 로봇이 누워 있는 사람을 인식한 뒤 해당 위치로 접근하고 정지 및 알림을 수행하는 기능 구현을 목표로 한다.

### Dataset

기본 학습 데이터셋은 Roboflow Universe의 People Falls Dataset v1을 사용하였다.

* Dataset source: https://universe.roboflow.com/project/people-falls/dataset/1
* Task: Object Detection
* Format: YOLOv8
* Class: `Fallen Person`
* Train dataset: 547장
* Validation dataset: 154장
* Test dataset: 75장

또한 실제 프로젝트 환경과 유사한 조건을 반영하기 위해, 직접 Storagy로봇 웹캠을 통해 촬영한 누워 있는 사람 이미지 8장을 추가로 라벨링하여 train 데이터셋에 포함하였다. 추가 데이터는 Roboflow에서 기존 데이터셋과 동일하게 YOLOv8 bounding box 형식으로 라벨링하였으며, 클래스명은 기존 `data.yaml`과 동일하게 `Fallen Person`으로 통일하였다.

최종 데이터셋 구조는 다음과 같다.

```text
people_falls_yolov8/
├── train/
│   ├── images/
│   └── labels/
├── valid/
│   ├── images/
│   └── labels/
├── test/
│   ├── images/
│   └── labels/
└── data.yaml
```

### Model Training

학습에는 YOLOv8 계열 중 가장 가벼운 nano 모델인 `YOLOv8n`을 사용하였다. 스토리지 로봇의 실시간 인식 환경을 고려하여, 정확도보다 추론 속도와 경량성을 우선적으로 고려하였다.

Colab 환경에서 다음과 같이 학습을 진행하였다.

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.train(
    data="/content/people_falls_yolov8/data.yaml",
    epochs=80,
    imgsz=640,
    batch=16,
    name="fallen_person_yolov8n"
)
```

학습 완료 후 생성된 최종 모델 파일은 다음 경로의 `best.pt`를 사용하였다.

```text
runs/detect/fallen_person_yolov8n/weights/best.pt
```

### Training Result

학습 결과는 다음과 같다.

| Metric    | Result |
| --------- | -----: |
| Precision | `0.98657` |
| Recall    | ` 0.95371` |
| mAP50     | `0.97234` |
| mAP50-95  | `0.92417` |

검증 이미지에 대한 예측 결과를 확인한 결과, 모델은 바닥에 누워 있는 사람을 `Fallen Person` 클래스로 탐지하는 것을 확인하였다. 추가로 직접 촬영한 이미지 8장을 train 데이터에 포함하여, 실제 실험 환경에서의 누운 사람 자세와 카메라 각도에 대한 적응성을 높이고자 하였다.

### Real-Time Inference Plan

학습된 `best.pt` 모델은 이후 웹캠 또는 스토리지 로봇의 RGB 카메라 입력에 적용할 예정이다. 실시간 추론에서는 YOLOv8n 모델을 사용하여 프레임 단위로 `Fallen Person` 객체를 탐지하고, 탐지된 bounding box의 중심 좌표와 depth 정보를 결합하여 로봇이 대상 위치로 접근하도록 확장할 계획이다.
