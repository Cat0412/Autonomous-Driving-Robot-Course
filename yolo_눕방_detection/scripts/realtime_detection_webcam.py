import argparse
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


PROJECT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_MODEL_PATH = PROJECT_DIR / "models" / "best.pt"


def parse_args():
    parser = argparse.ArgumentParser(description="YOLOv8 Fallen Person Webcam Inference")
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help=f"Path to YOLO model (default: {DEFAULT_MODEL_PATH})",
    )
    parser.add_argument("--camera", type=int, default=0, help="Webcam index. Usually 0 or 1")
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    return parser.parse_args()


def main():
    args = parse_args()

    model_path = args.model.expanduser().resolve()
    if not model_path.exists():
        raise FileNotFoundError(
            f"Model not found: {model_path}\n"
            "Use --model to specify the correct .pt file."
        )

    model = YOLO(str(model_path))

    cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open webcam index {args.camera}. "
            "Try --camera 1 or check webcam permission."
        )

    prev_time = time.time()

    print("Webcam started.")
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to read frame from webcam.")
            break

        results = model.predict(
            source=frame,
            conf=args.conf,
            imgsz=args.imgsz,
            verbose=False
        )

        annotated_frame = frame.copy()
        fallen_detected = False

        result = results[0]

        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = model.names[cls_id]

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                # 모델 클래스가 1개면 사실상 검출된 박스 전부 Fallen Person으로 보면 됨
                if class_name == "Fallen Person" or len(model.names) == 1:
                    fallen_detected = True

                label = f"{class_name} {conf:.2f}"

                cv2.rectangle(
                    annotated_frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    annotated_frame,
                    label,
                    (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        now = time.time()
        fps = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now

        cv2.putText(
            annotated_frame,
            f"FPS: {fps:.1f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2
        )

        if fallen_detected:
            cv2.putText(
                annotated_frame,
                "FALLEN PERSON DETECTED!",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3
            )

            # Windows에서 삐 소리 내고 싶으면 아래 주석 해제
            # import winsound
            # winsound.Beep(1000, 150)

        cv2.imshow("YOLOv8 Fallen Person Detection", annotated_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
