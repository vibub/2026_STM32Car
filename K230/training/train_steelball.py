from ultralytics import YOLO

DATA = "K230/training/steelball.yaml"
BASE_MODEL = "yolo11n.pt"


def main():
    model = YOLO(BASE_MODEL)
    model.train(
        data=DATA,
        imgsz=320,
        epochs=120,
        batch=32,
        device=0,
        workers=4,
        project="K230/training/runs",
        name="steelball_yolo11n_320",
        patience=30,
        cache=True,
    )
    best = "K230/training/runs/steelball_yolo11n_320/weights/best.pt"
    YOLO(best).export(format="onnx", imgsz=320, opset=11, simplify=True)


if __name__ == "__main__":
    main()
