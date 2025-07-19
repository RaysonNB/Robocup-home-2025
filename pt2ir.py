import sys, os

if len(sys.argv) == 1:
    print("Usage: python3 pt2ir.py <model>")
    sys.exit()

from ultralytics import YOLO
name = sys.argv[1]
source = f"{name}_openvino_model"
output = os.path.expanduser(f"~/models/openvino/{source}")

if os.path.exists("output"):
    print(f"Already exists {name}")

model = YOLO(name + ".pt")
model.export(format="openvino", half=False, dynamic=False)
os.makedirs(os.path.dirname(output), exist_ok=True)
os.rename(source, output)
# model = sys.argv[1]
