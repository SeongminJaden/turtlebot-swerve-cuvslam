"""Six-class baseline: color/shape pucks, work boxes, generic AprilTag regions."""
import argparse,json,shutil
from pathlib import Path
from ultralytics import YOLO
p=argparse.ArgumentParser()
p.add_argument('--data',required=True)
p.add_argument('--output',required=True)
p.add_argument('--model',default='/home/seongmin/Projects/yolov8-ubuntu24-practice/yolov8n.pt')
p.add_argument('--lr',type=float,help='Explicit AdamW fine-tuning learning rate')
p.add_argument('--epochs',type=int,default=30)
p.add_argument('--target-map50',type=float,help='Stop after five consecutive validation scores reach this target')
p.add_argument('--min-epochs',type=int,default=20)
p.add_argument('--mosaic',type=float,default=0.)
p.add_argument('--scale',type=float,default=.15)
p.add_argument('--weights',help='Transfer matching pretrained weights into a YAML architecture')
p.add_argument('--imgsz',type=int,default=640)
p.add_argument('--batch',type=int,default=4)
a=p.parse_args()
model=YOLO(a.model)
if a.weights:model.load(a.weights)
best_map50=-1.
recent_map50=[]
def save_map50_checkpoint(trainer):
    global best_map50
    score=float(trainer.metrics.get('metrics/mAP50(B)',0.))
    recent_map50.append(score)
    if a.target_map50 is not None and trainer.epoch+1>=a.min_epochs and len(recent_map50)>=5 and min(recent_map50[-5:])>=a.target_map50:
        trainer.stop=True
    if score>best_map50 and trainer.last.exists():
        best_map50=score
        shutil.copy2(trainer.last,trainer.wdir/'best-map50.pt')
        (trainer.save_dir/'map50-selection.json').write_text(json.dumps({'split':'val','epoch':trainer.epoch+1,'map50':score,'criterion':'highest validation mAP50, no test data'},indent=2))
model.add_callback('on_model_save',save_map50_checkpoint)
model.train(optimizer='AdamW' if a.lr else 'auto',lr0=a.lr or .001,cos_lr=True,warmup_epochs=1.,warmup_bias_lr=0.,data=a.data,epochs=a.epochs,imgsz=a.imgsz,batch=a.batch,device=0,workers=2,seed=3010,deterministic=True,project=a.output,name='yolo',exist_ok=False,amp=True,hsv_h=0.,hsv_s=.15,hsv_v=.25,fliplr=0.,flipud=0.,mosaic=a.mosaic,scale=a.scale,close_mosaic=5,save=True,plots=True)
