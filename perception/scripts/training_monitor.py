"""Read-only desktop GUI for the live dataset and two training runs."""
import argparse,csv,json,time
from PIL import Image,ImageDraw
from pathlib import Path
import gi
gi.require_version('Gtk','3.0')
from gi.repository import Gtk,GLib,GdkPixbuf
p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);p.add_argument('--run',type=Path,required=True);a=p.parse_args()
window=Gtk.Window(title='Mission perception — YOLO / CenterPose training')
window.set_default_size(1050,730)
window.connect('destroy',Gtk.main_quit)
outer=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=10);outer.set_border_width(16);window.add(outer)
title=Gtk.Label(label='800 train · 100 validation · 100 test | actual mission AprilTags');outer.pack_start(title,False,False,0)
status=Gtk.Label(label='Reading training status…');outer.pack_start(status,False,False,0)
progress=Gtk.ProgressBar();progress.set_show_text(True);outer.pack_start(progress,False,False,0)
row=Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL,spacing=12);outer.pack_start(row,True,True,0)
image=Gtk.Image();row.pack_start(image,True,True,0)
right=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=12);row.pack_start(right,True,True,0)
metrics=Gtk.Label();metrics.set_xalign(0);right.pack_start(metrics,False,False,0)
plot=Gtk.Image();right.pack_start(plot,True,True,0)
footer=Gtk.Label(label='Synthetic baseline only · Closing this monitor does not stop training');outer.pack_start(footer,False,False,0)
series=[];last_image=None

def draw():
    im=Image.new('RGB',(390,300),(25,30,40));pen=ImageDraw.Draw(im)
    pen.text((15,15),'Validation metrics (independent scales)',fill='white')
    for index,(name,values) in enumerate(series):
        if not values:continue
        lo=min(values);span=max(max(values)-lo,.00001)
        color=(50,210,165) if index==0 else (245,170,60)
        pts=[(25+340*i/max(19,len(values)-1),250-190*(v-lo)/span) for i,v in enumerate(values)]
        if len(pts)>1:pen.line(pts,fill=color,width=3)
        pen.text((15,265+index*16),f'{name}: {values[-1]:.4f}',fill=color)
    pix=GdkPixbuf.Pixbuf.new_from_bytes(GLib.Bytes.new(im.tobytes()),GdkPixbuf.Colorspace.RGB,False,8,390,300,390*3)
    plot.set_from_pixbuf(pix)

def refresh():
    global series,last_image
    try:
        state=(a.run/'status.txt').read_text().strip() if (a.run/'status.txt').exists() else 'GENERATING'
        counts={s:len(list((a.data/'images'/s).glob('*.png'))) for s in ['train','val','test']}
        status.set_text(f'{state} | images: train {counts["train"]}/800, val {counts["val"]}/100, test {counts["test"]}/100')
        total=sum(counts.values());progress.set_fraction(min(1,total/1000));progress.set_text(f'Dataset {total}/1000')
        lines=['Tag family: tag36h11','Puck: ID 0–11, 16 mm','Box center: ID 12–14, 48 mm','Pucks fully supported on box top','Cuboids: six resting faces / random yaw',''];series=[]
        path=a.run/'yolo/results.csv'
        if path.exists():
            with path.open() as f:rows=list(csv.DictReader(f))
            if rows:
                values=[float(r['metrics/mAP50(B)']) for r in rows];series.append(('YOLO mAP50',values));lines += [f'YOLO epoch {rows[-1]["epoch"]}',f'validation mAP50: {values[-1]:.4f}','']
        path=a.run/'centerpose/history.json'
        if path.exists():
            rows=json.loads(path.read_text())
            if rows:
                values=[r['val_loss'] for r in rows];series.append(('CenterPose val loss',values));lines += [f'CenterPose epoch {rows[-1]["epoch"]}',f'validation loss: {values[-1]:.4f}']
        evaluation=a.run/'test-evaluation.json'
        if evaluation.exists():
            test=json.loads(evaluation.read_text());pose=test['centerpose']
            lines += ['',f'TEST YOLO mAP50: {test["yolo"]["metrics/mAP50(B)"]:.4f}',f'POSE oracle median: {pose["translation_error_mm_median_valid_only"]:.2f} mm','CenterPose NOT approved for control']
        telemetry=a.run/'gpu-telemetry.csv'
        if telemetry.exists():
            with telemetry.open() as stream:gpu_rows=list(csv.DictReader(stream))[-60:]
            usage=[float(r[' utilization.gpu [%]'].strip().split()[0]) for r in gpu_rows if r.get(' utilization.gpu [%]')]
            if usage:lines += ['',f'GPU last recorded: {usage[-1]:.0f}% | last minute: {sum(usage)/len(usage):.1f}%']
        if evaluation.exists():
            pipeline=test.get('end_to_end',{})
            if pipeline.get('median_mm_valid_only') is not None:
                lines += [f'YOLO + pose median: {pipeline["median_mm_valid_only"]:.2f} mm',f'Within 10 mm (all): {100*pipeline["all_objects_within_10mm"]:.1f}%']
        metrics.set_text('\n'.join(lines));draw()
        paths=sorted((a.data/'images').glob('*/*.png'),key=lambda p:p.stat().st_mtime)
        current=paths[(int(time.monotonic())//3)%len(paths)] if paths and (a.data/'COMPLETE').exists() else (paths[-1] if paths else None)
        if current and current!=last_image:
            pix=GdkPixbuf.Pixbuf.new_from_file_at_scale(str(current),540,540,True);image.set_from_pixbuf(pix);last_image=current
    except (OSError,ValueError,KeyError,GLib.Error):pass
    return True
window.show_all();refresh();GLib.timeout_add(1500,refresh);Gtk.main()
