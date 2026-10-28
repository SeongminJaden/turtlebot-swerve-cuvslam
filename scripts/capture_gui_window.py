"""Capture an explicitly selected lecture app X11 window; never capture the desktop."""
import argparse,ctypes as C,time,subprocess,sys,json
from pathlib import Path
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('--window',type=lambda x:int(x,0),required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=20);p.add_argument('--armed-file',type=Path);p.add_argument('--start-file',type=Path);p.add_argument('--until-file',type=Path);p.add_argument('--fps',type=int,default=10);a=p.parse_args()
x=C.CDLL('libX11.so.6');x.XOpenDisplay.restype=C.c_void_p;d=x.XOpenDisplay(None)
class XI(C.Structure):
 _fields_=[('width',C.c_int),('height',C.c_int),('xoffset',C.c_int),('format',C.c_int),('data',C.c_void_p),('byte_order',C.c_int),('bitmap_unit',C.c_int),('bitmap_bit_order',C.c_int),('bitmap_pad',C.c_int),('depth',C.c_int),('bytes_per_line',C.c_int),('bits_per_pixel',C.c_int),('red_mask',C.c_ulong),('green_mask',C.c_ulong),('blue_mask',C.c_ulong)]
x.XGetGeometry.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong)]+[C.POINTER(C.c_int)]*2+[C.POINTER(C.c_uint)]*4
root=C.c_ulong();xx=C.c_int();yy=C.c_int();w=C.c_uint();h=C.c_uint();border=C.c_uint();depth=C.c_uint();x.XGetGeometry(d,a.window,C.byref(root),C.byref(xx),C.byref(yy),C.byref(w),C.byref(h),C.byref(border),C.byref(depth))
x.XGetImage.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int];x.XGetImage.restype=C.POINTER(XI);x.XDestroyImage.argtypes=[C.POINTER(XI)]
def grab():
 q=x.XGetImage(d,a.window,0,0,w.value,h.value,C.c_ulong(-1).value,2)
 if not q:raise RuntimeError('Cannot capture selected window')
 z=q.contents
 if z.bits_per_pixel!=32:raise RuntimeError(f'Unsupported pixel format {z.bits_per_pixel}')
 im=Image.frombytes('RGB',(z.width,z.height),C.string_at(z.data,z.bytes_per_line*z.height),'raw','BGRX',z.bytes_per_line,1);x.XDestroyImage(q);return im
if a.armed_file:
 a.armed_file.parent.mkdir(parents=True,exist_ok=True);a.armed_file.write_text('ready')
if a.start_file:
 deadline=time.monotonic()+120.
 while not a.start_file.exists():
  if time.monotonic()>deadline:raise RuntimeError('Capture start marker timed out')
  time.sleep(.05)
start_stamp=json.loads(a.start_file.read_text()).get('stamp_ns') if a.start_file else None
im=grab();a.output.parent.mkdir(parents=True,exist_ok=True);im.save(a.output.with_suffix('.png'))
if a.seconds>0:
 import imageio_ffmpeg
 cmd=[imageio_ffmpeg.get_ffmpeg_exe(),'-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w.value}x{h.value}','-r',str(a.fps),'-i','-','-an','-vf','pad=ceil(iw/2)*2:ceil(ih/2)*2','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output.with_suffix('.mp4'))]
 proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=subprocess.DEVNULL);begin=time.monotonic()
 for i in range(int(a.seconds*a.fps)):
  proc.stdin.write(grab().tobytes())
  if a.until_file and a.until_file.exists():break
  time.sleep(max(0,begin+(i+1)/a.fps-time.monotonic()))
 proc.stdin.close();proc.wait();assert proc.returncode==0
 a.output.with_suffix('.json').write_text(json.dumps({'capture':'actual X11 app window pixels','start_sim_stamp_ns':start_stamp,'window_id':hex(a.window),'fps':a.fps,'seconds':(i+1)/a.fps,'wall_seconds':time.monotonic()-begin,'size':[w.value,h.value]},indent=2))
print('GUI_CAPTURE_COMPLETE',a.output,flush=True)
