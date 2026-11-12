"""Official TAO CenterPose DLA34 architecture with a custom crop training harness.

Predicts geometric center heatmap and ordered 3D cuboid corner projections.
Metric pose comes from predicted corners + calibrated PnP and known dimensions.
This checkpoint is not a drop-in Isaac ROS/TAO exported model.
"""
import argparse
import json
from pathlib import Path
import random
import sys
from types import SimpleNamespace
import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from third_party.centerpose.centerpose import CenterPose_DLA34

SIZE=128
GRID=32
CORNERS=np.array([[x,y,z] for x in [-.015,.015] for y in [-.015,.015] for z in [-.03,.03]],np.float64)

def make_model():
    return CenterPose_DLA34(SimpleNamespace(down_ratio=4,final_kernel=1,last_level=5,head_conv=64,out_channel=0,use_convGRU=False,use_pretrained=False))

def crop_image(image, box, jitter=None, crop_scale=1.5, shift=(0.,0.)):
    x1,y1,x2,y2=box
    side=max(x2-x1,y2-y1)*crop_scale
    cx,cy=(x1+x2)/2,(y1+y2)/2
    cx+=shift[0]*side;cy+=shift[1]*side
    if jitter is not None:
        cx+=jitter.uniform(-.05,.05)*side;cy+=jitter.uniform(-.05,.05)*side
        side*=jitter.uniform(.9,1.1)
    matrix=np.array([[SIZE/side,0,SIZE/2-cx*SIZE/side],[0,SIZE/side,SIZE/2-cy*SIZE/side]],np.float32)
    crop=cv2.warpAffine(image,matrix,(SIZE,SIZE),borderValue=(114,114,114))
    tensor=torch.from_numpy(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB).transpose(2,0,1).copy()).float()/255
    return tensor,matrix

class Samples(Dataset):
    def __init__(self,manifest,split):
        self.root=manifest.parent;self.split=split;self.items=[]
        data=json.loads(manifest.read_text())
        self.images={}
        self.fx=data['camera_intrinsics']['fx']
        for sample in data['samples']:
            if sample['split']!=split:continue
            for pose in sample['poses']:
                if not pose.get('pose_supervision_valid',True):continue
                boxes=[b for b in sample['boxes'] if b['class_id']==pose['class_id']]
                if boxes:self.items.append((sample['image'],pose,boxes[0]['xyxy']))
        for image,_,_ in self.items:
            if image not in self.images:self.images[image]=cv2.imread(str(self.root/image))
    def __len__(self):return len(self.items)
    def __getitem__(self,index):
        image,pose,box=self.items[index]
        x,m=crop_image(self.images[image],box,random if self.split=='train' else None)
        pts=np.array(pose['projected_cuboid'],np.float32)
        pts=(pts@m[:,:2].T+m[:,2])/4
        ct=pts[0]; ij=np.clip(ct.astype(int),0,GRID-1)
        yy,xx=np.mgrid[:GRID,:GRID]
        heat=np.exp(-((xx-ij[0])**2+(yy-ij[1])**2)/2).astype(np.float32)[None]
        hps=(pts[1:]-ij).reshape(-1).astype(np.float32)
        dimension=max(pose.get('dimensions_m',[.03,.03,.06]))
        depth=np.log(pose['center_camera_m'][2]*SIZE/(m[0,0]*self.fx*dimension))
        return x,torch.from_numpy(heat),torch.tensor(ij),torch.from_numpy(hps),torch.tensor(ct-ij,dtype=torch.float32),torch.tensor(depth,dtype=torch.float32)

# Proper rotations of a square prism: preserve long axis, allow its reversal.
# Same object IDs on every face do not establish an ordered corner identity.
def symmetry_permutations():
    points=CORNERS/np.array([.015,.015,.03])
    permutations=[]
    for k in range(4):
        angle=k*np.pi/2
        rz=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        for flip in (np.eye(3),np.diag([1,-1,-1])):
            rotated=points@(rz@flip).T
            permutations.append(np.argmin(np.linalg.norm(rotated[:,None]-points[None],axis=2),axis=1))
    return np.array(permutations)

SYMMETRIES=symmetry_permutations()

def loss_for(out,heat,ij,hps,reg,depth):
    prob=out['hm'].sigmoid().clamp(1e-5,1-1e-5)
    pos=heat.eq(1);neg=heat.lt(1)
    hm=-(torch.log(prob)*(1-prob)**2*pos+torch.log(1-prob)*prob**2*(1-heat)**4*neg).sum()/max(1,len(ij))
    idx=torch.arange(len(ij),device=ij.device)
    pred=out['hps'][idx,:,ij[:,1],ij[:,0]]
    offset=out['reg'][idx,:,ij[:,1],ij[:,0]]
    permutations=torch.as_tensor(SYMMETRIES,device=hps.device)
    targets=hps.reshape(-1,8,2)[:,permutations].reshape(-1,8,16)
    errors=nn.functional.smooth_l1_loss(pred[:,None,:].expand_as(targets),targets,reduction='none').mean(-1)
    corner_loss=errors.min(dim=1).values.mean()
    depth_loss=nn.functional.smooth_l1_loss(out['scale'][idx,0,ij[:,1],ij[:,0]],depth,beta=.05)
    return hm+corner_loss+nn.functional.l1_loss(offset,reg)+5*depth_loss

def load_model(path,device='cuda'):
    ck=torch.load(path,map_location=device,weights_only=False)
    model=make_model().to(device);model.load_state_dict(ck['model']);model.eval();model.metric_depth_head=bool(ck.get('metric_depth_head',False));model.inference_tta=bool(ck.get('inference_tta',False))
    if 'fallback_model' in ck:
        fallback=make_model().to(device)
        fallback.load_state_dict(ck['fallback_model']);fallback.eval()
        # Inference-only model must not become part of trainable state_dict.
        object.__setattr__(model,'pose_fallback',fallback)
    return model

@torch.inference_mode()
def infer_single(model,image,box,K,device='cuda',dimensions_m=(.03,.03,.06),crop_scale=1.5,shift=(0.,0.)):
    x,m=crop_image(image,box,crop_scale=crop_scale,shift=shift)
    out=model(x[None].to(device))[0]
    return decode_prediction(model,out,m,K,dimensions_m)

def decode_prediction(model,out,m,K,dimensions_m):
    heat=out['hm'][0,0].sigmoid();flat=int(heat.argmax());y,x=divmod(flat,GRID)
    score=float(heat[y,x]);ij=np.array([x,y],np.float32)
    ct=(ij+out['reg'][0,:,y,x].cpu().numpy())*4
    corners=(out['hps'][0,:,y,x].cpu().numpy().reshape(8,2)+ij)*4
    inv=cv2.invertAffineTransform(m)
    pixels=corners@inv[:,:2].T+inv[:,2];center_px=ct@inv[:,:2].T+inv[:,2]
    object_corners=CORNERS*np.asarray(dimensions_m)/np.array([.03,.03,.06])
    ok,rvec,tvec=cv2.solvePnP(object_corners,pixels.astype(np.float64),K,np.zeros(5),flags=cv2.SOLVEPNP_EPNP)
    result=dict(source='centerpose_dla34_custom_crop',confidence=score,center_px=center_px.tolist(),corners_px=pixels.tolist(),valid=False)
    if ok:
        rvec,tvec=cv2.solvePnPRefineLM(object_corners,pixels.astype(np.float64),K,np.zeros(5),rvec,tvec)
        reproj,_=cv2.projectPoints(object_corners,rvec,tvec,K,np.zeros(5))
        error=float(np.linalg.norm(reproj.reshape(-1,2)-pixels,axis=1).mean())
        valid=bool(tvec[2,0]>0 and np.isfinite(tvec).all() and error<10 and score>=.25)
        result.update(valid=valid,center_camera_m=tvec.reshape(3).tolist(),rotation_matrix=cv2.Rodrigues(rvec)[0].tolist(),reprojection_error_px=error)
    if getattr(model,'metric_depth_head',False):
        log_depth=float(out['scale'][0,0,y,x])
        depth=float(np.exp(log_depth)*K[0,0]*max(dimensions_m)*m[0,0]/SIZE)
        ray=np.linalg.solve(K,np.r_[center_px,1.])
        result['pnp_valid']=result['valid']
        result['pnp_center_camera_m']=result.get('center_camera_m')
        result['center_camera_m']=(ray*depth).tolist()
        result['translation_source']='learned scale-normalized depth and center ray'
        result['translation_valid']=bool(np.isfinite(depth) and depth>0 and score>=.25)
        result['rotation_valid']=result['pnp_valid']
        result['valid']=result['translation_valid'] and result['rotation_valid']
    result.setdefault('translation_valid',result['valid'])
    result.setdefault('rotation_valid',result['valid'])
    return result

@torch.inference_mode()
def infer_crop_views(model,image,box,K,device,dimensions_m,mode='all7_mean'):
    views=[(1.5,(0,0)),(1.35,(0,0)),(1.65,(0,0)),(1.5,(-.04,0)),
           (1.5,(.04,0)),(1.5,(0,-.04)),(1.5,(0,.04))]
    if mode=='scale3_median':views=views[:3]
    crops=[crop_image(image,box,crop_scale=scale,shift=shift) for scale,shift in views]
    outputs=model(torch.stack([x for x,_ in crops]).to(device))[0]
    return [decode_prediction(model,{key:value[j:j+1] for key,value in outputs.items()},matrix,K,dimensions_m)
            for j,(_,matrix) in enumerate(crops)]

@torch.inference_mode()
def infer(model,image,box,K,device='cuda',dimensions_m=(.03,.03,.06),views='single'):
    if views not in ('single','all7_mean','scale3_median'):
        raise ValueError(f'Unknown pose inference views: {views}')
    if views=='single':return infer_single(model,image,box,K,device,dimensions_m)
    estimates=infer_crop_views(model,image,box,K,device,dimensions_m,mode=views)
    result=estimates[0]
    points=[e['center_camera_m'] for e in estimates if e['translation_valid']
            and np.isfinite(e['center_camera_m']).all()]
    result['translation_valid']=bool(points)
    result['translation_views_used']=len(points)
    result['translation_source']=views+' of valid metric translations'
    if points:result['center_camera_m']=(np.median if views=='scale3_median' else np.mean)(points,axis=0).tolist()
    # Rotation/corners retain the primary view: symmetric corner orders cannot be averaged.
    result['valid']=result['translation_valid'] and result['rotation_valid']
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume',type=Path);p.add_argument('--epochs',type=int,default=40);p.add_argument('--batch',type=int,default=16)
    p.add_argument('--lr',type=float,default=.0005)
    p.add_argument('--workers',type=int,default=0)
    p.add_argument('--checkpoint-every',type=int,default=5)
    p.add_argument('--cosine',action='store_true',help='Decay learning rate to 10 percent over the requested epochs')
    a=p.parse_args()
    if a.checkpoint_every<1 or a.workers<0:p.error('checkpoint-every must be positive and workers nonnegative')
    a.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(4);torch.manual_seed(317);np.random.seed(317);random.seed(317)
    model=load_model(a.resume) if a.resume else make_model().cuda();opt=torch.optim.Adam(model.parameters(),lr=a.lr)
    scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(opt,T_max=max(1,a.epochs),eta_min=a.lr*.1) if a.cosine else None
    train=DataLoader(Samples(a.manifest,'train'),batch_size=a.batch,shuffle=True,num_workers=a.workers,pin_memory=True,persistent_workers=a.workers>0)
    val=DataLoader(Samples(a.manifest,'val'),batch_size=a.batch,num_workers=a.workers,pin_memory=True,persistent_workers=a.workers>0)
    best=float('inf');history=[]
    if a.resume:
        model.metric_depth_head=True
        model.eval();vtotal=0
        with torch.no_grad():
            for batch in val:
                x,heat,ij,hps,reg,depth=[v.cuda(non_blocking=True) for v in batch]
                vtotal+=float(loss_for(model(x)[0],heat,ij,hps,reg,depth))
        best=vtotal/len(val)
        torch.save(dict(metric_depth_head=True,symmetry_aware=True,model=model.state_dict(),epoch=0,scope='Initial weights retained unless validation improves',default_puck_dimensions_m=[.03,.03,.06],box_dimensions_m=[[.205,.205,.07],[.410,.410,.07]]),a.output/'best.pt')
        print(dict(initial_val_loss=best),flush=True)
    for epoch in range(a.epochs):
        model.train();total=0
        for batch in train:
            x,heat,ij,hps,reg,depth=[v.cuda(non_blocking=True) for v in batch]
            out=model(x)[0];loss=loss_for(out,heat,ij,hps,reg,depth)
            opt.zero_grad();loss.backward();nn.utils.clip_grad_norm_(model.parameters(),10);opt.step();total+=float(loss.detach())
        model.eval();vtotal=0
        with torch.no_grad():
            for batch in val:
                x,heat,ij,hps,reg,depth=[v.cuda(non_blocking=True) for v in batch];vtotal+=float(loss_for(model(x)[0],heat,ij,hps,reg,depth))
        row=dict(epoch=epoch+1,train_loss=total/len(train),val_loss=vtotal/len(val),lr=opt.param_groups[0]['lr']);history.append(row);print(row,flush=True)
        if row['val_loss']<best:
            best=row['val_loss'];torch.save(dict(metric_depth_head=True,symmetry_aware=True,model=model.state_dict(),epoch=epoch+1,scope='Custom crop harness; hm/hps/reg and normalized metric depth supervised; symmetry-aware corners',default_puck_dimensions_m=[.03,.03,.06],box_dimensions_m=[[.205,.205,.07],[.410,.410,.07]]),a.output/'best.pt')
        if (epoch+1)%a.checkpoint_every==0 or epoch+1==a.epochs:
            torch.save(dict(metric_depth_head=True,symmetry_aware=True,model=model.state_dict(),epoch=epoch+1,optimizer=opt.state_dict(),default_puck_dimensions_m=[.03,.03,.06],box_dimensions_m=[[.205,.205,.07],[.410,.410,.07]],scope='DLA34 custom crop harness; not native TAO export'),a.output/'last.pt')
        (a.output/'history.json').write_text(json.dumps(history,indent=2))
        if scheduler:scheduler.step()

if __name__=='__main__':main()
