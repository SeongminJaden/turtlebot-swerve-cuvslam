"""Measure actual training throughput without changing any saved checkpoint."""
import argparse,json,subprocess,threading,time,gc
from pathlib import Path
import numpy as np
import torch
from torch.utils.data import DataLoader
from train_centerpose import Samples,load_model,loss_for

def main():
    p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=20);a=p.parse_args()
    torch.set_num_threads(4)
    samples=Samples(a.manifest,'train');telemetry=[]
    process=subprocess.Popen(['nvidia-smi','--query-gpu=utilization.gpu','--format=csv,noheader,nounits','-lms','200'],stdout=subprocess.PIPE,text=True)
    def collect():
        for line in process.stdout:
            try:telemetry.append((time.monotonic(),float(line.strip())))
            except ValueError:pass
    thread=threading.Thread(target=collect,daemon=True);thread.start();results=[]
    try:
        for batch_size,workers in ((32,0),(32,2),(64,2),(64,4)):
            torch.manual_seed(317)
            model=load_model(a.checkpoint).train();optimizer=torch.optim.Adam(model.parameters(),lr=.0001)
            loader=DataLoader(samples,batch_size=batch_size,shuffle=True,num_workers=workers,pin_memory=workers>0,persistent_workers=workers>0)
            iterator=iter(loader)
            def step():
                nonlocal iterator
                try:batch=next(iterator)
                except StopIteration:iterator=iter(loader);batch=next(iterator)
                x,heat,ij,hps,reg,depth=[v.cuda(non_blocking=workers>0) for v in batch]
                loss=loss_for(model(x)[0],heat,ij,hps,reg,depth)
                optimizer.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),10);optimizer.step()
                return len(x)
            for _ in range(8):step()
            torch.cuda.synchronize();start=time.monotonic();objects=0
            while time.monotonic()-start<a.seconds:objects+=step()
            torch.cuda.synchronize();end=time.monotonic()
            values=[value for stamp,value in telemetry if start<=stamp<=end]
            result={'batch':batch_size,'workers':workers,'seconds':end-start,'objects_per_second':objects/(end-start),'gpu_mean_percent':float(np.mean(values)),'gpu_median_percent':float(np.median(values)),'samples':len(values),'scope':'training batches including loader and optimizer; excludes validation/checkpoint IO; all weight updates discarded'}
            results.append(result);print(json.dumps(result),flush=True)
            a.output.write_text(json.dumps(results,indent=2))
            del iterator,loader,optimizer,model;gc.collect();torch.cuda.empty_cache()
    finally:process.terminate();process.wait();thread.join(timeout=2)

if __name__=='__main__':main()
