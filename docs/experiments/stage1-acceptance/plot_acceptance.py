import contextlib,io,runpy,sys
from pathlib import Path
import numpy as np
from reportlab.graphics.shapes import Drawing,String
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.graphics import renderPDF
from reportlab.lib import colors
import pypdfium2 as pdfium
out=Path(sys.argv[1]);sources=sys.argv[2:];drawing=Drawing(1160,1220)
drawing.add(String(45,1194,'Two-run acceptance | raw speed, trailing 0.25s mean and target',fontSize=19))
drawing.add(String(45,1173,'Rotor rpm; chart displays every third raw sample; metrics use all samples. Timing uncertainty ~26ms.',fontSize=10))
for col,source in enumerate(sources):
 sys.argv=[str(out/'analyze_acceptance.py'),source,str(out/f'run{col+1}')]
 with contextlib.redirect_stdout(io.StringIO()):ns=runpy.run_path(sys.argv[0],run_name='__main__')
 for run in ns['results']:
  motor=run['motor'];start=run['start_s'];a=ns['window'](ns['raw'][motor],start,start+18);moving=[];target=[]
  for phase in run['phases']:
   b=ns['window'](ns['raw'][motor],phase['first_diagnostic_s'],phase['last_diagnostic_s']);t=b[:,0];v=b[:,1];left=np.searchsorted(t,t-.25,side='left');cs=np.r_[0,np.cumsum(v)];ma=(cs[1:]-cs[left])/(np.arange(len(v))+1-left)
   moving.extend((float(tt-start),float(vv)) for tt,vv in zip(t[::5],ma[::5]));target.extend([(phase['first_diagnostic_s']-start,phase['target']),(phase['last_diagnostic_s']-start,phase['target'])])
  x=65+col*565;y=890-(motor-1)*280;p=LinePlot();p.x=x;p.y=y;p.width=490;p.height=210
  p.data=[[(float(t-start),float(v)) for t,v in a[::3,:2]],moving,target]
  p.xValueAxis.valueMin=0;p.xValueAxis.valueMax=18;p.xValueAxis.valueStep=2;p.yValueAxis.valueMin=-800;p.yValueAxis.valueMax=800;p.yValueAxis.valueStep=200
  for idx,color in enumerate([colors.HexColor('#80a9d0'),colors.HexColor('#154a80'),colors.red]):p.lines[idx].strokeColor=color;p.lines[idx].strokeWidth=.45 if idx==0 else 1
  drawing.add(p);drawing.add(String(x,y+229,f'Motor {motor} | run {col+1} | raw=light blue, mean=dark blue, target=red',fontSize=10))
with pdfium.PdfDocument(renderPDF.drawToString(drawing)) as pdf:pdf[0].render(scale=1.3).to_pil().save(out/'speed-traces.png')
print('saved speed-traces.png')
