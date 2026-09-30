from PIL import Image, ImageDraw, ImageFont
from pathlib import Path
root=Path(r'H:/fluent/live_cases/benchmark_A_stationary/animations')
font=ImageFont.load_default()
for kind in ('velocity','pressure'):
    paths=sorted((root/f'frames_{kind}').glob(f'{kind}_*.png'))
    out=[]
    for i,p in enumerate(paths):
        im=Image.open(p).convert('RGB')
        d=ImageDraw.Draw(im)
        d.rectangle((20,1020,220,1065),fill='white')
        d.text((30,1030),f't = {(i+1)*0.20:.2f} ms',fill='black',font=font)
        out.append(im)
    gif=root/('velocity_magnitude.gif' if kind=='velocity' else 'static_pressure.gif')
    out[0].save(gif,save_all=True,append_images=out[1:],duration=120,loop=0,optimize=False)
    print(gif, len(out), gif.stat().st_size, out[0].size)
