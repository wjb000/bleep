#!/usr/bin/env python3
"""Download vendor CAD that this repo does not redistribute into cad/vendor_fetched/ (git-ignored):
Seeed Studio XIAO ESP32S3 STEP (used for the controller mesh in the viewer model). Run: python cad/fetch_vendor.py"""
import io, os, urllib.request, zipfile
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vendor_fetched'); os.makedirs(OUT, exist_ok=True)
XIAO = 'https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/seeed-studio-xiao-esp32s3-3d_model.zip'
req = urllib.request.Request(XIAO, headers={'User-Agent': 'Mozilla/5.0 bleep fetch_vendor'})
z = zipfile.ZipFile(io.BytesIO(urllib.request.urlopen(req, timeout=120).read()))
for n in z.namelist():
    if n.lower().endswith(('.step', '.stp')) and not n.startswith('__MACOSX'):
        open(os.path.join(OUT, 'xiao_esp32s3.step'), 'wb').write(z.read(n)); print('->', OUT + '/xiao_esp32s3.step'); break
