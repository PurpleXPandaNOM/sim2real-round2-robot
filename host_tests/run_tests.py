#!/usr/bin/env python3
"""Requires Python 3 and g++. Copies source to a temporary tree; never changes shipping lock."""
from pathlib import Path
import shutil, subprocess, tempfile
root=Path(__file__).resolve().parent.parent
for enabled, profile in ((False,0),(True,0),(True,1)):
    with tempfile.TemporaryDirectory(prefix='round2-test-') as tmp:
        dest=Path(tmp)
        shutil.copytree(root/'Round2_Postman',dest/'Round2_Postman')
        shutil.copytree(root/'host_tests',dest/'host_tests')
        if enabled:
            cfg=dest/'Round2_Postman/Config.h'
            cfg.write_text(cfg.read_text().replace('HARDWARE_CONFIRMED=false','HARDWARE_CONFIRMED=true'))
        cfg=dest/'Round2_Postman/Config.h'
        cfg.write_text(cfg.read_text().replace('WIRING_PROFILE=0',f'WIRING_PROFILE={profile}'))
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-x','c++','-c',
            '-I'+str(dest/'host_tests/stubs'),str(dest/'Round2_Postman/Round2_Postman.ino'),
            '-o',str(dest/'sketch.o')],check=True)
        exe=dest/'test'
        subprocess.run(['g++','-std=c++17','-O1','-g','-Wall','-Wextra','-Werror',
            '-fsanitize=address,undefined','-fno-omit-frame-pointer',
            '-I'+str(dest/'host_tests/stubs'),str(dest/'host_tests/test.cpp'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
