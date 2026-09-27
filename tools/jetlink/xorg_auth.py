"""Create a local X11 cookie without putting it in logs or a repository."""
import os
from pathlib import Path
import pwd
import secrets
import subprocess
import sys

owner = pwd.getpwnam(sys.argv[1])
path = Path('/run/carrot-jetlink-xorg/Xauthority')
path.touch(mode=0o600)
subprocess.run(['/usr/bin/xauth', '-f', str(path), 'add', ':1', '.', secrets.token_hex(16)], check=True)
os.chown(path, owner.pw_uid, owner.pw_gid)
os.chmod(path, 0o600)
