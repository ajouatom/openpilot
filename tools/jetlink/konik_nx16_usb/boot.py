"""읽기 전용 rootfs를 바꾸지 않고 검증된 DATA 프로그램을 runtime systemd로 연결한다."""
from __future__ import annotations

import hashlib
from pathlib import Path, PurePosixPath
import re

DATA_BASE = '/data/carrot-usb-internet'
PROGRAM = DATA_BASE + '/program.py'
DATA_UNIT = DATA_BASE + '/runtime.service'
RUNTIME_UNIT = '/run/systemd/system/carrot-usb-internet.service'
UNIT_NAME = 'carrot-usb-internet.service'
TEMPLATE_SHA = '3c8abdc884ee8a03faa82737734906429ed7aca3b42ce227a0fbd3b9f01a631d'
BEGIN = b'# BEGIN CARROT_USB_INTERNET_BOOT_V1'
END = b'# END CARROT_USB_INTERNET_BOOT_V1'

# 이 고정 stdlib 본문은 아래 API와 실제 continue heredoc이 동일하게 사용한다.
_CORE = r'''
import hashlib,json,os,re,stat,subprocess,tempfile
from pathlib import Path
DATA_BASE='/data/carrot-usb-internet'
PROGRAM=DATA_BASE+'/program.py'
DATA_UNIT=DATA_BASE+'/runtime.service'
RUNTIME_UNIT='/run/systemd/system/carrot-usb-internet.service'
UNIT_NAME='carrot-usb-internet.service'
SEARCH=('/etc/systemd/system.control','/run/systemd/system.control','/run/systemd/transient',
        '/run/systemd/generator.early','/etc/systemd/system','/etc/systemd/system.attached',
        '/run/systemd/system','/run/systemd/system.attached','/run/systemd/generator',
        '/usr/local/lib/systemd/system','/usr/lib/systemd/system','/lib/systemd/system','/run/systemd/generator.late')
class BootError(RuntimeError): pass
def need(value,code):
    if not value: raise BootError(code)
def fixed(root,absolute):
    need(absolute in (DATA_BASE,PROGRAM,DATA_UNIT,RUNTIME_UNIT), 'write_path_outside_data_or_runtime')
    return Path(root)/absolute.lstrip('/')
def digest(raw): return hashlib.sha256(raw).hexdigest()
def expected(value):
    need(isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value) is not None,'invalid_expected_sha')
    return value
def safe_dir(path,owner,mode=None):
    info=path.lstat()
    need(stat.S_ISDIR(info.st_mode) and info.st_uid==owner and not info.st_mode&0o022,'directory_owner_or_type')
    if mode is not None: need(stat.S_IMODE(info.st_mode)==mode,'directory_mode')
def read_blob(path,sha,owner):
    expected(sha)
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    try:
        info=os.fstat(fd)
        need(stat.S_ISREG(info.st_mode) and info.st_uid==owner and info.st_nlink==1 and
             stat.S_IMODE(info.st_mode)==0o644 and 0<info.st_size<=131072,'file_owner_mode_or_size')
        raw=os.read(fd,131073)
        after=os.fstat(fd)
        named=path.lstat()
        fields=('st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns','st_uid','st_mode','st_nlink')
        need(len(raw)==info.st_size and all(getattr(info,k)==getattr(after,k) for k in fields) and
             (named.st_dev,named.st_ino)==(info.st_dev,info.st_ino),'file_changed_during_read')
        need(digest(raw)==sha,'file_sha_changed')
        return raw,info
    finally: os.close(fd)
def new_file(path,raw,owner):
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o644)
    with os.fdopen(fd,'wb') as stream:
        os.fchmod(stream.fileno(),0o644)
        if os.geteuid()==0: os.fchown(stream.fileno(),owner,0 if owner==0 else -1)
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
def fsync_dir(path):
    fd=os.open(path,os.O_RDONLY|os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)
def install_data(program,unit,program_sha=None,unit_sha=None,root='/',owner=0):
    need(os.geteuid()==owner,'root_required')
    program_sha=expected(program_sha or digest(program));unit_sha=expected(unit_sha or digest(unit))
    need(digest(program)==program_sha and digest(unit)==unit_sha,'payload_sha_changed')
    need(0<len(program)<=131072 and 0<len(unit)<=131072,'payload_size')
    base=fixed(root,DATA_BASE)
    need(stat.S_ISDIR(base.parent.lstat().st_mode),'data_mount_not_directory')
    created_base=False
    if not base.exists() and not base.is_symlink():
        base.mkdir(mode=0o755);os.chmod(base,0o755);created_base=True
    safe_dir(base,owner,0o755)
    values=((PROGRAM,program,program_sha),(DATA_UNIT,unit,unit_sha))
    for name,raw,sha in values:
        path=fixed(root,name)
        if path.exists() or path.is_symlink(): read_blob(path,sha,owner)
    made=[]
    for name,raw,sha in values:
        path=fixed(root,name)
        if not path.exists():
            new_file(path,raw,owner);made.append(name)
    fsync_dir(base)
    return {'data_base_created':created_base,'created_files':made,'files':{PROGRAM:program_sha,DATA_UNIT:unit_sha}}
def foreign_units(root):
    for directory in SEARCH:
        base=Path(root)/directory.lstrip('/')
        unit=base/UNIT_NAME
        if str(directory+'/'+UNIT_NAME)!=RUNTIME_UNIT:
            need(not unit.exists() and not unit.is_symlink(),'foreign_unit_shadow')
        drop=base/(UNIT_NAME+'.d')
        need(not drop.is_symlink(),'foreign_dropin_symlink')
        if drop.exists(): need(drop.is_dir() and not any(drop.iterdir()),'foreign_unit_dropin')
def runtime_dir(root,owner):
    directory=fixed(root,RUNTIME_UNIT).parent
    safe_dir(directory.parent,owner)
    if not directory.exists() and not directory.is_symlink():
        directory.mkdir(mode=0o755);os.chmod(directory,0o755)
    safe_dir(directory,owner)
    return directory
def default_run(argv):
    timeout=25 if len(argv)>1 and argv[1]=='stop' else 10
    return subprocess.run(argv,stdin=subprocess.DEVNULL,capture_output=True,text=True,check=True,timeout=timeout,
                          env={'PATH':'/usr/sbin:/usr/bin:/sbin:/bin','LANG':'C','LC_ALL':'C'}).stdout.strip()
def loaded_guard(root,run,allow_absent=False):
    fragment=run(['/usr/bin/systemctl','show','-p','FragmentPath','--value',UNIT_NAME]).strip()
    drop=run(['/usr/bin/systemctl','show','-p','DropInPaths','--value',UNIT_NAME]).strip()
    need(not drop,'loaded_foreign_dropin')
    if allow_absent and not fragment: return False
    need(fragment==str(fixed(root,RUNTIME_UNIT)),'loaded_foreign_fragment')
    return True
def bootstrap(unit_sha,program_sha,root='/',owner=0,runner=None):
    need(os.geteuid()==owner,'root_required');expected(unit_sha);expected(program_sha)
    base=fixed(root,DATA_BASE);safe_dir(base,owner,0o755)
    read_blob(fixed(root,PROGRAM),program_sha,owner)
    raw,_=read_blob(fixed(root,DATA_UNIT),unit_sha,owner)
    foreign_units(root)
    directory=runtime_dir(root,owner);target=fixed(root,RUNTIME_UNIT)
    copied=False
    if target.exists() or target.is_symlink(): read_blob(target,unit_sha,owner)
    else:
        fd,name=tempfile.mkstemp(prefix='.carrot-usb-unit-',dir=directory)
        try:
            with os.fdopen(fd,'wb') as stream:
                os.fchmod(stream.fileno(),0o644)
                if os.geteuid()==0: os.fchown(stream.fileno(),owner,0 if owner==0 else -1)
                stream.write(raw);stream.flush();os.fsync(stream.fileno())
            # 외부 파일이 생겼다면 덮지 않는다. 같은 디렉터리 hard-link는 배타 생성이다.
            os.link(name,target,follow_symlinks=False);copied=True
        finally: Path(name).unlink(missing_ok=True)
        fsync_dir(directory)
    run=runner or default_run
    run(['/usr/bin/systemctl','daemon-reload'])
    loaded_guard(root,run)
    run(['/usr/bin/systemctl','--no-block','start',UNIT_NAME])
    return {'status':'runtime_start_requested','runtime_copied':copied,'rootfs_modified':False}
def rollback(unit_sha,root='/',owner=0,runner=None):
    need(os.geteuid()==owner,'root_required');expected(unit_sha)
    target=fixed(root,RUNTIME_UNIT)
    if not target.exists() and not target.is_symlink(): return {'status':'already_absent'}
    foreign_units(root);safe_dir(target.parent,owner)
    _,before=read_blob(target,unit_sha,owner)
    run=runner or default_run
    if loaded_guard(root,run,allow_absent=True): run(['/usr/bin/systemctl','stop',UNIT_NAME])
    after=target.lstat()
    need(all(getattr(before,k)==getattr(after,k) for k in
             ('st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns','st_uid','st_mode','st_nlink')),'runtime_changed_before_remove')
    target.unlink();fsync_dir(target.parent)
    run(['/usr/bin/systemctl','daemon-reload'])
    return {'status':'runtime_removed','data_payload_preserved':True,'rootfs_modified':False}
'''


def _space():
    namespace = {}
    exec(compile(_CORE, '<carrot-usb-boot-stdlib>', 'exec'), namespace)
    return namespace


def install_data(program, unit, program_sha=None, unit_sha=None, root='/', owner=0):
    return _space()['install_data'](program, unit, program_sha, unit_sha, root, owner)


def bootstrap(unit_sha, program_sha, root='/', owner=0, runner=None):
    return _space()['bootstrap'](unit_sha, program_sha, root, owner, runner)


def rollback(unit_sha, root='/', owner=0, runner=None):
    return _space()['rollback'](unit_sha, root, owner, runner)


def _sha(value):
    if not isinstance(value, str) or re.fullmatch('[0-9a-f]{64}', value) is None:
        raise ValueError('invalid_expected_sha')
    return value


def _program_code(program_sha, path=PROGRAM, owner=0):
    _sha(program_sha)
    parent = str(PurePosixPath(path).parent)
    return ("import os,stat,hashlib;from pathlib import Path;"
            f"p={str(path)!r};d={parent!r};u={owner!r};"
            "ds=os.lstat(d);assert stat.S_ISDIR(ds.st_mode) and ds.st_uid==u and stat.S_IMODE(ds.st_mode)==0o755;"
            "fd=os.open(p,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK);s=os.fstat(fd);"
            "assert stat.S_ISREG(s.st_mode) and s.st_uid==u and s.st_nlink==1 and stat.S_IMODE(s.st_mode)==0o644 and 0<s.st_size<=131072;"
            "b=os.read(fd,131073);t=os.fstat(fd);n=os.lstat(p);os.close(fd);"
            "assert len(b)==s.st_size and all(getattr(s,k)==getattr(t,k) for k in "
            "('st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns','st_mode','st_uid','st_nlink')) and (n.st_dev,n.st_ino)==(s.st_dev,s.st_ino);"
            f"assert hashlib.sha256(b).hexdigest()=={program_sha!r};"
            "exec(compile(b,p,'exec'),{'__name__':'__main__','__file__':p})")


def make_unit(template, program_sha, expected_template_sha=TEMPLATE_SHA):
    _sha(program_sha)
    _sha(expected_template_sha)
    if not isinstance(template, bytes) or hashlib.sha256(template).hexdigest() != expected_template_sha:
        raise ValueError('unit_template_changed')
    lines = template.splitlines(keepends=True)
    positions = [index for index, line in enumerate(lines) if line.startswith(b'ExecStart=')]
    if len(positions) != 1:
        raise ValueError('unit_execstart_count')
    code = _program_code(program_sha).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')
    if '\n' in code or '\r' in code or '$' in code:
        raise ValueError('unit_loader_control_character')
    lines[positions[0]] = ('ExecStart=/usr/bin/python3 -I -S -B -c "' + code + '"\n').encode()
    return b''.join(lines)


def boot_code(unit_sha, program_sha):
    _sha(unit_sha)
    _sha(program_sha)
    return _CORE + f'\nbootstrap({unit_sha!r},{program_sha!r})\n'


def _block(unit_sha, program_sha):
    code = boot_code(unit_sha, program_sha).encode()
    terminator = 'CARROT_USB_BOOT_' + hashlib.sha256(code).hexdigest()[:16].upper()
    return (BEGIN + b'\n' + b"if ! sudo -n /usr/bin/python3 -I -S -B - <<'" + terminator.encode() + b"'\n" + code +
            terminator.encode() + b'\nthen\n  printf "%s\\n" "USB Internet optional boot setup unavailable; continuing normal startup." >&2\nfi\n' + END + b'\n')


def make_continue(original, unit_sha, program_sha):
    _sha(unit_sha)
    _sha(program_sha)
    if not isinstance(original, bytes) or len(original) > 131072 or not original.startswith(b'#!'):
        raise ValueError('continue_shebang_or_size')
    first = original.find(b'\n')
    if first < 2 or first > 4096 or b'\r' in original[:first] or b'\x00' in original:
        raise ValueError('continue_shebang_boundary')
    insertion = first + 1
    block = _block(unit_sha, program_sha)
    if BEGIN in original or END in original:
        if (original.count(BEGIN) == original.count(END) == 1 and original[insertion:].startswith(block)):
            return original
        raise ValueError('continue_foreign_or_duplicate_marker')
    return original[:insertion] + block + original[insertion:]
