"""실제 helper의 내장 Python을 임시 state/proc와 가짜 signal로 검사한다."""
import pytest
import builtins
import errno
import signal
import sys
from pathlib import Path
from types import SimpleNamespace
pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="Linux /proc and no-follow file checks")

SCRIPT = Path(__file__).with_name('setup_mobile.sh')
REAL_PATH = Path

def embedded_script():
    text = SCRIPT.read_text(encoding='utf-8')
    return text.split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]

class FakeProcess:
    """커널·실제 PID를 전혀 사용하지 않는 작은 실제 파일 fixture."""

    def __init__(self, root):
        self.root = root
        self.state = root / 'state'
        self.leases = root / 'leases'
        self.proc = root / 'proc' / '4242'
        self.state.mkdir()
        self.leases.mkdir()
        self.proc.mkdir(parents=True)
        self.pidfile = self.state / 'dnsmasq.pid'
        self.owner = self.state / 'dnsmasq.owner'
        self.pidfile.write_text('4242\n')
        self.owner.write_text('{"pid": 4242, "start": "999"}')
        self.args = ['dnsmasq', f'--pid-file={self.pidfile}', '--port=0', '--conf-file=/dev/null', f"--dhcp-leasefile={self.leases / 'dnsmasq.leases'}"]
        (self.proc / 'cmdline').write_bytes(b'\x00'.join(s.encode() for s in self.args) + b'\x00')
        (self.proc / 'exe').symlink_to('/synthetic/dnsmasq')
        self.write_stat('999')
        self.stat_reads = 0
        self.fail_stat_at = None
        self.exit_on_failure = True
        self.state_change = None
        self.kill_count = 0
        self.exit_on_kill = True

    def write_stat(self, start):
        (self.proc / 'stat').write_text('4242 (dnsmasq) ' + ' '.join(['S'] + ['0'] * 18 + [start]))

    def remove_process(self):
        for p in ('cmdline', 'exe', 'stat'):
            (self.proc / p).unlink(missing_ok=True)
        self.proc.rmdir()

    def kill(self, pid, sig):
        if pid != 4242 or sig != signal.SIGTERM:
            raise AssertionError('합성 PID 외 signal 금지')
        self.kill_count += 1
        if self.exit_on_kill:
            self.remove_process()

    def run(self, action='stop'):
        fixture = self

        class SyntheticPath(type(REAL_PATH())):

            def read_text(self, *args, **kwargs):
                if str(self) == str(fixture.proc / 'stat'):
                    fixture.stat_reads += 1
                    if fixture.stat_reads == fixture.fail_stat_at:
                        if fixture.exit_on_failure:
                            fixture.remove_process()
                        if fixture.state_change:
                            fixture.state_change()
                        raise FileNotFoundError(errno.ENOENT, '합성 프로세스 종료', str(self))
                return super().read_text(*args, **kwargs)

        def synthetic_path(*args):
            if args == ('/proc',):
                args = (str(fixture.root / 'proc'),)
            return SyntheticPath(*args)

        def isolated_import(name, *args, **kwargs):
            if name == 'pathlib':
                return SimpleNamespace(Path=synthetic_path)
            return builtins.__import__(name, *args, **kwargs)
        scope = {'__name__': '__main__', '__builtins__': {**vars(builtins), '__import__': isolated_import}}

        def unavailable_pidfd(*args):
            raise OSError(errno.ENOSYS, '합성 구형 커널')
        with pytest.MonkeyPatch.context() as monkeypatch:
            monkeypatch.setattr('os.kill', self.kill)
            monkeypatch.setattr('os.pidfd_open', unavailable_pidfd, raising=False)
            monkeypatch.setattr(sys, 'argv', ['helper', str(self.state), action, str(self.leases)])
            try:
                exec(compile(embedded_script(), '<helper-dhcp-process>', 'exec'), scope)
            except SystemExit as exc:
                return exc.code
        return 0

class TestCleanupRace:

    @pytest.fixture(autouse=True)
    def temporary_root(self, tmp_path):
        self.root = tmp_path

    def fixture(self, name='case'):
        root = self.root / name
        root.mkdir()
        return FakeProcess(root)

    def test_exit_during_first_stat_read(self):
        f = self.fixture()
        f.fail_stat_at = 1
        assert f.run() == 0
        assert f.kill_count == 0
        assert not f.pidfile.exists()
        assert not f.owner.exists()

    def test_exit_after_signal_before_wait_stat(self):
        f = self.fixture()
        f.exit_on_kill = False
        f.fail_stat_at = 3
        assert f.run() == 0
        assert f.kill_count == 1
        assert not f.pidfile.exists()
        assert not f.owner.exists()

    def test_live_or_reused_process_remains_rejected(self):
        for kind in ('live_enoent', 'reused', 'exe', 'args', 'owner', 'owner_exit', 'missing_owner_exit'):
            f = self.fixture(kind)
            if kind == 'live_enoent':
                f.fail_stat_at, f.exit_on_failure = (1, False)
            elif kind == 'reused':
                f.write_stat('1000')
            elif kind == 'exe':
                (f.proc / 'exe').unlink()
                (f.proc / 'exe').symlink_to('/synthetic/other-program')
            elif kind == 'args':
                (f.proc / 'cmdline').write_bytes(b'dnsmasq\x00--other\x00')
            elif kind == 'missing_owner_exit':
                f.owner.unlink()
                f.fail_stat_at = 1
            else:
                f.owner.write_text('{"pid": 4243, "start": "999"}')
                if kind == 'owner_exit':
                    f.fail_stat_at = 1
            before = (f.pidfile.read_bytes(), f.owner.read_bytes() if f.owner.exists() else None)
            assert f.run() != 0
            assert f.kill_count == 0
            assert (f.pidfile.read_bytes(), f.owner.read_bytes() if f.owner.exists() else None) == before

    def test_changed_state_is_preserved(self):
        for kind in ('content', 'same_bytes_new_inode'):
            f = self.fixture(kind)
            old = f.owner.read_bytes()

            def change(f=f, kind=kind, old=old):
                if kind == 'content':
                    f.owner.write_text('{"pid": 4242, "start": "replacement"}')
                else:
                    replacement = f.state / 'replacement'
                    replacement.write_bytes(old)
                    replacement.replace(f.owner)
            f.state_change, f.fail_stat_at = (change, 1)
            assert f.run() != 0
            assert f.pidfile.exists()
            assert f.owner.exists()
            assert f.kill_count == 0

    def test_record_and_alive_exit_do_not_become_stop_success(self):
        for action in ('record', 'alive'):
            f = self.fixture(action)
            f.fail_stat_at = 1
            assert f.run(action) != 0
            assert f.pidfile.exists()
            assert f.owner.exists()
            assert f.kill_count == 0

    def test_original_owned_stop_and_absent_stop_stay_idempotent(self):
        f = self.fixture()
        assert f.run() == 0
        assert f.kill_count == 1
        assert f.run() == 0
        assert f.kill_count == 1
