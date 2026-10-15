"""Read-only RTDE output subscription. No input recipes or motion methods."""
import copy
from collections import deque
import threading
import time
import math
import os
import sys
from contextlib import contextmanager
import smoke


@contextmanager
def quiet_native_auth():
    """Hide SDK auth frames without callbacks into Python worker threads."""
    sys.stdout.flush();sys.stderr.flush()
    saved=[os.dup(1),os.dup(2)]
    with open(os.devnull,'w') as sink:
        try:
            os.dup2(sink.fileno(),1);os.dup2(sink.fileno(),2)
            yield
        finally:
            os.dup2(saved[0],1);os.dup2(saved[1],2)
            for fd in saved:os.close(fd)


class Telemetry:
    FIELDS = ['R1_actual_q', 'R1_actual_qd', 'R1_target_q',
              'R1_target_qd', 'R1_target_qdd', 'R1_robot_mode',
              'R1_safety_mode', 'runtime_state']

    def __init__(self, host, user, password):
        import pyaubo_sdk as sdk
        self.client = sdk.RtdeClient()
        self.lock = threading.Lock()
        self.pending = deque(maxlen=20000)
        self.last = None
        self.error = None
        self.topic = None
        try:
            with quiet_native_auth():
                self.client.connect(host, 30010)
                smoke.require(self.client.hasConnected(), 'RTDE connect failed')
                try:self.client.login(user, password)
                except Exception:raise smoke.Refused('RTDE authentication failed') from None
                smoke.require(self.client.hasLogined(), 'RTDE authentication failed')
            # false means controller -> client, as in AUBO's official driver.
            self.topic = self.client.setTopic(False, self.FIELDS, 100.0, 0)
            smoke.require(self.topic >= 0, 'RTDE output recipe rejected')
            smoke.require(self.client.subscribe(self.topic, self._receive) == 0, 'RTDE subscribe failed')
            deadline = time.monotonic() + 3
            while self.last is None and time.monotonic() < deadline:time.sleep(.02)
            self.latest()
        except BaseException:
            self.close()
            raise

    def _receive(self, parser):
        try:
            sample = dict(received=time.monotonic(),
                          q=smoke.vector(parser.popVectorDouble(), 'RTDE q'),
                          qd=smoke.vector(parser.popVectorDouble(), 'RTDE qd'),
                          target_q=smoke.vector(parser.popVectorDouble(), 'RTDE target q'),
                          target_qd=smoke.vector(parser.popVectorDouble(), 'RTDE target qd'),
                          target_qdd=smoke.vector(parser.popVectorDouble(), 'RTDE target qdd'),
                          robot_mode=smoke.enum_name(parser.popRobotModeType()),
                          safety_mode=smoke.enum_name(parser.popSafetyModeType()),
                          runtime=smoke.enum_name(parser.popRuntimeState()))
            with self.lock:
                if len(self.pending) == self.pending.maxlen:
                    self.error = 'RTDE consumer fell behind'
                self.last = sample
                self.pending.append(sample)
        except BaseException as e:
            self.error = type(e).__name__ + ': ' + str(e)

    def latest(self):
        with self.lock:
            smoke.require(self.error is None, 'RTDE parser failed: ' + str(self.error))
            smoke.require(self.last is not None, 'No RTDE feedback')
            sample = copy.deepcopy(self.last)
        smoke.require(0 <= time.monotonic()-sample['received'] <= .25, 'RTDE feedback stale')
        return sample

    def drain(self):
        self.latest()
        with self.lock:
            samples = list(self.pending)
            self.pending.clear()
        return samples

    def close(self):
        try:
            if self.topic is not None and self.topic>=0:self.client.removeTopic(False, self.topic)
        except Exception:pass
        try:self.client.disconnect()
        except Exception:pass


if __name__ == '__main__':
    import getpass
    import json
    from pathlib import Path
    config=smoke.load_config(Path(__file__).with_name('commission.local.json'))
    username=input('SDK username [aubo]: ').strip() or 'aubo'
    password=getpass.getpass('SDK password (not saved): ')
    log=smoke.Log('rtde_readonly_10s')
    stream=None
    try:
        stream=Telemetry(config['host'],username,password)
        samples=[]
        end=time.monotonic()+10
        while time.monotonic()<end:
            for s in stream.drain():log.event('rtde_readonly',**s);samples.append(s)
            time.sleep(.02)
        print(json.dumps(dict(read_only=True,samples=len(samples),
            initial_q_deg=[math.degrees(x) for x in samples[0]['q']],
            final_q_deg=[math.degrees(x) for x in samples[-1]['q']],
            peak_actual_speed_deg_s=max(abs(math.degrees(x)) for s in samples for x in s['qd']),
            peak_target_speed_deg_s=max(abs(math.degrees(x)) for s in samples for x in s['target_qd']),
            robot_mode=samples[-1]['robot_mode'],safety_mode=samples[-1]['safety_mode']),indent=2))
        print('Log:',log.path)
    finally:
        if stream:stream.close()
