"""After successful exact replay, export/build/deploy only; never submit motion."""
from pathlib import Path
import hashlib,json,shutil,subprocess,sys,time
ROOT=Path(__file__).resolve().parent
WORK=ROOT.parents[2]
OLD=ROOT.parent/'fullflow_4x_webui_01'
until=time.monotonic()+90
while not (ROOT/'replay_01/run.json').exists():
 if time.monotonic()>until:raise TimeoutError('Replay has not finished')
 time.sleep(.5)
status=json.loads((ROOT/'replay_01/run.json').read_text())
assert status['success'] and status['shutdown_clean'],status
accept=json.loads((ROOT/'replay_01/acceptance.json').read_text())
assert accept['passed'] and accept['sequence_evidence']['passed'] and accept['fixed_orientation']['passed']
def run(name,args):
 with (ROOT/name).open('x') as f:subprocess.run(args,cwd=WORK,stdout=f,stderr=subprocess.STDOUT,check=True)
 print(name+' passed',flush=True)
run('export.log',[sys.executable,'scripts/task1_real_scene/save_simulation_trajectory.py','--run',str(ROOT/'replay_01'),'--output',str(ROOT/'saved_trajectory'),'--trajectory-math',str(ROOT/'player_j4_06/trajectory.py')])
run('controller_interpolation_check.log',[sys.executable,str(ROOT/'check_controller_samples.py')])
run('package.log',[sys.executable,'scripts/task2_real_scene/prepare_fullflow.py','--saved',str(ROOT/'saved_trajectory'),'--retiming',str(ROOT/'adaptive_retime_01'),'--manifest-reference',str(OLD/'commissioning/01_ABC_fullflow_4x_webui/manifest.json'),'--adapter',str(ROOT/'player_j4_06'),'--output',str(ROOT/'commissioning'),'--motion-reference',str(ROOT.parent/'fullflow_2x_01/physical_validation.json'),'--j4-tracking-guard-deg','0.6','--tracking-request',str(ROOT/'tracking_policy_request.json')])
shutil.copyfile(OLD/'commissioning/remote_bridge.py',ROOT/'commissioning/remote_bridge.py')
run('deployment.log',[sys.executable,str(ROOT/'deploy_verified.py')])
record=json.loads((ROOT/'commissioning/commissioning.json').read_text());entry=record['packages'][0]
package=ROOT/'commissioning'/entry['directory'];manifest=json.loads((package/'manifest.json').read_text())
deployment=json.loads((ROOT/'deployment.json').read_text())
settings=json.loads((OLD/'hardware_config.json').read_text())
settings.update(package=str(package),claim_file=str(ROOT/'webui_claim.json'),preflight_file=str(ROOT/'webui_preflight.json'),
    remote_bridge=deployment['remote_directory']+'/remote_bridge.py',remote_package=deployment['remote_directory']+'/'+entry['directory'],
    manifest_sha256=entry['manifest_sha256'],source_trajectory_sha256=hashlib.sha256((ROOT/'saved_trajectory/task_two_trajectory.json').read_bytes()).hexdigest(),
    trajectory_label='平滑提速 · 当前停点重新起步 · 仿真通过',
    previous_task_record=str(WORK/'outputs/webui_laptop_20261014/runs/20261015T002754_360343Z/ui-11f0b12b-d6da-4390-80c6-b2ee682c31c1.json'))
with (ROOT/'hardware_config.json').open('x') as f:json.dump(settings,f,ensure_ascii=False,indent=2);f.write('\n')
print(json.dumps(dict(status='EXACT_REPLAY_EXPORT_DEPLOYMENT_PASSED',plan_id=entry['plan_id'],tracking_guard_deg=manifest['tracking_guard_deg'],motion_commands_sent=0),indent=2))
