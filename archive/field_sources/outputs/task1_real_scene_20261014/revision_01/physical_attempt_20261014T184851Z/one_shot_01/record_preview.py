"""Record the existing 1 Hz camera preview without reopening the camera."""
from pathlib import Path
import csv
import json
import time
import cv2

out = Path(__file__).resolve().parent
source = Path(json.loads(Path('/tmp/e2h_camera_preview_current.json').read_text())['output'])/'latest_color.jpg'
start = time.monotonic()
frame = cv2.imread(str(source))
height, width = frame.shape[:2]
writer = cv2.VideoWriter(str(out/'physical_observation.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 5, (width, height))
assert writer.isOpened()
index = 0
try:
    with (out/'physical_observation_times.csv').open('w', newline='') as f:
        table = csv.writer(f)
        table.writerow(['frame', 'elapsed_wall_s', 'read_unix_s', 'source_modified_unix_s'])
        while not (out/'stop_recording').exists() and time.monotonic()-start < 1800:
            current = cv2.imread(str(source))
            if current is not None:
                frame = current
            writer.write(frame)
            table.writerow([index, time.monotonic()-start, time.time(), source.stat().st_mtime])
            f.flush()
            index += 1
            time.sleep(max(0., start+index/5-time.monotonic()))
finally:
    writer.release()
    (out/'recording.json').write_text(json.dumps({'frames': index, 'video_fps': 5, 'source_preview_rate_hz': 1,
        'method': 'Existing camera preview held between updates; wall-time playback; timestamps in CSV'}, indent=2)+'\n')
