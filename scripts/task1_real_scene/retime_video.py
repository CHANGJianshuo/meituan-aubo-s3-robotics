#!/usr/bin/env python3
"""Restore 1x playback using exact matches of the recorder's rendered clock."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def retime(run):
    source = run/'overview.mp4'
    output = run/'overview_realtime.mp4'
    if output.exists():
        raise ValueError('Refusing to replace an existing playback video')
    camera = json.loads((run/'camera.json').read_text())
    fps = 5.
    last_tick = round(camera['last_sim_time']*fps)
    # The camera runs at 5 Hz, and record_camera.py draws this known font.
    # Require an exact binary mask match; never infer time from frame count.
    templates = {}
    for tick in range(last_tick+1):
        canvas = np.full((40, 520), 255, dtype=np.uint8)
        cv2.putText(canvas, f'V1 Gazebo | simulation t={tick/fps:.2f} s',
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .7, 30, 2)
        key = np.packbits(canvas[12:32, 310:510] < 120).tobytes()
        if key in templates:
            raise ValueError('Timestamp templates are not unique')
        templates[key] = tick
    cap = cv2.VideoCapture(str(source))
    frames, ticks = [], []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        mask = cv2.cvtColor(frame[12:32, 310:510], cv2.COLOR_BGR2GRAY) < 120
        key = np.packbits(mask).tobytes()
        if key not in templates:
            raise ValueError(f'Clock pixels do not match at frame {len(frames)}')
        tick = templates[key]
        if ticks and tick <= ticks[-1]:
            raise ValueError('Clock is not strictly increasing')
        frames.append(frame)
        ticks.append(tick)
    cap.release()
    if len(frames) != camera['frames'] or not ticks or ticks[-1] != last_tick:
        raise ValueError('Camera metadata and decoded video disagree')
    h, w = frames[0].shape[:2]
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    if not writer.isOpened():
        raise ValueError('Cannot create playback video')
    index = 0
    for tick in range(ticks[0], ticks[-1]+1):
        if index+1 < len(ticks) and ticks[index+1] == tick:
            index += 1
        writer.write(frames[index])
    writer.release()
    with (run/'video_frame_times.csv').open('w', newline='') as f:
        csv.writer(f).writerows([['source_frame', 'simulation_seconds', 'exact_template_pixel_difference']]
                               + [[i, t/fps, 0] for i, t in enumerate(ticks)])
    result = dict(source=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  output=output.name, output_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                  source_frames=len(frames), output_frames=ticks[-1]-ticks[0]+1, fps=fps,
                  first_sim_time_s=ticks[0]/fps, last_sim_time_s=ticks[-1]/fps,
                  playback_speed=1, all_timestamp_matches_exact=True,
                  method='Exact rendered simulation clock matching; hold the latest frame for missing frames on a uniform 0.2 s timeline; source unchanged')
    (run/'video_timing.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(retime(args.run), indent=2))
