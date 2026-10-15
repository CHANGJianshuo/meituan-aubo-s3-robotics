#!/usr/bin/env python3
"""Live D435i preview and timestamped RGB/depth evidence. No robot interface.

Run on the operating system that owns the camera. A request_capture.json file in
--output can request a snapshot: {"name": "before_j6", "frames": 30}.
Images stay in the native color optical frame; raw and aligned depth are retained.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import re
import time
from pathlib import Path

import sys
sys.path.append('/home/operator/meituan/eye_to_hand_calib/vendor/py312')

import cv2
import numpy as np
import pyrealsense2 as rs


def intrinsics(profile):
    i = profile.as_video_stream_profile().get_intrinsics()
    return dict(width=i.width, height=i.height, fx=i.fx, fy=i.fy,
                ppx=i.ppx, ppy=i.ppy, model=str(i.model), coeffs=list(i.coeffs))


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2), encoding='utf-8')
    # A WSL reader can briefly hold the destination without FILE_SHARE_DELETE.
    # Keep complete-file publication, but tolerate that Windows sharing race.
    for attempt in range(51):
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt == 50:
                raise
            time.sleep(.01)


def save_capture(pending, metadata):
    """Compress evidence outside the live preview/camera heartbeat loop."""
    target = pending['target']
    np.savez_compressed(target / 'frames.npz',
                        color_rgb=np.asarray(pending['colors']),
                        aligned_depth_u16=np.asarray(pending['depths']),
                        raw_depth_u16=np.asarray(pending['raw_depths']))
    cv2.imwrite(str(target / 'color.png'),
                cv2.cvtColor(pending['colors'][-1], cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(target / 'aligned_depth.png'), pending['depths'][-1])
    atomic_json(target / 'capture.json',
                dict(camera=metadata, frames=pending['stamps'],
                     complete=True, motion_association=None))
    return dict(event='capture_saved', path=str(target), frames=pending['count'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--serial')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    devices = list(rs.context().query_devices())
    if args.serial:
        devices = [d for d in devices
                   if d.get_info(rs.camera_info.serial_number) == args.serial]
    if len(devices) != 1:
        raise RuntimeError(f'Expected one selected RealSense, found {len(devices)}')
    device = devices[0]
    serial = device.get_info(rs.camera_info.serial_number)
    info = {}
    for key in ['name', 'serial_number', 'firmware_version', 'usb_type_descriptor',
                'product_line', 'physical_port']:
        field = getattr(rs.camera_info, key)
        if device.supports(field):
            info[key] = device.get_info(field)
    attempts = [(1280, 720, 1280, 720, 6)]
    failures = []
    for cw, ch, dw, dh, fps in attempts:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, cw, ch, rs.format.rgb8, fps)
        config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
        try:
            profile = pipeline.start(config)
            break
        except RuntimeError as error:
            failures.append(str(error))
    else:
        raise RuntimeError('Cannot start camera: ' + '; '.join(failures))
    depth_profile = profile.get_stream(rs.stream.depth)
    color_profile = profile.get_stream(rs.stream.color)
    ext = depth_profile.get_extrinsics_to(color_profile)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    metadata = dict(
        schema_version=1, data_kind='REAL_CAMERA_CAPTURE', device=info,
        host_time_domain='unix_wall_clock_ns; not robot-synchronized',
        optical_axes='x right, y down, z forward', depth_unit_m=depth_scale,
        color_intrinsics=intrinsics(color_profile),
        raw_depth_intrinsics=intrinsics(depth_profile),
        T_color_depth=dict(rotation_column_major=list(ext.rotation),
                           translation_m=list(ext.translation)),
        color_encoding='RGB8', depth_encoding='uint16_z16',
        fps_requested=fps, mount='external; pose not calibrated',
        camera_to_robot_calibrated=False, startup_failures=failures,
        software=dict(opencv=cv2.__version__, numpy=np.__version__,
                      pyrealsense2=getattr(rs, '__version__', 'unknown')))
    atomic_json(args.output / 'camera.json', metadata)
    print(json.dumps({'event': 'camera_started', 'device': info,
                      'output': str(args.output)}, ensure_ascii=True), flush=True)
    align = rs.align(rs.stream.color)
    title = 'MTC D435i - LIVE RGB - S snapshot / Q close'
    depth_title = 'MTC D435i - LIVE DEPTH (0.2 to 2.0 m)'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1152, 648)
    cv2.namedWindow(depth_title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(depth_title, 576, 324)
    cv2.moveWindow(title, 30, 40)
    cv2.moveWindow(depth_title, 1210, 40)
    last_status = 0.0
    frame_count = 0
    pending = None
    writer = ThreadPoolExecutor(max_workers=1)
    save_future = None
    previous = time.monotonic()
    failure = None
    try:
        while True:
            frames = pipeline.wait_for_frames(4000)
            received_ns = time.time_ns()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            aligned_depth_frame = aligned.get_depth_frame()
            raw_depth_frame = frames.get_depth_frame()
            if not color_frame or not aligned_depth_frame or not raw_depth_frame:
                continue
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(aligned_depth_frame.get_data())
            raw_depth = np.asanyarray(raw_depth_frame.get_data())
            frame_count += 1
            now = time.monotonic()
            frame_meta = dict(
                host_received_ns=received_ns,
                color_number=color_frame.get_frame_number(),
                depth_number=raw_depth_frame.get_frame_number(),
                color_timestamp_ms=color_frame.get_timestamp(),
                depth_timestamp_ms=raw_depth_frame.get_timestamp(),
                color_timestamp_domain=str(color_frame.get_frame_timestamp_domain()),
                depth_timestamp_domain=str(raw_depth_frame.get_frame_timestamp_domain()))
            if 'aligned_depth_intrinsics' not in metadata:
                metadata['aligned_depth_intrinsics'] = intrinsics(aligned_depth_frame.profile)
                atomic_json(args.output / 'camera.json', metadata)
            request = args.output / 'request_capture.json'
            if save_future is not None and save_future.done():
                print(json.dumps(save_future.result()), flush=True)
                save_future = None
            if request.is_file() and pending is None and save_future is None:
                try:
                    task = json.loads(request.read_text(encoding='utf-8'))
                    name = task['name']
                    count = task.get('frames', 1)
                    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', name):
                        raise ValueError('Invalid capture name')
                    if not isinstance(count, int) or not 1 <= count <= 90:
                        raise ValueError('frames must be 1..90')
                    target = args.output / name
                    target.mkdir(exist_ok=False)
                    pending = dict(target=target, count=count, colors=[],
                                   depths=[], raw_depths=[], stamps=[])
                    request.unlink()
                except Exception as error:
                    atomic_json(args.output / 'request_error.json',
                                dict(error=str(error), at_ns=received_ns))
                    request.rename(args.output / f'request_rejected_{received_ns}.json')
            if pending is not None:
                pending['colors'].append(color.copy())
                pending['depths'].append(depth.copy())
                pending['raw_depths'].append(raw_depth.copy())
                pending['stamps'].append(frame_meta)
                if len(pending['stamps']) >= pending['count']:
                    save_future = writer.submit(save_capture, pending, metadata.copy())
                    pending = None
            bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            display = bgr.copy()
            h, w = depth.shape
            patch = depth[h//2-3:h//2+4, w//2-3:w//2+4]
            valid = patch[patch > 0]
            center_m = float(np.median(valid) * depth_scale) if valid.size else None
            cv2.drawMarker(display, (w//2, h//2), (0, 255, 255), cv2.MARKER_CROSS, 18, 1)
            label = f'LIVE | {w}x{h} | center depth: {center_m:.3f} m' if center_m else 'LIVE | center depth invalid'
            cv2.rectangle(display, (0, 0), (w, 43), (20, 20, 20), -1)
            cv2.putText(display, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            metric = depth.astype(np.float32) * depth_scale
            scaled = np.clip((metric - 0.2) / 1.8 * 255, 0, 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
            depth_color[depth == 0] = 0
            cv2.imshow(title, display)
            cv2.imshow(depth_title, depth_color)
            if now - last_status > 1.0:
                cv2.imwrite(str(args.output / 'latest_color.jpg'), bgr)
                cv2.imwrite(str(args.output / 'latest_depth_preview.jpg'), depth_color)
                atomic_json(args.output / 'status.json',
                            dict(state='streaming', **frame_meta, frame_count=frame_count,
                                 center_depth_m=center_m,
                                 valid_depth_fraction=float(np.mean(depth > 0)),
                                 host_frame_interval_s=now - previous))
                last_status = now
            previous = now
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord('s') and not request.exists():
                atomic_json(request, dict(name=f'manual_{received_ns}', frames=1))
    except BaseException as error:
        failure = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        try:
            pipeline.stop()
        except RuntimeError:
            # A physical USB disconnect may already have stopped the pipeline.
            pass
        cv2.destroyAllWindows()
        writer.shutdown(wait=True)
        atomic_json(args.output / 'status.json',
                    dict(state='error' if failure else 'stopped', error=failure,
                         host_received_ns=time.time_ns(), frame_count=frame_count))


if __name__ == '__main__':
    main()
