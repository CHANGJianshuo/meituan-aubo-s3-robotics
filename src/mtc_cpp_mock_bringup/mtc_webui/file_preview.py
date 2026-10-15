"""Read the existing laptop camera previews without opening the camera device.

This optional C++ Mock display source provides no perception or robot feedback.
"""
from io import BytesIO
from pathlib import Path

from PIL import Image

from mtc_webui.cpp_mock import CppMockBackend


class FilePreviewBackend(CppMockBackend):
    def __init__(self, *args, preview_dir, **kwargs):
        super().__init__(*args, **kwargs)
        self.preview_dir = Path(preview_dir).resolve()
        self.preview_stamps = {}

    def _read_preview(self, kind, filename):
        path = self.preview_dir / filename
        try:
            before = path.stat()
            if self.preview_stamps.get(kind) == before.st_mtime_ns:
                return
            if not 0 < before.st_size <= 8 * 1024 * 1024:
                raise ValueError("invalid preview size")
            jpeg = path.read_bytes()
            after = path.stat()
            if (before.st_mtime_ns, before.st_size) != (after.st_mtime_ns, after.st_size):
                raise ValueError("preview changed while reading")
            if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
                raise ValueError("incomplete JPEG")
            with Image.open(BytesIO(jpeg)) as image:
                image.load()
                width, height = image.size
            age_ms = (self.wall_clock() * 1e9 - before.st_mtime_ns) / 1e6
            self.frame(kind, dict(topic="本机相机预览文件（只读）", width=width, height=height,
                preview="RGB 预览" if kind == "color" else "深度伪彩预览，非原始深度数值"),
                jpeg, "local-preview:" + str(path), before.st_mtime_ns, age_ms)
            self.preview_stamps[kind] = before.st_mtime_ns
        except (OSError, ValueError):
            self.invalidate(kind)
            self.preview_stamps.pop(kind, None)

    def snapshot(self):
        with self.lock:
            self._read_preview("color", "latest_color.jpg")
            self._read_preview("depth", "latest_depth_preview.jpg")
            result = super().snapshot()
            result["local_preview"] = dict(directory=str(self.preview_dir),
                role="现场图像仅供观察；不用于 Mock 感知、轨迹定位或实机控制")
            return result
