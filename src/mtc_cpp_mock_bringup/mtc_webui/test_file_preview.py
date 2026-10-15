"""Camera display must not refresh stale files or invent hardware state."""
from io import BytesIO
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from PIL import Image
from mtc_webui.cpp_mock import cpp_config
from mtc_webui.file_preview import FilePreviewBackend
from mtc_webui.mock_backend import ApiError


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "latest_color.jpg"
        self.now = 100.
        self.backend = FilePreviewBackend(cpp_config('/isolated', 161),
            preview_dir=self.directory.name, clock=lambda: self.now,
            wall_clock=lambda: self.now)
        self.backend.connection_update(connected=True, action_ready=True, child_ready=True)
        stream = BytesIO()
        Image.new('RGB', (16, 12), 'red').save(stream, format='JPEG')
        self.jpeg = stream.getvalue()
        self.path.write_bytes(self.jpeg)
        os.utime(self.path, ns=(100_000_000_000, 100_000_000_000))

    def test_camera_cannot_enable_task_or_fabricate_robot(self):
        result = self.backend.snapshot()
        self.assertTrue(result['camera']['color']['fresh'])
        self.assertFalse(result['robot']['available'])
        self.assertFalse(result['tool']['available'])
        self.assertFalse(result['operations']['submit'])
        self.assertEqual(self.backend.camera_frame('color'), self.jpeg)
        self.assertIn('local-preview:', result['camera']['color']['source'])

    def test_same_file_does_not_become_fresh_again(self):
        self.backend.snapshot()
        self.now += 4
        self.assertFalse(self.backend.snapshot()['camera']['color']['fresh'])
        with self.assertRaises(ApiError): self.backend.camera_frame('color')
        os.utime(self.path, ns=(104_000_000_000, 104_000_000_000))
        self.assertTrue(self.backend.snapshot()['camera']['color']['fresh'])

    def test_partial_or_missing_file_removes_cached_frame(self):
        self.backend.snapshot()
        self.path.write_bytes(self.jpeg[:-20])
        self.assertFalse(self.backend.snapshot()['camera']['color']['available'])
        self.path.unlink()
        self.assertFalse(self.backend.snapshot()['camera']['color']['available'])
        with self.assertRaises(ApiError): self.backend.camera_frame('color')

    def test_old_or_future_file_is_not_fresh_on_first_read(self):
        for stamp in [90_000_000_000, 110_000_000_000]:
            os.utime(self.path, ns=(stamp, stamp))
            self.assertFalse(self.backend.snapshot()['camera']['color']['fresh'])


if __name__ == '__main__':
    unittest.main()
