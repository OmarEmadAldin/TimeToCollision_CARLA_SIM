import os
import cv2
import config
class VideoRecorder:
    def __init__(self, output_path=None, fps=None):

        self.output_path = output_path
        self.fps = fps or config.OUTPUT_VIDEO_FPS
        self._writer = None
        self._frame_count = 0

        if self.output_path is not None:
            out_dir = os.path.dirname(self.output_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)

    @property
    def enabled(self):
        return self.output_path is not None

    def write(self, frame):
        """Write a single BGR frame. No-op if recording is disabled."""
        if not self.enabled:
            return

        if self._writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
            print(f"[VideoRecorder] Recording to {self.output_path} at {self.fps} FPS...")

        self._writer.write(frame)
        self._frame_count += 1

    def release(self):
        """Finalize and close the video file. Safe to call multiple times."""
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            print(f"[VideoRecorder] Saved {self._frame_count} frames to {self.output_path}")

    @property
    def frame_count(self):
        return self._frame_count


def default_output_path(filename=None):

    filename = filename or config.OUTPUT_VIDEO_FILENAME
    return os.path.join(config.OUTPUT_DIR, filename)