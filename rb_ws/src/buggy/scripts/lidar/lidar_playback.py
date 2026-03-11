import argparse
from datetime import datetime
import time

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

# GLFW fallback key codes for arrow keys.
KEY_LEFT = 263
KEY_RIGHT = 262


def parse_args():
    parser = argparse.ArgumentParser(
        description="Play merged lidar frames in Open3D with media-style controls."
    )
    parser.add_argument(
        "--input",
        default="sc_feb_21_26_roll_1.npz",
        help="NPZ file containing a 'frames' array.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=2.0,
        help="Open3D point size.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Playback framerate in merged frames per second.",
    )
    parser.add_argument(
        "--merge-size",
        type=int,
        default=4,
        help="Number of consecutive raw frames to merge into one displayed frame.",
    )
    parser.add_argument(
        "--skip-size",
        type=int,
        default=30,
        help="Number of merged frames to jump on skip backward/forward.",
    )
    return parser.parse_args()


def ensure_xyz(frame):
    arr = np.asarray(frame)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError(f"Expected frame with shape (N, >=3), got {arr.shape}")
    return arr[:, :3].astype(np.float64, copy=False)


def get_key_down_constant():
    if hasattr(gui.KeyEvent, "DOWN"):
        return gui.KeyEvent.DOWN
    if hasattr(gui.KeyEvent, "Type") and hasattr(gui.KeyEvent.Type, "DOWN"):
        return gui.KeyEvent.Type.DOWN
    return None


def key_matches(event, key_name, fallback_char=None, fallback_code=None):
    if key_name is not None:
        key_enum = getattr(gui.KeyName, key_name, None)
        if key_enum is not None and event.key == key_enum:
            return True
    if fallback_char is not None and event.key == ord(fallback_char):
        return True
    if fallback_code is not None and event.key == fallback_code:
        return True
    return False


class LidarPlaybackApp:
    def __init__(self, args):
        self.args = args
        self.file_data = np.load(args.input, allow_pickle=True)
        if "frames" not in self.file_data:
            raise KeyError(f"{args.input} does not contain a 'frames' array")
        self.frames = self.file_data["frames"]
        if len(self.frames) == 0:
            raise ValueError(f"{args.input} has no frames")
        if args.merge_size < 1:
            raise ValueError("--merge-size must be >= 1")
        if args.fps <= 0:
            raise ValueError("--fps must be > 0")
        if args.skip_size < 1:
            raise ValueError("--skip-size must be >= 1")

        self.timestamps = None
        if "timestamps" in self.file_data and len(self.file_data["timestamps"]) == len(self.frames):
            self.timestamps = np.asarray(self.file_data["timestamps"], dtype=np.float64)

        self.merge_size = int(args.merge_size)
        self.skip_size = int(args.skip_size)
        self.merged_frame_count = (len(self.frames) + self.merge_size - 1) // self.merge_size
        self.merged_cache = {}
        self.state = {"idx": 0, "paused": False, "direction": 1}

        app = gui.Application.instance
        self.window = app.create_window("Lidar Playback", 1280, 720)
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene_widget.scene.set_background([0.02, 0.02, 0.02, 1.0])
        self.window.add_child(self.scene_widget)

        self.status_label = gui.Label("")
        self.manual_label = gui.Label(self.controls_text())
        self.window.add_child(self.status_label)
        self.window.add_child(self.manual_label)

        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"
        self.material.point_size = float(args.point_size)

        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(self.merged_frame(0))
        self.pcd.paint_uniform_color([0.1, 0.85, 1.0])
        self.scene_widget.scene.add_geometry("points", self.pcd, self.material)
        bounds = self.pcd.get_axis_aligned_bounding_box()
        self.scene_widget.setup_camera(60.0, bounds, bounds.get_center())

        self.window.set_on_layout(self.on_layout)
        self.window.set_on_key(self.on_key)
        if hasattr(self.window, "set_on_tick_event"):
            self.window.set_on_tick_event(self.on_tick)
        else:
            raise RuntimeError("Open3D GUI in this environment does not support tick events")

        self.target_dt = 1.0 / float(self.args.fps)
        self.next_tick = time.perf_counter() + self.target_dt
        self.update_frame(0)

    def controls_text(self):
        return (
            "Controls\n"
            "Space: Play/Pause\n"
            "F: Play Forward\n"
            "B: Play Reverse\n"
            "Left/J: Skip Back\n"
            "Right/L: Skip Forward\n"
            "R: Rewind + Pause\n"
            "Q or Esc: Quit"
        )

    def merged_frame(self, block_idx):
        if block_idx in self.merged_cache:
            return self.merged_cache[block_idx]
        start = block_idx * self.merge_size
        end = min(start + self.merge_size, len(self.frames))
        merged = np.vstack([ensure_xyz(self.frames[i]) for i in range(start, end)])
        self.merged_cache[block_idx] = merged
        return merged

    def frame_timestamp_text(self, merged_idx):
        if self.timestamps is not None:
            start = merged_idx * self.merge_size
            end = min(start + self.merge_size, len(self.frames))
            ts0 = float(self.timestamps[start])
            ts1 = float(self.timestamps[end - 1])
            t0 = datetime.fromtimestamp(ts0).strftime("%H:%M:%S.%f")[:-3]
            t1 = datetime.fromtimestamp(ts1).strftime("%H:%M:%S.%f")[:-3]
            return f"ts {t0} -> {t1}"
        simulated = merged_idx / float(self.args.fps)
        return f"playback t={simulated:0.2f}s"

    def update_status(self):
        idx = self.state["idx"]
        start_raw = idx * self.merge_size + 1
        end_raw = min((idx + 1) * self.merge_size, len(self.frames))
        mode = "PAUSED" if self.state["paused"] else "PLAY"
        direction = ">>" if self.state["direction"] > 0 else "<<"
        self.status_label.text = (
            f"{mode} {direction}  "
            f"Merged {idx + 1}/{self.merged_frame_count}  "
            f"Raw {start_raw}-{end_raw}  "
            f"{self.frame_timestamp_text(idx)}"
        )
        self.window.set_needs_layout()

    def update_frame(self, index):
        clamped_idx = max(0, min(index, self.merged_frame_count - 1))
        self.state["idx"] = clamped_idx
        self.pcd.points = o3d.utility.Vector3dVector(self.merged_frame(clamped_idx))
        self.scene_widget.scene.remove_geometry("points")
        self.scene_widget.scene.add_geometry("points", self.pcd, self.material)
        self.update_status()

    def step(self, delta):
        self.update_frame(self.state["idx"] + delta)

    def on_layout(self, layout_context):
        rect = self.window.content_rect
        self.scene_widget.frame = rect

        em = layout_context.theme.font_size
        pad = int(0.5 * em)
        constraints = gui.Widget.Constraints()

        status_pref = self.status_label.calc_preferred_size(layout_context, constraints)
        self.status_label.frame = gui.Rect(
            rect.x + pad,
            rect.y + pad,
            status_pref.width,
            status_pref.height,
        )

        manual_pref = self.manual_label.calc_preferred_size(layout_context, constraints)
        self.manual_label.frame = gui.Rect(
            rect.x + rect.width - manual_pref.width - pad,
            rect.y + pad,
            manual_pref.width,
            manual_pref.height,
        )

    def on_key(self, event):
        key_down = get_key_down_constant()
        if key_down is not None and event.type != key_down:
            return False

        if key_matches(event, "SPACE", " "):
            self.state["paused"] = not self.state["paused"]
            self.update_status()
            return True
        if key_matches(event, "F", "F"):
            self.state["direction"] = 1
            self.state["paused"] = False
            self.update_status()
            return True
        if key_matches(event, "B", "B"):
            self.state["direction"] = -1
            self.state["paused"] = False
            self.update_status()
            return True
        if (
            key_matches(event, "LEFT", fallback_code=KEY_LEFT)
            or key_matches(event, "LEFT_ARROW")
            or key_matches(event, "J", "J")
        ):
            self.step(-self.skip_size)
            return True
        if (
            key_matches(event, "RIGHT", fallback_code=KEY_RIGHT)
            or key_matches(event, "RIGHT_ARROW")
            or key_matches(event, "L", "L")
        ):
            self.step(self.skip_size)
            return True
        if key_matches(event, "R", "R"):
            self.state["paused"] = True
            self.state["direction"] = 1
            self.update_frame(0)
            return True
        if key_matches(event, "Q", "Q") or key_matches(event, "ESCAPE"):
            self.window.close()
            return True

        return False

    def on_tick(self):
        now = time.perf_counter()
        if now < self.next_tick:
            return False

        self.next_tick = now + self.target_dt
        if self.state["paused"]:
            return False

        next_idx = self.state["idx"] + self.state["direction"]
        if next_idx < 0 or next_idx >= self.merged_frame_count:
            self.state["paused"] = True
            self.update_status()
            return True

        self.update_frame(next_idx)
        return True


def main():
    args = parse_args()
    app = gui.Application.instance
    app.initialize()
    LidarPlaybackApp(args)
    app.run()


if __name__ == "__main__":
    main()
