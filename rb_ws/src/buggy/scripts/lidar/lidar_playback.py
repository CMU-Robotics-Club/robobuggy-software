import argparse
from contextlib import redirect_stdout
from datetime import datetime
import io
import time

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

from lidar_visualizer import (
    euclidean_clustering,
    ground_plane_segmentation,
    filter_points_in_circle,
    filter_points_by_angle,
    create_grid
)

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
        default=5.0,
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
        default=50,
        help="Number of merged frames to jump on skip backward/forward.",
    )
    parser.add_argument(
        "--cluster-eps",
        type=float,
        default=0.35,
        help="DBSCAN epsilon for euclidean clustering.",
    )
    parser.add_argument(
        "--cluster-min-points",
        type=int,
        default=20,
        help="DBSCAN min_points for euclidean clustering.",
    )
    parser.add_argument(
        "--cluster-min-height",
        type=float,
        default=0.5,
        help="Reject clusters whose bounding-box height is below this value (meters).",
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
        if args.cluster_eps <= 0:
            raise ValueError("--cluster-eps must be > 0")
        if args.cluster_min_points < 1:
            raise ValueError("--cluster-min-points must be >= 1")
        if args.cluster_min_height < 0:
            raise ValueError("--cluster-min-height must be >= 0")

        self.timestamps = None
        if "timestamps" in self.file_data and len(self.file_data["timestamps"]) == len(self.frames):
            self.timestamps = np.asarray(self.file_data["timestamps"], dtype=np.float64)

        self.merge_size = int(args.merge_size)
        self.skip_size = int(args.skip_size)
        self.cluster_eps = float(args.cluster_eps)
        self.cluster_min_points = int(args.cluster_min_points)
        self.cluster_min_height = float(args.cluster_min_height)
        self.merged_frame_count = (len(self.frames) + self.merge_size - 1) // self.merge_size
        self.merged_cache = {}
        self.state = {
            "idx": 0,
            "paused": False,
            "direction": 1,
            "cluster_count": 0,
            "cluster_point_count": 0,
        }

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

        self.ground_material = rendering.MaterialRecord()
        self.ground_material.shader = "defaultUnlit"
        self.ground_material.point_size = float(args.point_size)
        self.ground_material.base_color = [1.0, 0.0, 0.0, 1.0]

        self.non_ground_material = rendering.MaterialRecord()
        self.non_ground_material.shader = "defaultUnlit"
        self.non_ground_material.point_size = float(args.point_size)
        self.non_ground_material.base_color = [1.0, 1.0, 1.0, 1.0]

        self.cluster_points_material = rendering.MaterialRecord()
        self.cluster_points_material.shader = "defaultUnlit"
        self.cluster_points_material.point_size = float(args.point_size) + 1.5
        self.cluster_points_material.base_color = [0.1, 1.0, 0.1, 1.0]

        self.cluster_box_material = rendering.MaterialRecord()
        self.cluster_box_material.shader = "unlitLine"
        if hasattr(self.cluster_box_material, "line_width"):
            self.cluster_box_material.line_width = 2.0
        self.cluster_box_material.base_color = [0.1, 1.0, 0.1, 1.0]

        self.axes_material = rendering.MaterialRecord()
        self.axes_material.shader = "defaultUnlit"

        self.grid_material = rendering.MaterialRecord()
        self.grid_material.shader = "unlitLine"
        if hasattr(self.grid_material, "line_width"):
            self.grid_material.line_width = 1.0

        initial_frame = self.merged_frame(0)
        init_ground, init_non_ground = ground_plane_segmentation(initial_frame.copy())

        self.g_pcd = o3d.geometry.PointCloud()
        self.g_pcd.points = o3d.utility.Vector3dVector(init_ground)
        self.ng_pcd = o3d.geometry.PointCloud()
        self.ng_pcd.points = o3d.utility.Vector3dVector(init_non_ground)
        self.cluster_pcd = o3d.geometry.PointCloud()
        self.cluster_pcd.points = o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
        self.cluster_box_names = []
        self.scene_widget.scene.add_geometry("ground_points", self.g_pcd, self.ground_material)
        self.scene_widget.scene.add_geometry("non_ground_points", self.ng_pcd, self.non_ground_material)
        self.scene_widget.scene.add_geometry("cluster_points", self.cluster_pcd, self.cluster_points_material)

        center = np.array([0.0, 0.0, -0.5])
        up = np.array([0.0, 0.0, 1.0])      
        view = np.array([1.0, -1.0, 0.5])

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=center)
        grid = create_grid(size=15, n=15, z=center[2])
        self.scene_widget.scene.add_geometry("axes", axes, self.axes_material)
        self.scene_widget.scene.add_geometry("grid", grid, self.grid_material)

        init_bounds_pcd = o3d.geometry.PointCloud()
        init_bounds_pcd.points = o3d.utility.Vector3dVector(initial_frame)
        bounds = init_bounds_pcd.get_axis_aligned_bounding_box()
        self.scene_widget.setup_camera(60.0, bounds, np.array(center))   # center the camera at world origin
        eye = center + (view / np.linalg.norm(view) * 5.0)
        self.scene_widget.look_at(center, eye, up)

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
            f"Clusters {self.state['cluster_count']} ({self.state['cluster_point_count']} pts)  "
            f"{self.frame_timestamp_text(idx)}"
        )
        self.window.set_needs_layout()

    def update_frame(self, index):
        clamped_idx = max(0, min(index, self.merged_frame_count - 1))
        self.state["idx"] = clamped_idx
        full_frame = self.merged_frame(clamped_idx)
        try:
            g, ng_clean = ground_plane_segmentation(full_frame.copy())
        except Exception:
            g = np.empty((0, 3), dtype=np.float64)
            ng_clean = full_frame

        ng_filtered = filter_points_in_circle(ng_clean, radius=8)
        # ng_filtered = filter_points_by_angle(ng_filtered, min_angle=17/16*np.pi, max_angle=0)   # filter out left side due to left-passing behavior; ignores pushers on left side

        clusters = []
        boxes = []
        if len(ng_filtered) >= self.cluster_min_points:
            try:
                with redirect_stdout(io.StringIO()):
                    clusters, boxes, _ = euclidean_clustering(
                        ng_filtered,
                        eps=self.cluster_eps,
                        min_points=self.cluster_min_points,
                        min_height=self.cluster_min_height,
                    )
            except Exception:
                clusters = []
                boxes = []
        self.state["cluster_count"] = len(boxes)
        if clusters:
            cluster_points = np.vstack(clusters)
        else:
            cluster_points = np.empty((0, 3), dtype=np.float64)
        self.state["cluster_point_count"] = len(cluster_points)

        self.g_pcd.points = o3d.utility.Vector3dVector(g)
        self.ng_pcd.points = o3d.utility.Vector3dVector(ng_clean)
        self.cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
        self.scene_widget.scene.remove_geometry("ground_points")
        self.scene_widget.scene.remove_geometry("non_ground_points")
        self.scene_widget.scene.remove_geometry("cluster_points")
        self.scene_widget.scene.add_geometry("ground_points", self.g_pcd, self.ground_material)
        self.scene_widget.scene.add_geometry("non_ground_points", self.ng_pcd, self.non_ground_material)
        self.scene_widget.scene.add_geometry("cluster_points", self.cluster_pcd, self.cluster_points_material)

        for name in self.cluster_box_names:
            self.scene_widget.scene.remove_geometry(name)
        self.cluster_box_names = []
        for i, box in enumerate(boxes):
            name = f"cluster_box_{i}"
            lines = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(box)
            line_count = len(lines.lines)
            lines.colors = o3d.utility.Vector3dVector(
                np.tile(np.array([[0.1, 1.0, 0.1]]), (line_count, 1))
            )
            self.scene_widget.scene.add_geometry(name, lines, self.cluster_box_material)
            self.cluster_box_names.append(name)

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
