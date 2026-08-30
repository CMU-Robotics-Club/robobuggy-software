#!/usr/bin/env python3
"""
buggy_lidar.py
--------------
ROS2 node: PointCloud2 messages or saved lidar files → numpy frames →
360° merge (sliding window) → ground segmentation → circle filter →
clustering → ellipse filter → best-cluster identification → publish result.

Key design decisions
--------------------
- Sliding window merging produces a fresh 360° cloud on every incoming frame
  rather than only once every N-frame batch.
- Bounding boxes are recomputed AFTER ellipse filtering so that the index
  passed to identify_best_cluster always matches the cluster list.
- ROS clock is used for all timing and synthetic playback headers so bag
  replay / sim time stays consistent.
- create_cloud_xyz32 receives points.tolist() — numpy arrays are not accepted.
"""

import collections
from pathlib import Path

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from lidar_helpers import (
    ground_plane_segmentation2 as ground_plane_segmentation,
    euclidean_clustering2       as euclidean_clustering,
    filter_points_in_circle,
    filter_clusters,
    identify_best_cluster,
)


class BuggyLidar(Node):

    def __init__(self):
        super().__init__("buggy_lidar")

        # ---------------- Parameters ----------------
        # All tunable values live here so they can be changed live via
        # `ros2 param set /buggy_lidar <name> <value>` without restarting.
        self.declare_parameter("input_topic",             "/velodyne_points")
        self.declare_parameter("input_file",              "")
        self.declare_parameter("playback_rate_hz",        10.0)
        self.declare_parameter("loop_input_file",         False)
        self.declare_parameter("playback_frame_id",       "velodyne")
        self.declare_parameter("frames_to_merge",        4)    # how many partial scans to stack into one 360° cloud
        self.declare_parameter("circle_radius",          8.0)  # metres — discard everything beyond this from the buggy
        self.declare_parameter("ellipse_half_width",     2.0)  # metres — lateral half-axis of the obstacle search zone
        self.declare_parameter("ellipse_half_depth",     5.0)  # metres — forward half-axis of the obstacle search zone
        self.declare_parameter("ellipse_forward_offset", 1.5)  # metres — shift the ellipse centre forward of the buggy
        self.declare_parameter("dbscan_eps",             0.35) # metres — DBSCAN neighbourhood radius
        self.declare_parameter("dbscan_min_points",      20)   # minimum points to form a valid cluster

        frames = self._frames_to_merge()

        # Sliding window — deque auto-evicts the oldest frame on each push.
        # During normal operation every new frame immediately produces a fresh
        # merged cloud once the window is filled, keeping detection rate at the
        # full incoming frame rate instead of once per N-frame batch.
        self.frame_buffer: collections.deque = collections.deque(maxlen=frames)
        self.lidar_sub = None
        self.playback_timer = None
        self.playback_frames = None
        self.playback_index = 0
        self.input_file_path = None
        self._bag_reader = None
        self._bag_deserialize_message = None
        self._bag_message_type = None
        self._last_input_header = None

        # ---------------- QoS ----------------
        # Velodyne driver publishes with BEST_EFFORT reliability.
        # Mismatching this (e.g. using the default RELIABLE) causes ROS2 to
        # silently drop every message — the node runs but receives nothing.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- Publishers ----------------
        self.filtered_pub = self.create_publisher(          # non-ground points inside circle radius — useful for RViz debug
            PointCloud2, "/lidar/filtered", 10
        )
        self.obstacle_cloud_pub = self.create_publisher(    # raw points of the single best obstacle cluster
            PointCloud2, "/lidar/obstacle_cloud", 10
        )
        self.obstacle_centroid_pub = self.create_publisher( # (x, y, z) centroid of the best cluster — consumed by the planner
            PointStamped, "/lidar/obstacle_centroid", 10
        )

        input_file = str(self.get_parameter("input_file").value).strip()
        if input_file:
            self._configure_file_input(input_file)
        else:
            self._configure_topic_input(sensor_qos)

        self.get_logger().info("Buggy LiDAR node ready.")

    def _configure_topic_input(self, sensor_qos: QoSProfile):
        topic = str(self.get_parameter("input_topic").value).strip() or "/velodyne_points"

        # Receives one partial rotation per message from the Velodyne driver.
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            topic,
            self.lidar_callback,
            sensor_qos,
        )
        self.get_logger().info(f"Listening for lidar data on topic: {topic}")

    def _configure_file_input(self, input_file: str):
        path = Path(input_file).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Input file does not exist: {path}")

        self.input_file_path = path
        rate_hz = self._playback_rate_hz()

        if self._is_bag_path(path):
            self._open_bag_reader()
            self.playback_timer = self.create_timer(1.0 / rate_hz, self._play_next_bag_frame)
            self.get_logger().info(
                f"Playing lidar data from bag: {path} "
                f"(topic={self._bag_topic()}, rate={rate_hz:.2f} Hz)"
            )
            return

        self.playback_frames = self._load_frames_from_file(path)
        if not self.playback_frames:
            raise ValueError(f"No lidar frames found in file: {path}")

        self.playback_timer = self.create_timer(1.0 / rate_hz, self._play_next_loaded_frame)
        self.get_logger().info(
            f"Playing lidar data from file: {path} "
            f"({len(self.playback_frames)} frames @ {rate_hz:.2f} Hz)"
        )

    def _is_bag_path(self, path: Path) -> bool:
        suffix = path.suffix.lower()
        if path.is_dir():
            return (path / "metadata.yaml").exists()
        return suffix in {".mcap", ".db3"}

    def _bag_topic(self) -> str:
        return str(self.get_parameter("input_topic").value).strip() or "/velodyne_points"

    def _frames_to_merge(self) -> int:
        frames = int(self.get_parameter("frames_to_merge").value)
        if frames < 1:
            self.get_logger().warn("frames_to_merge must be >= 1. Using 1 instead.")
            return 1
        return frames

    def _playback_rate_hz(self) -> float:
        rate = float(self.get_parameter("playback_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("playback_rate_hz must be > 0")
        return rate

    def _sync_frame_buffer(self) -> int:
        frames_needed = self._frames_to_merge()
        if self.frame_buffer.maxlen != frames_needed:
            self.frame_buffer = collections.deque(self.frame_buffer, maxlen=frames_needed)
        return frames_needed

    def _reset_frame_buffer(self):
        self.frame_buffer = collections.deque(maxlen=self._frames_to_merge())

    def _make_playback_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("playback_frame_id").value).strip() or "velodyne"
        return header

    def _load_frames_from_file(self, path: Path):
        suffix = path.suffix.lower()

        if suffix == ".npz":
            with np.load(path, allow_pickle=True) as file_data:
                if "frames" in file_data.files:
                    raw_frames = file_data["frames"]
                elif "points" in file_data.files:
                    raw_frames = file_data["points"]
                elif len(file_data.files) == 1:
                    raw_frames = file_data[file_data.files[0]]
                else:
                    raise KeyError(
                        f"{path} must contain a 'frames' array or a single point-cloud array"
                    )
        elif suffix == ".npy":
            raw_frames = np.load(path, allow_pickle=True)
        else:
            raise ValueError(
                f"Unsupported input file type: {path.suffix}. "
                "Use .npz/.npy for array playback or .mcap/.db3 for ROS bags."
            )

        return self._normalise_loaded_frames(raw_frames, str(path))

    def _normalise_loaded_frames(self, raw_frames, source: str):
        array = np.asarray(raw_frames)

        if array.ndim == 2 and array.dtype != object:
            return [self._ensure_xyz(array, source)]

        if array.ndim == 3 and array.dtype != object:
            return [self._ensure_xyz(frame, f"{source}[{idx}]") for idx, frame in enumerate(array)]

        if array.dtype == object:
            frames = []
            for idx, frame in enumerate(np.ravel(array)):
                frames.append(self._ensure_xyz(frame, f"{source}[{idx}]"))
            return frames

        raise ValueError(
            f"Unsupported lidar array shape from {source}: {array.shape}. "
            "Expected (N, 3+), (F, N, 3+), or an object array of frames."
        )

    def _ensure_xyz(self, frame, source: str) -> np.ndarray:
        array = np.asarray(frame)
        if array.ndim != 2 or array.shape[1] < 3:
            raise ValueError(f"Expected {source} to have shape (N, 3+), got {array.shape}")
        xyz = array[:, :3]
        valid_mask = np.isfinite(xyz).all(axis=1)
        return xyz[valid_mask].astype(np.float32, copy=False)

    def _open_bag_reader(self):
        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except ImportError as exc:
            raise RuntimeError(
                "Bag playback requires rosbag2_py, rclpy.serialization, and rosidl_runtime_py."
            ) from exc

        storage_id = self._bag_storage_id()
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=str(self.input_file_path),
            storage_id=storage_id,
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader.open(storage_options, converter_options)

        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        bag_topic = self._bag_topic()
        if bag_topic not in topic_types:
            available = ", ".join(sorted(topic_types)) or "<none>"
            raise RuntimeError(
                f"Topic {bag_topic} not found in bag {self.input_file_path}. "
                f"Available topics: {available}"
            )

        self._bag_reader = reader
        self._bag_deserialize_message = deserialize_message
        self._bag_message_type = get_message(topic_types[bag_topic])

    def _bag_storage_id(self) -> str:
        metadata_path = self.input_file_path / "metadata.yaml" if self.input_file_path.is_dir() else None
        if metadata_path and metadata_path.exists():
            for line in metadata_path.read_text(encoding="utf-8").splitlines():
                stripped = line.strip()
                if stripped.startswith("storage_identifier:"):
                    storage_id = stripped.split(":", 1)[1].strip()
                    if storage_id:
                        return storage_id

        if self.input_file_path.suffix.lower() == ".mcap":
            return "mcap"
        return "sqlite3"

    def _restart_file_playback(self, message: str):
        self.playback_index = 0
        self._reset_frame_buffer()
        self.get_logger().info(message)

    def _flush_partial_buffer(self):
        frames_needed = self._frames_to_merge()
        if not (0 < len(self.frame_buffer) < frames_needed):
            return

        header = self._last_input_header or self._make_playback_header()
        self.get_logger().info(
            f"Flushing final partial scan {len(self.frame_buffer)}/{frames_needed} frames from playback."
        )
        self.run_pipeline(np.vstack(self.frame_buffer), header)

    def _play_next_loaded_frame(self):
        if self.playback_frames is None:
            return

        if self.playback_index >= len(self.playback_frames):
            self._flush_partial_buffer()
            if bool(self.get_parameter("loop_input_file").value):
                self._restart_file_playback("Restarting lidar file playback from the beginning.")
            else:
                self.get_logger().info("Reached end of lidar input file. Playback stopped.")
                self.playback_timer.cancel()
                return

        frame = self.playback_frames[self.playback_index]
        self.playback_index += 1
        self.process_frame(frame, self._make_playback_header())

    def _play_next_bag_frame(self):
        if self._bag_reader is None:
            return

        bag_topic = self._bag_topic()
        while self._bag_reader.has_next():
            topic, data, _ = self._bag_reader.read_next()
            if topic != bag_topic:
                continue
            msg = self._bag_deserialize_message(data, self._bag_message_type)
            self.lidar_callback(msg)
            return

        self._flush_partial_buffer()
        if bool(self.get_parameter("loop_input_file").value):
            self._reset_frame_buffer()
            self._open_bag_reader()
            self.get_logger().info("Restarting lidar bag playback from the beginning.")
        else:
            self.get_logger().info("Reached end of lidar bag. Playback stopped.")
            self.playback_timer.cancel()

    # ---------------------------------------------------
    # Lidar Callback
    # Fires on every incoming PointCloud2 message (~10 Hz).
    # Converts the ROS message to numpy, slides it into the frame
    # buffer, and triggers the pipeline once the buffer is full.
    # ---------------------------------------------------
    def lidar_callback(self, msg: PointCloud2):
        t_cb = self.get_clock().now()

        # Deserialise PointCloud2 → (N, 3) float32 numpy array.
        # skip_nans drops invalid returns (e.g. out-of-range or intensity-only points).
        frame = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if frame.size == 0:
            return

        self.process_frame(frame, msg.header)

        dt_cb = (self.get_clock().now() - t_cb).nanoseconds / 1e6
        self.get_logger().info(
            f"Total callback time: {dt_cb:.1f} ms",
            throttle_duration_sec=1.0,
        )

    def process_frame(self, frame: np.ndarray, header):
        if frame.size == 0:
            return

        frames_needed = self._sync_frame_buffer()
        self._last_input_header = header

        # .copy() is required — without it the deque holds a view into the
        # ROS message buffer which may be overwritten before we vstack it.
        self.frame_buffer.append(frame.copy())

        # Hold off until the sliding window is fully populated.
        # After the first fill this condition is never true again.
        if len(self.frame_buffer) < frames_needed:
            self.get_logger().info(
                f"Buffering frames {len(self.frame_buffer)}/{frames_needed}",
                throttle_duration_sec=2.0,
            )
            return

        # Stack all N frames row-wise → one (N_total, 3) array representing
        # a complete 360° scan ready for the perception pipeline.
        combined = np.vstack(self.frame_buffer)   # shape: (N_total, 3)

        self.run_pipeline(combined, header)

    # ---------------------------------------------------
    # Main Pipeline
    # Runs the full perception stack on a merged 360° cloud.
    # Each stage feeds its output directly into the next.
    # Parameters are re-read every cycle so live tuning takes effect
    # immediately without restarting the node.
    # ---------------------------------------------------
    def run_pipeline(self, data: np.ndarray, header):
        t0 = self.get_clock().now()

        radius     = self.get_parameter("circle_radius").value
        half_width = self.get_parameter("ellipse_half_width").value
        half_depth = self.get_parameter("ellipse_half_depth").value
        fwd_offset = self.get_parameter("ellipse_forward_offset").value
        eps        = self.get_parameter("dbscan_eps").value
        min_pts    = self.get_parameter("dbscan_min_points").value

        # -------- Stage 1: Ground Segmentation --------
        # Open3D RANSAC fits a plane to the lowest points and splits the cloud
        # into ground (inliers) and non-ground (everything above the plane).
        # ~20-80 ms depending on point density — the pipeline's most expensive step.
        try:
            _ground, nonground = ground_plane_segmentation(data)
        except Exception as e:
            self.get_logger().warn(f"Ground segmentation failed: {e}")
            return

        # -------- Stage 2: Circle Filter --------
        # Discard non-ground points beyond `circle_radius` metres in XY.
        # Removes distant noise and keeps the DBSCAN neighbourhood density
        # uniform, which improves cluster quality.
        ng_circle = filter_points_in_circle(nonground, radius=radius)

        # Publish the filtered cloud so RViz / downstream nodes can visualise
        # what the pipeline is actually working with.
        self.publish_cloud(ng_circle, header, self.filtered_pub)

        if len(ng_circle) < min_pts:
            self.get_logger().warn("Too few points after circle filter.")
            return

        # -------- Stage 3: Clustering --------
        # Voxel-downsampled DBSCAN groups nearby non-ground points into
        # candidate obstacle clusters. Returns only clusters that pass
        # volume and height sanity checks inside euclidean_clustering2.
        try:
            clusters, _boxes, _labels = euclidean_clustering(
                ng_circle, eps=eps, min_points=min_pts
            )
        except Exception as e:
            self.get_logger().warn(f"Clustering failed: {e}")
            return

        # -------- Stage 4: Ellipse Filter --------
        # Keeps only clusters whose centroid falls inside a forward-biased
        # ellipse in front of the buggy — ignores obstacles behind or far
        # to the sides that we don't need to react to.
        ranged_clusters = filter_clusters(
            clusters,
            half_width=half_width,
            half_depth=half_depth,
            forward_offset=fwd_offset,
        )

        if not ranged_clusters:
            self.get_logger().info("No clusters in ellipse.", throttle_duration_sec=1.0)
            return

        # -------- Stage 5: Recompute boxes for the FILTERED subset --------
        # The boxes returned by euclidean_clustering cover ALL clusters before
        # ellipse filtering. After filter_clusters removes entries, those box
        # indices no longer align with ranged_clusters — passing them directly
        # to identify_best_cluster would score the wrong geometry per cluster.
        # We rebuild boxes only for the clusters that survived Stage 4.
        ranged_boxes = []
        for c in ranged_clusters:
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(c)
            ranged_boxes.append(pc.get_axis_aligned_bounding_box())

        # -------- Stage 6: Best Cluster --------
        # Scores each surviving cluster by distance, forward angle, and size.
        # Returns the single most-likely obstacle, or None if all scores
        # exceed the maximum reasonable threshold defined in lidar_helpers.
        best = identify_best_cluster(ranged_clusters, ranged_boxes)

        if best is not None:
            self.publish_cloud(best, header, self.obstacle_cloud_pub)
            centroid = np.mean(best, axis=0)   # (x, y, z) centre of mass of the cluster
            self.publish_centroid(centroid, header)
            self.get_logger().info(
                f"Obstacle @ ({centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f}) m"
                f"  |  {len(best)} pts",
                throttle_duration_sec=0.5,
            )
        else:
            self.get_logger().info("No obstacle detected.", throttle_duration_sec=1.0)

        dt = (self.get_clock().now() - t0).nanoseconds / 1e6
        self.get_logger().info(
            f"Pipeline | raw={len(data)}  ng={len(nonground)}  "
            f"circle={len(ng_circle)}  clusters={len(clusters)}  "
            f"ranged={len(ranged_clusters)}  {dt:.1f} ms",
            throttle_duration_sec=1.0,
        )

    # ---------------------------------------------------
    # Publish Helpers
    # ---------------------------------------------------
    def publish_cloud(self, points: np.ndarray, header, publisher):
        """Wraps a (N, 3) numpy array as a PointCloud2 message and publishes it.
        The header is reused from the original Velodyne message so timestamp
        and frame_id stay consistent with the rest of the ROS graph."""
        if len(points) == 0:
            return
        # create_cloud_xyz32 requires a Python list of (x, y, z) tuples —
        # passing a raw numpy array causes a type error in some ROS2 distros.
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points.tolist())
        publisher.publish(cloud_msg)

    def publish_centroid(self, centroid: np.ndarray, header):
        """Publishes the (x, y, z) centre of mass of the best obstacle cluster
        as a PointStamped. This is the primary output consumed by the planner —
        a single coordinate is much lighter than a full point cloud."""
        msg = PointStamped()
        msg.header = header
        msg.point.x = float(centroid[0])
        msg.point.y = float(centroid[1])
        msg.point.z = float(centroid[2])
        self.obstacle_centroid_pub.publish(msg)


# ---------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = BuggyLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
