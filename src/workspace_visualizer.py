#!/usr/bin/env python3
import rclpy, numpy as np, pathlib
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


class WSViewer(Node):
    def __init__(self):
        super().__init__("ws_viewer")

        # ---------- parameters ----------
        self.declare_parameter("txt_path", "my_WS_file_6.txt")
        self.declare_parameter("frame_id", "regulated/link0")
        self.declare_parameter(
            "z_layers", [-0.5, -0.45, -0.4, -0.35, -0.3, -0.25, -0.2]
        )
        self.declare_parameter("slice_thickness", 0.05)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("reduce_mode", "avg")
        voxel = float(self.get_parameter("voxel_size").value)
        mode = str(self.get_parameter("reduce_mode").value)

        txt_path = pathlib.Path(self.get_parameter("txt_path").value)
        frame_id = self.get_parameter("frame_id").value
        z_layers = np.asarray(self.get_parameter("z_layers").value, dtype=float)
        dz = float(self.get_parameter("slice_thickness").value)

        # ---------- load workspace ----------
        xyz, yoshi = self._load_ws(txt_path)
        xyz_vox, yoshi_vox = self._voxelize(xyz, yoshi, voxel, mode)

        self.y_lo, self.y_hi = np.percentile(yoshi_vox, [1, 99])
        # self.y_lo, self.y_hi = yoshi.min(), yoshi.max()

        # ---------- build clouds & publishers ----------
        self.slice_clouds, self.slice_pubs = [], []
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        for i, z0 in enumerate(z_layers):
            mask = np.abs(xyz_vox[:, 2] - z0) < dz / 2.0
            if not np.any(mask):
                self.get_logger().warn(f"No points near z={z0:+.2f}")
                continue

            # make same height
            xyz_vox[mask, 2] = z0
            cloud_i = self._make_cloud(
                xyz_vox[mask], yoshi_vox[mask], frame_id, self.y_lo, self.y_hi
            )
            self.slice_clouds.append(cloud_i)

            topic = f"workspace_cloud_Z{i}"
            pub = self.create_publisher(PointCloud2, topic, qos)
            self.slice_pubs.append(pub)

            self.get_logger().info(
                f"Slice {i}: z={z0:+.2f} → {mask.sum()} pts  topic={topic}"
            )

        # ---------- periodic publish ----------
        self.create_timer(1.0, self._timer_cb)

    # --------------------------------------------------
    def _timer_cb(self):
        stamp = self.get_clock().now().to_msg()
        for cloud, pub in zip(self.slice_clouds, self.slice_pubs):
            cloud.header.stamp = stamp
            pub.publish(cloud)

    def _voxelize(self, xyz, yoshi, voxel, mode="avg"):
        xyz_min = xyz.min(axis=0)
        idx = np.floor((xyz - xyz_min) / voxel).astype(np.int32)

        mult = np.array([1, idx[:, 0].max() + 1, 0], dtype=np.int64)
        mult[2] = mult[1] * (idx[:, 1].max() + 1)
        key = (idx * mult).sum(axis=1)

        order = np.argsort(key)
        key_sorted = key[order]
        xyz_sorted, y_sorted = xyz[order], yoshi[order]

        bounds = np.flatnonzero(np.diff(key_sorted)) + 1
        splits = np.split(np.arange(len(key_sorted)), bounds)

        xyz_centers, y_red = [], []
        for g in splits:
            pts = xyz_sorted[g]
            yv = y_sorted[g]
            xyz_centers.append(pts.mean(axis=0))  # voxel center
            y_red.append(yv.mean() if mode == "avg" else yv.max())

        return np.vstack(xyz_centers).astype(np.float32), np.asarray(y_red, np.float32)

    # -------------- file loader -----------------------
    def _load_ws(self, path):
        xs, ys, zs, yv = [], [], [], []
        with open(path) as f:
            for ln in f:
                tok = [t for t in ln.rstrip("\n").split("\t") if t]
                if len(tok) < 7:
                    continue
                try:
                    xs.append(float(tok[0]))
                    ys.append(float(tok[1]))
                    zs.append(float(tok[2]))
                    yv.append(float(tok[6]))
                except ValueError:
                    continue
        xyz = np.column_stack((xs, ys, zs)).astype(np.float32)
        yoshi = np.asarray(yv, dtype=np.float32)
        return xyz, yoshi

    # -------------- cloud builder --------------------
    def _make_cloud(self, xyz, yoshi, frame_id, y_lo, y_hi):
        # percentile stretch for better contrast
        # p1, p99 = np.percentile(yoshi, [1, 99])
        # t = np.clip((yoshi - p1) / (p99 - p1 + 1e-12), 0.0, 1.0)
        y_lo, y_hi = self.y_lo, self.y_hi
        t = np.clip((yoshi - y_lo) / (y_hi - y_lo + 1e-12), 0.0, 1.0)

        def jet(v):
            f4 = 4 * v
            r = np.clip(f4 - 1.5, 0, 1) - np.clip(f4 - 3.5, 0, 1)
            g = np.clip(f4 - 0.5, 0, 1) - np.clip(f4 - 2.5, 0, 1)
            b = np.clip(f4 + 0.5, 0, 1) - np.clip(f4 - 1.5, 0, 1)
            return (r * 255, g * 255, b * 255)

        rgb = np.asarray([jet(v) for v in t], dtype=np.uint32)
        rgb32 = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
        rgbf = rgb32.view(np.float32)

        pts = np.column_stack((xyz, rgbf))

        fields = []
        for nm, off in zip(("x", "y", "z", "rgb"), (0, 4, 8, 12)):
            pf = PointField()
            pf.name, pf.offset = nm, off
            pf.datatype, pf.count = PointField.FLOAT32, 1
            fields.append(pf)

        hdr = Header()
        hdr.frame_id = frame_id
        return pc2.create_cloud(hdr, fields, pts)


def main():
    rclpy.init()
    viewer = WSViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
