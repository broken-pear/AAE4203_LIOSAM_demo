"""离线单元测试：验证 sensor_prep.py 里最容易出错的两块

1) _read_xyz(): 按 offsets 用 numpy 从 PointCloud2 原始 buffer 里读出 x/y/z/intensity
2) 输出打包：point_step=24 / ring@16(UINT16) / time@20(FLOAT32) 的布局，
   以及 ring / time 的计算结果是否正确

不需要 ROS 环境：rclpy 和 sensor_msgs 都被打了桩。
"""
import importlib.util
import math
import os
import sys
import types

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------- 打桩 rclpy
rclpy = types.ModuleType("rclpy")
node_mod = types.ModuleType("rclpy.node")
qos_mod = types.ModuleType("rclpy.qos")


class _Node:
    def __init__(self, *a, **k):
        pass


class _QoSProfile:
    def __init__(self, *a, **k):
        pass


node_mod.Node = _Node
qos_mod.QoSProfile = _QoSProfile
qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
rclpy.node = node_mod
rclpy.qos = qos_mod

sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")


class _PointField:
    INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = 1, 2, 3, 4, 5, 6, 7, 8

    def __init__(self, name="", offset=0, datatype=0, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class _Msg:
    pass


sensor_msgs_msg.PointField = _PointField
sensor_msgs_msg.Imu = _Msg
sensor_msgs_msg.PointCloud2 = _Msg
sensor_msgs.msg = sensor_msgs_msg
sensor_msgs_msg.__name__ = "sensor_msgs.msg"

sys.modules["rclpy"] = rclpy
sys.modules["rclpy.node"] = node_mod
sys.modules["rclpy.qos"] = qos_mod
sys.modules["sensor_msgs"] = sensor_msgs
sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

PATH = os.path.join(REPO_ROOT, "ros2_ws", "src", "lio_sam_gz_bringup",
                    "scripts", "sensor_prep.py")

spec = importlib.util.spec_from_file_location("sensor_prep", PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

fails = []


def check(cond, msg):
    print(("  [OK]   " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


# ------------------------------------------------- 1) 造一帧"Gazebo 风格"点云
N_SCAN, N_AZ = 16, 360
HALF_FOV = 15.0
rows = []
for r in range(N_SCAN):
    elev = math.radians(-HALF_FOV + 2.0 * r)
    for a in range(N_AZ):
        az = -math.pi + 2.0 * math.pi * a / N_AZ
        d = 5.0
        rows.append((d * math.cos(elev) * math.cos(az),
                     d * math.cos(elev) * math.sin(az),
                     d * math.sin(elev),
                     float(r * 10 + a % 7)))
arr = np.array(rows, dtype=np.float32)

# gz 的 PointCloudPacked：x,y,z,intensity 四个 float32，point_step=16
gz_dtype = np.dtype({"names": ["x", "y", "z", "intensity"],
                     "formats": ["<f4"] * 4,
                     "offsets": [0, 4, 8, 12], "itemsize": 16})
gz_buf = np.empty(len(arr), dtype=gz_dtype)
gz_buf["x"], gz_buf["y"], gz_buf["z"] = arr[:, 0], arr[:, 1], arr[:, 2]
gz_buf["intensity"] = arr[:, 3]

fake = types.SimpleNamespace(
    width=len(arr), height=1, point_step=16, is_bigendian=False,
    fields=[types.SimpleNamespace(name=n, offset=o, datatype=7, count=1)
            for n, o in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))],
    data=gz_buf.tobytes(),
)

print("=== 1) _read_xyz 解析 ===")
got = mod._read_xyz(fake)
check(got is not None, "有返回值")
x, y, z, inten = got
check(np.allclose(x, arr[:, 0]) and np.allclose(y, arr[:, 1]) and np.allclose(z, arr[:, 2]),
      "x/y/z 与源数据一致")
check(np.allclose(inten, arr[:, 3]), "intensity 与源数据一致")
check(x.dtype == np.float32, f"dtype 是 float32（实际 {x.dtype}）")

# --------------------------------------- 2) 复现 on_cloud 里的 ring / time 计算
print("\n=== 2) ring / time 计算 ===")
r = np.sqrt(x * x + y * y + z * z)
good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (r > 0.15) & (r < 45.0)
check(bool(np.all(good)), f"所有点都通过 range/finite 过滤（{good.sum()}/{len(good)}）")
x, y, z, inten = x[good], y[good], z[good], inten[good]

elev_deg = np.degrees(np.arctan2(z, np.hypot(x, y)))
step = HALF_FOV * 2.0 / max(N_SCAN - 1, 1)
ring = np.clip(np.rint((elev_deg + HALF_FOV) / step), 0, N_SCAN - 1).astype(np.uint16)
expect_ring = np.repeat(np.arange(N_SCAN, dtype=np.uint16), N_AZ)
check(np.array_equal(ring, expect_ring), "ring 与真实线号完全一致（16×360 全部命中）")

period = 0.1
az = np.arctan2(y, x)
t = ((az + math.pi) / (2.0 * math.pi) * period).astype(np.float32)
check(np.all(t >= 0) and np.all(t <= period + 1e-6), f"time 落在 [0,{period}] 内")
check(np.allclose(t[:N_AZ], np.linspace(0, period, N_AZ, endpoint=False), atol=1e-6),
      "time 随方位角单调（第一线符合预期）")

# ------------------------------------------------------------ 3) 输出打包布局
print("\n=== 3) 输出 PointCloud2 布局 ===")
dt = mod._OUT_DTYPE
check(dt.itemsize == 24, f"point_step = 24（实际 {dt.itemsize}）")
check(dt.fields["ring"][1] == 16 and dt.fields["ring"][0] == np.dtype("<u2"),
      f"ring @16, dtype uint16（实际 offset={dt.fields['ring'][1]}, {dt.fields['ring'][0]}）")
check(dt.fields["time"][1] == 20 and dt.fields["time"][0] == np.dtype("<f4"),
      f"time @20, dtype float32（实际 offset={dt.fields['time'][1]}, {dt.fields['time'][0]}）")

out = np.empty(len(x), dtype=dt)
out["x"], out["y"], out["z"] = x, y, z
out["intensity"] = inten
out["ring"] = ring
out["time"] = t
data = out.tobytes()
check(len(data) == 24 * len(x), f"buffer 大小 = 24 × N（{len(data)} = 24×{len(x)}）")

# 按 PCL/LIO-SAM 的读法（字段名 + offset）解析回来
back = np.frombuffer(data, dtype=dt, count=len(x))
check(np.allclose(back["x"], x, atol=1e-6), "回读 x 一致")
check(np.allclose(back["ring"], ring), "回读 ring 一致")
check(np.allclose(back["time"], t, atol=1e-7), "回读 time 一致")

# 再检查 PointField 描述和 numpy dtype 完全对齐
offsets = {f.name: (f.offset, f.datatype) for f in mod._OUT_FIELDS}
expect = {"x": (0, 7), "y": (4, 7), "z": (8, 7), "intensity": (12, 7),
          "ring": (16, 4), "time": (20, 7)}
check(offsets == expect, f"PointField 偏移/类型与 dtype 对齐: {offsets}")

# ----------------------------------------------- 4) 边界情况：空云 / 缺字段
print("\n=== 4) 边界情况 ===")
empty = types.SimpleNamespace(width=0, height=1, point_step=16, is_bigendian=False,
                              fields=fake.fields, data=b"")
check(mod._read_xyz(empty) is None, "空点云返回 None（不崩）")
nofield = types.SimpleNamespace(width=1, height=1, point_step=16, is_bigendian=False,
                                fields=[types.SimpleNamespace(name="x", offset=0,
                                                             datatype=7, count=1)],
                                data=b"\x00" * 16)
check(mod._read_xyz(nofield) is None, "缺 y/z 字段返回 None（不崩）")

print("\n=== 结论 ===")
print("ALL PASS" if not fails else f"{len(fails)} FAILED")
sys.exit(1 if fails else 0)
