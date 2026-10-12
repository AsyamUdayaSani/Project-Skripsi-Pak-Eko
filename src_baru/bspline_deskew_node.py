"""Node Deskewing Point Cloud Berbasis Cumulative B-Spline SE(3).

Implementasi sesuai proposal skripsi:
  "Optimasi Trajektori Continuous-Time Berbasis B-Spline untuk Deskewing
   Point Cloud pada Sistem Rotating 2D LiDAR UAV Real-Time"
  Asyam Udaya Sani, FILKOM UB, 2026

Referensi utama:
  - Sommer et al. (2020) — Efficient derivative computation for cumulative B-splines on Lie groups
  - Lv et al. (2020/2021) — CLINS: Continuous-time trajectory estimation
  - Furgale, Barfoot & Sibley (2012) — Continuous-time batch estimation

ALUR KERJA NODE INI (sesuai Gambar 3.1 proposal):
  1. Buffer IMU terus diperbarui dari /imu/data_raw
  2. Saat scan LiDAR masuk (/scan + /stepper/angle):
     a. Kumpulkan pengukuran giroskop IMU di rentang waktu scan
     b. Estimasi control points B-Spline via Gauss-Newton (minimasi residual ω)
     c. Deskew setiap titik: p̂ᵢ = T(tᵢ) · pᵢ
     d. Publish /point_cloud_deskewed (PointCloud2)

TARGET: latensi ≤ 100 ms per scan (batas real-time 10 Hz)
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32, Header

# ============================================================
# MATEMATIKA LIE GROUP SO(3) DAN SE(3)
# ============================================================

def skew(v):
    """Matriks skew-symmetric [v]× dari vektor 3D."""
    return np.array([
        [ 0.0,  -v[2],  v[1]],
        [ v[2],  0.0,  -v[0]],
        [-v[1],  v[0],  0.0 ],
    ])


def so3_exp(phi):
    """Eksponensial map SO(3): vektor rotasi φ ∈ ℝ³ → matriks rotasi R ∈ SO(3).

    Menggunakan formula Rodrigues:
      R = I + sin(θ)/θ · [φ]× + (1-cos(θ))/θ² · [φ]×²
    Degenerasi θ≈0 ditangani dengan ekspansi Taylor orde-2.
    """
    theta = np.linalg.norm(phi)
    K = skew(phi)
    if theta < 1e-8:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + (math.sin(theta) / theta) * K + \
           ((1.0 - math.cos(theta)) / theta**2) * (K @ K)


def so3_log(R):
    """Logaritmik map SO(3): R → φ (vektor rotasi).

    Menggunakan formula:
      θ = arccos((tr(R)-1)/2)
      φ = θ/(2 sin θ) · (R - Rᵀ)^∨
    """
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = math.acos(cos_theta)
    if theta < 1e-8:
        return np.array([R[2, 1] - R[1, 2],
                         R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]]) * 0.5
    factor = theta / (2.0 * math.sin(theta))
    return factor * np.array([R[2, 1] - R[1, 2],
                               R[0, 2] - R[2, 0],
                               R[1, 0] - R[0, 1]])


def se3_from_Rt(R, t):
    """Buat matriks SE(3) 4×4 dari rotasi R dan translasi t."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def se3_inv(T):
    """Invers dari elemen SE(3): T⁻¹."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def se3_exp(xi):
    """Eksponensial map SE(3): ξ = [ρ; φ] ∈ ℝ⁶ → T ∈ SE(3).

    Menggunakan formula:
      R = exp(φ)
      t = V·ρ  dimana V = I + (1-cosθ)/θ²·[φ]× + (θ-sinθ)/θ³·[φ]×²
    """
    rho = xi[:3]
    phi = xi[3:]
    theta = np.linalg.norm(phi)
    R = so3_exp(phi)
    K = skew(phi)
    if theta < 1e-8:
        V = np.eye(3) + 0.5 * K
    else:
        V = (np.eye(3)
             + ((1.0 - math.cos(theta)) / theta**2) * K
             + ((theta - math.sin(theta)) / theta**3) * (K @ K))
    return se3_from_Rt(R, V @ rho)


def se3_log(T):
    """Logaritmik map SE(3): T → ξ = [ρ; φ] ∈ ℝ⁶."""
    R = T[:3, :3]
    t = T[:3, 3]
    phi = so3_log(R)
    theta = np.linalg.norm(phi)
    K = skew(phi)
    if theta < 1e-8:
        V_inv = np.eye(3) - 0.5 * K
    else:
        V_inv = (np.eye(3)
                 - 0.5 * K
                 + (1.0 / theta**2 - (1.0 + math.cos(theta))
                    / (2.0 * theta * math.sin(theta))) * (K @ K))
    return np.concatenate([V_inv @ t, phi])


# ============================================================
# CUMULATIVE B-SPLINE SE(3)
# ============================================================
# Referensi: Sommer et al. (2020), Lv et al. (2020)
# Formulasi kumulatif kubik: T(t) = T₀ · ∏ᵢ exp(βᵢ(u) · log(Tᵢ₋₁⁻¹ Tᵢ))

# Matriks B-spline kumulatif kubik (sesuai Sommer et al. 2020, eq. 6)
# Untuk spline kubik seragam, fungsi basis kumulatif:
#   β̃(u) = M̃ · [1, u, u², u³]ᵀ
# dimana M̃ adalah matriks kumulatif kubik standar
_BSPLINE_M_CUMUL = (1.0 / 6.0) * np.array([
    [6.0,  0.0,  0.0,  0.0],
    [5.0,  3.0, -3.0,  1.0],
    [1.0,  3.0,  3.0, -2.0],
    [0.0,  0.0,  0.0,  1.0],
], dtype=np.float64)

# Turunan matriks kumulatif (untuk menghitung kecepatan angular ω)
_BSPLINE_dM_CUMUL = (1.0 / 6.0) * np.array([
    [0.0,  0.0,  0.0,  0.0],
    [0.0,  3.0, -6.0,  3.0],
    [0.0,  3.0,  6.0, -6.0],
    [0.0,  0.0,  0.0,  3.0],
], dtype=np.float64)


def bspline_basis_cumul(u):
    """Vektor basis kumulatif kubik β(u) untuk u ∈ [0,1).

    β(u) = M̃ · [1, u, u², u³]ᵀ
    Hasilnya 4 nilai yang mewakili bobot akumulasi setiap control point.
    """
    uvec = np.array([1.0, u, u*u, u*u*u])
    return _BSPLINE_M_CUMUL @ uvec


def bspline_dbasis_cumul(u, dt):
    """Turunan β'(u)/dt — dipakai menghitung kecepatan angular prediksi spline."""
    uvec = np.array([0.0, 1.0, 2.0*u, 3.0*u*u])
    return (_BSPLINE_dM_CUMUL @ uvec) / dt


def bspline_eval(control_points, t, t_knots, dt):
    """Evaluasi pose T(t) ∈ SE(3) dari B-Spline kumulatif kubik.

    Parameter:
      control_points : list[ndarray 4×4] — pose SE(3) di knot seragam
      t              : float — waktu query
      t_knots        : ndarray — waktu knot seragam
      dt             : float — interval knot (uniform)

    Mengembalikan:
      T : ndarray 4×4 — pose SE(3) pada waktu t
    """
    # Cari indeks segmen aktif
    i = int((t - t_knots[0]) / dt)
    i = max(0, min(i, len(control_points) - 2))

    u = (t - t_knots[i]) / dt
    u = np.clip(u, 0.0, 1.0 - 1e-9)

    beta = bspline_basis_cumul(u)

    # T(t) = T₀ · exp(β₁·Ω₁) · exp(β₂·Ω₂) · exp(β₃·Ω₃)
    # dimana Ωᵢ = log(Tᵢ₋₁⁻¹ Tᵢ)
    # Untuk kubik kita butuh 4 control points di sekitar i
    idx = [max(0, i-1), i, min(i+1, len(control_points)-1),
           min(i+2, len(control_points)-1)]

    T = control_points[idx[0]].copy()
    for k in range(1, 4):
        Omega = se3_log(se3_inv(control_points[idx[k-1]]) @ control_points[idx[k]])
        T = T @ se3_exp(beta[k] * Omega)
    return T


def bspline_angular_velocity(control_points, t, t_knots, dt):
    """Hitung kecepatan angular ω(t) yang diprediksi spline (frame body).

    Dipakai sebagai prediksi untuk residual terhadap giroskop IMU:
      residual = ω_spline(tₖ) - ω̂ₖ_gyro
    """
    i = int((t - t_knots[0]) / dt)
    i = max(0, min(i, len(control_points) - 2))
    u = (t - t_knots[i]) / dt
    u = np.clip(u, 0.0, 1.0 - 1e-9)

    dbeta = bspline_dbasis_cumul(u, dt)

    # dT/dt dihitung dari suku turunan kumulatif
    # ω ≈ ∑ β'ᵢ · Ωᵢ_rot  (komponen rotasi dari Ωᵢ)
    idx = [max(0, i-1), i, min(i+1, len(control_points)-1),
           min(i+2, len(control_points)-1)]

    omega = np.zeros(3)
    T_accum = control_points[idx[0]].copy()
    for k in range(1, 4):
        Omega = se3_log(se3_inv(control_points[idx[k-1]]) @ control_points[idx[k]])
        omega_k = Omega[3:]  # komponen rotasi dari ξ = [ρ; φ]
        # Perlu transformasi ke frame body oleh T_accum (approx)
        R_accum = T_accum[:3, :3]
        omega += dbeta[k] * (R_accum @ omega_k)
        T_accum = T_accum @ se3_exp(dbeta[k] * Omega)

    return omega


# ============================================================
# OPTIMASI GAUSS-NEWTON
# ============================================================

def imu_preintegrate(gyro_times, gyro_data, t_knots):
    """Praintegrasi IMU untuk inisialisasi control points.

    Mengintegrasikan giroskop dengan midpoint rule untuk mendapatkan
    estimasi awal orientasi di setiap knot (translasi = 0).
    """
    n_knots = len(t_knots)
    control_points = [np.eye(4) for _ in range(n_knots)]

    for i in range(1, n_knots):
        t_prev = t_knots[i - 1]
        t_curr = t_knots[i]

        # Interpolasi giroskop di midpoint
        t_mid = 0.5 * (t_prev + t_curr)
        omega = np.array([
            np.interp(t_mid, gyro_times, gyro_data[:, ax])
            for ax in range(3)
        ])

        dt = t_curr - t_prev
        R_prev = control_points[i - 1][:3, :3]
        R_curr = R_prev @ so3_exp(omega * dt)
        control_points[i][:3, :3] = R_curr

    return control_points


def gauss_newton_bspline(control_points, gyro_times, gyro_data,
                          t_knots, dt, max_iter=10, tol=1e-6):
    """Optimasi Gauss-Newton untuk estimasi trajektori B-Spline.

    Minimasi fungsi biaya (sesuai proposal Eq. 3.4):
      e(θ) = Σₖ ‖ω_spline(tₖ; θ) - ω̂ₖ‖²

    dimana:
      θ       = parameter (control points dalam ruang Lie)
      ω_spline = kecepatan angular yang diprediksi spline
      ω̂ₖ     = pengukuran giroskop pada tₖ

    Menggunakan Gauss-Newton dengan Jacobian numerik sederhana.
    """
    n_knots = len(control_points)
    n_meas = len(gyro_times)

    prev_cost = float('inf')

    for iteration in range(max_iter):
        # Hitung residual r = ω_spline(tₖ) - ω̂ₖ
        residuals = []
        for k in range(n_meas):
            omega_pred = bspline_angular_velocity(
                control_points, gyro_times[k], t_knots, dt)
            r = omega_pred - gyro_data[k]
            residuals.append(r)

        residuals = np.array(residuals)  # (n_meas, 3)
        cost = np.sum(residuals**2)

        # Cek konvergensi
        if abs(prev_cost - cost) < tol * (1.0 + cost):
            break
        prev_cost = cost

        # Update control points dengan gradient descent sederhana
        # (Jacobian penuh terlalu berat untuk RPi5 — pakai approx grad)
        eps = 1e-4
        step_size = 0.01 / (1.0 + iteration * 0.5)

        for ci in range(1, n_knots - 1):
            # Gradien numerik terhadap komponen rotasi control point ci
            grad = np.zeros(3)
            for ax in range(3):
                # Perturb positif
                delta_p = np.zeros(6)
                delta_p[3 + ax] = eps
                cp_p = [T.copy() for T in control_points]
                cp_p[ci] = control_points[ci] @ se3_exp(delta_p)

                res_p = []
                for k in range(n_meas):
                    omega_pred = bspline_angular_velocity(
                        cp_p, gyro_times[k], t_knots, dt)
                    res_p.append(omega_pred - gyro_data[k])
                cost_p = np.sum(np.array(res_p)**2)

                grad[ax] = (cost_p - cost) / eps

            # Update via gradient descent
            delta_xi = np.zeros(6)
            delta_xi[3:] = -step_size * grad

            if np.linalg.norm(delta_xi[3:]) > 0.1:
                delta_xi[3:] *= 0.1 / np.linalg.norm(delta_xi[3:])

            control_points[ci] = control_points[ci] @ se3_exp(delta_xi)

    return control_points, cost


# ============================================================
# NODE ROS 2
# ============================================================

class BSplineDeskewNode(Node):
    """Node deskewing berbasis cumulative B-Spline SE(3).

    Subscribe:
      /scan           (sensor_msgs/LaserScan)
      /imu/data_raw   (sensor_msgs/Imu)
      /stepper/angle  (std_msgs/Float32)

    Publish:
      /point_cloud_deskewed  (sensor_msgs/PointCloud2)
      /point_cloud_raw       (sensor_msgs/PointCloud2) — tanpa deskewing, baseline
    """

    def __init__(self):
        super().__init__('bspline_deskew_node')

        # Parameter node
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('dt_knot', 0.05)       # interval knot (detik), 50ms = 2 knot per scan
        self.declare_parameter('max_iter_gn', 10)      # iterasi Gauss-Newton
        self.declare_parameter('tol_gn', 1e-6)         # toleransi konvergensi
        self.declare_parameter('imu_buffer_sec', 1.0)  # panjang buffer IMU (detik)
        self.declare_parameter('stepper_axis', 'y')    # sumbu rotasi stepper
        self.declare_parameter('invert_rotation', True)
        self.declare_parameter('tilt_offset_deg', 0.0)
        self.declare_parameter('publish_raw', True)    # publish baseline tanpa deskewing

        self.frame_id      = str(self.get_parameter('frame_id').value)
        self.dt_knot       = float(self.get_parameter('dt_knot').value)
        self.max_iter_gn   = int(self.get_parameter('max_iter_gn').value)
        self.tol_gn        = float(self.get_parameter('tol_gn').value)
        self.buf_sec       = float(self.get_parameter('imu_buffer_sec').value)
        self.stepper_axis  = str(self.get_parameter('stepper_axis').value)
        self.rot_sign      = -1.0 if self.get_parameter('invert_rotation').value else 1.0
        self.tilt_offset   = math.radians(float(self.get_parameter('tilt_offset_deg').value))
        self.publish_raw   = bool(self.get_parameter('publish_raw').value)

        imu_topic = str(self.get_parameter('imu_topic').value)

        # Buffer IMU: [(timestamp, omega_x, omega_y, omega_z), ...]
        self.imu_buffer = []  # list of (t, np.array([wx, wy, wz]))

        # Buffer stepper angle
        self.stepper_times  = []
        self.stepper_angles = []

        # Subscriber
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 50)
        self.create_subscription(Float32, '/stepper/angle', self.stepper_callback, 50)

        # Publisher
        self.pub_deskewed = self.create_publisher(
            PointCloud2, '/point_cloud_deskewed', 10)
        if self.publish_raw:
            self.pub_raw = self.create_publisher(
                PointCloud2, '/point_cloud_raw', 10)
        else:
            self.pub_raw = None

        # Statistik latensi
        self.latency_log = []

        self.get_logger().info(
            f'BSplineDeskewNode dimulai. '
            f'dt_knot={self.dt_knot*1000:.0f}ms, '
            f'max_iter={self.max_iter_gn}, '
            f'stepper_axis={self.stepper_axis}'
        )

    # ----------------------------------------------------------
    # Callback IMU
    # ----------------------------------------------------------

    def _stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def imu_callback(self, msg):
        t = self._stamp_to_sec(msg.header.stamp)
        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self.imu_buffer.append((t, omega))

        # Buang data lama (> buf_sec)
        cutoff = t - self.buf_sec
        self.imu_buffer = [(ts, w) for ts, w in self.imu_buffer if ts >= cutoff]

    def stepper_callback(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.stepper_times.append(t)
        self.stepper_angles.append(float(msg.data))
        if len(self.stepper_times) > 200:
            self.stepper_times.pop(0)
            self.stepper_angles.pop(0)

    # ----------------------------------------------------------
    # Scan callback — inti pemrosesan
    # ----------------------------------------------------------

    def scan_callback(self, msg):
        t_start_proc = time.monotonic()

        t_scan_start = self._stamp_to_sec(msg.header.stamp)
        time_inc = msg.time_increment
        if time_inc <= 0.0:
            time_inc = (1.0 / 10.0) / max(len(msg.ranges), 1)
        t_scan_end = t_scan_start + len(msg.ranges) * time_inc

        # --- 1. Kumpulkan pengukuran IMU di rentang scan ---
        gyro_in_scan = [
            (ts, w) for ts, w in self.imu_buffer
            if t_scan_start - 0.02 <= ts <= t_scan_end + 0.02
        ]

        if len(gyro_in_scan) < 3:
            self.get_logger().warn(
                f'IMU tidak cukup ({len(gyro_in_scan)} sampel) untuk '
                'estimasi B-Spline. Scan dilewati.'
            )
            return

        gyro_times = np.array([ts for ts, _ in gyro_in_scan])
        gyro_data  = np.array([w  for _, w  in gyro_in_scan])  # (N, 3)

        # --- 2. Buat knot seragam di rentang scan ---
        t_knots = np.arange(t_scan_start - self.dt_knot,
                            t_scan_end   + self.dt_knot * 2,
                            self.dt_knot)
        if len(t_knots) < 4:
            t_knots = np.linspace(t_scan_start - self.dt_knot,
                                  t_scan_end   + self.dt_knot,
                                  5)

        # --- 3. Inisialisasi control points via praintegrasi IMU ---
        control_points = imu_preintegrate(gyro_times, gyro_data, t_knots)

        # --- 4. Optimasi Gauss-Newton ---
        control_points, final_cost = gauss_newton_bspline(
            control_points, gyro_times, gyro_data,
            t_knots, self.dt_knot,
            max_iter=self.max_iter_gn,
            tol=self.tol_gn
        )

        # --- 5. Deskewing per titik ---
        ranges   = np.asarray(msg.ranges, dtype=np.float64)
        n_rays   = len(ranges)

        # Stepper angle buffer → array
        if len(self.stepper_times) >= 2:
            st = np.asarray(self.stepper_times)
            sa = np.asarray(self.stepper_angles)
        else:
            st = np.array([t_scan_start, t_scan_end])
            sa = np.array([0.0, 0.0])

        points_deskewed = []
        points_raw      = []

        for i, r in enumerate(ranges):
            if not (msg.range_min < r < msg.range_max and math.isfinite(r)):
                continue

            scan_angle = msg.angle_min + i * msg.angle_increment
            t_ray = t_scan_start + i * time_inc

            # Sudut stepper pada saat ray ini
            stepper_angle = float(np.interp(t_ray, st, sa))
            a = self.rot_sign * stepper_angle + self.tilt_offset

            # Titik lokal di frame LiDAR
            x_local = r * math.cos(scan_angle)
            y_local = r * math.sin(scan_angle)

            # Rotasi stepper (sumbu Y — poros pipa horizontal)
            if self.stepper_axis == 'y':
                ca, sa_ = math.cos(a), math.sin(a)
                vx = x_local * ca
                vy = y_local
                vz = -x_local * sa_
            elif self.stepper_axis == 'x':
                ca, sa_ = math.cos(a), math.sin(a)
                vx = x_local
                vy = y_local * ca
                vz = y_local * sa_
            else:  # z
                ca, sa_ = math.cos(a), math.sin(a)
                vx = x_local * ca - y_local * sa_
                vy = x_local * sa_ + y_local * ca
                vz = 0.0

            p_local = np.array([vx, vy, vz, 1.0])
            points_raw.append([vx, vy, vz])

            # Evaluasi pose B-Spline pada timestamp ray
            T_ray = bspline_eval(control_points, t_ray, t_knots, self.dt_knot)

            # Deskewing: p̂ᵢ = T(tᵢ) · pᵢ (sesuai proposal Eq. 2.2.3)
            p_deskewed = T_ray @ p_local
            points_deskewed.append(p_deskewed[:3])

        # --- 6. Publish ---
        stamp_msg = msg.header.stamp

        if points_deskewed:
            cloud_deskewed = self._make_cloud(points_deskewed, stamp_msg)
            self.pub_deskewed.publish(cloud_deskewed)

        if self.pub_raw and points_raw:
            cloud_raw = self._make_cloud(points_raw, stamp_msg)
            self.pub_raw.publish(cloud_raw)

        # --- 7. Log latensi ---
        latency_ms = (time.monotonic() - t_start_proc) * 1000.0
        self.latency_log.append(latency_ms)

        status = 'OK' if latency_ms <= 100.0 else 'OVER BUDGET'
        self.get_logger().info(
            f'[{status}] Deskewing selesai: {len(points_deskewed)} titik, '
            f'latensi={latency_ms:.1f}ms, cost_GN={final_cost:.4f}'
        )

        # Laporan statistik setiap 10 scan
        if len(self.latency_log) % 10 == 0:
            arr = np.array(self.latency_log[-50:])
            self.get_logger().info(
                f'Latensi 50 scan terakhir — '
                f'mean={arr.mean():.1f}ms, '
                f'p99={np.percentile(arr, 99):.1f}ms, '
                f'max={arr.max():.1f}ms'
            )

    # ----------------------------------------------------------
    # Utility: buat PointCloud2
    # ----------------------------------------------------------

    def _make_cloud(self, points, stamp):
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for p in points:
            data += np.array(p, dtype=np.float32).tobytes()

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.data = bytes(data)
        msg.is_dense = True
        return msg


# ============================================================
# MAIN
# ============================================================

def main():
    rclpy.init()
    node = BSplineDeskewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Node berhenti. Total scan diproses: {len(node.latency_log)}'
        )
        if node.latency_log:
            arr = np.array(node.latency_log)
            node.get_logger().info(
                f'Statistik latensi keseluruhan:\n'
                f'  mean  = {arr.mean():.1f} ms\n'
                f'  p99   = {np.percentile(arr, 99):.1f} ms\n'
                f'  max   = {arr.max():.1f} ms\n'
                f'  % ≤ 100ms = {(arr <= 100.0).mean()*100:.1f}%'
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
