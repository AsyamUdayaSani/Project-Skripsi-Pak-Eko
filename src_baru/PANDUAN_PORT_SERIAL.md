# Panduan Port Serial — LiDAR & IMU

Cara LiDAR dan IMU dikenali di Raspi, kenapa dulu bermasalah, dan cara
memperbaikinya kalau kambuh.

Dibuat 3 Agustus 2026.

---

## Jawaban singkat kalau ditanya

**Apa yang diubah?** Dua hal.

1. **Aturan udev.** Dulu mencocokkan VID:PID `10c4:ea60` — dan kedua adapter
   memakai VID:PID yang sama, jadi tidak bisa dibedakan. Sekarang mencocokkan
   **serial number**, yang unik per perangkat.
2. **`sllidar_c1_launch.py` baris 12.** Dulu dipaku ke `/dev/ttyUSB1`, sekarang
   ke `/dev/rplidar` — nama yang dibuat aturan udev di atas.

**Dampaknya?** Nomor `ttyUSB0`/`ttyUSB1` tidak lagi berpengaruh. LiDAR dan IMU
selalu ditemukan, apa pun urutan colok USB atau urutan boot.

**Buktinya?** Satu perintah:

```bash
ls -l /dev/rplidar /dev/imu_usb
```

Harus menunjuk dua device yang **berbeda**:

```
/dev/imu_usb -> ttyUSB1
/dev/rplidar -> ttyUSB0
```

Kalau keduanya menunjuk device yang sama, berarti rusak lagi.

---

## Peta perangkat

| Adapter | Serial | Perangkat | Nama tetap |
|---|---|---|---|
| CP2102**N** | `6aa92f87fbe5ed11b5f6d3a80b2af5ab` | RPLIDAR C1 | `/dev/rplidar` |
| CP2102 | `0001` | IMU WIT | `/dev/imu_usb` |

Keduanya Silicon Labs, VID:PID sama-sama `10c4:ea60`. **Hanya serial yang
membedakan** — itu sebabnya aturan lama gagal.

---

## Gejala kalau rusak

```
[sllidar_node]  Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!
[wit_ros2_imu]  SerialException: device reports readiness to read but returned
                no data (device disconnected or multiple access on port?)
```

Kedua sensor gagal bersamaan. Yang terjadi: node LiDAR membuka port IMU, IMU
kehilangan portnya karena direbut. Satu penyebab, dua gejala.

Dulu ini muncul kalau Raspi dinyalakan tanpa monitor/keyboard/mouse — tapi
pemicunya bisa apa saja yang menggeser urutan enumerasi USB.

---

## Cara memperbaiki

### 1. Pastikan siapa yang mana

```bash
ls -l /dev/serial/by-id/
```

Kalau serialnya tidak cocok dengan tabel di atas (adapter diganti), uji langsung
— yang menyebut `SLLidar health status : OK` itulah LiDAR:

```bash
pkill -f sllidar_node; pkill -f wit_ros2_imu; sleep 1
```

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/serial/by-id/NAMA-DI-SINI
```

### 2. Tulis aturan udev

Buang aturan lama kalau masih ada:

```bash
sudo rm -f /etc/udev/rules.d/imu_usb.rules /etc/udev/rules.d/rplidar.rules
```

Tulis yang baru — **satu perintah satu baris**, jangan pakai heredoc:

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="6aa92f87fbe5ed11b5f6d3a80b2af5ab", MODE="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-lidar-imu.rules
```

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE="0666", SYMLINK+="imu_usb"' | sudo tee -a /etc/udev/rules.d/99-lidar-imu.rules
```

Perhatikan `tee -a` (huruf `a`) di perintah kedua. Tanpa itu, baris pertama
tertimpa.

### 3. Periksa isinya sebelum lanjut

```bash
cat -n /etc/udev/rules.d/99-lidar-imu.rules
```

Harus **tepat 2 baris**, dan **keduanya** dimulai dengan `SUBSYSTEM=="tty",`.

> Ini bukan langkah opsional. Perintah panjang sering terpotong saat ditempel
> lewat SSH, dan potongannya menghasilkan berkas cacat yang *tetap terlihat
> berhasil* — symlink-nya benar, tapi karena kebetulan. Selalu `cat -n`.

### 4. Terapkan

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger && sleep 2 && ls -l /dev/rplidar /dev/imu_usb
```

### 5. Arahkan launch LiDAR ke nama tetap

```bash
sed -i "s|default='/dev/ttyUSB1'|default='/dev/rplidar'|" ~/ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py
```

```bash
cd ~/ros2_ws && colcon build --packages-select sllidar_ros2 && source install/setup.bash
```

Satu baris ini memperbaiki ketiga launch yang meng-include berkas tersebut:
`sweep_mapping`, `sweep_mappingimu`, dan `stepper_controller/mapping_system`.

Berkas itu **file Python** — tidak dikompilasi. Build yang sukses **tidak**
membuktikan `sed`-nya berhasil, jadi periksa keduanya:

```bash
grep -n "ttyUSB\|rplidar" ~/ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py ~/ros2_ws/install/sllidar_ros2/share/sllidar_ros2/launch/sllidar_c1_launch.py
```

Harus dua baris, keduanya `default='/dev/rplidar'`. Yang di `install/` itu yang
benar-benar dipakai saat runtime.

---

## Cara membuktikan sudah benar

**Cek 1 — symlink menunjuk device berbeda**

```bash
ls -l /dev/rplidar /dev/imu_usb
```

**Cek 2 — scan berjalan**

```bash
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=1
```

Tidak ada `SL_RESULT_OPERATION_TIMEOUT`, tidak ada `multiple access on port`,
tidak ada `/scan KOSONG`. Muncul `Published cloud: <ratusan ribu> titik`.

**Cek 3 — bertahan setelah reboot** ← yang paling menentukan

Symlink udev ditetapkan saat perangkat dienumerasi, yaitu **saat boot**.
`udevadm trigger` hanya menirunya. Selama Cek 3 belum dilakukan, semua bukti di
atas masih di atas kertas.

```bash
sudo reboot
```

Tunggu ~40 detik, SSH lagi, ulangi Cek 1. Idealnya reboot dalam keadaan tanpa
monitor/keyboard/mouse — itu kondisi yang dulu gagal.

---

## Kalau LiDAR atau adapternya diganti

Serial-nya berbeda, aturan tidak lagi cocok, `/dev/rplidar` hilang. Perbaikannya:
ambil serial baru dari `ls -l /dev/serial/by-id/`, lalu ganti angkanya di
`/etc/udev/rules.d/99-lidar-imu.rules`.

Itu harga dari pencocokan per-serial. Alternatifnya — mencocokkan per posisi port
USB fisik (`KERNELS=="..."`) — justru rusak kalau kabelnya pindah colokan, jadi
untuk alat yang akan naik drone, per-serial lebih tepat.

---

## Jangan sentuh sisi IMU

`/dev/imu_usb` dipakai langsung oleh `wit_ros2_imu.py:153` dan berfungsi.
Biarkan.

Jebakan kalau suatu saat tergoda "merapikan" node IMU supaya membaca parameter:

```python
# rviz_and_imu.launch.py:10-11  — dikirim ke node, tapi DIABAIKAN
parameters=[{'port': '/dev/ttyUSB0'},     # <-- ini port LiDAR!
            {"baud": 9600}]                # <-- kode sebenarnya pakai 460800
```

Dua nilai itu aman **justru karena diabaikan** (`wit_ros2_imu.py:143-144`, baris
`get_parameter` dikomentari). Begitu node dibuat membacanya, IMU langsung
diarahkan ke port LiDAR dengan baud yang salah — kesalahan yang sama, arah
terbalik. Kalau mau dirapikan, `rviz_and_imu.launch.py` **harus** diperbaiki
dalam perubahan yang sama.

---

## Yang tidak ada hubungannya

Tidak ada satu pun yang membuka serial, jadi aman dari semua ini:

- `mapping_3d_sweep.py`, `mapping_3d_imu.py`
- `stepper_sweep_node.py` (GPIO, bukan serial), `bag_recorder_node.py`
- `cloudcom/`, `pointcloud_studio/` — bekerja dari berkas mcap
- FAST-LIO — bekerja dari topic, bukan port

---

## Cadangan aturan lama

Isi `/etc/udev/rules.d/` sebelum perbaikan tersimpan di laptop:
`~/riset td/backup/udev-raspi-2026-08-03/`, lengkap dengan cara mengembalikannya.

Mengembalikannya berarti mengembalikan bug-nya — symlink akan acak lagi tiap
boot. Hanya masuk akal kalau perbaikan ini ternyata lebih buruk.
