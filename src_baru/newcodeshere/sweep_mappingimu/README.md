# sweep_mappingimu

Mapping 3D dengan **koreksi orientasi IMU**, untuk rig yang dipasang di drone.

Kalau drone miring atau bergetar saat scan berjalan, titik LiDAR jatuh di tempat
yang salah. Paket ini memutar balik setiap sinar memakai orientasi rangka dari
IMU, sehingga peta tetap tegak walau rangkanya goyang.

```bash
ros2 launch sweep_mappingimu sweep_mappingimu.launch.py sweeps:=1
```

## Hubungannya dengan `sweep_mapping`

Paket ini **menumpang** `sweep_mapping`, bukan menggantikannya. Yang ada di sini
cuma satu node baru (`mapping_3d_imu`) dan satu launch file. Stepper, bag
recorder, dan tata letak pointcloud tetap dipakai dari `sweep_mapping` — jadi
perbaikan di sana otomatis berlaku di sini dan keduanya tidak bisa menyimpang
diam-diam.

| | `sweep_mapping` | `sweep_mappingimu` |
|---|---|---|
| Dipakai untuk | rig diam (tripod, meja) | rig di drone |
| Sumber orientasi | sudut stepper saja | sudut stepper **×** orientasi IMU |
| IMU | direkam ke bag, tidak dibaca | dibaca dan dipakai per sinar |

## Yang dikoreksi dan yang tidak

**Dikoreksi:** kemiringan rangka (roll dan pitch). Keduanya diambil dari
orientasi IMU yang mengacu ke gravitasi, jadi tidak pernah drift berapa lama pun
scan berjalan.

**Tidak dikoreksi:** perpindahan posisi. Node ini menganggap drone diam di satu
titik. Kalau drone melayang bergeser 20 cm selama sweep, pergeseran itu masuk ke
peta sebagai galat dan tidak ada yang bisa dilakukan node ini. Untuk drone yang
benar-benar berpindah, yang dibutuhkan adalah SLAM/LIO (FAST-LIO), bukan node ini.

**Yaw: mati secara default.** Yaw berasal dari magnetometer, dan magnetometer di
dekat motor + ESC drone tidak bisa dipercaya. Yaw palsu akan merusak peta yang
tadinya sudah bagus. Nyalakan dengan `use_yaw:=true` hanya kalau uji lapangan
menunjukkan bacaannya bersih; saat menyala, yaw dihitung relatif terhadap awal
sweep, bukan terhadap utara magnetik.

## Kalibrasi bias pemasangan (sekali saja)

IMU tidak terpasang persis sejajar dengan bidang pindai LiDAR. Selisih tetap itu
harus dimasukkan sekali:

1. Taruh rig di meja yang **datar**.
2. Jalankan `ros2 run sweep_mappingimu mapping_3d_imu`.
3. Baris log pertama saat pesan IMU masuk melaporkan roll dan pitch saat itu.
   Karena rig sedang datar, dua angka itu **adalah** bias pemasangannya.
4. Masukkan apa adanya:

```bash
ros2 launch sweep_mappingimu sweep_mappingimu.launch.py \
    sweeps:=1 imu_roll_offset_deg:=-1.4 imu_pitch_offset_deg:=0.7
```

## Argumen launch

Selain semua argumen `sweep_mapping` (`sweeps`, `rpm`, `direction`,
`steps_per_rev`, `gear_ratio`, `record`, `bag_dir`, `foxglove`, …):

| Argumen | Default | Keterangan |
|---|---|---|
| `imu_topic` | `/imu/data_raw` | Topic `sensor_msgs/Imu` yang dipakai. |
| `use_yaw` | `false` | Ikut mengoreksi yaw. Baca peringatan di atas dulu. |
| `imu_roll_offset_deg` | `0.0` | Bias pemasangan sumbu roll (lihat kalibrasi). |
| `imu_pitch_offset_deg` | `0.0` | Bias pemasangan sumbu pitch. |

## Kalau IMU mati saat terbang

Titik **tetap dikumpulkan** tanpa koreksi — membuang data penerbangan lebih mahal
daripada melaporkan apa adanya. Tapi node akan:

* menulis `ERROR` tiap siklus health check,
* menghitung titik yang tak terkoreksi, dan
* menyebut jumlahnya di setiap baris `Published cloud` serta saat node berhenti.

Kalau angka itu bukan nol, hasilnya setara `sweep_mapping` biasa dan **tidak boleh
diklaim terkoreksi**.

## Catatan teknis

**Kenapa interpolasi per sinar?** `mapping_3d_old.py` memakai satu orientasi IMU
untuk seluruh pesan scan (~450 sinar, ~100 ms). Untuk rig statis itu cukup. Untuk
drone yang bergetar, 100 ms sudah cukup lama untuk membuat blur — jadi di sini
orientasi diinterpolasi per sinar, sama seperti perlakuan terhadap sudut stepper.

**Susunan rotasi.** `R_z(yaw) · R_y(pitch) · R_x(roll) · R_x(sudut_stepper)`,
konvensi ZYX yang sama dengan `get_quaternion_from_euler` di `wit_ros2_imu.py`.
Dikerjakan elemen-per-elemen dengan numpy, bukan lewat N buah matriks 3×3.

**Yaw dijaga kontinu.** Selisih antar sampel dibungkus ke ±π lalu dijumlahkan,
supaya `np.interp` tidak menghasilkan putaran palsu saat bacaan melewati batas
wrap ±180°.

**Sumber waktu belum seragam** (diwarisi dari `mapping_3d_sweep.py`): sudut
stepper dan IMU dicap dengan waktu terima, sedangkan sinar LiDAR dengan
`header.stamp` dari driver. Kalau driver punya latensi tetap, ada bias kecil
antara keduanya. Belum diukur di rig ini.
