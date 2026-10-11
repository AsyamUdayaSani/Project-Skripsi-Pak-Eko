# sweep_mapping

Mapping 3D dengan **jumlah sweep yang bisa diatur lewat parameter**. Motor berhenti
sendiri setelah N sweep, tepat di posisi awal (LiDAR lurus lagi), lalu semua titik dari
N sweep itu di-publish sebagai **satu pointcloud utuh**.

Menjawab TODO no.1 dan no.5 di `guide.pdf`:
> 1. Atur kode untuk sekali muter aja gausah muter terus-terusan
> 5. Coba bandingin 1x sweep sampai emang di berapa kali sweep baru bagusnya berapa

Paket ini **berdiri sendiri**. `stepper_controller`, `wit_ros2_imu`, dan `sllidar_ros2`
tidak diubah sedikit pun — sistem lama tetap bisa dijalankan seperti biasa.

---

## Pasang di Raspberry Pi

Copy folder `sweep_mapping` ke direktori `src`:

```bash
# dari laptop
scp -r sweep_mapping <user>@<ip-raspi>:~/ros2_ws/src/

# di Raspi
cd ~/ros2_ws
colcon build --packages-select sweep_mapping
source install/setup.bash
```

## Cara jalanin

```bash
source install/setup.bash
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=3
```

Itu saja. Launch ini menyalakan IMU, LiDAR, stepper, mapping, dan foxglove_bridge —
sama seperti `mapping_system.launch.py`, tapi versi sweep-aware.

**Sebelum menekan Enter: luruskan dulu LiDAR-nya.** Posisi fisik saat node menyala
dipakai sebagai titik nol. Tidak ada limit switch/encoder, jadi kode tidak punya cara
lain untuk tahu di mana "sejajar" yang sebenarnya.

### Contoh lain

```bash
# 1 sweep saja (default)
ros2 launch sweep_mapping sweep_mapping.launch.py

# 5 sweep, motor lebih cepat
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=5 delay:=0.010

# muter terus tanpa henti (perilaku lama)
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=0

# tanpa foxglove (hemat CPU kalau cuma mau ros2 bag)
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=3 foxglove:=false
```

### Lihat progres tanpa buka viewer

```bash
ros2 topic echo /stepper/sweep_count
```

---

## Auto-record

Rekaman **jalan sendiri** begitu launch dinyalakan, dan **berhenti sendiri** setelah
target sweep tercapai. Tidak perlu terminal kedua, tidak perlu mikir nama file.

Hasilnya tersimpan di `~/bags/` dengan nama ber-index otomatis:

```
~/bags/scan_0001_3sweep/
~/bags/scan_0002_3sweep/
~/bags/scan_0003_1sweep/
~/bags/scan_0004_5sweep/
```

Index diambil dari nomor terbesar yang sudah ada di folder, lalu +1 — jadi tidak akan
pernah menimpa rekaman lama, termasuk bag lamamu yang namanya bebas seperti
`test_lab_01`. Jumlah sweep ikut ditulis di nama supaya gampang membandingkan hasil
1x vs 3x vs 5x sweep tanpa membuka isinya.

Di akhir rekaman, terminal menampilkan:

```
[bag_recorder]: === REKAMAN TERSIMPAN === scan_0001_3sweep (412.7 MB)
[bag_recorder]: Lokasi : /home/raspitampan/bags/scan_0001_3sweep
[bag_recorder]: Putar  : ros2 bag play /home/raspitampan/bags/scan_0001_3sweep
```

### Mengatur rekaman

```bash
# tanpa rekam sama sekali
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=3 record:=false

# simpan ke folder lain, awalan nama sendiri
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=3 \
  bag_dir:=~/bags/lab bag_prefix:=lab

# kompres zstd (file jauh lebih kecil, CPU Pi lebih berat)
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=3 compress:=true
```

### Topic yang direkam

`/scan`, `/imu/data_raw`, `/stepper/angle`, `/stepper/steps`, `/stepper/status`,
`/stepper/sweep_count`, `/stepper/sweep_done`, `/map_3d`, `/odom`, `/tf`, `/tf_static`.

`/odom` memang belum ada selama belum pakai SLAM — rosbag2 cuma menunggu topic itu
muncul, tidak error. Dibiarkan supaya rekaman tetap valid kalau nanti `rtabmap_odom`
dipasang. Daftar ini bisa diganti lewat parameter `topics` pada node `bag_recorder`.

### Kalau Ctrl+C sebelum sweep selesai

Rekaman tetap ditutup rapi (`metadata.yaml` ditulis lengkap), jadi bag-nya tetap bisa
diputar. Yang terekam ya sampai titik kamu menekan Ctrl+C.

---

## Parameter launch

| Argumen | Default | Keterangan |
|---|---|---|
| `sweeps` | `1` | Jumlah sweep. 1 sweep = LiDAR muter 360°. `0` = muter terus tanpa henti. |
| `delay` | `0.018` | Detik per step motor. Makin kecil makin cepat (dan makin jarang titiknya). |
| `direction` | `1` | Arah putar motor (`1` atau `0`). |
| `steps_per_rev` | `1600` | Step motor per satu putaran motor (setting microstepping driver). |
| `gear_ratio` | `0.5` | Pulley motor / pulley LiDAR. 30T/60T = 0.5. |
| `return_home_on_abort` | `true` | Ctrl+C di tengah sweep → motor muter balik ke posisi awal. |
| `invert_rotation` | `true` | `true` = rotasi −sudut, sama persis dengan `mapping_3d.py` yang selama ini dipakai. Set `false` kalau hasil scan ternyata kebalik/kecermin. |
| `publish_every_sweep` | `true` | Publish cloud kumulatif tiap sweep selesai, bukan cuma di akhir. |
| `foxglove` | `true` | Nyalakan `foxglove_bridge`. |
| `record` | `true` | Auto `ros2 bag record`, berhenti sendiri saat sweep selesai. |
| `bag_dir` | `~/bags` | Folder penyimpanan rekaman. |
| `bag_prefix` | `scan` | Awalan nama bag → `scan_0001_3sweep`. |
| `compress` | `false` | Kompres rekaman dengan zstd. |

### Berapa lama satu sweep?

`steps_per_sweep = steps_per_rev / gear_ratio` = `1600 / 0.5` = **3200 step**.
Dengan `delay=0.018` → **≈ 58 detik per sweep**, ±260 ribu titik, ±6 MB.
Jadi `sweeps:=5` ≈ 5 menit dan ±31 MB.

Karena berhenti tepat di kelipatan 3200 step, sudut LiDAR dijamin kembali ke 0.000° —
tidak ada sisa miring, berapa pun jumlah sweep-nya.

---

## Topic

| Topic | Tipe | Arah | Keterangan |
|---|---|---|---|
| `/map_3d` | `PointCloud2` | keluar | Cloud kumulatif. Tiap pesan berisi **seluruh** titik sejauh ini. Frame `base_link`. |
| `/stepper/sweep_count` | `Int32` | keluar | Sweep ke berapa yang sudah selesai (latched). |
| `/stepper/sweep_done` | `Bool` | keluar | `true` saat semua sweep beres (latched). |
| `/stepper/angle` | `Float32` | keluar | Sudut LiDAR (rad), sama seperti node lama. |
| `/stepper/steps`, `/stepper/status` | | keluar | Sama seperti node lama. |
| `/stepper/enable`, `/stepper/direction`, `/stepper/speed` | | masuk | Sama seperti node lama, masih berfungsi. |

---

## Catatan teknis

**Kenapa `frame_id` jadi `base_link`, bukan `lidar_tilt`?**
Titik sudah di-de-rotasi ke frame diam sebelum disimpan, jadi frame yang benar memang
`base_link`. `mapping_3d.py` yang lama memakai `lidar_tilt` dan hasilnya kebetulan tetap
benar hanya karena publish-nya selalu terjadi pas sudut ≈0 (di titik wrap-around).
Di sini cloud di-publish di sudut sembarang, jadi ketergantungan itu harus dibuang.
Tampilan di RViz/Foxglove tetap sama.

**Perhitungan titik divektorisasi dengan numpy.**
Sudah diverifikasi menghasilkan byte yang **identik persis** dengan versi lama
(`scipy` + `struct.pack` per titik). Loop Python per titik akan makan ~10 detik untuk
1,3 juta titik dan membuat Pi tersendat.

**Jangan pakai `np.concatenate` untuk menggabungkan chunk titik.**
Dtype titik punya padding 2 byte setelah `ring` (itemsize 24 agar cocok dengan
`struct.pack('ffffH2xf')`). `np.concatenate` memampatkannya jadi 22 byte, sehingga data
tidak lagi cocok dengan `point_step=24` dan cloud jadi kacau di viewer. Kode ini memakai
prealokasi + salin per-chunk. Ada `assert` yang menjaga hal ini.

**`sweep_done` di-publish `false` saat node nyala, `true` saat selesai.**
Topic ini latched (`TRANSIENT_LOCAL`) supaya subscriber yang telat nyala tetap kebagian.
Konsekuensinya, tanpa state awal `false`, node yang baru nyala bisa membaca `true` sisa
sesi sebelumnya lalu langsung menghentikan rekaman. Bug ini ketangkap saat tes dan sudah
ditutup dengan mengumumkan `false` di awal.

**Kenapa recorder tidak langsung berhenti saat `sweep_done`?**
`mapping_3d_sweep` baru mem-publish cloud finalnya 0.5 detik setelah `sweep_done`
(menunggu scan yang masih di jalan). Recorder menunggu `stop_delay` (default 3 detik)
supaya cloud final itu ikut masuk rekaman. Sudah diverifikasi lewat tes.

**Ctrl+C di tengah sweep.**
Motor muter balik ke posisi 0° lewat jalur terpendek (mundur kalau belum lewat separuh
sweep, maju kalau sudah). Karena ada pembalikan arah, backlash belt/pulley bisa membuat
posisi akhir meleset sedikit — kalau butuh presisi tinggi, biarkan sweep selesai normal.
Matikan dengan `return_home_on_abort:=false` kalau tidak mau perilaku ini.

## Menjalankan node satu-satu (tanpa launch)

```bash
ros2 run sweep_mapping stepper_sweep_node --ros-args -p num_sweeps:=3 -p delay:=0.018
ros2 run sweep_mapping mapping_3d_sweep
ros2 run sweep_mapping bag_recorder_node --ros-args -p num_sweeps:=3 -p bag_dir:=~/bags
```
