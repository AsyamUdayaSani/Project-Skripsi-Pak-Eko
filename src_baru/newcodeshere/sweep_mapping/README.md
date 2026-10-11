# sweep_mapping

Mapping 3D dengan **jumlah sweep yang bisa diatur lewat parameter**. Motor berhenti
sendiri setelah N sweep, tepat di posisi awal (LiDAR lurus lagi), lalu semua titik dari
N sweep itu di-publish sebagai **satu pointcloud utuh**.

Menjawab TODO no.1 dan no.5 di `guide.pdf`:
> 1. Atur kode untuk sekali muter aja gausah muter terus-terusan
> 5. Coba bandingin 1x sweep sampai emang di berapa kali sweep baru bagusnya berapa

## Mapping 180° (versi sekarang)

Satu sweep = **180° dipetakan**, lalu **180° pulang cepat tanpa dipetakan** sampai
LiDAR kembali ke 0°.

Kenapa: bidang scan RPLIDAR C1 sudah 360°, dan sumbu putar stepper ada di bidang
itu. Memutar bidang 180° saja sudah menyapu seluruh ruangan. 180° berikutnya menyapu
ruang yang **sama** dari sisi sebaliknya. Selisih kalibrasi sekecil apa pun (sumbu
tidak pas, offset sudut, backlash) membuat dua salinan itu tidak berimpit, sehingga
objek muncul dobel.

- Kecepatan fase mapping tetap diatur `rpm`/`delay`, artinya sama seperti dulu.
  `rpm:=2` = 15 detik untuk 180° yang dipetakan.
- Fase pulang pakai `return_rpm` (default 9, maksimal ~9,4), dengan ramp naik/turun
  `return_ramp_steps`. Tidak pernah lebih lambat dari fase mapping.
- `mapping_3d_sweep` membuang setiap sinar yang waktunya jatuh di fase pulang, dengan
  acuan `/stepper/mapping_active`.

**Versi lama (360° dipetakan) masih ada**, dengan akhiran `1`:

```bash
ros2 launch sweep_mapping sweep_mapping1.launch.py sweeps:=1 delay:=0.009375
```

File lamanya: `launch/sweep_mapping1.launch.py`, `stepper_sweep_node1.py`,
`mapping_3d_sweep1.py`. Paket `sweep_mappingimu` sengaja tetap memakai
`stepper_sweep_node1`, karena `mapping_3d_imu` belum membuang fase pulang.

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
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=5 delay:=0.009375

# sama cepatnya, ditulis sebagai RPM
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=5 rpm:=2.0

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
| `sweeps` | `1` | Jumlah sweep. 1 sweep = 180° dipetakan + 180° pulang. `0` = muter terus tanpa henti. |
| `delay` | `0.018` | Detik per step motor selama fase mapping. Makin kecil makin cepat (dan makin jarang titiknya). Cara lama, tetap berlaku. |
| `rpm` | `0.0` | Alternatif yang lebih mudah dibaca: putaran per menit piringan LiDAR selama fase mapping. Dipakai **hanya kalau diisi > 0**; kalau tidak, `delay` yang menentukan. Dibatasi di ~9,4 RPM oleh ketelitian `time.sleep`. |
| `return_rpm` | `9.0` | Kecepatan fase pulang 180° (tanpa mapping). Dikunci di ~9,4 dan tidak pernah lebih lambat dari fase mapping. |
| `return_ramp_steps` | `200` | Step untuk naik/turun kecepatan di awal dan akhir fase pulang. `0` = langsung loncat. |
| `direction` | `1` | Arah putar motor (`1` atau `0`). |
| `steps_per_rev` | `1600` | Step motor per satu putaran motor (setting microstepping driver). |
| `gear_ratio` | `0.5` | Pulley motor / pulley LiDAR. 30T/60T = 0.5. |
| `return_home_on_abort` | `true` | Ctrl+C di tengah sweep → motor muter balik ke posisi awal. |
| `invert_rotation` | `true` | `true` = rotasi −sudut, sama persis dengan `mapping_3d.py` yang selama ini dipakai. Set `false` kalau hasil scan ternyata kebalik/kecermin. |
| `publish_every_sweep` | `true` | Publish cloud kumulatif tiap sweep selesai, bukan cuma di akhir. |
| `foxglove` | `true` | Nyalakan `foxglove_bridge`. |
| `tilt_offset_deg` | `0.0` | Koreksi kemiringan tetap kalau posisi awal LiDAR tidak datar. |
| `record` | `true` | Auto `ros2 bag record`, berhenti sendiri saat sweep selesai. |
| `bag_dir` | `~/bags` | Folder penyimpanan rekaman. |
| `bag_prefix` | `scan` | Awalan nama bag → `scan_0001_3sweep`. |
| `compress` | `false` | Kompres rekaman dengan zstd. |

### Berapa lama satu sweep?

`steps_per_sweep = steps_per_rev / gear_ratio` = `1600 / 0.5` = **3200 step**:
1600 step dipetakan, 1600 step pulang.
Dengan `delay=0.018` → **≈ 29 detik mapping** + ≈ 4–5 detik pulang. RPLIDAR C1
mengeluarkan 5 kHz pada `scan_mode` DenseBoost, jadi ≈ **144 ribu titik** per sweep.
Jumlah itu separuh dari versi 360°, tapi versi 360° memang memotret ruang yang sama dua kali.

Konversinya: `delay = 0,01875 / rpm`. Jadi `delay:=0.009375` sama dengan `rpm:=2.0`
— 15 detik mapping.

Catatan: tiap step juga mem-publish sudut dan TF, dan itu makan ±0,7 ms di laptop
(di Pi mungkin lebih). Waktu nyata jadi sedikit lebih lama dari hitungan di atas.
Kode lama juga begitu. Sudut tetap benar karena dihitung dari jumlah step, bukan dari waktu.

Karena berhenti tepat di kelipatan 3200 step, sudut LiDAR dijamin kembali ke 0.000° —
tidak ada sisa miring, berapa pun jumlah sweep-nya.

---

## Topic

| Topic | Tipe | Arah | Keterangan |
|---|---|---|---|
| `/map_3d` | `PointCloud2` | keluar | Cloud kumulatif. Tiap pesan berisi **seluruh** titik sejauh ini. Frame `base_link`. |
| `/stepper/sweep_count` | `Int32` | keluar | Naik saat fase mapping 180° sebuah sweep selesai (latched). |
| `/stepper/sweep_done` | `Bool` | keluar | `true` saat sweep terakhir sudah pulang ke 0° (latched). |
| `/stepper/mapping_active` | `Bool` | keluar | `true` selama fase mapping, `false` selama fase pulang (latched). Ikut direkam di bag. |
| `/stepper/angle` | `Float32` | keluar | Sudut LiDAR (rad), sama seperti node lama. |
| `/stepper/steps`, `/stepper/status` | | keluar | Sama seperti node lama. |
| `/stepper/enable`, `/stepper/direction`, `/stepper/speed` | | masuk | Sama seperti node lama, masih berfungsi. |

---

## Kalau hasil peta miring

Sudut nol stepper adalah **posisi fisik LiDAR saat node dinyalakan**, bukan posisi datar
sebenarnya. Tidak ada limit switch atau encoder. Miring 5° saat start → peta miring 5°.

Cara mengoreksi:

1. Ekspor hasilnya ke `.pcd`, buka di CloudCompare, ukur kemiringan lantai/dinding.
2. Jalankan ulang dengan nilai berlawanan tandanya:

```bash
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=3 tilt_offset_deg:=-5.0
```

Nilainya menggeser sudut seluruh titik terhadap sumbu putar stepper. Sudah diverifikasi
memutar tepat sesuai nilai (`+7.0` → `+7.000°`, `-12.5` → `-12.500°`) tanpa mengubah
jarak antar titik.

Kalau miringnya **bertambah tiap sweep** (bukan tetap), penyebabnya bukan offset ini
melainkan motor kehilangan step. Uji dengan menempel selotip penanda di LiDAR, jalankan
`sweeps:=5`, lalu lihat apakah penanda kembali ke posisi semula. Kalau meleset,
perbesar `delay` (motor terlalu cepat sehingga slip) atau periksa ketegangan belt.

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
