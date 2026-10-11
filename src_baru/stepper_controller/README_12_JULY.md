# Catatan Perubahan Kode (Changelog 12 Juli)

Berikut adalah daftar faktual dari semua perubahan kode pada file yang telah dilakukan pada hari ini (12 Juli):

## 1. `mapping_3d_fastlio_scan.py` (FILE BARU)
**Status:** Dibuat baru
*   **Perubahan:** 
    *   Membuat *node* baru yang mengimplementasikan arsitektur **Per-Scan Publishing**.
    *   **Menghapus buffer akumulasi:** Variabel `self.current_sweep_points` dihapus. Sistem tidak lagi mengumpulkan data selama 1 putaran penuh (Full Sweep).
    *   **Publishing Instan:** `PointCloud2` langsung di-publish setiap kali menerima 1 pesan `LaserScan` (~10Hz).
    *   **Timestamp Relatif:** `offset_time` direvisi agar hanya menghitung selisih waktu dalam satu garis *scan* (nilai mulai dari `0.0` hingga `~0.073` detik).
    *   Rotasi fisik dari motor stepper menggunakan `scipy.spatial.transform.Rotation` tetap dipertahankan.

## 2. `setup.py` (DIMODIFIKASI)
**Status:** Berhasil disimpan
*   **Perubahan:** 
    *   Menambahkan baris `entry_points` baru di dalam `console_scripts` agar *node* baru bisa dieksekusi.
    *   Kode yang ditambahkan: `'mapping_3d_fastlio_scan = stepper_controller.mapping_3d_fastlio_scan:main'`

## 3. `stepper_node.py` (DIMODIFIKASI LALU DI-REVERT)
**Status:** Kembali ke kode asli (`time.sleep`)
*   **Perubahan yang Sempat Dilakukan:** 
    *   Menulis ulang blok `motor_loop()` untuk menghapus fungsi *software delay* `time.sleep()`.
    *   Mengimplementasikan fungsi *Hardware PWM* menggunakan `lgpio.tx_pwm(self.h, STEP_PIN, int(target_freq), 50.0)`.
    *   Mengubah cara menghitung `current_step`. Daripada mengandalkan `+= 1` di dalam iterasi *loop*, perhitungan diubah menggunakan waktu presisi `time.monotonic()` dengan rumus `steps_taken = elapsed * target_freq`.
*   **Alasan Revert:** 
    *   Mendapatkan pesan *Error* `[ros2run]: Segmentation fault` saat dieksekusi.
    *   Driver `lgpio` mengalami *crash* saat mencoba memanggil `tx_pwm` pada `STEP_PIN=17` karena pin tersebut tidak memiliki kemampuan *Hardware PWM* di *chip* RP1 Raspberry Pi 5 (hanya pin 12, 13, 18, 19).

## 4. `spark_fast_lio/config/sllidar_stepper.yaml` (DIMODIFIKASI LALU DI-REVERT)
**Status:** Kembali ke pengaturan asli
*   **Perubahan yang Sempat Dilakukan:** 
    *   Mengubah parameter `filter_size_map` dari `0.3` menjadi `0.05`.
    *   Menambahkan (uncomment) parameter `point_filter_num: 1`.
*   **Alasan Revert:**
    *   Dikembalikan oleh *user* ke bentuk awal (`0.3` dan dikomen) untuk keperluan pengujian *baseline*.

---

## 5. Glosarium & Konsep Operasional FAST-LIO

Berdasarkan pengujian dan analisis *log* terminal hari ini, berikut adalah penjelasan parameter vital yang mencerminkan keberhasilan atau kegagalan *mapping*:

### A. `feats_down` (Features Downsampled)
Merupakan jumlah total titik LiDAR yang berhasil masuk ke algoritma pencocokan FAST-LIO dalam satu *frame* **setelah** melewati filter voxel (`filter_size_map`) dan filter kerapatan (`point_filter_num`).
*   **Jika terlalu kecil (cth: 92 poin):** Berarti *setting* YAML terlalu agresif membuang titik. LiDAR menjadi buta karena resolusinya terlalu kasar.
*   **Jika besar (cth: 11.731 poin):** Berarti *setting* YAML sudah rapat (contoh voxel 5cm). Sensor melihat bentuk geometri ruangan dengan sangat tajam dan kaya.

### B. `effective` (Effective Points)
Merupakan jumlah titik (dari *feats_down*) yang **berhasil menemukan pasangannya** di dalam memori peta IKD-Tree, dan akan dipakai untuk mengoreksi posisi (Odometri) dari sensor IMU.
*   **Proses Mencari Pasangan:** Titik LiDAR baru akan dicari 5 tetangga terdekatnya di peta lama. Jika kelima tetangga tersebut dikalkulasi terbukti membentuk bidang datar/tembok (lolos uji `esti_plane` dengan batas kerataan 10 cm), titik tersebut dianggap sukses berpasangan (*effective*).
*   **Makna `effective = 0`:** Berarti **tidak ada satupun** titik yang berhasil menemukan tembok yang rata di dalam Peta. Hal ini umumnya terjadi karena titik terlalu menyebar (Voxel besar, radius terlampaui), atau karena posisi IMU sudah terlanjur bergeser melayang (*drift*), membuat peta di otak menjadi sangat tebal. Jika nilainya 0, sistem gagal melakukan koreksi posisi.

### C. Alur Pemrosesan Peta (Map Flow)
Urutan perjalanan data dari Lidar hingga menjadi File Peta 3D:
1.  **Voxelization & Downsampling:** Data masuk ke FAST-LIO lalu langsung di-potong sesuai dengan aturan `filter_size_map` dan `point_filter_num`.
2.  **Matching (Pencocokan):** Titik dicocokkan dengan Peta IKD-Tree (menghasilkan angka `effective`).
3.  **State Update:** Jika `effective` mencukupi, Kalman Filter akan mengunci dan membenarkan posisi gerak robot di dunia virtual.
4.  **Injeksi Memori (IKD-Tree Update):** Titik-titik baru yang posisinya sudah dikoreksi lalu disuntikkan secara permanen untuk memperbesar peta IKD-Tree di memori RAM.
5.  **Visualisasi RViz:** Secara bersamaan, titik baru di-publish ke ROS Topic `/cloud_registered`. (Di RViz, peta bisa ditumpuk agar terlihat utuh menggunakan pengaturan `Decay Time = 10000`).
6.  **Ekspor File (PCD Save):** Ketika *node* FAST-LIO ditekan mati (Ctrl+C), parameter `pcd_save_en: true` akan memicu FAST-LIO mengekstrak semua titik di IKD-Tree menjadi satu buah file ekstensif `.pcd` yang tersimpan di *hardisk*.
