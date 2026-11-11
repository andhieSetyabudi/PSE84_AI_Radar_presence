# PSoC Edge 60 GHz Radar Presence

Aplikasi ini menjalankan deteksi gerakan makro dan mikro menggunakan sensor radar XENSIV BGT60TR13C pada KIT_PSE84_AI. Hasil deteksi ditampilkan melalui debug UART.

## Arsitektur

Workspace terdiri dari tiga project yang bekerja bersama:

| Project | Tugas |
| --- | --- |
| `proj_cm33_s` | Secure boot dan konfigurasi keamanan perangkat. |
| `proj_cm33_ns` | Inisialisasi platform dan mengaktifkan core CM55. LED board juga berkedip dari project ini. |
| `proj_cm55` | Menginisialisasi radar melalui SPI, mengambil sampel FIFO, menjalankan algoritma presence, dan mencetak event ke UART. |

Konfigurasi register sensor berada di [proj_cm55/presence_radar_setting.h](proj_cm55/presence_radar_setting.h). Parameter algoritma, seperti rentang bin dan threshold, berada di `default_config` pada [proj_cm55/main.c](proj_cm55/main.c).

## Kebutuhan

- ModusToolbox 3.9 dan GNU Arm Embedded Compiler 14.2.1 (`GCC_ARM`) untuk konfigurasi build yang sudah diuji.
- KIT_PSE84_AI dengan sensor XENSIV BGT60TR13C yang terpasang dan terhubung ke pin radar sesuai BSP.
- Kabel USB KitProg3 dan terminal serial.

Source konfigurasi radar saat ini ditujukan untuk BGT60TR13C; sensor atau board lain mungkin memerlukan perubahan BSP, pin, dan register konfigurasi.

## Build dan program

Buka `Radar_presence.code-workspace` di VS Code. Jalankan task dari project yang sesuai:

1. `Build` pada folder `proj_cm55` untuk membangun aplikasi CM55 dan image gabungan.
2. `Build & Program:proj_cm33_s` untuk project CM33 secure.
3. `Build & Program:proj_cm33_ns` untuk project CM33 non-secure.
4. `Program` pada folder `proj_cm55` untuk memprogram image aplikasi CM55.

Pastikan semua project menggunakan konfigurasi `Debug` dan target `APP_KIT_PSE84_AI` yang sama. Task `Build` CM55 yang diuji menghasilkan image gabungan di `build/project_hex/`.

## Melihat output

Hubungkan terminal ke COM port KitProg3 dengan pengaturan `115200 baud, 8-N-1`. Setelah startup, output event dapat berbentuk:

```text
[INFO] macro presence <range_bin> <timestamp_ms>
[INFO] micro presence <range_bin> <timestamp_ms>
[INFO] absence <timestamp_ms>
```

`range_bin` adalah indeks jarak deteksi, bukan penghitung event. Timestamp menunjukkan waktu event dalam milidetik.

## Penyesuaian deteksi

Ubah `min_range_bin`, `max_range_bin`, `macro_threshold`, atau `micro_threshold` di `default_config` pada [proj_cm55/main.c](proj_cm55/main.c) untuk menyetel algoritma presence. Ubah tabel register di [proj_cm55/presence_radar_setting.h](proj_cm55/presence_radar_setting.h) hanya jika parameter akuisisi sensor memang perlu diganti; pastikan nilai frekuensi dan jumlah sampel tetap konsisten dengan konfigurasi algoritma.

Optimizer dari contoh CE241611 upstream tidak digunakan dalam project ini. Contoh tersebut memiliki arsitektur FreeRTOS dan alur akuisisi berbeda, jadi komponennya tidak dapat ditambahkan langsung tanpa porting.

## Lisensi

Lihat [LICENSE](LICENSE) untuk ketentuan lisensi. Komponen Infineon tetap tunduk pada lisensi dan disclaimer yang menyertai masing-masing komponen.
