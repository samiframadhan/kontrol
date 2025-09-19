import pygame
import math
import zmq
import logging
# Pustaka baru untuk deserialisasi MessagePack
import msgpack

# --- Konfigurasi Logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# --- Konfigurasi Pygame ---
pygame.init()
WIDTH, HEIGHT = 800, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("LIDAR Raw Data Visualizer (MessagePack)")
font = pygame.font.SysFont('monospace', 20, bold=True)

# --- Variabel State ---
# latest_points sekarang akan berisi list dictionary dengan key x_m, y_m, quality
latest_points = []
safety_status = "AMAN"

# --- Konfigurasi Plot & Warna ---
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
SCALE_PIXELS_PER_METER = 50.0  
MAX_DISTANCE_METERS = 10.0
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 200, 0)
RED_VEHICLE = (255, 0, 0)
GRID_COLOR = (40, 40, 40)
BOUNDARY_COLOR = (0, 100, 255)
UNSAFE_COLOR = (255, 100, 0)

# --- Konfigurasi Zona Aman (Sesuai dengan yang akan digunakan di control.py) ---
SAFETY_ZONE = {'min_x_m': -0.75, 'max_x_m': 0.75, 'min_y_m': 0.0, 'max_y_m': 2.0}

# --- Setup ZMQ ---
context = zmq.Context()
socket = context.socket(zmq.SUB)
# Hubungkan ke URL tempat program Rust mempublikasikan data
# Alamat ini harus sama dengan yang ada di program Rust. `ultra_simple.rs` menggunakan port 5551.
socket.connect("tcp://127.0.0.1:5551")
socket.setsockopt_string(zmq.SUBSCRIBE, "") 
logging.info("Visualizer siap. Menunggu data MessagePack dari Rust...")

# --- Fungsi Helper untuk Menggambar ---
def draw_safety_zone(surface, zone, color):
    """Menggambar kotak zona aman."""
    points_m = [
        (zone['min_x_m'], zone['max_y_m']),
        (zone['max_x_m'], zone['max_y_m']),
        (zone['max_x_m'], zone['min_y_m']),
        (zone['min_x_m'], zone['min_y_m'])
    ]
    
    points_pixels = []
    for x_m, y_m in points_m:
        px = int(x_m * SCALE_PIXELS_PER_METER + CENTER_X)
        py = int(-y_m * SCALE_PIXELS_PER_METER + CENTER_Y) # Y dibalik
        points_pixels.append((px, py))
        
    pygame.draw.lines(surface, color, True, points_pixels, 2)

# --- Loop Utama Pygame ---
running = True
while running:
    # --- Handle Events ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # --- Terima Data ---
    try:
        # MODIFIKASI: Terima data sebagai satu bagian (bukan multipart)
        packed_data = socket.recv(flags=zmq.NOBLOCK)
        
        # MODIFIKASI: Deserialisasi data menggunakan msgpack.unpackb()
        # Data dari Rust adalah list dari dictionary.
        raw_points_polar = msgpack.unpackb(packed_data)
        
        # MODIFIKASI: Konversi data dari polar (angle, distance) ke kartesius (x, y)
        print(f"Menerima {len(raw_points_polar)} titik dari Rust.")
        print(f"Contoh titik: {raw_points_polar[5] if raw_points_polar else 'Tidak ada titik'}")
        cartesian_points = []
        index = 0
        point_nums = len(raw_points_polar)
        for point in raw_points_polar:
            index += 1
            angle_rad = index / point_nums * 2 * math.pi  # Sudut dalam radian
            angle_rad += math.pi  # Offset agar 0 derajat mengarah ke atas
            distance_m = point[1]

            # Abaikan titik yang jaraknya 0 untuk menghindari titik menumpuk di tengah
            if distance_m > 0.0:
                # Konversi ke koordinat kartesius
                # Sumbu Y LIDAR biasanya mengarah ke depan
                x_m = distance_m * math.sin(angle_rad)
                y_m = distance_m * math.cos(angle_rad)
                
                cartesian_points.append({
                    'x_m': x_m,
                    'y_m': y_m,
                    'quality': point[2]
                })
        
        latest_points = cartesian_points
        
        # Evaluasi Keamanan Langsung di sini untuk tujuan visualisasi
        is_unsafe = False
        for point in latest_points:
            # MODIFIKASI: Gunakan akses dictionary `point['x_m']`
            if (SAFETY_ZONE['min_x_m'] <= point['x_m'] <= SAFETY_ZONE['max_x_m']) and \
               (SAFETY_ZONE['min_y_m'] <= point['y_m'] <= SAFETY_ZONE['max_y_m']):
                is_unsafe = True
                break
        
        safety_status = "TIDAK AMAN" if is_unsafe else "AMAN"

    except zmq.Again:
        pass  # Tidak ada data baru, lanjutkan

    # --- Proses Menggambar ---
    screen.fill(BLACK)  
    
    # Gambar grid jarak
    for r in range(1, int(MAX_DISTANCE_METERS) + 1):
        pygame.draw.circle(screen, GRID_COLOR, (CENTER_X, CENTER_Y), r * SCALE_PIXELS_PER_METER, 1)
    
    # Gambar zona aman
    draw_safety_zone(screen, SAFETY_ZONE, BOUNDARY_COLOR)

    # Gambar titik-titik data LIDAR
    for point in latest_points:
        # Kecerahan warna berdasarkan kualitas
        green_value = 40 + int((point['quality'] / 255.0) * 215)
        point_color = (0, green_value, 0)
        
        # Konversi koordinat kartesius (meter) ke pixel layar
        screen_x = int(point['x_m'] * SCALE_PIXELS_PER_METER + CENTER_X)
        screen_y = int(-point['y_m'] * SCALE_PIXELS_PER_METER + CENTER_Y) # Y perlu dibalik
        
        pygame.draw.circle(screen, point_color, (screen_x, screen_y), 2)
    
    # Gambar titik merah di tengah sebagai representasi kendaraan/sensor
    pygame.draw.circle(screen, RED_VEHICLE, (CENTER_X, CENTER_Y), 5)

    # Tampilkan teks status di layar
    status_color = UNSAFE_COLOR if safety_status == "TIDAK AMAN" else GREEN
    status_text = font.render(f'STATUS: {safety_status}', True, status_color)
    screen.blit(status_text, (10, 10))
    
    # Tampilkan jumlah titik yang diterima
    points_count_text = font.render(f'Points: {len(latest_points)}', True, WHITE)
    screen.blit(points_count_text, (10, 40))

    pygame.display.flip()

# --- Cleanup ---
pygame.quit()
socket.close()
context.term()
logging.info("Visualizer ditutup.")