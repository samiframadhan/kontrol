import pygame
import math
import zmq
import logging

# Impor Protobuf yang akan diterima
import obstacle_data_pb2

# --- Konfigurasi Logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# --- Konfigurasi Pygame ---
pygame.init()
WIDTH, HEIGHT = 800, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("LIDAR Processed Data Visualizer")
font = pygame.font.SysFont('monospace', 20, bold=True)

# --- Variabel State ---
latest_points = []
safety_status = "AMAN"

# --- Konfigurasi Plot & Warna ---
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
SCALE_PIXELS_PER_METER = 50.0  
MAX_DISTANCE_METERS = 8.0
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 200, 0)
RED_VEHICLE = (255, 0, 0)
GRID_COLOR = (40, 40, 40)
BOUNDARY_COLOR = (0, 100, 255)
UNSAFE_COLOR = (255, 100, 0)

# --- Konfigurasi Zona Aman (Sesuai dengan yang akan digunakan di control.py) ---
# Sebaiknya ini juga dibaca dari config.yaml, namun untuk visualizer kita hardcode
SAFETY_ZONE = {'min_x_m': -1.75, 'max_x_m': 1.75, 'min_y_m': 0.0, 'max_y_m': 2.0}

# --- Setup ZMQ ---
context = zmq.Context()
socket = context.socket(zmq.SUB)
# Hubungkan ke URL tempat lidar_processor_node mempublikasikan data
socket.connect("tcp://192.168.55.1:5571")
# Berlangganan ke topik data halangan
socket.setsockopt_string(zmq.SUBSCRIBE, "/dev/lidar_depan") 
logging.info("Visualizer siap. Menunggu data dari Lidar Processor Node...")

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
        # Konversi dari meter ke pixel
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
        # Terima data multipart (topik, pesan protobuf)
        topic, serialized_msg = socket.recv_multipart(flags=zmq.NOBLOCK)
        
        # Buat objek Protobuf dan parse pesannya
        lidar_scan_msg = obstacle_data_pb2.LidarScan()
        lidar_scan_msg.ParseFromString(serialized_msg)
        
        # Simpan list titik untuk digambar
        latest_points = lidar_scan_msg.points

        # Evaluasi Keamanan Langsung di sini untuk tujuan visualisasi
        is_unsafe = False
        for point in latest_points:
            # Cek apakah titik berada di dalam batas zona aman
            if (SAFETY_ZONE['min_x_m'] <= point.x_m <= SAFETY_ZONE['max_x_m']) and \
               (SAFETY_ZONE['min_y_m'] <= point.y_m <= SAFETY_ZONE['max_y_m']):
                is_unsafe = True
                break  # Ditemukan 1 titik, cukup, hentikan loop.
        
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

    # Gambar titik-titik data LIDAR (jika ada)
    for point in latest_points:
        # Kecerahan warna berdasarkan kualitas
        green_value = 40 + int((point.quality / 255.0) * 215)
        point_color = (0, green_value, 0)
        
        # Konversi koordinat kartesius (meter) ke pixel layar
        screen_x = int(point.x_m * SCALE_PIXELS_PER_METER + CENTER_X)
        screen_y = int(-point.y_m * SCALE_PIXELS_PER_METER + CENTER_Y) # Y perlu dibalik
        
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