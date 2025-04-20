
import os
import time
import math
import csv
import serial
import pynmea2
import RPi.GPIO as GPIO
from itertools import product

# ─── CONFIGURATION ─────────────────────────────────────────────────────────────
DATA_DIR = "/home/pi/data"
OUTPUT_FILE = os.path.join(DATA_DIR, "all_samples.csv")
SAMPLING_POINTS_FILE = os.path.join(DATA_DIR, "sampling_points.csv")
LAND_BOUNDARY_FILE = os.path.join(DATA_DIR, "land_boundary.csv")
S = 5  # Spacing in meters (adjust based on sensor coverage area)

LEFT_FWD, LEFT_REV = 17, 18
RIGHT_FWD, RIGHT_REV = 22, 23
LEFT_ENC, RIGHT_ENC = 5, 6
DRILL_PIN, PROBE_PIN = 24, 25

WHEEL_D_CM = 5.0
TICKS_PER_REV = 20
WHEEL_C_CM = math.pi * WHEEL_D_CM
TICKS_PER_CM = TICKS_PER_REV / WHEEL_C_CM
REV_DIST_CM = 60.0
TARGET_TICKS_REV = int(REV_DIST_CM * TICKS_PER_CM)

left_ticks = 0
right_ticks = 0

# ─── GPIO SETUP ────────────────────────────────────────────────────────────────
def tick_callback(ch):
    global left_ticks, right_ticks
    if ch == LEFT_ENC: left_ticks += 1
    else: right_ticks += 1

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([LEFT_FWD, LEFT_REV, RIGHT_FWD, RIGHT_REV], GPIO.OUT)
    GPIO.setup([LEFT_ENC, RIGHT_ENC], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENC, GPIO.RISING, callback=tick_callback)
    GPIO.add_event_detect(RIGHT_ENC, GPIO.RISING, callback=tick_callback)
    GPIO.setup([DRILL_PIN, PROBE_PIN], GPIO.OUT)

def cleanup():
    GPIO.cleanup()

# ─── MOTOR CONTROL ─────────────────────────────────────────────────────────────
def forward():
    GPIO.output([LEFT_FWD, RIGHT_FWD], 1)
    GPIO.output([LEFT_REV, RIGHT_REV], 0)

def reverse():
    GPIO.output([LEFT_FWD, RIGHT_FWD], 0)
    GPIO.output([LEFT_REV, RIGHT_REV], 1)

def stop():
    GPIO.output([LEFT_FWD, LEFT_REV, RIGHT_FWD, RIGHT_REV], 0)

def reverse_60cm():
    global left_ticks, right_ticks
    left_ticks = right_ticks = 0
    reverse()
    while (left_ticks + right_ticks) / 2 < TARGET_TICKS_REV:
        time.sleep(0.005)
    stop()

# ─── GPS (RTK NEO-M8P-2) ───────────────────────────────────────────────────────
def read_rtk_gps():
    ser = serial.Serial("/dev/ttyAMA0", 9600, timeout=1)
    start_time = time.time()
    while time.time() - start_time < 10:
        line = ser.readline().decode('ascii', errors='ignore')
        if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
            try:
                msg = pynmea2.parse(line)
                return {
                    "lat": msg.latitude,
                    "lon": msg.longitude,
                    "alt": float(msg.altitude)
                }
            except:
                continue
    return {"lat": float('nan'), "lon": float('nan'), "alt": float('nan')}

# ─── NPK SENSOR ────────────────────────────────────────────────────────────────
def read_npk():
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
    ser.write(b'GET_NPK\n')
    line = ser.readline().decode().strip()
    ser.close()
    try:
        vals = [float(v) for v in line.split(',')]
        return {
            "N": vals[0], "P": vals[1], "K": vals[2],
            "pH": vals[3], "Moist": vals[4], "EC": vals[5], "Temp": vals[6]
        }
    except:
        return {}

# ─── ACTUATORS ─────────────────────────────────────────────────────────────────
def do_drill():
    GPIO.output(DRILL_PIN, 1)
    time.sleep(3)
    GPIO.output(DRILL_PIN, 0)

def do_probe():
    GPIO.output(PROBE_PIN, 1)
    time.sleep(2)
    GPIO.output(PROBE_PIN, 0)

# ─── GRID GENERATION ───────────────────────────────────────────────────────────
def is_inside(point, polygon):
    testx, testy = point
    nvert = len(polygon)
    vertx = [p[0] for p in polygon]
    verty = [p[1] for p in polygon]
    c = 0
    for i in range(nvert):
        j = i - 1 if i != 0 else nvert - 1
        if ((verty[i] > testy) != (verty[j] > testy)) and (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]):
            c += 1
    return c % 2 == 1

def generate_sampling_points(boundary, S):
    lats = [p[0] for p in boundary]
    lons = [p[1] for p in boundary]
    min_lat = min(lats)
    max_lat = max(lats)
    min_lon = min(lons)
    max_lon = max(lons)
    φ_avg = (min_lat + max_lat) / 2
    Δlat_deg = S / 111320
    Δlon_deg = S / (111320 * math.cos(math.radians(φ_avg)))
    n_lat = int((max_lat - min_lat) / Δlat_deg) + 1
    n_lon = int((max_lon - min_lon) / Δlon_deg) + 1
    lat_values = [min_lat + i * Δlat_deg for i in range(n_lat)]
    lon_values = [min_lon + j * Δlon_deg for j in range(n_lon)]
    grid_points = list(product(lat_values, lon_values))
    sampling_points = [p for p in grid_points if is_inside(p, boundary)]
    return sampling_points

def save_to_csv(points, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["idx", "lat", "lon"])
        for idx, p in enumerate(points, start=1):
            writer.writerow([idx, p[0], p[1]])

# ─── DATA STORAGE ──────────────────────────────────────────────────────────────
def ensure_data_dir():
    os.makedirs(DATA_DIR, exist_ok=True)

def save_all_samples(samples):
    fieldnames = ["idx", "lat", "lon", "alt", "N", "P", "K", "pH", "Moist", "EC", "Temp"]
    with open(OUTPUT_FILE, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            flat_sample = {
                "idx": sample["idx"],
                "lat": sample["position"]["lat"],
                "lon": sample["position"]["lon"],
                "alt": sample["position"]["alt"],
                "N": sample["npk"].get("N", float('nan')),
                "P": sample["npk"].get("P", float('nan')),
                "K": sample["npk"].get("K", float('nan')),
                "pH": sample["npk"].get("pH", float('nan')),
                "Moist": sample["npk"].get("Moist", float('nan')),
                "EC": sample["npk"].get("EC", float('nan')),
                "Temp": sample["npk"].get("Temp", float('nan')),
            }
            writer.writerow(flat_sample)

# ─── NAVIGATION (PLACEHOLDER) ──────────────────────────────────────────────────
def navigate_to(target_lat, target_lon):
    print(f"Navigating to {target_lat}, {target_lon}")
    # Implement navigation logic here (e.g., use GPS and motor control to reach target)

# ─── HAVERSINE DISTANCE ────────────────────────────────────────────────────────
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

# ─── MAIN EXECUTION ────────────────────────────────────────────────────────────
def main():
    setup_gpio()
    ensure_data_dir()

    # Generate sampling points if not already present
    if not os.path.exists(SAMPLING_POINTS_FILE):
        with open(LAND_BOUNDARY_FILE, 'r') as f:
            reader = csv.reader(f)
            boundary = [(float(row[0]), float(row[1])) for row in reader]
        points = generate_sampling_points(boundary, S)
        save_to_csv(points, SAMPLING_POINTS_FILE)

    all_samples = []
    with open(SAMPLING_POINTS_FILE, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            idx = int(row["idx"])
            target_lat = float(row["lat"])
            target_lon = float(row["lon"])
            navigate_to(target_lat, target_lon)
            # Wait until close enough
            while True:
                current_gps = read_rtk_gps()
                if current_gps["lat"] is float('nan') or current_gps["lon"] is float('nan'):
                    time.sleep(1)
                    continue
                dist = haversine(current_gps["lat"], current_gps["lon"], target_lat, target_lon)
                if dist < 1:  # 1 meter tolerance
                    break
                time.sleep(1)
            # Collect data
            do_drill()
            reverse_60cm()
            do_probe()
            gps = read_rtk_gps()
            npk = read_npk()
            payload = {
                "idx": idx,
                "position": gps,
                "npk": npk
            }
            all_samples.append(payload)
            print(f"Point {idx} recorded.")
            time.sleep(1)

    save_all_samples(all_samples)
    print(f"✅ All samples saved to {OUTPUT_FILE}")
    cleanup()

if _name_ == "_main_":
    main()
