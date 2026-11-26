import network
import socket
import ujson
import time
import gc
import os
import machine
import ustruct

DATA_FILE = "data.json"
HTML_FILE = "index.html"
PICO_IP = "192.168.4.1"

# ---------------------------------------------------------
# 1. Software I2C Setup (Fixes Swapped SDA/SCL GPIOs in Software)
# ---------------------------------------------------------
# Hardware I2C on RP2350 strictly forces GP4/GP20 = SDA and GP5/GP21 = SCL.
# Because SDA and SCL are swapped on the connector, machine.SoftI2C lets us
# assign any GPIO to SDA and SCL in software without repinning the cable!

def init_best_i2c():
    # Test swapped GP5/GP4 first (J2/J3), then swapped GP21/GP20 (J5), plus normal order fallbacks
    candidates = [
        ("J2/J3 Swapped (SDA=GP5, SCL=GP4)", 5, 4),
        ("J5 Swapped (SDA=GP21, SCL=GP20)", 21, 20),
        ("J2/J3 Normal (SDA=GP4, SCL=GP5)", 4, 5),
        ("J5 Normal (SDA=GP20, SCL=GP21)", 20, 21),
    ]
    
    for label, sda_gpio, scl_gpio in candidates:
        try:
            sda = machine.Pin(sda_gpio, machine.Pin.IN, machine.Pin.PULL_UP)
            scl = machine.Pin(scl_gpio, machine.Pin.IN, machine.Pin.PULL_UP)
            time.sleep_ms(10)
            # 100kHz SoftI2C makes 12-byte IMU reads take ~1.2ms while still supporting PA1010D clock-stretching
            bus = machine.SoftI2C(sda=sda, scl=scl, freq=100000, timeout=100000)
            found = bus.scan()
            if found:
                print(f"I2C Active on {label} -> Found: {[hex(x) for x in found]}")
                return bus, label
        except Exception as e:
            print(f"Bus check error on {label}: {e}")

    # Default to J2/J3 with swapped SDA=GP5, SCL=GP4 if no sensors are plugged in yet
    print("No I2C devices responded at boot; defaulting to SoftI2C(sda=GP5, scl=GP4)")
    sda = machine.Pin(5, machine.Pin.IN, machine.Pin.PULL_UP)
    scl = machine.Pin(4, machine.Pin.IN, machine.Pin.PULL_UP)
    return machine.SoftI2C(sda=sda, scl=scl, freq=100000, timeout=100000), "SDA=GP5, SCL=GP4"

i2c, active_bus_label = init_best_i2c()

IMU_ADDR = 0x6A  # ISM330DHCX default I2C address (alt is 0x6B)
GPS_ADDR = 0x10  # PA1010D default I2C address

telemetry = {
    "i2c_devices": [],
    "imu_ok": False,
    "gps_ok": False,
    "imu_hz": 0,
    "gps": {
        "fix": False,
        "lat": 0.0,
        "lon": 0.0,
        "speed_mph": 0.0,
        "speed_kmh": 0.0,
        "heading": 0.0,
        "sats": 0,
        "alt_m": 0.0
    },
    "imu": {
        "accel_x": 0.0,
        "accel_y": 0.0,
        "accel_z": 0.0,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0
    },
    "imu_batch": []
}

imu_initialized = False
gps_initialized = False
imu_fail_streak = 0
gps_fail_streak = 0
last_bus_scan_ms = 0
imu_sample_count = 0
imu_rate_window_ms = time.ticks_ms()

def init_gps():
    global gps_initialized
    try:
        # Tell PA1010D to only output RMC and GGA sentences every 1 update ($PMTK314)
        # Reduces I2C buffer load and power spikes significantly
        cmd = b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
        i2c.writeto(GPS_ADDR, cmd)
        gps_initialized = True
        telemetry["gps_ok"] = True
        print("PA1010D GPS configured at 0x10")
    except Exception:
        gps_initialized = False

def init_imu():
    global imu_initialized
    for addr in (0x6A, 0x6B):
        try:
            who = i2c.readfrom_mem(addr, 0x0F, 1)[0]
            if who == 0x6B:
                # CTRL3_C (0x12): Enable BDU (Block Data Update) + Auto-increment register address
                i2c.writeto_mem(addr, 0x12, b'\x44')
                # CTRL1_XL (0x10): 208Hz ODR (0x50) + +/- 8g full scale (0x0C) -> 0x5C
                i2c.writeto_mem(addr, 0x10, b'\x5c')
                # CTRL2_G (0x11): 208Hz ODR (0x50) + +/- 2000 dps full scale (0x0C) -> 0x5C
                i2c.writeto_mem(addr, 0x11, b'\x5c')
                imu_initialized = addr
                print(f"ISM330DHCX initialized at 0x{addr:02X} (208Hz ODR)")
                return
        except Exception:
            pass
    imu_initialized = False

def scan_and_update_sensors():
    global imu_initialized, gps_initialized, imu_fail_streak, gps_fail_streak, last_bus_scan_ms, imu_sample_count, imu_rate_window_ms
    now = time.ticks_ms()

    # Only run a full I2C bus scan on boot or every 5 seconds if both sensors are missing.
    if not telemetry["i2c_devices"] or ((not telemetry["imu_ok"] and not telemetry["gps_ok"]) and time.ticks_diff(now, last_bus_scan_ms) >= 5000):
        try:
            found = i2c.scan()
            if found:
                telemetry["i2c_devices"] = [f"0x{d:02X}" for d in found]
                if GPS_ADDR in found:
                    telemetry["gps_ok"] = True
        except Exception:
            pass
        last_bus_scan_ms = now

    # 1. High-Speed 100Hz IMU Poll
    if not imu_initialized:
        init_imu()
    if imu_initialized:
        try:
            raw = i2c.readfrom_mem(imu_initialized, 0x22, 12)
            gx, gy, gz, ax, ay, az = ustruct.unpack('<hhhhhh', raw)
            ax_g = round(ax * 0.000244, 3)
            ay_g = round(ay * 0.000244, 3)
            az_g = round(az * 0.000244, 3)
            gx_d = round(gx * 0.061, 1)
            gy_d = round(gy * 0.061, 1)
            gz_d = round(gz * 0.061, 1)

            telemetry["imu"]["accel_x"] = ax_g
            telemetry["imu"]["accel_y"] = ay_g
            telemetry["imu"]["accel_z"] = az_g
            telemetry["imu"]["gyro_x"]  = gx_d
            telemetry["imu"]["gyro_y"]  = gy_d
            telemetry["imu"]["gyro_z"]  = gz_d
            telemetry["imu_ok"] = True
            imu_fail_streak = 0

            # Append to high-rate batch buffer (capped at 12 samples between HTTP reads)
            batch = telemetry["imu_batch"]
            batch.append((ax_g, ay_g, az_g))
            if len(batch) > 12:
                batch.pop(0)

            # Calculate actual hardware IMU sampling Hz
            imu_sample_count += 1
            elapsed = time.ticks_diff(now, imu_rate_window_ms)
            if elapsed >= 1000:
                telemetry["imu_hz"] = round((imu_sample_count * 1000) / elapsed)
                imu_sample_count = 0
                imu_rate_window_ms = now
        except Exception:
            imu_fail_streak += 1
            if imu_fail_streak >= 15:
                telemetry["imu_ok"] = False
                imu_initialized = False

    # 2. GPS Poll (debounced: runs only once every 200ms inside update_gps)
    if not gps_initialized:
        init_gps()
    if update_gps():
        telemetry["gps_ok"] = True
        gps_fail_streak = 0
    else:
        gps_fail_streak += 1
        # 40 consecutive failed 200ms reads (~8 seconds) before declaring GPS disconnected
        if gps_fail_streak >= 40:
            telemetry["gps_ok"] = False
            telemetry["gps"]["fix"] = False
            gps_initialized = False

gps_buf = ""
last_gps_poll_ms = 0

def parse_nmea_lat(deg_str, hemi):
    if not deg_str or len(deg_str) < 4: return 0.0
    val = float(deg_str[:2]) + (float(deg_str[2:]) / 60.0)
    return -val if hemi == 'S' else val

def parse_nmea_lon(deg_str, hemi):
    if not deg_str or len(deg_str) < 5: return 0.0
    val = float(deg_str[:3]) + (float(deg_str[3:]) / 60.0)
    return -val if hemi == 'W' else val

def update_gps():
    global gps_buf, last_gps_poll_ms
    now = time.ticks_ms()
    # Read GPS I2C FIFO every 200ms using 32-byte chunks (PA1010D recommended I2C transaction size)
    if time.ticks_diff(now, last_gps_poll_ms) < 200:
        return True
    last_gps_poll_ms = now

    try:
        raw = i2c.readfrom(GPS_ADDR, 32)
        if not telemetry["i2c_devices"] or "0x10" not in telemetry["i2c_devices"]:
            if "0x10" not in telemetry["i2c_devices"]:
                telemetry["i2c_devices"].append("0x10")

        ascii_chars = "".join(chr(b) for b in raw if 32 <= b <= 126 or b in (10, 13))
        if not ascii_chars:
            return True

        gps_buf += ascii_chars

        if len(gps_buf) > 384:
            gps_buf = gps_buf[-192:]

        while "\n" in gps_buf:
            line, gps_buf = gps_buf.split("\n", 1)
            line = line.strip()

            if line.startswith("$GPRMC") or line.startswith("$GNRMC"):
                p = line.split(",")
                if len(p) >= 9:
                    telemetry["gps"]["fix"] = (p[2] == 'A')
                    if p[2] == 'A':
                        telemetry["gps"]["lat"] = round(parse_nmea_lat(p[3], p[4]), 6)
                        telemetry["gps"]["lon"] = round(parse_nmea_lon(p[5], p[6]), 6)
                        knots = float(p[7]) if p[7] else 0.0
                        telemetry["gps"]["speed_mph"] = round(knots * 1.15078, 1)
                        telemetry["gps"]["speed_kmh"] = round(knots * 1.852, 1)
                        telemetry["gps"]["heading"] = round(float(p[8]), 1) if p[8] else 0.0

            elif line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                p = line.split(",")
                if len(p) >= 10:
                    if p[6] and p[6] != '0':
                        telemetry["gps"]["fix"] = True
                    telemetry["gps"]["sats"] = int(p[7]) if p[7] else 0
                    telemetry["gps"]["alt_m"] = round(float(p[9]), 1) if p[9] else 0.0
        return True
    except Exception:
        return False

# Initial scan on boot
scan_and_update_sensors()
print("Initial I2C0 Scan (GP4/GP5):", telemetry["i2c_devices"])

# ---------------------------------------------------------
# 2. Flash Storage Persistence (Cars & Tracks)
# ---------------------------------------------------------
def load_data():
    try:
        with open(DATA_FILE, "r") as f:
            data = ujson.load(f)
            if "cars" in data and "tracks" in data:
                return data
    except Exception:
        pass
    initial = {"cars": [], "tracks": []}
    save_data(initial)
    return initial

def save_data(data):
    try:
        with open(DATA_FILE, "w") as f:
            ujson.dump(data, f)
    except Exception as e:
        print("Flash write error:", e)

# ---------------------------------------------------------
# 3. Start Wi-Fi Access Point
# ---------------------------------------------------------
time.sleep(0.5)
ap = network.WLAN(network.AP_IF)
ap.active(False)
time.sleep(0.1)
ap.active(True)
ap.config(essid="PitWall-Timer", password="pitwallpassword")

timeout = 10
while not ap.active() and timeout > 0:
    time.sleep(0.5)
    timeout -= 1

if not ap.active():
    print("Failed to initialize AP.")
else:
    print(f"AP Online! Captive portal active at http://{PICO_IP}")

# ---------------------------------------------------------
# 4. Setup Sockets (DNS UDP 53, HTTP TCP 80)
# ---------------------------------------------------------
dns_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dns_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
dns_sock.bind(('', 53))
dns_sock.setblocking(False)

http_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
http_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
http_sock.bind(('', 80))
http_sock.listen(4)
http_sock.setblocking(False)

def handle_dns():
    try:
        data, addr = dns_sock.recvfrom(512)
        if len(data) < 12:
            return
        tx_id = data[:2]
        idx = 12
        while idx < len(data) and data[idx] != 0:
            idx += data[idx] + 1
        idx += 5
        if idx > len(data):
            return
        question = data[12:idx]
        qtype = data[idx - 4 : idx - 2]

        if qtype == b"\x00\x01":
            ancount = b"\x00\x01"
            answer = b"\xc0\x0c\x00\x01\x00\x01\x00\x00\x00\x3c\x00\x04\xc0\xa8\x04\x01"
        else:
            ancount = b"\x00\x00"
            answer = b""

        response = tx_id + b"\x81\x80\x00\x01" + ancount + b"\x00\x00\x00\x00" + question + answer
        dns_sock.sendto(response, addr)
    except OSError:
        pass

def handle_http(conn):
    try:
        conn.settimeout(1.5)
        req_bytes = b""
        while b"\r\n\r\n" not in req_bytes:
            chunk = conn.recv(512)
            if not chunk:
                break
            req_bytes += chunk

        if not req_bytes:
            conn.close()
            return

        header_part, _, body_part = req_bytes.partition(b"\r\n\r\n")
        header_str = header_part.decode('utf-8', 'ignore')
        lines = header_str.split("\r\n")
        if not lines or not lines[0]:
            conn.close()
            return

        parts = lines[0].split(" ")
        if len(parts) < 2:
            conn.close()
            return

        method, path = parts[0], parts[1]
        host = ""
        content_length = 0
        for line in lines[1:]:
            l_lower = line.lower()
            if l_lower.startswith("host:"):
                host = line.split(":", 1)[1].strip()
            elif l_lower.startswith("content-length:"):
                content_length = int(line.split(":")[1].strip())

        if "favicon" in path or "apple-touch-icon" in path:
            conn.sendall(b"HTTP/1.1 204 No Content\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")

        elif method == "GET" and path == "/api/telemetry":
            payload = ujson.dumps(telemetry).encode('utf-8')
            # Clear batch after serializing so the next request gets fresh 100Hz samples
            telemetry["imu_batch"].clear()
            header = (
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json\r\n"
                f"Content-Length: {len(payload)}\r\n"
                "Connection: close\r\n\r\n"
            )
            conn.sendall(header.encode('utf-8') + payload)

        elif method == "GET" and path == "/api/data":
            data = load_data()
            payload = ujson.dumps(data).encode('utf-8')
            header = (
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json\r\n"
                f"Content-Length: {len(payload)}\r\n"
                "Connection: close\r\n\r\n"
            )
            conn.sendall(header.encode('utf-8') + payload)

        elif method == "POST" and path == "/api/data":
            body = body_part
            while len(body) < content_length:
                chunk = conn.recv(content_length - len(body))
                if not chunk:
                    break
                body += chunk
            try:
                new_data = ujson.loads(body.decode('utf-8'))
                if "cars" in new_data and "tracks" in new_data:
                    save_data(new_data)
                    conn.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 15\r\nConnection: close\r\n\r\n{\"status\":\"ok\"}")
                else:
                    conn.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
            except Exception:
                conn.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")

        elif not host.startswith(PICO_IP) or path in ["/hotspot-detect.html", "/generate_204", "/connecttest.txt", "/redirect"]:
            redirect = f"HTTP/1.1 302 Found\r\nLocation: http://{PICO_IP}/\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
            conn.sendall(redirect.encode('utf-8'))

        elif method == "GET" and (path == "/" or path == "/index.html"):
            try:
                file_size = os.stat(HTML_FILE)[6]
                header = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/html\r\n"
                    f"Content-Length: {file_size}\r\n"
                    "Connection: close\r\n\r\n"
                )
                conn.sendall(header.encode('utf-8'))
                with open(HTML_FILE, "rb") as f:
                    while True:
                        buf = f.read(512)
                        if not buf:
                            break
                        conn.sendall(buf)
            except Exception as e:
                print("Error serving file:", e)
                conn.sendall(b"HTTP/1.1 500 Internal Error\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
        else:
            conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")

    except Exception:
        pass
    finally:
        conn.close()

# ---------------------------------------------------------
# 5. Non-Blocking Event Loop (100Hz / 10ms IMU Sampling)
# ---------------------------------------------------------
last_sensor_poll = time.ticks_ms()
last_gc_ms = time.ticks_ms()

try:
    while True:
        handle_dns()

        try:
            conn, addr = http_sock.accept()
            handle_http(conn)
        except OSError:
            pass

        now = time.ticks_ms()
        # Poll IMU every 10ms (100Hz); GPS self-throttles to 200ms inside update_gps()
        if time.ticks_diff(now, last_sensor_poll) >= 10:
            scan_and_update_sensors()
            last_sensor_poll = now

        # Periodic garbage collection every 2 seconds instead of every HTTP frame
        if time.ticks_diff(now, last_gc_ms) >= 2000:
            gc.collect()
            last_gc_ms = now

        time.sleep_ms(1)
finally:
    dns_sock.close()
    http_sock.close()