#!/usr/bin/env python3
"""
GNSS NMEA to Prometheus Exporter
Reads NMEA sentences from UM980 receiver and exports metrics to Prometheus
"""

import socket
import time
import threading
import math
from datetime import datetime
from prometheus_client import start_http_server, Gauge, Counter, Info
import pynmea2

# Configuration
import os
GNSS_HOST = os.environ.get("GNSS_HOST", "192.168.68.68")
GNSS_PORT = int(os.environ.get("GNSS_PORT", 27))
PROMETHEUS_PORT = int(os.environ.get("PROMETHEUS_PORT", 8000))

# Prometheus Metrics
# Position metrics
latitude = Gauge('gnss_latitude_degrees', 'Current latitude in decimal degrees')
longitude = Gauge('gnss_longitude_degrees', 'Current longitude in decimal degrees')
altitude = Gauge('gnss_altitude_meters', 'Altitude above mean sea level in meters')

# Quality metrics
fix_quality = Gauge('gnss_fix_quality', 'GPS fix quality (0=invalid, 1=GPS fix, 2=DGPS fix, etc.)')
hdop = Gauge('gnss_hdop', 'Horizontal Dilution of Precision')
pdop = Gauge('gnss_pdop', 'Position Dilution of Precision')
vdop = Gauge('gnss_vdop', 'Vertical Dilution of Precision')
satellites_used = Gauge('gnss_satellites_used', 'Number of satellites used in solution')
satellites_visible = Gauge('gnss_satellites_visible', 'Number of satellites visible', ['constellation'])

# Base station specific metrics
station_position_error_m = Gauge('gnss_station_position_error_meters', 'Position error from reference in meters')
station_position_variance_lat = Gauge('gnss_station_position_variance_lat_meters', 'Latitude variance from reference (meters)')
station_position_variance_lon = Gauge('gnss_station_position_variance_lon_meters', 'Longitude variance from reference (meters)')
station_position_variance_alt = Gauge('gnss_station_position_variance_alt_meters', 'Altitude variance from reference (meters)')
station_fix_loss_count = Counter('gnss_station_fix_loss_count_total', 'Total number of fix losses')
station_time_since_last_move = Gauge('gnss_station_time_since_last_position_change_seconds', 'Seconds since last position change')

# Satellite individual metrics
satellite_elevation = Gauge('gnss_satellite_elevation_degrees', 'Satellite elevation above horizon', ['satellite_id', 'constellation'])
satellite_azimuth = Gauge('gnss_satellite_azimuth_degrees', 'Satellite azimuth direction', ['satellite_id', 'constellation'])
satellite_snr = Gauge('gnss_satellite_snr_db', 'Satellite signal-to-noise ratio in dB', ['satellite_id', 'constellation'])

# Satellite polar coordinates (for map visualization)
satellite_x = Gauge('gnss_satellite_x', 'Satellite X position (polar projection)', ['satellite_id', 'constellation'])
satellite_y = Gauge('gnss_satellite_y', 'Satellite Y position (polar projection)', ['satellite_id', 'constellation'])

# Accuracy metrics (from GNGST)
position_error_lat = Gauge('gnss_position_error_lat_meters', 'Latitude position error in meters')
position_error_lon = Gauge('gnss_position_error_lon_meters', 'Longitude position error in meters')
position_error_alt = Gauge('gnss_position_error_alt_meters', 'Altitude position error in meters')
rms_std_dev = Gauge('gnss_rms_std_dev_meters', 'RMS standard deviation of position')

# Velocity metrics
speed_knots = Gauge('gnss_speed_knots', 'Speed over ground in knots')
speed_kmh = Gauge('gnss_speed_kmh', 'Speed over ground in km/h')
course = Gauge('gnss_course_degrees', 'Course over ground in degrees')

# Counters
messages_received = Counter('gnss_messages_received_total', 'Total NMEA messages received', ['sentence_type'])
parse_errors = Counter('gnss_parse_errors_total', 'Total parsing errors', ['sentence_type'])

# Info metrics
receiver_info = Info('gnss_receiver', 'GNSS receiver information')
receiver_info.info({'model': 'UM980', 'host': GNSS_HOST, 'port': str(GNSS_PORT)})

# Last update timestamp
last_update = Gauge('gnss_last_update_timestamp', 'Unix timestamp of last successful update')


class GNSSReader:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.buffer = ""
        # Base station reference position
        self.reference_position = None  # Will be set on first fix
        self.last_position = None
        self.last_position_time = None
        self.last_fix_quality = 0
        self.position_samples = []  # For variance calculation
        
    def connect(self):
        """Establish connection to GNSS receiver"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((self.host, self.port))
            print(f"✓ Connected to GNSS receiver at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close connection to GNSS receiver"""
        if self.socket:
            self.socket.close()
            self.socket = None
            print("✓ Disconnected from GNSS receiver")
    
    def read_line(self):
        """Read a complete NMEA sentence from socket"""
        while '\n' not in self.buffer:
            try:
                data = self.socket.recv(1024).decode('ascii', errors='ignore')
                if not data:
                    return None
                self.buffer += data
            except socket.timeout:
                return None
            except Exception as e:
                print(f"✗ Read error: {e}")
                return None
        
        # Extract one line
        line, self.buffer = self.buffer.split('\n', 1)
        return line.strip()
    
    def parse_gngst(self, sentence):
        """Parse GNGST (GNSS Pseudorange Error Statistics)
        $GNGST,123944.00,3.36,2.61,2.07,21.1472,1.813,1.592,2.474*74
        """
        try:
            parts = sentence.split(',')
            if len(parts) >= 8:
                rms = float(parts[2]) if parts[2] else None
                std_major = float(parts[3]) if parts[3] else None
                std_minor = float(parts[4]) if parts[4] else None
                std_lat = float(parts[6]) if parts[6] else None
                std_lon = float(parts[7]) if parts[7] else None
                std_alt = float(parts[8].split('*')[0]) if parts[8] else None
                
                if rms is not None:
                    rms_std_dev.set(rms)
                if std_lat is not None:
                    position_error_lat.set(std_lat)
                if std_lon is not None:
                    position_error_lon.set(std_lon)
                if std_alt is not None:
                    position_error_alt.set(std_alt)
                    
                return True
        except Exception as e:
            print(f"✗ Error parsing GNGST: {e}")
            return False
    
    def parse_gpgsv(self, sentence):
        """Parse GSV (Satellites in View) - GPS/GLONASS/BeiDou/Galileo
        Format: $GPGSV,numMsg,msgNum,totalSats,sat1,elev1,azim1,snr1,...*checksum
        Example: $GPGSV,2,1,07,11,19,078,18,29,55,183,46,25,72,348,48,28,33,310,42,1*62
        """
        try:
            parts = sentence.split(',')
            
            # Determine constellation
            constellation = 'Unknown'
            if sentence.startswith('$GPGSV'):
                constellation = 'GPS'
            elif sentence.startswith('$GLGSV'):
                constellation = 'GLONASS'
            elif sentence.startswith('$GBGSV'):
                constellation = 'BeiDou'
            elif sentence.startswith('$GAGSV'):
                constellation = 'Galileo'
            
            if len(parts) >= 4:
                # Get total number of satellites
                total_sats = int(parts[3])
                satellites_visible.labels(constellation=constellation).set(total_sats)
                
                # Parse individual satellite data (4 satellites per message max)
                # Starting from index 4, each satellite has 4 fields: ID, elevation, azimuth, SNR
                for i in range(4, len(parts), 4):
                    if i + 3 < len(parts):
                        sat_id = parts[i].strip()
                        elevation = parts[i + 1].strip()
                        azimuth = parts[i + 2].strip()
                        snr = parts[i + 3].strip()
                        
                        # Remove checksum from last field if present
                        if '*' in snr:
                            snr = snr.split('*')[0]
                        
                        # Only export if we have valid data
                        if sat_id and elevation and azimuth:
                            try:
                                sat_id_str = f"{constellation[0]}{sat_id}"  # e.g., G29, R82, C25
                                elev_val = float(elevation)
                                azim_val = float(azimuth)
                                
                                satellite_elevation.labels(
                                    satellite_id=sat_id_str, 
                                    constellation=constellation
                                ).set(elev_val)
                                
                                satellite_azimuth.labels(
                                    satellite_id=sat_id_str, 
                                    constellation=constellation
                                ).set(azim_val)
                                
                                # SNR might be empty for satellites without lock
                                if snr:
                                    snr_val = float(snr)
                                    satellite_snr.labels(
                                        satellite_id=sat_id_str, 
                                        constellation=constellation
                                    ).set(snr_val)
                                
                                # Calculate polar coordinates for map visualization
                                elev_rad = math.radians(elev_val)
                                azim_rad = math.radians(azim_val)
                                r = 1  # Fixed radius for projection
                                x = r * math.cos(azim_rad) * math.sin(elev_rad)
                                y = r * math.sin(azim_rad) * math.sin(elev_rad)
                                
                                satellite_x.labels(
                                    satellite_id=sat_id_str,
                                    constellation=constellation
                                ).set(x)
                                satellite_y.labels(
                                    satellite_id=sat_id_str,
                                    constellation=constellation
                                ).set(y)
                            except ValueError:
                                pass  # Skip invalid numeric values
                
                return True
        except Exception as e:
            print(f"✗ Error parsing GSV: {e}")
            return False
    
    def parse_sentence(self, line):
        """Parse NMEA sentence and update Prometheus metrics"""
        if not line.startswith('$'):
            return
        
        # Get sentence type
        sentence_type = line[1:6]  # e.g., GNRMC, GNGST
        messages_received.labels(sentence_type=sentence_type).inc()
        
        try:
            # Handle special sentences not well supported by pynmea2
            if sentence_type == 'GNGST':
                if self.parse_gngst(line):
                    last_update.set(time.time())
                return
            
            if sentence_type in ['GPGSV', 'GLGSV', 'GBGSV', 'GAGSV']:
                if self.parse_gpgsv(line):
                    last_update.set(time.time())
                return
            
            # Parse with pynmea2
            msg = pynmea2.parse(line)
            
            # GNRMC - Recommended Minimum Navigation Information
            if isinstance(msg, pynmea2.types.talker.RMC):
                if msg.latitude and msg.longitude:
                    latitude.set(msg.latitude)
                    longitude.set(msg.longitude)
                if msg.spd_over_grnd is not None:
                    speed_knots.set(msg.spd_over_grnd)
                    speed_kmh.set(msg.spd_over_grnd * 1.852)
                if msg.true_course is not None:
                    course.set(msg.true_course)
                last_update.set(time.time())
            
            # GNGGA - Global Positioning System Fix Data
            elif isinstance(msg, pynmea2.types.talker.GGA):
                if msg.latitude and msg.longitude:
                    latitude.set(msg.latitude)
                    longitude.set(msg.longitude)
                    
                    # Base station monitoring
                    current_position = (msg.latitude, msg.longitude, msg.altitude if msg.altitude else 0)
                    
                    # Set reference position on first valid fix
                    if self.reference_position is None and msg.gps_qual and int(msg.gps_qual) >= 1:
                        self.reference_position = current_position
                        print(f"✓ Base station reference position set: {current_position}")
                    
                    # Monitor position variance
                    if self.reference_position:
                        lat_error_m = (msg.latitude - self.reference_position[0]) * 111000  # ~111km per degree
                        lon_error_m = (msg.longitude - self.reference_position[1]) * 111000 * math.cos(math.radians(msg.latitude))
                        alt_error_m = (msg.altitude if msg.altitude else 0) - self.reference_position[2]
                        
                        station_position_variance_lat.set(lat_error_m)
                        station_position_variance_lon.set(lon_error_m)
                        station_position_variance_alt.set(alt_error_m)
                        
                        # Total position error in meters
                        total_error = math.sqrt(lat_error_m**2 + lon_error_m**2 + alt_error_m**2)
                        station_position_error_m.set(total_error)
                        
                        # Track time since last movement
                        if self.last_position != current_position:
                            self.last_position = current_position
                            self.last_position_time = time.time()
                        elif self.last_position_time:
                            time_since_move = time.time() - self.last_position_time
                            station_time_since_last_move.set(time_since_move)
                
                if msg.altitude is not None:
                    altitude.set(msg.altitude)
                if msg.gps_qual is not None:
                    fix_quality_val = int(msg.gps_qual)
                    fix_quality.set(fix_quality_val)
                    
                    # Detect fix loss
                    if self.last_fix_quality >= 1 and fix_quality_val == 0:
                        station_fix_loss_count.inc()
                        print(f"✗ Fix lost at {datetime.now().isoformat()}")
                    elif self.last_fix_quality == 0 and fix_quality_val >= 1:
                        print(f"✓ Fix recovered at {datetime.now().isoformat()}")
                    self.last_fix_quality = fix_quality_val
                
                if msg.num_sats is not None:
                    satellites_used.set(int(msg.num_sats))
                if msg.horizontal_dil is not None:
                    hdop.set(float(msg.horizontal_dil))
                last_update.set(time.time())
            
            # GNGSA - GNSS DOP and Active Satellites
            elif isinstance(msg, pynmea2.types.talker.GSA):
                # Update satellite count if available
                sats = [s for s in [msg.sv_id01, msg.sv_id02, msg.sv_id03, msg.sv_id04,
                                    msg.sv_id05, msg.sv_id06, msg.sv_id07, msg.sv_id08,
                                    msg.sv_id09, msg.sv_id10, msg.sv_id11, msg.sv_id12] if s]
                if sats:
                    satellites_used.set(len(sats))
                
                # Extract DOP values
                if msg.pdop is not None:
                    pdop.set(float(msg.pdop))
                if msg.hdop is not None:
                    hdop.set(float(msg.hdop))
                if msg.vdop is not None:
                    vdop.set(float(msg.vdop))
                
                last_update.set(time.time())
                
        except pynmea2.ParseError as e:
            parse_errors.labels(sentence_type=sentence_type).inc()
            print(f"✗ Parse error for {sentence_type}: {e}")
        except Exception as e:
            parse_errors.labels(sentence_type=sentence_type).inc()
            print(f"✗ Unexpected error parsing {sentence_type}: {e}")
    
    def run(self):
        """Main loop to read and process NMEA sentences"""
        self.running = True
        reconnect_delay = 5
        
        while self.running:
            if not self.socket:
                if not self.connect():
                    print(f"Retrying in {reconnect_delay} seconds...")
                    time.sleep(reconnect_delay)
                    continue
            
            line = self.read_line()
            if line is None:
                print("✗ Connection lost, reconnecting...")
                self.disconnect()
                time.sleep(reconnect_delay)
                continue
            
            if line:
                self.parse_sentence(line)
    
    def stop(self):
        """Stop the reader"""
        self.running = False
        self.disconnect()


def main():
    """Main entry point"""
    print("=" * 60)
    print("GNSS NMEA to Prometheus Exporter")
    print("=" * 60)
    print(f"GNSS Receiver: {GNSS_HOST}:{GNSS_PORT}")
    print(f"Metrics endpoint: http://0.0.0.0:{PROMETHEUS_PORT}/metrics")
    print("=" * 60)
    
    # Start Prometheus HTTP server
    start_http_server(PROMETHEUS_PORT)
    print(f"✓ Prometheus metrics server started on port {PROMETHEUS_PORT}")
    
    # Create and start GNSS reader
    reader = GNSSReader(GNSS_HOST, GNSS_PORT)
    
    # Run in thread
    reader_thread = threading.Thread(target=reader.run, daemon=True)
    reader_thread.start()
    
    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n✓ Shutting down...")
        reader.stop()
        reader_thread.join(timeout=5)
        print("✓ Goodbye!")


if __name__ == "__main__":
    main()
