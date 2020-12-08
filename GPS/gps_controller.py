import serial
import time
import adafruit_gps
import threading

class GPS_Handler(threading.Thread):
    def __init__(self, connect_flag=True):
        self.gps = None
        self.is_connected = False
        self.last_update_time = 0
        threading.Thread.__init__(self)
        if connect_flag:
            self.connect()

    def connect(self):
        uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        #Create a GPS module instance.
        self.gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
        # Turn on the basic GGA and RMC info (what you typcally want)
        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

        # Set update rate to once a second (1hz) which is what you typically want.
        self.gps.send_command(b"PMTK220,1000")
        # Or decrease to once every two seconds by doubling the millisecond value.
        # Be sure to also increase your UART timeout above!
        # gps.send_command(b'PMTK220,2000')
        # You can also speed up the rate, but don't go too fast or else you can lose
        # data during parsing.  This would be twice a second (2hz, 500ms delay):
        # gps.send_command(b'PMTK220,500')
        self.is_connected = True
        self.update_gps()
    def update_gps(self):
        # Make sure to call gps.update() every loop iteration and at least twice
        # as fast as data comes from the GPS unit (usually every second).
        # This returns a bool that's true if it parsed new data (you can ignore it
        # though if you don't care and instead look at the has_fix property).
        if self.is_connected:
            self.gps.update()
            new_time_update = time.monotonic()
            if new_time_update - self.last_update_time  >= 1.0:
                if not self.gps.has_fix:
                    # Try again if we don't have a fix yet.
                    print("Waiting for fix...")
                    return
                self.last_update_time = new_time_update
        else:
            return'Not Running'
    def get_gps_data(self):
        gps_output = dict()
        gps_output['timestamp'] = self.last_update_time
        gps_output['longitude'] = self.gps.longitude
        gps_output['latitude'] = self.gps.latitude
        if self.gps.altitude_m is not None:
            gps_output['altitude'] = self.gps.altitude_m
        else:
            gps_output['altitude'] = 'N/A'
        return gps_output

    def print_current_coordinate(self):
        print("=" * 40)  # Print a separator line.
        print(
            "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(                self.gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
                self.gps.timestamp_utc.tm_mday,  # struct_time object that holds
                self.gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                self.gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                self.gps.timestamp_utc.tm_min,  # month!
                self.gps.timestamp_utc.tm_sec,
            ))
        print("Latitude: {0:.6f} degrees".format(self.gps.latitude))
        print("Longitude: {0:.6f} degrees".format(self.gps.longitude))
        print("Fix quality: {}".format(self.gps.fix_quality))
        print("Fix quality: {}".format(self.gps.fix_quality))
        # Some attributes beyond latitude, longitude and timestamp are optional
        # and might not be present.  Check if they're None before trying to use!
        if self.gps.satellites is not None:
            print("# satellites: {}".format(self.gps.satellites))
        if self.gps.altitude_m is not None:
            print("Altitude: {} meters".format(self.gps.altitude_m))
        if self.gps.speed_knots is not None:
            print("Speed: {} knots".format(self.gps.speed_knots))
        if self.gps.track_angle_deg is not None:
            print("Track angle: {} degrees".format(self.gps.track_angle_deg))
        if self.gps.horizontal_dilution is not None:
            print("Horizontal dilution: {}".format(self.gps.horizontal_dilution))
        if self.gps.height_geoid is not None:
            print("Height geo ID: {} meters".format(self.gps.height_geoid))


if __name__ == '__main__':
    gpsp = GPS_Handler() # create the thread
    gpsp.start()
    while True:
        gpsp.update_gps()
        print(gpsp.get_gps_data())
        time.sleep(1)