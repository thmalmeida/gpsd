#!/usr/bin/python3
# Autor - thmalmeida on 20240819
# ref.: https://gpsd.gitlab.io/gpsd/NMEA.html

import serial
import threading
import os           # for clear screan only

# Data structure in class format
class GNSS_GGA: # GGA - Global Positioning System Fix Data - Time, Position and fix related data for a GPS receiver.
    def __init__(self):
        self.hour = 0           # Field 1 - UTC of this position report, hh is hours, mm is minutes, ss.ss is seconds.
        self.min = 0
        self.sec = 0
        self.utc_time = 0
        
        self.latitude = 0       # Field 2 - Latitude, dd is degrees, mm.mm is minutes
        self.latdeg_dec = 0.0
        self.latdeg = 0
        self.latmin = 0.0
        self.latsec = 0.0
        self.lat_index = '_ '   # Field 3 - N or S (North or South)
        
        self.longitude = 0      # Field 4 - Longitude, dd is degrees, mm.mm is minutes 
        self.londeg_dec = 0.0
        self.londeg = 0
        self.lonmin = 0.0
        self.lonsec = 0.0
        self.lon_index = '_'    # Field 5 - E or W (East or West)

        self.quality = 0        # Field 6 - GPS Quality Indicator (non null)
                                    # 0 - fix not available,
                                    # 1 - GPS fix,
                                    # 2 - Differential GPS fix (values above 2 are 2.3 features)
                                    # 3 = PPS fix
                                    # 4 = Real Time Kinematic
                                    # 5 = Float RTK
                                    # 6 = estimated (dead reckoning)
                                    # 7 = Manual input mode
                                    # 8 = Simulation mode
        self.n_satts = 0        # Field 7 - Number of satellites in use, 00 - 12
        self.hdop = 0.0         # Field 8 - Horizontal Dilution of precision (meters)
        self.altitude = 0       # Field 8 - Antenna Altitude above/below mean-sea-level (geoid) (in meters)
        self.units_pos = 0      # Field 10- Units of antenna altitude, meters
        self.geoidal_separation = 0 
        self.units_gs = 0
        self.age_of_correction = 0 
        self.checksum = 0

        def decode(str_cmd):
            case "GGA":	# Time, position, fix type data.
            if str_cmd[2] != "":
            gpsd0.hour = int(str_cmd[1][0:2])
            gpsd0.min = int(str_cmd[1][2:4])
            gpsd0.sec = int(str_cmd[1][4:6])
            gpsd0.latdeg = float(str_cmd[2][:2])
            gpsd0.latdeg_dec = gpsd0.latdeg + float(str_cmd[2][2:])/60

            # gpsd0.latmin = str_cmd[2][2:4]
            # gpsd0.latsec = str_cmd[2][5:]*60
            # gpsd0.latitude = float(str_cmd[2])/100
            gpsd0.lat_index = str_cmd[3]

            # gpsd0.longitude = float(str_cmd[4])/100
            gpsd0.londeg = float(str_cmd[4][:3])
            gpsd0.londeg_dec = gpsd0.londeg + float(str_cmd[4][3:])/60
            gpsd0.lon_index = str_cmd[5]
            gpsd0.quality = int(str_cmd[6])
            gpsd0.num_satt = int(str_cmd[7])
            gpsd0.hdop = float(str_cmd[8])
            gpsd0.altitude = str_cmd[9]
            gpsd0.units_pos = str_cmd[10]
            gpsd0.geoidal_separation = str_cmd[11]
            gpsd0.units_gs = str_cmd[12]
            gpsd0.age_of_correction = str_cmd[13]
            # correction_station_ID = str_cmd[14]
            gpsd0.checksum = str_cmd[14][2:]
class GNSS_GSA: # GSA - GPS DOP and active satellites
    def __init__(self):
        self.sel_mode = "_"         # Field 1 - Selection mode: M=Manual, forced to operate in 2D or 3D, A=Automatic, 2D/3D
        self.mode = 0;			    # Field 2 - Mode (1 = no fix, 2 = 2D fix, 3 = 3D fix)
        self.satt_id_group = []     # Field 3 to 14 - ID of 1st to 12th satellite used for fix
        for i in range(0,12):
            self.satt_id_group.append(0)
        self.pdop = 0.0             # Field 15 - PDOP
        self.hdop = 0.0             # Field 16 - HDOP
        self.vdop = 0.0             # Field 17 - VDOP
        self.sys_id = 0	            # Field 18 - System ID (NMEA 4.11), see above
                                        # 1 = GPS L1C/A, L2CL, L2CM
                                        # 2 = GLONASS L1 OF, L2 OF
                                        # 3 = Galileo E1C, E1B, E5 bl, E5 bQ
                                        # 4 = BeiDou B1I D1, B1I D2, B2I D1, B2I D12
        self.num_used = 0
        self.num_visible = 0
    
    def decode(str_cmd):
        gpsd0.s_mode = int(ans2[2])
        sys_id = int(ans2[18][:1])         # Constellation type.
        satt_id_group = []
        satt_id_group_num = 0
        for x in range(3,15):
        try:
        satt_id_group.append(int(ans2[x]))
        satt_id_group_num += 1
        # gpsd0.satt_id[gpsd0.sys_id - 1][x-3] = int(ans2[x])
        except:
        satt_id_group.append(0)                        # gpsd0.satt_id[gpsd0.sys_id - 1][x-3] = 0

        gpsd0.satt_id[sys_id-1] = satt_id_group
class GNSS_RMC: # RMC - Recommended Minimum Navigation Information
    def __init__(self):
        self.utc_hour = 0           # Field 1 - UTC of position fix, hh is hours, mm is minutes, ss.ss is seconds.
        self.utc_min = 0
        self.utc_sec = 0.0
        self.status = '_'           # Field 2 - Status, A = Valid, V = Warning
        # Field 3 - Latitude, dd is degrees. mm.mm is minutes.
        # Field 4 - N or S
        # Field 5 - Longitude, ddd is degrees. mm.mm is minutes.
        # Field 6 - E or W
        # Field 7 - Speed over ground, knots
        # Field 8 - Track made good, degrees true
        # Field 9 - Date, ddmmyy
        # Field 10- Magnetic Variation, degrees
        # Field 11- E or W
        # Field 12- FAA mode indicator (NMEA 2.3 and later)
        # Field 13- Nav Status (NMEA 4.1 and later) A=autonomous, D=differential, E=Estimated, M=Manual input mode N=not valid, S=Simulator, V = Valid
        # Field 14- Checksum
        # Example: $GNRMC,001031.00,A,4404.13993,N,12118.86023,W,0.146,,100117,,,A*7B
class GNSS_VTG: # VTG - Track made good and Ground speed
    def __init__(self):
        self.course = 0.0       # Field 1 - Course over ground, degrees True
            # Field 2 - T = True
            # Field 3 - Course over ground, degrees Magnetic
            # Field 4 - M = Magnetic
            # Field 5 - Speed over ground, knots
            # Field 6 - N = Knots
        self.speed = 0.0        # Field 7 - Speed over ground, km/h
            # Field 8 - K = Kilometers Per Hour
            # Field 9 - FAA mode indicator (NMEA 2.3 and later)
            # Field 10- Checksum
class GNSS_GSV: # GSV - Satellites in view
    def __init__(self):
        self.n = 0          # Field 1 - total number of GSV sentences to be transmitted in this group
        self.i = 0          # Field 2 - Sentence number, 1-9 of this GSV message within current group. Current GSV msg
        self.n_satt = 0     # Field 3 - total number of satellites in view (leading zeros sent)
        self.satt_id = 0    # Field 4 - satellite ID or PRN number (leading zeros sent)
        self.elevation = 0  # Field 5 - elevation in degrees (-90 to 90) (leading zeros sent)
        self.azimuth = 0    # Field 6 - azimuth in degrees to true north (000 to 359) (leading zeros sent)
        self.snr = 0        # Field 7 - SNR in dB (00-99) (leading zeros sent) more satellite info quadruples like 4-7 n-1) Signal ID (NMEA 4.11) n) checksum

    class SATT_INFO:
        def __init__(self):
            self.id = 0
            self.elevation = 0
            self.azimuth = 0
            self.snr = 0
class GNSS_ZDA: # ZDA - Time & Date - UTC, day, month, year and local time zone
    def __init__(self): # $--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*hh<CR><LF>
        self.UTC_time = "0000"  # Field 1 - UTC time (hours, minutes, seconds, may have fractional subseconds)
        self.day = 1            # Field 2 - day 01 to 31
        self.month = 1          # Field 3 - month 01 to 12
        self.year = 2000        # Field 4 - Year (4 digits)
        self.loc_zone_hour = 0  # Field 5 - Local zone description, 00 to +- 13 hours
        self.loc_zone_min = 00  # Field 6 - Local zone minutes description, 00 to 59, apply same sign as local hours
        self.checksum = 0
class GNSS_GLL: # GLL - Geographic Position - Latitude/Longitude
    def __init__(self): #  $--GLL,ddmm.mm,a,dddmm.mm,a,hhmmss.ss,a,m*hh<CR><LF>
        self.lat = 0        # Field 1 - Latitude, dd is degrees, mm.mm is minutes
        self.lat_i = '_'    # Field 2 - N or S (North or South)
        self.lon = 0.0      # Field 3 - Longitude, dd is degrees, mm.mm is minutes
        self.lon_i = '_'    # Field 4 - E or W (East or West)
        self.utc = 0        # Field 5 - UTC of this position, hh is hours, mm is minutes, ss.ss is seconds
        self.status = 'V'   # Field 6 - Status A - Data Valid, V - Data Invalid
        self.mode = 0       # Field 7 - FAA mode indicator (NMEA 2.3 and later) ??
        self.checksum = 0   # Field 8 - checksum
class GPSD:
    def __init__(self):
        self.gga = GNSS_GGA()
        self.gsa = GNSS_GSA()
        self.gll = GNSS_GLL()


# Global variables
gpsd0 = GPSD()
serial_port = '/dev/ttyUSB0'    # serial uart port
speed = 38400                   # baud rate: 4800, 9600, 38400, 921600
fuse = -3
# str0 = list()
# res_rx = ""

# Functions
def summary(gpsd):
    os.system('clear')
    print(str(gpsd.hour+fuse).zfill(2) + ":" + str(gpsd.min).zfill(2) + ":" + str(gpsd.sec).zfill(2))
    print("latitude: " + "{0:.6f}".format(gpsd.latdeg_dec) + " " + str(gpsd.lat_index))
    print("longitude: " + "{0:.6f}".format(gpsd.londeg_dec) + " " + str(gpsd.lon_index))
    print("Altitude: " + str(gpsd.altitude))
    print("N satt: " + str(gpsd.num_satt))
    print("GPS: ")
    print("BD : ")
    print("GL : ")
    print("GS : ")
def run_serial():
    try:
        s = serial.Serial(serial_port, speed)
        print("Serial start reading...")
        while True:
            res_rx = s.readline()
            # str0.append(res_rx)
            # print(str0[len(str0)-1])
            # flag_rx = 1
            # print(res_rx.decode("utf-8"))
            print(res_rx)
    except:
        print("Can't stablished serial communication")
def read_file():
    file = open("gnss_data_02.txt", "r")
    ans0 = file.readline()
def main0():
    try:
        s = serial.Serial(serial_port, speed)
        print("Serial port reading...")

        while True:
            res_rx = s.readline()
            # str_cmd = res_rx.decode("utf-8")
            cmd_to_parse = res_rx.decode("utf-8")

            # if len(str0) > 1:
            #     print(str0[0])
            #     # decode(str0[0])
            #     # try:
            #     str0.pop()
            #     # finally:
            #         # print("nothing")

            # p1 = str_cmd.find("$")			    # find the start point
            # p2 = str_cmd.find("\r\n")			# find the end point
            # cmd_to_parse = str_cmd[p1+1:p2]  	# reconstruct the string
            # cmd_to_parse = str_cmd[p1+1:]  	# reconstruct the string
            ans2 = cmd_to_parse.split(',')	   	# split data in one array
            # str_cmd = str_cmd[p2:]            # remove the first command line

            cmd = ans2[0][1:]	                # get the main command removing $ character
            cmd_satt = cmd[0:2]	        	# get satt constelation
            cmd_type = cmd[2:5]	        	# get subcommand type

            # process it
            match cmd_type:
                case "GSA":	# GSA - GPS DOP and active satellites
                    a = 0
                
                case "GSV":
                    gpsd0.num_gsv = 0

                case "ZDA":
                    gpsd0.UTC_time = 0

                case "VTG":
                    try:
                        gpsd0.speed_kmph = float(ans2[7])
                    except:
                        gpsd0.speed_kmph = 0
                case "GLL":
                    summary(gpsd0)
                    # a=0
                    # print("nothing found")
    except:
        print("Can't stablished serial communication2")

# run_serial()
main0()

# thread_1 = threading.Thread(target=run_serial)
# thread_1.start()
# thread_1.join()
