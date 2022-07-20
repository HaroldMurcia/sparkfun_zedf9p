#!/usr/bin/env python3

from sys import platform
import sys
import os
import socket
from io import BufferedReader
from threading import Thread, Lock, Event
from queue import Queue
from time import sleep
from datetime import datetime, timedelta
from base64 import b64encode
from io import TextIOWrapper, BufferedWriter
from serial import Serial
from pynmeagps import NMEAMessage, GET
from pyubx2 import UBXReader, RTCM3_PROTOCOL, ERR_IGNORE
from pygnssutils._version import __version__ as VERSION
from pygnssutils.exceptions import ParameterError
from pygnssutils.globals import (
    VERBOSITY_LOW,
    VERBOSITY_MEDIUM,
    DEFAULT_BUFSIZE,
    LOGLIMIT,
    MAXPORT,
    DISCONNECTED,
    NOGGA,
    OUTPORT_NTRIP,
)
from pyrtcm import (RTCM_MSGIDS,RTCMParseError,RTCMMessageError,RTCMTypeError,)
from pygnssutils import (GNSSNTRIPClient, GNSSReader, NMEA_PROTOCOL,UBX_PROTOCOL,RTCM3_PROTOCOL,protocol,)

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sparkfun_zedf9p.msg import nav_status
from sparkfun_zedf9p.msg import gngsa
from sparkfun_zedf9p.msg import gnrmc

###### GLOBAL VARIABLES
USERAGENT = f"PYGNSSUTILS NTRIP Client/{VERSION}"
NTRIP_HEADERS = {
    "Ntrip-Version": "Ntrip/2.0",
    "User-Agent": USERAGENT,
}
FIXES = {
    "3D": 1,
    "2D": 2,
    "RTK FIXED": 4,
    "RTK FLOAT": 5,
    "RTK": 5,
    "DR": 6,
    "NO FIX": 0,
}

######


class GPS_ZED_F9P(object):

    def __init__(self, app=None, **kwargs):
        self.__app = app  # Reference to calling application class (if applicable)
        self._validargs = True
        self._ntrip_thread = None
        self.rth_thread=None
        self.kth_thread = None
        self.RTCM_1005_FLAG = False
        self.ROVER_ECEF_X=0
        self.ROVER_ECEF_Y=0
        self.ROVER_ECEF_Z=0
        self.ROVER_LAT=0
        self.ROVER_LONG=0
        self.ROVER_ALT=0
        self.ROVER_ALT_MSL=0
        self.hAcc=0
        self.vAcc=0
        self.pAcc=0
        self.PRINT_FULL = False
        self.SERIAL_PORT = "/dev/ttyACM0"
        self.BAUDRATE = 921600
        self.serial_lock = Lock()
        self.serial=0
        self.serialTIMEOUT=0
        # NTRIP server parameters - AMEND AS REQUIRED:
        # Ideally, self.MOUNTPOINT should be <30 km from location.
        # persist settings to allow any calling app to retrieve them
        self.ntrip_queue = Queue()
        self._settings = {
            "server": "",
            "port": "2101",
            "mountpoint": "",
            "distance": "",
            "version": "2.0",
            "user": "anon",
            "password": "password",
            "ggainterval": "None",
            "sourcetable": [],
            "reflat": "",
            "reflon": "",
            "refalt": "",
            "refsep": "",
        }
        try:
            self._verbosity = int(kwargs.get("verbosity", VERBOSITY_MEDIUM))
            self._logtofile = int(kwargs.get("logtofile", 0))
            self._logpath = kwargs.get("logpath", ".")
        except (ParameterError, ValueError, TypeError) as err:
            self._do_log(
                f"Invalid input arguments {kwargs}\n{err}\nType gnssntripclient -h for help.",
                VERBOSITY_LOW,)
            self._validargs = False
        self.output = kwargs.get("output", None)
        # ROS MSG
        self.gpsmsg=NavSatFix()
        self.gpsECEF=Odometry()
        self.gpsLLAccuracies=nav_status()
        self.gpsECEFAccuracies=nav_status()
        self.gpsgngsa=gngsa()
        self.gpsgnrmc=gnrmc()
        #ROS Publishers
        self.LLA_pub=rospy.Publisher('/zedf9p/LLA_zedf9p', NavSatFix, queue_size=100)
        self.ECEF_pub=rospy.Publisher('/zedf9p/ECEF_zedf9p', Odometry, queue_size=100)
        self.LLAccuracies_pub=rospy.Publisher('/zedf9p/LLAccuracies', nav_status, queue_size=100)
        self.ECEFAccuracies_pub=rospy.Publisher('/zedf9p/ECEFAccuracies', nav_status, queue_size=100)
        self.GNGSA_pub=rospy.Publisher('/zedf9p/GNGSA', gngsa, queue_size=100)
        self.GNRMC_pub=rospy.Publisher('/zedf9p/GNRMC', gnrmc, queue_size=100)
        #
        self.TIMEOUT = 10
        self._socket = None
        self._connected = False
        self._stopevent = Event()
        self._ntrip_thread = None
        self._last_gga = datetime.now()
        #
        
        

    @property
    def settings(self):
        """
        Getter for NTRIP settings.
        """
        return self._settings

    def __enter__(self):
        """
        Context manager enter routine.
        """
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit routine.
        Terminates threads in an orderly fashion.
        """
        self.stop()


    def stop(self):
        """
        Close NTRIP server connection.
        """
        self._stop_read_thread()
        self._connected = False


    def _stop_read_thread(self):
            """
            Stop NTRIP reader thread.
            """
            if self._ntrip_thread is not None:
                self._stopevent.set()
                self._ntrip_thread = None


    def _app_notify(self):
        """
        THREADED
        If calling app is tkinter, generate event
        to notify app that data is available
        """

        if hasattr(self.__app, "appmaster"):
            if hasattr(self.__app.appmaster, "event_generate"):
                self.__app.appmaster.event_generate("<<ntrip_read>>")

    @property
    def connected(self):
        """
        Connection status getter.
        """
        return self._connected

    def _do_header(self, sock: socket, stopevent: Event) -> str:
            """
            THREADED
            Parse response header lines.
            :param socket sock: socket
            :param Event stopevent: stop event
            :return: return status or error message
            :rtype: str
            """
            stable = []
            data = "Initial Header"
            while data and not stopevent.is_set():
                try:
                    data = sock.recv(DEFAULT_BUFSIZE)
                    header_lines = data.decode(encoding="utf-8").split("\r\n")
                    for line in header_lines:
                        # if sourcetable request, populate list
                        if line.find("STR;") >= 0:  # sourcetable entry
                            strbits = line.split(";")
                            if strbits[0] == "STR":
                                strbits.pop(0)
                                stable.append(strbits)
                        elif line.find("ENDSOURCETABLE") >= 0:  # end of sourcetable
                            self._settings["sourcetable"] = stable
                            self._get_closest_mountpoint()
                            self._do_log("Complete sourcetable follows...\n")
                            for lines in self._settings["sourcetable"]:
                                self._do_log(lines, VERBOSITY_MEDIUM, False)
                            return "1"
                        elif (
                                line.find("401 Unauthorized") >= 0
                                or line.find("403 Forbidden") >= 0
                                or line.find("404 Not Found") >= 0
                        ):
                            return line
                        elif line == "":
                            break
                except UnicodeDecodeError:
                    data = False
            return "0"

    def _do_data(self, sock: socket, stopevent: Event, ggainterval: int, output: object ):
        """
        THREADED
        Read and parse incoming NTRIP RTCM3 data stream.
        :param socket sock: socket
        :param Event stopevent: stop event
        :param int ggainterval: GGA transmission interval seconds
        :param object output: output stream for RTCM3 messages
        """

        # UBXReader will wrap socket as SocketStream
        ubr = UBXReader(sock,protfilter=RTCM3_PROTOCOL,quitonerror=ERR_IGNORE,bufsize=DEFAULT_BUFSIZE,)
        raw_data = None
        parsed_data = None
        while not stopevent.is_set():
            try:
                raw_data, parsed_data = ubr.read()
                if raw_data is not None:
                    self._do_write(output, raw_data, parsed_data)
                self._send_GGA(ggainterval, output)

            except (RTCMMessageError,RTCMParseError,RTCMTypeError,) as err:
                parsed_data = f"Error parsing data stream {err}"
                self._do_write(output, raw_data, parsed_data)
                continue


    def _do_write(self, output: object, raw: bytes, parsed: object):
            """
            THREADED
            Send RTCM3 data to designated output medium.
            If output is Queue, will send both raw and parsed data.
            :param object output: writeable output medium for RTCM3 data
            :param bytes raw: raw data
            :param object parsed: parsed message
            """
            self._do_log(parsed, VERBOSITY_MEDIUM)
            if output is not None:
                if isinstance(output, (Serial, BufferedWriter)):
                    output.write(raw)
                elif isinstance(output, TextIOWrapper):
                    output.write(str(parsed))
                elif isinstance(output, Queue):
                    output.put((raw, parsed))
                elif isinstance(output, socket.socket):
                    output.sendall(raw)

            self._app_notify()  # notify any calling app that data is available

    def _do_log(self,message: object,loglevel: int = VERBOSITY_MEDIUM,timestamp: bool = True,):
        """
        THREADED
        Write timestamped log message according to verbosity and logfile settings.
        :param object message: message or object to log
        :param int loglevel: log level for this message (0,1,2)
        :param bool timestamp: prefix message with timestamp (Y/N)
        """
        if timestamp:
            message = f"{datetime.now()}: {str(message)}"
        else:
            message = str(message)
        if self._verbosity >= loglevel:
            if self._logtofile:
                self._cycle_log()
                with open(self._logpath, "a", encoding="UTF-8") as log:
                    log.write(message + "\n")
                    self._loglines += 1
            else:
                pass
                #print(message)

    def _cycle_log(self):
        """
        THREADED
        Generate new timestamped logfile path.
        """
        if not self._loglines % LOGLIMIT:
            tim = datetime.now().strftime("%Y%m%d%H%M%S")
            self._logpath = os.path.join(self._logpath, f"gnssntripclient-{tim}.log")
            self._loglines = 0


    @staticmethod
    def _formatGET(settings: dict) -> str:
        """
        THREADED
        Format HTTP GET Request.
        :param dict settings: settings dictionary
        :return: formatted HTTP GET request
        :rtype: str
        """
        mountpoint = settings["mountpoint"]
        version = settings["version"]
        user = settings["user"]
        password = settings["password"]

        if mountpoint != "":
            mountpoint = "/" + mountpoint  # sourcetable request
        user = user + ":" + password
        user = b64encode(user.encode(encoding="utf-8"))
        req = (
                f"GET {mountpoint} HTTP/1.1\r\n"
                + f"User-Agent: {USERAGENT}\r\n"
                + f"Authorization: Basic {user.decode(encoding='utf-8')}\r\n"
                + f"Ntrip-Version: Ntrip/{version}\r\n"
        )
        req += "\r\n"  # NECESSARY!!!
        return req.encode(encoding="utf-8")

    def _app_get_coordinates(self) -> tuple:
        """
        THREADED
        Get current coordinates from calling application.
        If not available, used fixed reference coordinates.
        :returns: tuple of (lat, lon, alt, sep)
        :rtype: tuple
        """

        if hasattr(self.__app, "get_coordinates"):
            conn_status, lat, lon, alt, sep = self.__app.get_coordinates()
            if conn_status != DISCONNECTED:
                return lat, lon, alt, sep
        lat = self._settings["reflat"]
        lon = self._settings["reflon"]
        alt = self._settings["refalt"]
        sep = self._settings["refsep"]
        return lat, lon, alt, sep

    def _formatGGA(self) -> tuple:
        """
        THREADED
        Format NMEA GGA sentence using pynmeagps. The raw string
        output is suitable for sending to an NTRIP socket.
        :return: tuple of (raw NMEA message as bytes, NMEAMessage)
        :rtype: tuple
        """
        # time will default to current UTC

        try:

            lat, lon, alt, sep = self._app_get_coordinates()
            lat = float(lat)
            lon = float(lon)
            NS = "N"
            EW = "E"
            if lat < 0:
                NS = "S"
            if lon < 0:
                EW = "W"

            parsed_data = NMEAMessage(
                "GP",
                "GGA",
                GET,
                lat=lat,
                NS=NS,
                lon=lon,
                EW=EW,
                quality=1,
                numSV=15,
                HDOP=0,
                alt=alt,
                altUnit="M",
                sep=sep,
                sepUnit="M",
                diffAge="",
                diffStation=0,
            )

            raw_data = parsed_data.serialize()
            return raw_data, parsed_data

        except ValueError:
            return None, None

    def _send_GGA(self, ggainterval: int, output: object):
        """
        THREADED
        Send NMEA GGA sentence to NTRIP server at prescribed interval.
        """
        if ggainterval != NOGGA:
            if datetime.now() > self._last_gga + timedelta(seconds=ggainterval):
                raw_data, parsed_data = self._formatGGA()
                if parsed_data is not None:
                    self._socket.sendall(raw_data)
                    self._do_write(output, raw_data, parsed_data)
                self._last_gga = datetime.now()

    
    def _start_read_thread(self,settings: dict,stopevent: Event,output: object,):
        """
        Start the NTRIP reader thread.
        """
        if self._connected:
            self._stopevent.clear()
            self._ntrip_thread = Thread(target=self._read_thread,args=(settings,stopevent,output,),daemon=True,)
            self._ntrip_thread.start()


    def _start_readZEDF9P_thread(self):
        """
        Start the serial reader thread.
        """
        self.rth_thread = Thread(target=self.read_gnss, args=(), daemon=True,)
        self.rth_thread.start()
        

    def _stop_readZEDF9P_thread(self):
        """
        Stop the serial reader thread.
        """
        if self.rth_thread is not None:
            self._stopevent.set()
            self.rth_thread= None





    def _start_writeZEDF9P_thread(self):
        """
        Start send thread.
        """
        self.kth_thread = Thread(target=self.send_gnss,args=(),daemon=True,)
        self.kth_thread.start()


    def _read_thread(self,settings: dict,stopevent: Event,output: object,):
        """
        THREADED
        Opens socket to NTRIP server and reads incoming data.
        :param dict settings: settings as dictionary
        :param Event stopevent: stop event
        :param object output: output stream for RTCM3 data
        """
        try:
            server = settings["server"]
            port = int(settings["port"])
            mountpoint = settings["mountpoint"]
            ggainterval = int(settings["ggainterval"])
            with socket.socket() as self._socket:
                self._socket.connect((server, port))
                self._socket.settimeout(self.TIMEOUT)
                self._socket.sendall(self._formatGET(settings))
                # send GGA sentence with request
                if mountpoint != "":
                    self._send_GGA(ggainterval, output)
                while not stopevent.is_set():
                    print("self._socket",self._socket)
                    rc = self._do_header(self._socket, stopevent)
                    if rc == "0":  # streaming RTMC3 data from mountpoint
                        self._do_log(f"Using mountpoint {mountpoint}\n")
                        self._do_data(self._socket, stopevent, ggainterval, output)
                    elif rc == "1":  # retrieved sourcetable
                        stopevent.set()
                        self._connected = False
                        self._app_update_status(False)
                    else:  # error message
                        stopevent.set()
                        self._connected = False
                        self._app_update_status(False, (f"Error!: {rc}", "red"))
        except (
                socket.gaierror,
                ConnectionRefusedError,
                ConnectionAbortedError,
                ConnectionResetError,
                BrokenPipeError,
                TimeoutError,
                OverflowError,
        ):
            stopevent.set()
            self._connected = False

    # Set to True to print entire GNSS/NTRIP message rather than just identity
    def read_gnss(self):
        """
        THREADED
        Reads and parses incoming GNSS data from receiver.
        """
        gnr = UBXReader(BufferedReader(self.serial),protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL | RTCM3_PROTOCOL),)     
        while not self._stopevent.is_set():
            try:
                if self.serial.in_waiting:
                    self.serial_lock.acquire()
                    (raw_data, parsed_data) = gnr.read()
                    self.serial_lock.release()
                    stamp_received = rospy.get_rostime()
                    if parsed_data:
                        ### NEW LAW Harold
                        if self.RTCM_1005_FLAG == True:
                            if "UBX(NAV-HPPOSECEF" in str(parsed_data):
                                msg = str(parsed_data).split(",")
                                ecefX = msg[4]
                                ecefX = ecefX.split("=")
                                ecefX = float(ecefX[1])
                                ecefY = msg[5]
                                ecefY = ecefY.split("=")
                                ecefY = float(ecefY[1])
                                ecefZ = msg[6]
                                ecefZ = ecefZ.split("=")
                                ecefZ = float(ecefZ[1])
                                self.pAcc = msg[8]
                                self.pAcc = self.pAcc.strip(")>")
                                self.pAcc = self.pAcc.split("=")
                                self.pAcc = float(self.pAcc[1])
                                self.ROVER_ECEF_X=ecefX
                                self.ROVER_ECEF_Y=ecefY
                                self.ROVER_ECEF_Z=ecefZ
                                self.gpsECEF.header.stamp = stamp_received
                                self.gpsECEF.header.frame_id="ublox"
                                self.gpsECEF.pose.pose.position.x=self.ROVER_ECEF_X
                                self.gpsECEF.pose.pose.position.y=self.ROVER_ECEF_Y
                                self.gpsECEF.pose.pose.position.z=self.ROVER_ECEF_Z
                                self.ECEF_pub.publish(self.gpsECEF)
                                #
                                self.gpsECEFAccuracies.header.stamp = stamp_received
                                self.gpsECEFAccuracies.header.frame_id="ublox"
                                self.gpsECEFAccuracies.position_acc = self.pAcc
                                self.ECEFAccuracies_pub.publish(self.gpsECEFAccuracies)                            
                                #print("\nECEF: ", self.ROVER_ECEF_X,self.ROVER_ECEF_Y,self.ROVER_ECEF_Z,self.pAcc)
                            if "UBX(NAV-POSLLH," in str(parsed_data):
                                msg = str(parsed_data).split(",")
                                longitude = msg[2]
                                longitude = longitude.split("=")
                                longitude = float(longitude[1])
                                latitude = msg[3]
                                latitude = latitude.split("=")
                                latitude = float(latitude[1])
                                altitude = msg[4]
                                altitude = altitude.split("=")
                                altitude = float(altitude[1])
                                altitude_MSL = msg[5]
                                altitude_MSL = altitude_MSL.split("=")
                                altitude_MSL = float(altitude_MSL[1])
                                self.hAcc = msg[6]
                                self.hAcc = self.hAcc.split("=")
                                self.hAcc = float(self.hAcc[1])
                                self.vAcc = msg[7]
                                self.vAcc = self.vAcc.strip(")>")
                                self.vAcc = self.vAcc.split("=")
                                self.vAcc = float(self.vAcc[1])
                                self.ROVER_LAT=latitude
                                self.ROVER_LONG=longitude
                                self.ROVER_ALT=altitude
                                self.ROVER_ALT_MSL=altitude_MSL
                                #
                                self._settings["reflat"] = latitude
                                self._settings["reflon"] = longitude
                                self._settings["refalt"] = altitude
                                #print("\nGPS: ", self.ROVER_LAT,self.ROVER_LONG,self.ROVER_ALT,self.ROVER_ALT_MSL,self.hAcc,self.vAcc)
                                self.gpsmsg.latitude=self.ROVER_LAT
                                self.gpsmsg.longitude=self.ROVER_LONG
                                self.gpsmsg.altitude=self.ROVER_ALT
                                self.gpsmsg.header.stamp = stamp_received
                                self.gpsmsg.header.frame_id="ublox"
                                self.LLA_pub.publish(self.gpsmsg)
                                #
                                self.gpsLLAccuracies.header.stamp = stamp_received
                                self.gpsLLAccuracies.header.frame_id="ublox"
                                self.gpsLLAccuracies.horizontal_acc = self.hAcc
                                self.gpsLLAccuracies.vertical_acc = self.vAcc
                                self.LLAccuracies_pub.publish(self.gpsLLAccuracies)
                            if "GNGSA" in str(parsed_data):
                                msg = str(parsed_data).split(",")
                                opMode=msg[1]
                                opMode=opMode.split("=")[1]
                                if (opMode=='A'):
                                    opMode="Automatic"
                                else:
                                    opMode="Manual"
                                navMode=msg[2]
                                navMode=navMode.split("=")[1]
                                if (navMode=='1'):
                                    navMode="Not available"
                                elif (navMode=='2'):
                                    navMode="2D"
                                elif (navMode=='3'):
                                    navMode="3D"
                                PDOP=msg[15]
                                PDOP=float(PDOP.split("=")[1])
                                HDOP=msg[16]
                                HDOP=float(HDOP.split("=")[1])
                                VDOP=msg[17]
                                VDOP=float(VDOP.split("=")[1])
                                #
                                self.gpsgngsa.header.stamp = stamp_received
                                self.gpsgngsa.header.frame_id="ublox"
                                self.gpsgngsa.opmode = opMode
                                self.gpsgngsa.navmode = navMode
                                self.gpsgngsa.pdop = PDOP
                                self.gpsgngsa.hdop = HDOP
                                self.gpsgngsa.vdop = VDOP
                                self.GNGSA_pub.publish(self.gpsgngsa)
                            if "GNRMC" in str(parsed_data):
                                msg = str(parsed_data).split(",")
                                utc_time = msg[1]
                                utc_time=utc_time.split("=")[1]
                                GNRMC_status= msg[2]
                                GNRMC_status=GNRMC_status.split("=")[1]
                                if (GNRMC_status=="A"):
                                    GNRMC_status="effective positioning"
                                elif (GNRMC_status=="V"):
                                    GNRMC_status="invalid positioning"
                                GNRMC_date= msg[9]
                                GNRMC_date=GNRMC_date.split("=")[1]
                                GNRMC_mode= msg[12]
                                GNRMC_mode=GNRMC_mode.split("=")[1]
                                if (GNRMC_mode=="A"):
                                    GNRMC_mode="autonomous positioning"
                                elif (GNRMC_mode=="D"):
                                    GNRMC_mode="differential mode"
                                elif (GNRMC_mode=="E"):
                                    GNRMC_mode="estimation"
                                elif (GNRMC_mode=="N"):
                                    GNRMC_mode="invalid data"
                                elif (GNRMC_mode=="M"):
                                    GNRMC_mode="manual input mode"
                                elif (GNRMC_mode=="S"):
                                    GNRMC_mode="simulation mode"
                                #
                                self.gpsgnrmc.header.stamp = stamp_received
                                self.gpsgnrmc.header.frame_id="ublox"
                                self.gpsgnrmc.utc_time = utc_time
                                self.gpsgnrmc.status = GNRMC_status
                                self.gpsgnrmc.utc_date = GNRMC_date
                                self.gpsgnrmc.mode = GNRMC_mode
                                #
                                self.GNRMC_pub.publish(self.gpsgnrmc)

                    else:
                        print("Not parsed_data")
            except Exception as err:
                print(f"Something went wrong in read thread {err}")
                break

    def send_gnss(self):
        """
        THREADED
        Reads RTCM3 data from message queue and sends it to receiver.
        """
        while not self._stopevent.is_set():
            try:
                raw_data, parsed_data = self.ntrip_queue.get()
                if protocol(raw_data) == RTCM3_PROTOCOL:
                    #
                    if "RTCM(1005" in str(parsed_data) and self.RTCM_1005_FLAG==False:
                        self.RTCM_1005_FLAG=True
                        print("\t ------------------------------------------------------------------\n")
                    #
                    if self.PRINT_FULL:
                        print(parsed_data)
                    else:
                        mns = f"NTRIP>> {parsed_data.identity} {RTCM_MSGIDS[parsed_data.identity]}"
                        rospy.loginfo(mns)
                        #print(f"NTRIP>> {parsed_data.identity} {RTCM_MSGIDS[parsed_data.identity]}")
                    self.serial_lock.acquire()
                    self.serial.write(raw_data)
                    self.serial_lock.release()
            except Exception as err:
                print(f"Something went wrong in send thread {err}")
                break
        

    def run(self, **kwargs) -> bool:
        try:
            user = os.getenv("NTRIP_USER", "anon")
            password = os.getenv("NTRIP_PASSWORD", "password")
            self._settings["server"] = server = kwargs.get("server", "")
            self._settings["port"] = port = int(kwargs.get("port", OUTPORT_NTRIP))
            self._settings["mountpoint"] = mountpoint = kwargs.get("mountpoint", "")
            self._settings["version"] = kwargs.get("version", "2.0")
            self._settings["user"] = kwargs.get("user", user)
            self._settings["password"] = kwargs.get("password", password)
            self._settings["ggainterval"] = int(kwargs.get("ggainterval", NOGGA))
            self._settings["reflat"] = kwargs.get("reflat", "")
            self._settings["reflon"] = kwargs.get("reflon", "")
            self._settings["refalt"] = kwargs.get("refalt", "")
            self._settings["refsep"] = kwargs.get("refsep", "")
            #output = kwargs.get("output", None)
            if server == "":
                raise ParameterError(f"Invalid server url {server}")
            if port > MAXPORT or port < 1:
                raise ParameterError(f"Invalid port {port}")      
        except (ParameterError, ValueError, TypeError) as err:
            pass
            self._do_log(
                f"Invalid input arguments {kwargs}\n{err}\n"
                + "Type gnssntripclient -h for help.",
                VERBOSITY_LOW,)
            self._validargs = False
        
        if self._validargs:
            self._connected = True
            #
            if mountpoint == "":
                print("Empty MountPoint")
                return False
            else:
                print(f"Opening serial port {self.SERIAL_PORT} @ {self.BAUDRATE}...\n")
                self._stopevent.clear()
                with Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=self.serialTIMEOUT) as self.serial:

                    print("Starting thread: NTRIPClient...\n")
                    self._start_read_thread( self._settings,self._stopevent, self.ntrip_queue)

                    print("Starting thread.: read ZED-F9P device..\n")
                    self._start_readZEDF9P_thread() 

                    print("Sending corrections to ZED-F9P")
                    self._start_writeZEDF9P_thread()
                    
                    while not rospy.is_shutdown():  # run until user presses CTRL-C
                        pass
        return True 



if __name__ == "__main__":
    # ROS
    kwargs = dict(arg.split("=") for arg in sys.argv[1:])
    rospy.init_node('ZEDF9P')

    try:
        gnc = GPS_ZED_F9P(None, **kwargs)
        streaming = gnc.run(**kwargs)
    except KeyboardInterrupt:
        pass
    gnc.stop()
    print("Terminated by user")