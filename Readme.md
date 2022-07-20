# # SparkFun GPS-RTK2 ROSnode

The SparkFun GPS-RTK2 raises the bar for high-precision GPS and is the latest in a line of powerful RTK boards featuring the ZED-F9P module from u-blox. The ZED-F9P is a top-of-the-line module for high accuracy GNSS and GPS location solutions including RTK that is capable of 10mm, three-dimensional accuracy. With this board, you will be able to know where your (or any object's) X, Y, and Z location is within roughly the width of your fingernail! The ZED-F9P is unique in that it is capable of both rover and base station operations. Utilizing our handy Qwiic system, no soldering is required to connect it to the rest of your system. However, we still have broken out 0.1"-spaced pins in case you prefer to use a breadboard.

Related information

 - [Official Sparkfun site](https://www.sparkfun.com/products/15136)
 - [Arduino Library for UBlox](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library)
 - [Official GitHub](https://github.com/sparkfun/Qwiic_GPS-RTK2)

## How to install?
Based on [Semu consulting](https://github.com/semuconsulting)
~~~
python3 -m pip install --upgrade pyubx2
sudo apt install python3-pip python3-tk python3-pil python3-pil.imagetk
python3 -m pip install --upgrade PyGPSClient
python3 -m pip install --upgrade pygnssutils
~~~
### Config LCD [3.5 Touch](https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(A))
~~~
git clone https://github.com/waveshare/LCD-show.git
chmod -R 755 LCD-show
cd LCD-show/
./LCD35-show
~~~

## How to execute?

### ZED-F9P base station

RTCM corrections: [ZED-F9P - Data Sheet, Section 1.4.2](https://cdn.sparkfun.com/assets/8/3/2/b/8/ZED-F9P_Data_Sheet.pdf)

* RTCM 1005 Stationary RTK reference station ARP
* RTCM 1074 GPS MSM4
* RTCM 1084 GLONASS MSM4
* RTCM 1094 Galileo MSM4
* RTCM 1124 BeiDou MSM4
* RTCM 1230 GLONASS code-phase biases


#### config
~~~
python3 f9p_basestation.py
python3 -m pygpsclient
~~~
~~~
# Harold
export PYGPSCLIENT_USER="user"
export PYGPSCLIENT_PASSWORD="password"
python3 -m pygpsclient
# mountpoint is pygpsclient
~~~

### ZED-F9P ROVER

~~~
sudo chmod 666 /dev/ttyACM0
~~~

* Put [high preicion mode](https://www.youtube.com/watch?v=az-rVHEIWyU)
* On [U-base M9 software](https://www.u-blox.com/en/product/u-center) config the following UBX-CFG-MSG:
  * 01-13 NAV-HPPSECEF [UART1 and USB]
  * 01-02 NAV-POSLLH [UART1 and USB]
  * Change rate to 10 Hz using GAL Time
  * in NAV5 change platform to Pedestrian

### How to run?

~~~
rosrun sparkfun_zedf9p NTRIP_client.py  server=.... [args]
~~~

args:
* server=[your ip address]
* port=[your port, default is 2101]
* user=[Ntrip user]
* password=[Ntrip passw]
* mountpoint=[Obtained from Ntrip account]
* ggainterval=0 
* verbosity=1 
* reflat=[Initial LAT] 
* reflon=[Initial LONG] 
* refalt=[Initial ALT] 
* refsep=1

### ROSbag to txt
~~~
rostopic echo -b 2022-07-06-15-24-38.bag -p /ECEF_zedf9p  > ECEFfile.txt
~~~