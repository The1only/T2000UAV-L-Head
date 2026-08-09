#Transponder for Drones and Sports planes

System Update Summary
	•	Added a computer-screen layout optimized for larger displays and desktop use.
	•	Added airspeed support, including Arduino firmware for the MS4525 differential airspeed sensor.
	•	Migrated the platform to ESP32-S3.

⸻

Supported Sensors and Subsystems

Barometer
	•	Provides altitude information.
	•	Supports:
	•	Internal barometer
	•	External BMP280 barometer
	•	External barometer can be connected via USB or WLAN through an Arduino.


⸻

Transponder
	•	Supports T2000-UAV Mode A/C transponder.
	•	Full support for all transponder functions.
	•	Arduino-based interfaces supported over:
	•	USB
	•	WLAN

⸻


This software creates a bridge to all the devices.
On PC and Android one can use USB-Serial and connect with cable or Bluetooth. However NOT on IOS (iPhone/iPad) these are restricted.  
On all systems one can use WLan bridges using ARDUINO and SSDP over Broadcast.

We uses the ESP32S3 as Sensor simulator and sensor inteface. 
Also the ESP32C6 works, the ESP32C3 is a low cost solution, using Wlan I have seen some working some not, USB seems fine for all.

It includes tree artificial horizon versions, camera, map, list of used frequencies and call signs, a RADAR, and a Transponder Mode C screen.
In addition up to 4 screens can be connected at the same time (Android + iPhone + iPad etc. in any combination)
The system supports both external IMU and internal Android and IOS IMU (prety bad on all devices). 
It can be used in a split screen on a pad together with Skydemon etc.
Soon to come an Autopilot interface (the screen is there already).

Takeoff and landing is automatically logged to screen and log file, as is the local QNH (by calculating it from the GPS altitude when at ground/not moving). 

![Image 02-10-2024 at 12 04](https://github.com/user-attachments/assets/cd66978a-af2e-43ad-bf76-4d7dfc3056ae)

![IMG_0923](https://github.com/user-attachments/assets/ab5e302f-c7a1-4116-b5f9-e0cb81a96349)

<<<<<<< HEAD

=======
>>>>>>> 69d3ffa3daead801b1e40fd761a8766854bc8e1d


