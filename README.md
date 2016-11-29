
===================================
MahoWii rev 2.4 
===================================

Hi guys!

Here is the last revision of my biggest project based on favorite Multiwii project.
In two words: it's pretty stable, improved and extended version of Multiwii.

Changes:
- INS (inertial navigation system) integrated with GPS and Baro

- completely different althold integrated with INS
- right altitude management by setting desired vario (vertical speed) with throttle stick
- single PID controller for hovering and altitude management
- stable hovering trottle estimator/corrector by I-part of alt PID controller
- smart land detector
- reduced effect of air-cushion to avoid incorrect raw baro values and jumps near the ground accordingly (integrated with INS)
- new SAFE_ALT_DURING_AH define. It helps to protect descending less than specified in define altitude. Activated when SAFE_ALT box activated in GUI.

- precised GPS-INS position hold
- possibility to fly with activated position hold and stop at predicted point on stick release (tunable by Pos-I value at GUI)
- wait for target altitude at RTH and WP navigation
- skip whole RTH cycle if distance less than RTH_RADIUS meters and make a landing
- safe and soft takeoff on RTH and WP Navigation
- for safety prevent arm if RTH or NAV switched on
- new auto config defines for UBLOX and MTK3339 modules

- precised gyro calibration + 3 axis acel calibration + filtered mag calibration to avoid min/max noise values
- frsky telemetry adopted for er9x-frsky TX firmware 
- 50hz PID output for althold, gps modes
- cycle time independent final PID controller 
- optimized math
- a lot of fixes 
- for more details please read release_note.txt, see config.h and also here http://forum.rcdesign.ru/blogs/83206/blog22332.html


Videos from my friend's youtube channel: 
https://www.youtube.com/user/artnesterof/videos

Blogs:
- http://forum.rcdesign.ru/blogs/83206/

Enjoy! ;)  

[![Donate](https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=NQ6D8YEWUV88S)


