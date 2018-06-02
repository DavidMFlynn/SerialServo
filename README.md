# SerialServo
Packet serial to R/C servo w/ absolute encoder
This is for one of my robot projects.
Features: TTL Serial interface, PWM output w/ 1/2 microsecond resolution, Current sensing, Absolute encoder SPI interface.
Features will be added as needed.

ToDo:

  Add params:
    
    
  Torque limit
  
  Fix C10 (Rev A), Shrink PCB (Ver Small Rev n/c), all smt, no switches, only sys LED
  Servo power bypass jumper.
  
  Mode 4, Gripper control, closed value, open value
    Move toward close until reached or max current
    Open to open value
    OpenValue = FastRevVal, CloseValue = FastRevVal
    Speed is used for close only
    Open may be greater than or less than close, RevDirFlag?
    