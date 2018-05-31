# SerialServo
Packet serial to R/C servo w/ absolute encoder
I'm just getting started. This is for one of my robot projects. 
Features: TTL Serial interface, PWM output w/ 1/2 microsecond resolution, Current sensing, Absolute encoder SPI interface.
Features will be added as needed.

ToDo:

  Add params:
    Mode 3 deadband 2..100
    Servo speed 0=full, 1..63 counts per cycle
    
  Torque limit
  
  Fix C10 (Rev A), Shrink PCB (Ver Small Rev n/c), all smt, no switches, only sys LED