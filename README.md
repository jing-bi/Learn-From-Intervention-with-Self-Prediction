# Repo for paper
### Road Condition
-    VOID = -1
-    LEFT = 1
-    RIGHT = 2
-    STRAIGHT = 3
-    LANEFOLLOW = 4
### Auto_control
- J: Start Collecting Data
- K: End Collecting Data
- Backspace: stop recording and respawn
- Three controller:
    1. trained network
    2. PID
    3. Manual
- PID -P/WASD-> Manual
- PID -I-> network
- Manual -P-> PID
- Manual -I-> network
- network -WASD/I-> Manual
- network -P -> PID

### Config
- Town07 npc 40

            speed: 30
            'K_P': 0.75,
            'K_D': 0.01,
            'K_I': 20,
- Town03 npc 80

            speed: 30
            'K_P': 0.5,
            'K_D': 0.002,
            'K_I': 20,
- Town04 npc 80  
          
            'K_P': 0.6,
            'K_D': 0.002,
            'K_I': 25,
### Control feature
- **respawn** after **collision** with anything
- turn back to **PID control** after **lane invasion**
- when turning back to PID, re-schedule waypoint based on current location 
- must turn right or left at intersection
- slow down when turning 
- hold contion until you press 2
