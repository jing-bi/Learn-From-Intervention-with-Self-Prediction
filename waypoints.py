
from time import time
from carladep import *
from carladep.functions import find_weather_presets, get_actor_display_name
from carladep.hud import HUD
from carladep.sensor import CollisionSensor, LaneInvasionSensor, GnssSensor, CameraManager






waypoints = map.generate_waypoints(distance)
for w in waypoints:
    world.world.debug.draw_string(w.transform.location, 'O', draw_shadow=False,
                                       color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                                       persistent_lines=True)