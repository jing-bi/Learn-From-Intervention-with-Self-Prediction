"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

Recording Keys:
    J   :   Start recording dataset
    K   :   Stop recording
    P   :   toggle autopilot
    I   :   network control

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake

    M            : toggle manual transmission
    ,/.          : gear up/down

    TAB          : change sensor position
    `            : next sensor
    [1-9]        : change to sensor [1-9]
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from time import time
from carladep import *
from carladep.functions import find_weather_presets, get_actor_display_name
from carladep.hud import HUD
from carladep.sensor import CollisionSensor, LaneInvasionSensor, GnssSensor, CameraManager
from navigation.roaming_agent import RoamingAgent
from agent.action_intervention import ActionInterventionAgent
from agent.intention_intervention import IntentionInterventionAgent
from policy.Branch import BranchNet
from policy.ResNet import BasicResNet
from policy.SelfPred import SelfPred


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, actor_filter, network):
        # ===========================
        self.network = network
        self.capture_frame = 0
        self.total_frame = 0
        self.render_frame = 0
        self.capture_true = False
        self.datafolder = ''

        self.cvs_writer = None
        self.displaybuffer = None
        self.controller = 'Manual'  # network/manual/PID
        self.condition = 3
        self.capture_fps = 5  # 4 frames save once
        self.recover_time = 0
        self.save_buff = []
        # =============================
        self.world = carla_world
        self.map = self.world.get_map()
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self):
        self.save_buff.clear()

        if self.hud.collision_flag or self.hud.simulation_time_init != None:
            self.controller = 'PID'
            self.capture_true = True
        self.controller = 'Manual'
        self.condition = 3
        self.hud.collision_flag = False
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = blueprint.get_attribute('color').recommended_values[1]
            blueprint.set_attribute('color', color)
        # Spawn the player.
        if self.player is not None:
            spawn_points = self.map.get_spawn_points()
            self.destroy()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        self.agent = RoamingAgent(self.player)
        # # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        # ======================================Recording=========================================================
        clock = pygame.time.Clock()
        self.render_frame += 1
        self.displaybuffer = pygame.transform.scale(display, (640, 360))
        if self.render_frame % self.capture_fps == 0 and self.capture_true:
            if len(self.save_buff) < 10:
                self.save_buff.append(display)
            else:
                self.save_buff.append(display)
                self.capture_frame += 1
                self.total_frame += 1
                imagename = f"{str(self.datafolder)}/{self.total_frame}.jpeg"
                pygame.image.save(pygame.transform.scale(self.save_buff.pop(), (640, 360)), imagename)
                t = self.player.get_transform()
                c = self.player.get_control()
                stage = 'M' if self.controller == 'PID' or self.controller == 'Manual' else 'P'
                row = {
                    'imagepath': f"{str(self.datafolder.stem)}/{self.capture_frame}.jpeg",
                    'Heading': t.rotation.yaw,
                    'Location': [t.location.x, t.location.y],
                    'Throttle': c.throttle,
                    'Steer': c.steer,
                    'Brake': c.brake,
                    'Stage': stage,  # 'P'=policy,'M'=Manual
                    'Condition': self.condition
                }
                self.cvs_writer.writerow(row)
        # ======================================End=========================================================
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        # if self.cvs_writer is not None:self.cvs_writer.close()
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world, experiment, start_in_Roaming):
        self._Roaming_enabled = start_in_Roaming
        self._experiment = experiment
        self.hold_condition = False
        self.datafolder = pathlib.Path(__file__).parent.parent / f"dataset/{self._experiment}/"

        current_iteration = max([int(i.stem[-1]) for i in self.datafolder.glob('iter*')])
        self.iter = f"iter_{current_iteration+1}"
        self.iter_folder = self.datafolder / self.iter
        world.network.load(f'{self._experiment}_{current_iteration}.pth.tar')
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()


        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def collsion_respawn(self, world):
        if world.capture_true == True:
            world.capture_frame = 0
            world.capture_true = False
            world.hud.notification('Waiting...')
        world.restart()

    def invasion_switch(self, world):
        world.hud.notification(f"Lane invasion Back to {world.controller}")
        if world.controller == 'Network': world.recover_time = time()
        world.controller = 'PID'
        world.agent.new_plan = True
        world.hud.laneinvasion_flag = False

    def parse_events(self, client, world, clock):
        period = 1 if world.condition not in [1, 2] else 3
        if world.controller == 'Network' and world.condition in [1, 2]:
            world.controller = 'PID'
            world.recover_time = 0
        if  world.controller == 'PID' and time()-world.recover_time>period and self._experiment not in ['baseline_1','baseline_4']:
        # if world.controller == 'PID' and time() - world.recover_time > period:
            world.controller = 'Network'
            world.recover_time = 0
        if world.hud.collision_flag:
            self.collsion_respawn(world)
        if world.hud.laneinvasion_flag:
            self.invasion_switch(world)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                # ====================================== jjjjjjjjjkkkkkkkkkk  =========================================================
                elif event.key == K_j and world.capture_true == False:
                    world.capture_true = True
                    world.hud.notification('Collecting data')
                    now = datetime.datetime.now().strftime("%m-%d-%H:%M:%S")
                    folder = self.iter_folder / f"{now}"
                    folder.mkdir(parents=True, exist_ok=True)
                    world.datafolder = folder
                    csvfile = open(f'{str(folder)}_RAW.csv', 'a')
                    fieldnames = ['imagepath', 'Heading', 'Location', 'Throttle', 'Steer', 'Brake', 'Stage',
                                  'Condition']
                    world.cvs_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    world.cvs_writer.writeheader()


                elif event.key == K_k and world.capture_true == True:

                    world.capture_frame = 0
                    world.capture_true = False
                    world.hud.notification('Waiting...')


                # ====================================== CONDITION =========================================================
                #                 1:left 2:right 3:stright
                elif event.key == K_2 and self.hold_condition == True:
                    self.hold_condition = False
                    if world.condition in [1, 2]:
                        world.condition = 3
                elif event.key == K_3 and self.hold_condition == False:
                    self.hold_condition = True

                    world.condition = 1
                elif event.key == K_4 and self.hold_condition == False:
                    self.hold_condition = True

                    world.condition = 2
                # ======================================BACKSPACE=========================================================
                elif event.key == K_BACKSPACE:
                    self.collsion_respawn(world)
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:

                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                # elif event.key > K_0 and event.key <= K_9:
                #     world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    currentIndex = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(currentIndex)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                # ====================================== PPPPPPPPPPPPP =========================================================

                elif event.key == K_p and not (pygame.key.get_mods() & KMOD_CTRL):
                    if world.controller in ['Manual', 'Network']:
                        world.hud.notification(f"{world.controller} -> PID")
                        world.controller = 'PID'
                        world.agent.new_plan = True
                        world.recover_time = time()

        keys = pygame.key.get_pressed()
        self._parse_vehicle_keys(keys, clock.get_time(), world)

        result = world.agent.run_step()
        control = result['control']
        if self.hold_condition == False and result['road'] in [1, 2]:
            self.hold_condition = True
            world.condition = result['road']

        if world.controller == 'PID':
            control.manual_gear_shift = False
            world.player.apply_control(control)
        elif world.controller == 'Network':
            # ======================================Network =========================================================
            state = pygame.surfarray.pixels3d(world.displaybuffer)
            brake, steer = world.network.inference(state, world.condition)

            # control.brake=brake
            control.steer = steer
            world.player.apply_control(control)
        elif world.controller == 'Manual':
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds, world):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]
        if any([keys[K_w], keys[K_s], keys[K_a], keys[K_d]]):
            if world.controller != 'Manual':
                world.hud.notification(f'{world.controller} -> Manual')
                world.controller = 'Manual'
                world.agent.new_plan = False

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    # @staticmethod
    def _is_quit_shortcut(self, key):

        if (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL):
            import shutil
            # shutil.rmtree(self.iter_folder)
            return True
        else:
            return False


# ==============================================================================
# -- game_loop() ---------------------------------------------------------
# ==============================================================================

def game_loop(args):
    pygame.init()
    pygame.font.init()
    print(f'Experiment: {args.experiment} with {args.policy}')
    if args.experiment in [1, 2, 3]:
        policies = {'basic': BasicResNet(), 'branch': BranchNet()}
        network = ActionInterventionAgent(policies[args.policy], args=args)
    else:
        policies = {'branch': SelfPred()}
        network = IntentionInterventionAgent(policies[args.policy],args= args)
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args.filter, network)
        controller = KeyboardControl(world, experiment=args.experiment, start_in_Roaming=False)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return

            # # as soon as the server is ready continue!
            # if not world.world.wait_for_tick(10.0):
            #     continue

            world.tick(clock)
            world.render(display)
            pygame.display.flip()


    finally:
        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '--port',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.bmw.grandtourer',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument("-a", "--robot", type=str,
                           choices=["Roaming", "Basic"],
                           help="select which robot to run",
                           default="Basic")
    argparser.add_argument('-g', '--GPU', nargs='+', type=int, default=[0])

    argparser.add_argument('--experiment', type=str, default='baseline_2')
    argparser.add_argument('-p', '--policy', type=str, default='branch')
    argparser.add_argument('-t', '--test', type=int, default=0)
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    # print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
