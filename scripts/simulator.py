import roslaunch
import rospy

import argparse

from os.path import abspath, isfile

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('-c', '--config',
                    help='A configuration file provided in json format')
parser.add_argument('-m', '--map',
                    help='A heightmap provided in pgm format')

args = parser.parse_args()

cli_args = ['emc_simulator', 'sim.launch']

if args.config:
    config_filename = abspath(args.config)
    print(f"user config file provided {config_filename}")
    if not isfile(config_filename):
        print("But file does not exist!")
        raise FileNotFoundError
    cli_args.append(f'config:={config_filename}')

if args.map:
    map_filename = abspath(args.map)
    print(f"user map file provided {map_filename}")
    if not isfile(map_filename):
        print("But file does not exist!")
        raise FileNotFoundError
    cli_args.append(f'map:={map_filename}')


roslaunch_args = cli_args[2:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

launch.start()

try:
    launch.spin()
finally:
    # After Ctrl+C, stop all nodes from running
    launch.shutdown()
