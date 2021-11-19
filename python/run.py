#!/usr/bin/env python

import argparse

from MapEnvironment import MapEnvironment
from tstar_wrapper import TstarWrapper

from IPython import embed

def main(planning_env):

    # init tstar planner.
    tstar = TstarWrapper()
    tstar.set_map(planning_env.map, planning_env.start, planning_env.goal)
    
    # Shortcut the path.
    # TODO (student): Do not shortcut when comparing the performance of algorithms. 
    # Comment this line out when collecting data over performance metrics.
    #plan_short = planner.ShortenPath(plan)

    # Visualize the final path.
    #planning_env.visualize_plan(plan_short)
    
    #embed()


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='tstar')

    parser.add_argument('-m', '--map', type=str, default='.\..\maps\map1.txt',
                        help='The environment to plan on')    
    parser.add_argument('-s', '--start', nargs='+', type=int, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=int, required=True)

    args = parser.parse_args()
 
    # First setup the environment and the robot.
    planning_env = MapEnvironment(args.map, args.start, args.goal)

    main(planning_env)
