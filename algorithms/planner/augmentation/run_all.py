import os
import sys

scenarios = os.listdir('/tmp/scenarios')
scenarios.sort()
for scenario in scenarios:
    print("Running scenario {}".format(scenario))
    cmd = "bazel run -c opt --copt='-Wno-unused-variable' offboard/simulation/simulator_main -- --sim_mode=dsim --scenario_path='/tmp/scenarios/{}' --vantage_save_pngs_to_path='/tmp/pngs/{}'".format(
        scenario, scenario)
    os.system(cmd)
