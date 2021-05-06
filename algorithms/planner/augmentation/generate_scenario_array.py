import datetime
import os
from absl import app
from absl import flags
from absl import logging
from onboard.planner.augmentation import generate_scenario
from onboard.planner.augmentation import augmentation_config_pb2
from google.protobuf import text_format

FLAGS = flags.FLAGS
flags.DEFINE_string("output_prefix", "/tmp/generated_scenario", "")
flags.DEFINE_string("batch", "0", "batch identifier")
flags.DEFINE_string(
    "params",
    "params { name: 'agent0_start_time' min: 12.0 max: 24.0 step: 0.25 }", "")


def GenerateForParamCombination(param_it, param_config):
    params = {}
    for i in range(0, len(param_it)):
        params[param_config[param_it[i]][0]] = param_config[param_it[i]][1]
        param_config = param_config[param_it[i]][2]

    batch_name = "batch_{}".format(FLAGS.batch)
    aug_name = "aug_" + "_".join(["{:02d}".format(i) for i in param_it[::-1]])

    scenario_name = "{}.{}.{}.pb.txt".format(FLAGS.output_prefix, batch_name,
                                             aug_name)
    params["aug_name"] = aug_name

    logging.info("Generating scenario {} with parameters: {}".format(
        scenario_name, params))
    generated_content = generate_scenario.Generate(params)
    with open(scenario_name, "w") as f:
        f.write(generated_content)


def main(argv):
    aug_config = augmentation_config_pb2.AugmentationConfigProto()
    text_format.Parse(FLAGS.params, aug_config)
    logging.info("Augmentation config: {}".format(aug_config))

    param_config = []
    param_num_steps = []
    for aug_param in aug_config.params:
        num_steps = int((aug_param.max - aug_param.min) / aug_param.step) + 1
        param_num_steps.append(num_steps)
        param_config = [(aug_param.name, aug_param.min + i * aug_param.step,
                         param_config) for i in range(0, num_steps)]
    logging.info("Number of steps for parameters: {}".format(param_num_steps))

    num_params = len(aug_config.params)
    param_it = [0] * num_params
    finished = False
    while not finished:
        GenerateForParamCombination(param_it, param_config)
        for i in range(0, num_params):
            param_it[i] += 1
            if param_it[i] < param_num_steps[num_params - 1 - i]:
                break
            param_it[i] = 0
            if i + 1 == num_params:
                finished = True


if __name__ == "__main__":
    app.run(main)
