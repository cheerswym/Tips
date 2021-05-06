from absl import app
from absl import flags
import urllib.parse

FLAGS = flags.FLAGS
flags.DEFINE_string("batch_name", "batch_default", "batch name")
flags.DEFINE_string("aug_name", "aug_default", "aug name")

flags.DEFINE_string(
    "template",
    "/qcraft/onboard/planner/testdata/scenarios/unprotected_right_agent_test/planner.unprotected_right.agent_aug_array.village_north.template.pb.txt",
    "")
flags.DEFINE_string("param_dict", "agent0_init_speed=15&agent0_init_route_s=5",
                    "")


def Generate(params_override):
    template_file = FLAGS.template
    with open(template_file, "r") as f:
        template_content = f.read()

    params = {}
    search_start = 0
    while True:
        pair_start = template_content.find("{{", search_start)
        if pair_start < 0:
            break
        pair_end = template_content.find("}}", pair_start + 2)
        if pair_end < 0:
            break
        param = template_content[(pair_start + 2):pair_end].split(":")
        key = param[0]
        default_value = param[1]
        params[key] = default_value
        template_content = template_content[:(
            pair_start + 2)] + key + template_content[pair_end:]
        search_start = pair_start + 2 + len(key)

    param_dict = urllib.parse.parse_qs(FLAGS.param_dict)
    for key, value in param_dict.items():
        params[key] = value[0]

    for key, value in params_override.items():
        params[key] = value

    generated_content = template_content
    for key, value in params.items():
        generated_content = generated_content.replace("{{" + key + "}}",
                                                      "{}".format(value))

    return generated_content


def main(argv):
    params = {"batch_name": FLAGS.batch_name, "aug_name": FLAGS.aug_name}
    generated_content = Generate(params)
    print(generated_content)


if __name__ == "__main__":
    app.run(main)
