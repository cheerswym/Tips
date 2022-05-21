# load and plot data
# run in docker:  bazel run //onboard/perception/tracking_validation:visualize_result

from onboard.perception.tracker.tracking_validation.proto import validation_pb2
from google.protobuf import text_format
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import animation
import sys
import os

fig = plt.figure()
ax = plt.axes(xlim=(-5, 15), ylim=(-5, 15))
meas_bbox = Rectangle((0, 0),
                      5.0,
                      2.5,
                      edgecolor='blue',
                      facecolor='none',
                      alpha=1,
                      label="camera_meas")
est_bbox = Rectangle((10, 0),
                     5.0,
                     2.5,
                     edgecolor='green',
                     facecolor='none',
                     alpha=1,
                     label="estimation")


def init():
    ax.add_patch(meas_bbox)
    ax.add_patch(est_bbox)
    plt.title("Extent Filter")
    plt.xlabel("position: x (m)")
    plt.ylabel("position: y (m)")
    plt.legend()

    return meas_bbox, est_bbox


def plot_track(meas, est):

    # ax = plt.gca()

    def animation_function(i):
        meas_bbox.set_width(meas[i][3])
        meas_bbox.set_height(meas[i][2])
        est_bbox.set_width(est[i][3])
        est_bbox.set_height(est[i][2])
        return meas_bbox, est_bbox

    anim = animation.FuncAnimation(fig,
                                   animation_function,
                                   init_func=init,
                                   frames=len(meas),
                                   interval=1)
    plt.show()
    anim.save('/hosthome/Desktop/test.gif', writer='imagemagick')


def load_pbtxt_file(path):
    """Read .pb.txt file.
    Args:
        path: Path to StringIntLabelMap proto text file (.pb.txt file).
    Returns:
        A StringIntLabelMapProto.
    Raises:
        ValueError: If path is not exist.
    """
    if not path:
        raise ValueError('`path` is not exist.')

    with open(path, 'r') as f:
        pbtxt_string = f.read()
        pbtxt = validation_pb2.TrackingValidationProto()
        try:
            text_format.Merge(pbtxt_string, pbtxt)
        except text_format.ParseError:
            pbtxt.ParseFromString(pbtxt_string)

    return pbtxt


if __name__ == '__main__':

    pbtxt_path = '/tmp/extent_data.pb.txt'
    pbtxt = load_pbtxt_file(pbtxt_path)

    time = np.zeros((len(pbtxt.measurements), 1))
    global meas
    meas = np.zeros((len(pbtxt.measurements), 4))
    i = 0
    for infos in pbtxt.measurements:
        for info in infos.measurements:
            if info.HasField("laser_measurement"):
                bbox = info.laser_measurement.detection_bounding_box
                # print(bbox)
                meas[i][0] = bbox.x
                meas[i][1] = bbox.y
                meas[i][2] = bbox.length
                meas[i][3] = bbox.width
                time[i][0] = i
                i += 1
    print("measurement done.")

    i = 0
    gt = np.zeros((len(pbtxt.measurements), 4))
    for info in pbtxt.ground_truths:
        for object_info in info.objects:
            # print(object_info.pos)
            gt[i][0] = object_info.pos.x
            gt[i][1] = object_info.pos.y
            gt[i][2] = object_info.bounding_box.length
            gt[i][3] = object_info.bounding_box.width
            i += 1
    print("groundtruth done.")

    i = 0
    global est
    est = np.zeros((len(pbtxt.measurements), 4))
    for info in pbtxt.estimators:
        for object_info in info.objects:
            # print(object_info.pos)
            est[i][0] = object_info.pos.x
            est[i][1] = object_info.pos.y
            est[i][2] = object_info.bounding_box.length
            est[i][3] = object_info.bounding_box.width
            i += 1
    print("estimation done.")

    # draw picture
    plot_track(meas, est)
