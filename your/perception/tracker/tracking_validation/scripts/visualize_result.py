# load and plot data
# run in docker:  bazel run //onboard/perception/tracking_validation:visualize_result

from onboard.perception.tracker.tracking_validation.proto import validation_pb2
from google.protobuf import text_format
import numpy as np
import matplotlib.pyplot as plt
import sys
import os


def plot_track(my_matrix):

    times = my_matrix.shape[0]
    statedim_shown = 0
    if my_matrix.shape[1] % 2 != 0:
        statedim_shown = (my_matrix.shape[1] - 1) / 3
    else:
        print(my_matrix.shape[1])
    statedim_shown = int(statedim_shown)

    my_matrix_t = my_matrix.transpose()
    gt = my_matrix_t[1:statedim_shown + 1, :]
    meas = my_matrix_t[statedim_shown + 1:2 * statedim_shown + 1, :]
    est = my_matrix_t[2 * statedim_shown + 1:3 * statedim_shown + 1, :]

    plt.subplot(211)
    plt.title("Car Model: UKF with CTRV Path")
    plt.xlabel("position: x (m)")
    plt.ylabel("position: y (m)")
    plt.plot(gt[0], gt[1], 'ro', label="gt")
    # use the for while there are more than one est
    plt.plot(meas[0], meas[1], 'bo', label="meas")
    plt.plot(est[0], est[1], 'go', label="estimation")
    plt.legend()

    plt.subplot(212)
    plt.xlabel("time sequence: t (s)")
    plt.ylabel("RMSE (m)")
    est_rmse = []
    for i in range(len(gt[0])):
        error = (gt[0][i] - est[0][i])**2 + (gt[1][i] - est[1][i])**2
        est_rmse.append(np.sqrt(error))
    print("RMSE of Estimation: ", np.mean(est_rmse))

    plt.plot(my_matrix_t[0], est_rmse, label="est_rmse")
    meas_rmse = []
    for i in range(len(gt[0])):
        error = (gt[0][i] - meas[0][i])**2 + (gt[1][i] - meas[1][i])**2
        meas_rmse.append(np.sqrt(error))
    print("RMSE of Measurement: ", np.mean(meas_rmse))

    plt.plot(my_matrix_t[0], meas_rmse, label="meas_rmse")
    plt.axis([0, gt.shape[1] + 2, 0, max(meas_rmse) * 1.5])
    plt.legend()

    plt.show()


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

    pbtxt_path = '/tmp/data.pb.txt'
    pbtxt = load_pbtxt_file(pbtxt_path)

    time = np.zeros((len(pbtxt.measurements), 1))
    meas = np.zeros((len(pbtxt.measurements), 2))
    i = 0
    for infos in pbtxt.measurements:
        for info in infos.measurements:
            if info.HasField("laser_measurement"):
                bbox = info.laser_measurement.detection_bounding_box
                # print(bbox)
                meas[i][0] = bbox.x
                meas[i][1] = bbox.y
                time[i][0] = i
                i += 1
    print("measurement done.")

    i = 0
    gt = np.zeros((len(pbtxt.measurements), 2))
    for info in pbtxt.ground_truths:
        for object_info in info.objects:
            # print(object_info.pos)
            gt[i][0] = object_info.pos.x
            gt[i][1] = object_info.pos.y
            i += 1
    print("groundtruth done.")

    i = 0
    est = np.zeros((len(pbtxt.measurements), 2))
    for info in pbtxt.estimators:
        for object_info in info.objects:
            # print(object_info.pos)
            est[i][0] = object_info.pos.x
            est[i][1] = object_info.pos.y
            i += 1
    print("estimation done.")

    # draw picture
    plot_track(np.hstack((time, gt, meas, est)))
