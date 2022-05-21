# load and plot data
# run in docker:  bazel run //onboard/perception/tracking_validation:visualize_result_imm

from onboard.perception.tracker.tracking_validation.proto import validation_imm_pb2
from google.protobuf import text_format
import numpy as np
import matplotlib.pyplot as plt
import time


def plot_track(my_matrix):
    print("my_matrix: ", my_matrix)
    # my_matrix here contains m_time, gt, meas, est
    # m_time of N*1, gt of N*4, meas of N*2,
    # est of N*4, and my_matrix of N*11
    m_times = my_matrix.shape[0]
    statedim_shown = 0
    if my_matrix.shape[1] % 2 != 0:
        statedim_shown = (my_matrix.shape[1] - 3) / 2
    else:
        print(my_matrix.shape[1])
    statedim_shown = int(statedim_shown)

    my_matrix_t = my_matrix.transpose()
    meas = my_matrix_t[1:3, :]
    gt = my_matrix_t[3:3 + statedim_shown, :]
    est = my_matrix_t[3 + statedim_shown:, :]

    plt.figure(1)
    plt.subplot(411)
    plt.title("IMM with CP+CA Vx/y=1, Accx/y=1")
    plt.xlabel("position: x (m)")
    plt.ylabel("position: y (m)")
    # use the for while there are more than one est
    plt.plot(gt[0], gt[1], 'ro', label="gt")
    plt.plot(meas[0], meas[1], 'bo', label="meas")
    plt.plot(est[0], est[1], 'go', label="est")

    plt.legend()

    plt.subplot(412)
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

    plt.subplot(413)
    plt.xlabel("time sequence: t / 10^-1 s")
    plt.ylabel("Vx m/s")
    vel_gt = []
    vel_est = []
    for i in range(len(gt[0])):
        vel_gt.append(np.sqrt(gt[2][i]**2 + gt[3][i]**2))
        vel_est.append(np.sqrt(est[2][i]**2 + est[3][i]**2))
    plt.plot(my_matrix_t[0], vel_gt, label="gt")
    plt.plot(my_matrix_t[0], vel_est, label="est")
    plt.axis([0, gt.shape[1] + 2, 0, max(vel_gt) * 1.5])
    plt.legend()


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
        pbtxt = validation_imm_pb2.TrackingValidationProto()
        try:
            text_format.Merge(pbtxt_string, pbtxt)
        except text_format.ParseError:
            pbtxt.ParseFromString(pbtxt_string)

    return pbtxt


if __name__ == '__main__':

    pbtxt_path = '/tmp/data_imm.pb.txt'
    pbtxt = load_pbtxt_file(pbtxt_path)

    i = 0
    filter_type = np.zeros((len(pbtxt.estimators), 1))
    for info in pbtxt.estimators:
        filter_type[i][0] = info.filter
        i += 1

    # we assume that it is only one output now.
    if filter_type[0][0] == 1:
        m_time = np.zeros((len(pbtxt.measurements), 1))
        meas = np.zeros((len(pbtxt.measurements), 2))
        i = 0
        for infos in pbtxt.measurements:
            for info in infos.measurements:
                if info.HasField("laser_measurement"):
                    bbox = info.laser_measurement.detection_bounding_box
                    # print(bbox)
                    meas[i][0] = bbox.x
                    meas[i][1] = bbox.y
                    m_time[i][0] = i
                    i += 1
        print("measurement done.")

        i = 0
        gt = np.zeros((len(pbtxt.measurements), 4))
        for info in pbtxt.ground_truths:
            for object_info in info.objects:
                # print(object_info.pos)
                gt[i][0] = object_info.pos.x
                gt[i][1] = object_info.pos.y
                gt[i][2] = object_info.vel.x
                gt[i][3] = object_info.vel.y
                i += 1
        print("groundtruth done.")

        i = 0
        est = np.zeros((len(pbtxt.measurements), 4))
        for info in pbtxt.estimators:
            for object_info in info.estimations:
                # print(object_info.pos)
                est[i][0] = object_info.est_state.pos.x
                est[i][1] = object_info.est_state.pos.y
                est[i][2] = object_info.est_state.vel.x
                est[i][3] = object_info.est_state.vel.y
                i += 1
        print("estimation done.")
        # draw picture
        plot_track(np.hstack((m_time, gt, meas, est)))
    elif filter_type[0][0] == 0:
        m_time = np.zeros((len(pbtxt.measurements), 1))
        meas = np.zeros((len(pbtxt.measurements), 2))
        i = 0
        for infos in pbtxt.measurements:
            for info in infos.measurements:
                if info.HasField("laser_measurement"):
                    bbox = info.laser_measurement.detection_bounding_box
                    # print(bbox)
                    meas[i][0] = bbox.x
                    meas[i][1] = bbox.y
                    print("meas: ", meas[i][0])
                    m_time[i][0] = i
                    i += 1
        print("measurement done.")

        i = 0
        gt = np.zeros((len(pbtxt.measurements), 4))
        for info in pbtxt.ground_truths:
            for object_info in info.objects:
                # print(object_info.pos)
                gt[i][0] = object_info.pos.x
                gt[i][1] = object_info.pos.y
                gt[i][2] = object_info.vel.x
                gt[i][3] = object_info.vel.y
                i += 1
        print("groundtruth done.")

        i = 0
        est = np.zeros((len(pbtxt.measurements), 4))
        filter_num = pbtxt.estimators[0].estimations[0].motion_num
        est_mu = np.zeros((filter_num, len(pbtxt.measurements)))
        for info in pbtxt.estimators:
            for info_est in info.estimations:
                est[i][0] = info_est.est_state.pos.x
                est[i][1] = info_est.est_state.pos.y
                est[i][2] = info_est.est_state.vel.x
                est[i][3] = info_est.est_state.vel.y
                j = 0
                for info_est_mu in info_est.est_mu.mu:
                    est_mu[j][i] = info_est_mu
                    j += 1
                i += 1
        print("estimation done.")
        print("mes:", meas)
        print("gt:", gt)
        # draw picture
        plot_track(np.hstack((m_time, meas, gt, est)))
        plt.subplot(414)
        plt.xlabel("time sequence: t (10-1 s)")
        plt.ylabel("model probability")
        # motion model used
        motion_name = ['CP', 'CA']
        for j in range(filter_num):
            plt.plot(m_time, est_mu[j], label="%s" % motion_name[j])
        plt.axis([0, est_mu.shape[1] + 2, -0.1, 1.1])
        plt.legend()
        plt.show()
