#!/usr/bin/env python3

import numbers

import numpy as np
from google.protobuf import text_format
from matplotlib import pyplot as plt

from offboard.eval.tools import qevent_loader
from offboard.simulation.simulation_test.proto import sim_snippet_pb2


def find_value_by_field_key(qevent, key):
    for field in qevent.fields:
        if field.key == key:
            return getattr(field, field.WhichOneof("value"))
    return None


def ToSimEventsBySimLog(qevents, start_offset, end_offset):
    sim_events = sim_snippet_pb2.SimSnippets()
    for qevent in qevents:
        sim_event = sim_snippet_pb2.SimSnippet()
        sim_event.sim_task_id = find_value_by_field_key(qevent, "task")
        sim_event.timestamp_interval.timestamp = qevent.timestamp
        sim_event.timestamp_interval.start_offset = start_offset
        sim_event.timestamp_interval.end_offset = end_offset
        sim_events.snippets.append(sim_event)
    return sim_events


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="This is a tool to load and convert a certain qevent field from sim job."
    )
    parser.add_argument("--job_id", help="The job id.", required=True)
    parser.add_argument("--qevent_name", help="The QEvent name.", required=True)
    parser.add_argument(
        "--field_name", help="The QEvent field name to be queried.", required=True
    )
    parser.add_argument("--csv", help="The output csv file path.")
    parser.add_argument(
        "--stats",
        help="Print statistics of the field values (numerical types only).",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "--hist",
        help="Output path for histogram of the field values (numerical types only).",
    )
    parser.add_argument("--sim", type=str, help="Export to simulation format.")
    args = parser.parse_args()
    if not (args.csv or args.stats or args.hist or args.sim):
        parser.error("No converting mode specified!")

    qevents = qevent_loader.load_qevents_by_job_id(
        args.job_id, args.qevent_name
    ).qevents

    info_list = list()
    for qevent in qevents:
        task = find_value_by_field_key(qevent, "task")
        val = find_value_by_field_key(qevent, args.field_name)
        if task is not None and val is not None:
            info_list.append([task, qevent.timestamp, val])

    if args.csv:
        with open(args.csv, 'w') as fout:
            fout.write("task,timestamp,value\n")
            for task, timestamp, val in info_list:
                fout.write(f"{task},{timestamp},{val}\n")
            print(f"CSV file written to {args.csv}")

    if args.stats or args.hist:
        # The following modes require numerical types.
        for task, timestamp, val in info_list:
            if not isinstance(val, numbers.Number):
                print(
                    f"Non-numerical value \"{val}\" of field \"{args.field_name}\" "
                    f"found in task {task} at {timestamp}, exiting..."
                )
                exit(0)
        val_arr = np.array(list(zip(*info_list))[-1])

        if args.stats:
            print(
                f"Range: [{np.min(val_arr)} {np.max(val_arr)}]\n"
                f"Quartiles: {np.percentile(val_arr, [25, 50, 75])}\n"
                f"Mean: {np.mean(val_arr)}\n"
                f"Std: {np.std(val_arr)}"
            )

        if args.hist:
            plt.hist(val_arr, density=True)
            plt.xlabel("Field value")
            plt.ylabel("Count")
            plt.grid(True)
            plt.savefig(args.hist, format="svg")
            print(f"Histogram saved to {args.hist}")

    if args.sim:
        sim_events = ToSimEventsBySimLog(qevents, 0.0, 0.0)
        with open(args.sim, 'w') as f:
            f.write(text_format.MessageToString(sim_events))
            print("Simulation snippets are written to %s" % args.sim)
