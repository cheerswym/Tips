#!/usr/bin/env python3

from datetime import datetime
import time
import os
import json
from google.protobuf import text_format

from offboard.simulation.simulation_test.proto import sim_snippet_pb2

__QCRAFT_DIR__ = "/qcraft"


def ModifyEvent(qevent):
    if "absolutetime" in qevent:
        qevent["timestamp"] = qevent["absolutetime"]
    # A hack in web vantage: use a float with six digits after float point to represent an absolute timestamp.
    event_time_six_point_seconds = "%.6f" % (float(qevent["timestamp"]) * 1e-6)
    qevent["webtime"] = event_time_six_point_seconds
    return qevent


def LoadJsonFile(filename):
    events = []
    with open(filename, 'r') as myfile:
        for line in myfile.readlines():
            line = line.strip()
            if len(line) == 0 or line[0] == '#':
                continue
            line = line.replace("'", '"')  # Replace ' to ".
            event = json.loads(line)
            event = ModifyEvent(event)
            events.append(event)
    return events


def GenerateWebVantageUrl(qevent):
    return "http://vantage.qcraftai.com/?run=%s&offset=%s" % (qevent["runname"],
                                                              qevent["webtime"])


def GenerateRunUrl(qevent):
    return "http://sim-dash.qcraftai.com/run_detail/%s" % qevent["runname"]


def ToHtml(qevents, html_filename):
    fw = open(html_filename, 'w')
    fw.write(
        """<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01//EN" "http://www.w3.org/TR/html4/strict.dtd">"""
    )
    fw.write("<html>")
    fw.write("""
    <head>
    <style>
        tr:nth-child(even) {
          background-color: #D6EEEE;
        }
        table th td {
          border: 1px solid white;
          border-collapse: collapse;
          padding: 15px;
        }
        th, td {
          padding-left: 30px;
          padding-right: 30px;
        }
    </style>
    </head>
    """)
    fw.write("<title> Run and timestamp </title>")
    fw.write("<body>")
    fw.write("<table>\n")
    fw.write(
        "<tr style='background-color: #96D4D4'><th>Name</th> <th>Run</th> <th>Timestamp</th><th>Web Vantage</th></tr>\n"
    )
    for qevent in qevents:
        webv_url = GenerateWebVantageUrl(qevent)
        name = '----' if 'name' not in qevent else qevent['name']
        fw.write(
            """<tr><th>%s</th> <th><a href="%s" target="_blank">%s</a></th> <th>%s</th> <th><a href=%s target="_blank">%s</a></th> </tr>\n"""
            % (name, GenerateRunUrl(qevent), qevent['runname'],
               int(qevent['timestamp']) * 1e-6, webv_url, webv_url))
    fw.write("</table>")
    fw.write("</body>")
    fw.write("</html>")
    fw.close()


def ToSimEvents(qevents, start_offset, end_offset):
    sim_events = sim_snippet_pb2.SimSnippets()
    for qevent in qevents:
        sim_event = sim_snippet_pb2.SimSnippet()
        sim_event.run_name = qevent["runname"]
        sim_event.timestamp_interval.timestamp = int(qevent["timestamp"])
        sim_event.timestamp_interval.start_offset = start_offset
        sim_event.timestamp_interval.end_offset = end_offset
        sim_events.snippets.append(sim_event)
    return sim_events


def ToUrls(qevents):
    results = []
    for qevent in qevents:
        results.append(GenerateWebVantageUrl(qevent))
    return "\n".join(results)


def ToCsv(qevents, csv_file):
    fw = open(csv_file, 'w')
    fw.write("qevent,runname,timestamp,webvantage\n")
    for qevent in qevents:
        fw.write("%s,%s,%s,%s\n" %
                 (qevent['name'], qevent['runname'], int(qevent['timestamp']) *
                  1e-6, GenerateWebVantageUrl(qevent)))
    fw.close()


def ToPlannerSnapshots(qevents, snapshot_dir):
    for qevent in qevents:
        dump_command = "cd %s && bash scripts/dump_snapshot.sh --run=%s --timestamp=%s --snapshot_path=%s" % (
            __QCRAFT_DIR__, qevent["runname"], qevent["timestamp"],
            snapshot_dir)
        ret = os.system(dump_command)
        if ret == 0:
            print("Created snapshot at %s:%s." %
                  (qevent["runname"], qevent["timestamp"]))
        else:
            print("Failed to convert %s:%s to snapshot." %
                  (qevent["runname"], qevent["timestamp"]))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "json_file",
        help="The json file name that each line contains a QEvent.")
    parser.add_argument("--url",
                        action='store_true',
                        help="Print url to console.")
    parser.add_argument("--html", type=str, help="Print output to a html file.")
    parser.add_argument("--csv", type=str, help="Export to CSV format.")
    parser.add_argument("--sim", type=str, help="Export to simulation format.")
    parser.add_argument("--snapshot_dir", type=str, help="Export to snapshots.")
    parser.add_argument("--sim_start_offset",
                        type=float,
                        default=-5.0,
                        help="Simulation start offset.")
    parser.add_argument("--sim_end_offset",
                        type=float,
                        default=15.0,
                        help="Simulation end offset.")

    args = parser.parse_args()

    qevents = LoadJsonFile(args.json_file)

    op = False
    if args.url:
        op = True
        print(ToUrls(qevents))

    if args.html:
        op = True
        ToHtml(qevents, args.html)
        print("Webpage is written to %s" % args.html)

    if args.csv:
        op = True
        ToCsv(qevents, args.csv)
        print("CSV file is written to %s" % args.csv)

    if args.snapshot_dir:
        op = True
        if not os.path.exists(args.snapshot_dir):
            os.makedirs(args.snapshot_dir)
        ToPlannerSnapshots(qevents, args.snapshot_dir)
        print("Snapshot files are written to %s" % args.snapshot_dir)

    if args.sim:
        op = True
        sim_events = ToSimEvents(qevents, args.sim_start_offset,
                                 args.sim_end_offset)
        with open(args.sim, 'w') as f:
            f.write(text_format.MessageToString(sim_events))
            print("Simulation snippets are written to %s" % args.sim)

    if not op:
        parser.print_help()
