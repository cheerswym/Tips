from datetime import datetime
import json
from typing import List

from google.protobuf import json_format
from google.protobuf import text_format
import grpc

from offboard.eval.proto import qevents_service_pb2
from offboard.eval.proto import qevents_service_pb2_grpc
from onboard.eval.proto import qevent_pb2 as qevent_proto

GRPC_CHANNEL_OF_SIM_SERVER = 'qevents.qcraftai.com:50051'
FIELD_TYPE_DICT = {
    "int64_val": qevent_proto.AggregateField.INT64,
    "double_val": qevent_proto.AggregateField.DOUBLE,
    "string_val": qevent_proto.AggregateField.STRING,
    "bool_val": qevent_proto.AggregateField.BOOL
}


def load_qevents_by_job_id(job_id: str,
                           event_name: str) -> qevent_proto.QEventListProto:
    '''
    Load qevents from sim-server which source_id is {job_id} and named {event_name}, convert qevents into list.
    '''
    channel = grpc.insecure_channel(GRPC_CHANNEL_OF_SIM_SERVER,
                                    options=[
                                        ('grpc.max_send_message_length', -1),
                                        ('grpc.max_receive_message_length', -1),
                                    ])
    stub = qevents_service_pb2_grpc.QEventsServiceStub(channel)
    response = stub.ListEvents(
        qevents_service_pb2.ListEventsRequest(
            filter=f"source_id={job_id} name={event_name}"))
    qevent_list = qevent_proto.QEventListProto()
    qevent_list.qevents.extend(response.events)
    return qevent_list


def load_qevents_by_timestamp(
        start_time: int, end_time: int,
        event_name: str) -> List[qevent_proto.QEventProto]:
    '''                         
    Load qevents from sim-server which timestamp in [start_time, end_time] and named {event_name}, convert qevents into list.
    '''
    start_time -= 1
    end_time += 1
    channel = grpc.insecure_channel(GRPC_CHANNEL_OF_SIM_SERVER)
    stub = qevents_service_pb2_grpc.QEventsServiceStub(channel)
    response = stub.ListEvents(
        qevents_service_pb2.ListEventsRequest(
            filter=
            f"timestamp>{start_time} timestamp<{end_time} name={event_name}"))
    return response.events


def load_qevents_by_datetime(start_date: datetime, end_date: datetime,
                             event_name: str) -> List[qevent_proto.QEventProto]:
    return load_qevents_by_timestamp(start_time=int(start_date.timestamp() *
                                                    1e6),
                                     end_time=int(end_date.timestamp() * 1e6),
                                     event_name=event_name)


def field_to_aggregate_field(
        field: qevent_proto.Field) -> qevent_proto.AggregateField:
    aggregate_field = qevent_proto.AggregateField()
    aggregate_field.key = field.key
    value_type = field.WhichOneof("value")
    aggregate_field.value = str(getattr(field, value_type))
    aggregate_field.type = FIELD_TYPE_DICT[value_type]
    return aggregate_field


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="""This is a tool to load qevent from sim job. 
    This is an example:
        bazel run //offboard/eval/tools:qevent_loader -- --qevent_name="aeb_signal_triggered" --job_id="1730068577201694208" --proto_dir="output.pb.txt" """
    )
    parser.add_argument("--job_id", help="The Job id.")
    parser.add_argument("--qevent_name", help="The QEvent name.")
    parser.add_argument("--proto_dir", help="The output QEvent_Proto file dir.")
    parser.add_argument("--json_dir", help="The output QEvent_Proto file dir.")

    args = parser.parse_args()

    qevent_list = load_qevents_by_job_id(args.job_id, args.qevent_name)

    if args.proto_dir:
        with open(args.proto_dir, 'w') as fout:
            fout.write(text_format.MessageToString(qevent_list))

    if args.json_dir:
        for qevent in qevent_list.qevents:
            for field in qevent.fields:
                qevent.aggregate_fields.append(field_to_aggregate_field(field))
            qevent.ClearField("fields")
        qevent_dict = json_format.MessageToDict(qevent_list)
        with open(args.json_dir, 'w') as fout:
            for qevent in qevent_dict["qevents"]:
                fout.write(json.dumps(qevent))
                fout.write('\n')
