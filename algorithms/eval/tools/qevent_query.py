#!/usr/bin/env python3

import bson.json_util as json_util
import pymongo

# Fields with value 0 will not be returned in query result.
__EVENT_FIELD_MASK__ = {'_id': 0, 'dbinserttime': 0, 'uid': 0}

__DB_HOST__ = "dds-8vb12de0083364541.mongodb.zhangbei.rds.aliyuncs.com"
__DB_PORT__ = 3717
__DB_NAME__ = "analysis_db_prod"
__COLLECTIONS__ = "qevent_proto_data"
__DB_USERNAME__ = "test"
__PASSWORD__ = "Q1ngZh0u"


def _ConnectMongoDb(host, port, db_name, collection, username, password):
    client = pymongo.MongoClient(
        f'mongodb://{username}:{password}@{host}:{port}/{db_name}')
    return client[db_name][collection]


# Query an event by name, with limited number of results.
def QueryQEventByNameLimitByN(client, event_name, num):
    return client.find({"name": event_name}, __EVENT_FIELD_MASK__).limit(num)


# Query without limiting the number of events.
def QueryQEventByName(client, event_name, limit):
    return client.find({"name": event_name}, __EVENT_FIELD_MASK__).limit(limit)


# A general qevent query interface that uses the `find` function.
def FindQEvents(client, find_args, limit):
    return client.find(find_args, __EVENT_FIELD_MASK__).limit(limit)


def AggregateQEvents(client, aggregate_args):
    return client.aggregate(aggregate_args)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="""This is a tool to help quickly query mongodb QEvents. 
    This is an example:
        bazel run //offboard/eval/tools:qevent_query -- --qevent_name "aeb_signal_triggered" --limit=5"""
    )
    parser.add_argument("--host",
                        type=str,
                        default=__DB_HOST__,
                        help="The mongodb host address.")
    parser.add_argument("--port",
                        type=int,
                        default=__DB_PORT__,
                        help="The mongodb port.")
    parser.add_argument("--db_name",
                        type=str,
                        default=__DB_NAME__,
                        help="The db name.")
    parser.add_argument("--collection",
                        type=str,
                        default=__COLLECTIONS__,
                        help="The qevent collection name.")
    parser.add_argument("--username",
                        type=str,
                        default=__DB_USERNAME__,
                        help="The db username.")
    parser.add_argument("--limit",
                        type=int,
                        help="Limit the number of returned documents.")
    parser.add_argument("--qevent_name", help="The QEvent name.")
    parser.add_argument("--find",
                        help="The arguments used in mongo DB `find` command.")
    parser.add_argument(
        "--aggregate",
        help="The arguments used in mongo DB `aggregate` command.")
    parser.add_argument("--password",
                        type=str,
                        default=__PASSWORD__,
                        help="The db password.")

    args = parser.parse_args()

    client = _ConnectMongoDb(args.host, args.port, args.db_name,
                             args.collection, args.username, args.password)

    if args.limit and args.limit > 0 and args.qevent_name:
        for event in QueryQEventByNameLimitByN(client, args.qevent_name,
                                               args.limit):
            print(json_util.dumps(event))
    elif args.find:
        for event in FindQEvents(client, json_util.loads(args.find),
                                 args.limit):
            print(json_util.dumps(event))
    elif args.aggregate:
        for event in AggregateQEvents(client, json_util.loads(args.aggregate)):
            print(json_util.dumps(event))
    else:
        parser.print_help()
