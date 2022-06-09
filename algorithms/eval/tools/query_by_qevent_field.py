"""
This script helps to query qevents based on specific field and conditions.
The output is a csv file with each entry as a runname and a timestamp of qevent_proto.

Please check the following link for env with presto.
https://qcraft.atlassian.net/wiki/spaces/WEIT/pages/1084981292/Presto.

To query the first 100 entries between 2021-12-17 and 2021-12-30 with field 'index'
of qevent 'traj_opt_ddp_postprocess_u' larger than 1 and smaller than 3, use:
bazel run //offboard/eval/tools:query_by_qevent_field -- --csv_path test.csv \
    --qevent traj_opt_ddp_postprocess_u --start_date 20211217 --end_date 20211230 \
    --field index --val_type int --conditions "> 1" "< 3" --limit 100

To query the first 50 entries between 2022-03-01 and 2022-04-10 with field 'car_id'
of qevent 'planner_delay_too_much' equals "Q1004", use:
bazel run //offboard/eval/tools:query_by_qevent_field -- --csv_path test.csv \
    --qevent planner_delay_too_much --start_date 20220301 --end_date 20220410 \
    --field car_id --val_type varchar --conditions "= 'Q1004'" --limit 50

If no condition is provided alongside field, all entries with the given qevent containing
the specified field will hit the query:
bazel run //offboard/eval/tools:query_by_qevent_field -- --csv_path test.csv \
    --qevent AEB_triggered --start_date 20220401 --end_date 20220410 \
    --field object_type --limit 50

For value type options, 'int', 'double' and 'varchar' would already satisfy common needs.
For all valid types refer to https://prestodb.io/docs/current/language/types.html.
"""

import sys

sys.path.append('/qcraft/offboard/db/presto/')

import pypresto


def query_from_presto(query: str,
                      catalog: str = 'mongodb-analysis-prod',
                      schema: str = 'analysis_db_prod'):
    """
    This function queries with a given statement using presto.
    It returns a pandas.Dataframe as value.
    """
    presto_cur = pypresto.PyPresto(catalog=catalog, schema=schema)
    df = presto_cur.fetch(sql_query=query)
    print(f"\nQuery finished, valid record num: {len(df)}")

    return df


def gen_query_statement(qevent: str,
                        start_date: str,
                        end_date: str,
                        field: str = None,
                        val_type: str = None,
                        conditions: list = None,
                        limit: int = None):
    statement = f"SELECT\n" \
                f"    runname,\n" \
                f"    timestamp\n" \
                f"FROM qevent_proto_data\n" \
                f"WHERE\n" \
                f"    name = \'{qevent}\'\n" \
                f"AND runname >= \'{start_date}\'\n" \
                f"AND runname <= \'{end_date}z\'"

    if field and not conditions:
        statement = statement + \
            f"\nAND any_match(transform(aggregatefields, x -> x.key),\n" \
            f"              x -> x = \'{field}\')"
    if field and val_type and conditions:
        for condition in conditions:
            statement = statement + \
                f"\nAND try_cast(\n" \
                f"        element_at(map(transform(aggregatefields, x -> x.key),\n" \
                f"                       transform(aggregatefields, x -> x.value)),\n" \
                f"                   \'{field}\')\n" \
                f"        AS {val_type}) {condition}"
    if limit:
        statement = statement + f"\nLIMIT {limit}"

    return statement


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv_path",
                        type=str,
                        required=True,
                        help="The output csv file path.")
    parser.add_argument("--qevent",
                        type=str,
                        required=True,
                        help="The QEvent name to be queried.")
    parser.add_argument("--start_date",
                        type=str,
                        required=True,
                        help="The earliest date of the desired runs.")
    parser.add_argument("--end_date",
                        type=str,
                        required=True,
                        help="The latest date of the desired runs.")
    parser.add_argument("--field",
                        type=str,
                        default=None,
                        help="The QEvent field to be queried.")
    parser.add_argument("--val_type",
                        type=str,
                        default=None,
                        help="The value type of the specified field.")
    parser.add_argument("--conditions",
                        default=None,
                        nargs='+',
                        help="The query conditions to be fulfilled.")
    parser.add_argument("--limit",
                        type=int,
                        default=None,
                        help="Maximal amount of query results.")

    args = parser.parse_args()

    query_statement = gen_query_statement(qevent=args.qevent,
                                          start_date=args.start_date,
                                          end_date=args.end_date,
                                          field=args.field,
                                          val_type=args.val_type,
                                          conditions=args.conditions,
                                          limit=args.limit)
    print(
        f"\nQuerying database with statement:\n{query_statement}\n"
        "This could be time-consuming if a large date interval is specified...")

    df = query_from_presto(query_statement)
    df.to_csv(args.csv_path, index=False)
    print(f"Queried data saved to {args.csv_path}")
