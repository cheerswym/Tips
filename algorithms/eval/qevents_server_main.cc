
#include <grpc++/grpc++.h>

#include "absl/strings/str_format.h"
#include "offboard/db/cassandra/cass_table_store.h"
#include "offboard/eval/proto/qevents_service.grpc.pb.h"
#include "offboard/eval/services/qevents/factory.h"
#include "offboard/eval/services/qevents/qevents.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/proto/port.pb.h"

DEFINE_string(cass_host_ip, "127.0.0.1", "The cassandra server address");
DEFINE_string(username, "", "The cassandra server username");
DEFINE_string(password, "", "The cassandra server user password");
DEFINE_string(keyspace, "qevent_db",
              "Database name. The keyspace in cassandra DB");
DEFINE_string(keyspace_lite, "qevent_db_lite",
              "Database name. The keyspace in cassandra DB");
DEFINE_string(qevents_server_addr, "0.0.0.0:50051", "QEvents server address");

DEFINE_int32(
    db_request_timeout_ms, 25000,
    "Specifies the amount of time in millseconds to wait before timing out an"
    "HTTP request. For example, consider increasing these times when "
    "transferring large files.");
DEFINE_int32(
    db_connect_timeout_ms, 3000,
    "Specifies the amount of time in millseconds to wait before timing out an"
    "HTTP request. For example, consider increasing these times when "
    "transferring large files.");

std::pair<qcraft::cassandra::CassandraClusterPtr,
          qcraft::cassandra::CassandraSessionPtr>
GetCassandraConnection() {
  qcraft::cassandra::CassandraClusterPtr cluster =
      qcraft::cassandra::CreateCassCluster(FLAGS_cass_host_ip,
                                           FLAGS_db_connect_timeout_ms,
                                           FLAGS_db_request_timeout_ms);
  qcraft::cassandra::CassandraSessionPtr session =
      qcraft::cassandra::CreateCassSession();
  cass_cluster_set_credentials(cluster.get(), FLAGS_username.c_str(),
                               FLAGS_password.c_str());
  CHECK(qcraft::cassandra::ConnectCassSession(cluster, session) == CASS_OK);
  return std::make_pair(cluster, session);
}

qcraft::cassandra::CassTableOptions GetCassandraOptions(
    const std::string &keyspace, const std::string &table) {
  static auto connection = GetCassandraConnection();

  return qcraft::cassandra::CassTableOptions{
      .keyspace = keyspace,
      .table_name = table,
      .cluster_ = connection.first,
      .session_ = connection.second,
  };
}

int main(int argc, char *argv[]) {
  qcraft::InitQCraft(&argc, &argv);

  std::string server_address =
      absl::StrFormat("0.0.0.0:%d", qcraft::QEVENT_SERVICE_PORT);
  qcraft::eval::QEventsServiceImpl service =
      qcraft::eval::QEventsServerWithCassTableOptions(
          GetCassandraOptions(FLAGS_keyspace, "qevents"),
          GetCassandraOptions(FLAGS_keyspace_lite, "qevents"),
          GetCassandraOptions(FLAGS_keyspace, "stats"));

  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  server->Wait();
}
