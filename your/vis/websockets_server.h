#ifndef ONBOARD_VIS_WEBSOCKETS_SERVER_H_
#define ONBOARD_VIS_WEBSOCKETS_SERVER_H_

#include <iostream>
#include <queue>
#include <set>
#include <string>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "absl/synchronization/mutex.h"

namespace qcraft::websocket {

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;
using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

using websocketpp::lib::condition_variable;
using websocketpp::lib::thread;
using websocketpp::lib::unique_lock;

/* on_open insert connection_hdl into channel
 * on_close remove connection_hdl from channel
 * on_message queue send to all channels
 */
enum action_type { SUBSCRIBE, UNSUBSCRIBE, MESSAGE, STOP };

struct action {
  action(action_type t, connection_hdl h) : type(t), hdl(h) {}
  action(action_type t, connection_hdl h, std::string m)
      : type(t), hdl(h), msg(m) {}

  action_type type;
  websocketpp::connection_hdl hdl;
  // server::message_ptr msg;
  std::string msg;
};

class WebsocketServer {
 public:
  WebsocketServer();

  ~WebsocketServer();

  void EnableHttpService();

  void Run(uint16_t port);

  void AsyncRun(uint16_t port);

  void Stop();

  void OnOpen(connection_hdl hdl);

  void OnClose(connection_hdl hdl);

  void OnMessage(connection_hdl hdl, server::message_ptr msg);

  void OnHttp(connection_hdl hdl);

  void BroadcastMessage(std::string msg);

  void ProcessMessages();

 private:
  typedef std::set<connection_hdl, std::owner_less<connection_hdl> > con_list;

  server m_server_;

  absl::Mutex m_action_mutex_;
  absl::Mutex m_con_mutex_;
  absl::CondVar m_action_cond_;

  con_list m_connections_ GUARDED_BY(m_con_mutex_);
  std::queue<action> m_actions_ GUARDED_BY(m_action_mutex_);

  std::thread thread_msg_process_;
  std::thread thread_main_loop_;
};

}  // namespace qcraft::websocket

#endif
