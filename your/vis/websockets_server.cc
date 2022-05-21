#include "onboard/vis/websockets_server.h"

#include <fstream>

#include "onboard/lite/logging.h"
#include "onboard/utils/thread_util.h"

namespace qcraft::websocket {

namespace {
constexpr int kMaxWebSocketErrCount = 3;
constexpr char sQViewDocRoot[] = "/qcraft/production/qview/templates";
constexpr char sQViewStaticRoot[] = "/qcraft/production/qview";
}  // namespace

WebsocketServer::WebsocketServer() {
  // Initialize Asio Transport
  m_server_.init_asio();
  m_server_.set_reuse_addr(true);

  // Register handler callbacks
  m_server_.set_open_handler(std::bind(&WebsocketServer::OnOpen, this, _1));
  m_server_.set_close_handler(std::bind(&WebsocketServer::OnClose, this, _1));
  m_server_.set_message_handler(
      std::bind(&WebsocketServer::OnMessage, this, _1, _2));
}

WebsocketServer::~WebsocketServer() {
  Stop();
  if (thread_msg_process_.joinable()) {
    thread_msg_process_.join();
  }

  if (thread_main_loop_.joinable()) {
    thread_main_loop_.join();
  }

  m_server_.stop();
}

void WebsocketServer::EnableHttpService() {
  m_server_.set_http_handler(std::bind(&WebsocketServer::OnHttp, this, _1));
}

void WebsocketServer::Run(uint16_t port) {
  QSetThreadName("WebProcessRun");
  m_server_.clear_access_channels(websocketpp::log::alevel::all);

  int ws_err_count = 0;
  while (ws_err_count < kMaxWebSocketErrCount) {
    try {
      // listen on specified port
      m_server_.listen(port);

      // Start the server accept loop
      m_server_.start_accept();

      // Start the ASIO io_service run loop
      m_server_.run();

      // success start service and return;
      return;
    } catch (const std::exception &e) {
      ws_err_count++;
      QLOG(ERROR) << " WebsocketServer listen failed " << ws_err_count
                  << " times, retry...";
      m_server_.stop();
      // retry starting service after 1000ms later
      sleep(1);
    } catch (...) {
      ws_err_count++;
      QLOG(ERROR) << " WebsocketServer listen failed " << ws_err_count
                  << " times, retry...";
      m_server_.stop();
      // retry starting service after 1000ms later
      sleep(3);
    }
  }

  QLOG(FATAL) << " WebsocketServer start failed " << ws_err_count
              << " times, service not avaliable "
              << " check with lsof -i:" << port;
}

void WebsocketServer::AsyncRun(uint16_t port) {
  // Start a thread to run the processing loop; broadcast msgs to clients
  // This part can be reconsitution to multi thead pool to process message
  // sending Also need to consider the drop out timing this thread
  thread_msg_process_ =
      std::thread(std::bind(&WebsocketServer::ProcessMessages, this));
  std::cout << "start WebsocketServer::thread_msg_process_ done\n";

  thread_main_loop_ = std::thread(std::bind(&WebsocketServer::Run, this, port));
  std::cout << "start WebsocketServer::thread_main_loop_ done\n";
}

void WebsocketServer::Stop() {
  {
    absl::MutexLock lock(&m_action_mutex_);
    m_actions_.push(action(STOP, connection_hdl(), ""));
  }
  m_action_cond_.Signal();
}

void WebsocketServer::OnOpen(connection_hdl hdl) {
  {
    absl::MutexLock lock(&m_action_mutex_);
    // std::cout << "on_open" << std::endl;
    m_actions_.push(action(SUBSCRIBE, hdl));
  }
  m_action_cond_.Signal();
}

void WebsocketServer::OnClose(connection_hdl hdl) {
  {
    absl::MutexLock lock(&m_action_mutex_);
    // std::cout << "on_close" << std::endl;
    m_actions_.push(action(UNSUBSCRIBE, hdl));
  }
  m_action_cond_.Signal();
}

void WebsocketServer::OnMessage(connection_hdl hdl, server::message_ptr msg) {
  // queue message up for sending by processing thread
  {
    absl::MutexLock lock(&m_action_mutex_);
    std::cout << "on_message" << std::endl;
    m_actions_.push(action(MESSAGE, hdl, msg->get_payload()));
  }
  m_action_cond_.Signal();
}

void WebsocketServer::OnHttp(connection_hdl hdl) {
  // Upgrade our connection handle to a full connection_ptr
  server::connection_ptr con = m_server_.get_con_from_hdl(hdl);

  std::string filename = con->get_resource();

  m_server_.get_alog().write(websocketpp::log::alevel::app,
                             "http request1: " + filename);

  auto find_param = filename.find("?");
  if (std::string::npos != find_param) {
    filename = filename.substr(0, find_param);
  }

  find_param = filename.find("qview-static");
  if (std::string::npos != find_param) {
    filename = filename.substr(find_param + strlen("qview-static"));
  }

  if (filename.back() == '/' ||
      std::string::npos != filename.find("index.html")) {
    filename = "/car_show.html";
  }

  if (std::string::npos != filename.find(".html")) {
    filename = sQViewDocRoot + filename;
  } else {
    filename = sQViewStaticRoot + filename;
  }

  m_server_.get_alog().write(websocketpp::log::alevel::app,
                             "http request2: " + filename);

  std::ifstream file;
  file.open(filename.c_str(), std::ios::in);
  do {
    if (!file) {
      break;
    }

    file.seekg(0, std::ios::end);
    // if the filename is a directory, file.tellg() will return
    // 0x8000000000000000, return 404 error response if the file.tellg() > 100M
    if (file.tellg() > 100 * 1000 * 1000) {
      file.close();
      break;
    }

    std::string response;
    response.reserve(file.tellg());
    file.seekg(0, std::ios::beg);
    response.assign((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
    file.close();
    con->set_body(response);
    con->set_status(websocketpp::http::status_code::ok);
    return;
  } while (0);

  // 404 error
  std::stringstream ss;
  ss << "<!doctype html><html><head>"
     << "<title>Error 404 (Resource not found)</title><body>"
     << "<h1>Error 404</h1>"
     << "<p>The requested URL " << filename
     << " was not found on this server.</p>"
     << "</body></head></html>";

  con->set_body(ss.str());
  con->set_status(websocketpp::http::status_code::not_found);
  return;
}

void WebsocketServer::BroadcastMessage(std::string msg) {
  // queue message up for sending by processing thread
  {
    absl::MutexLock lock(&m_action_mutex_);
    m_actions_.push(action(MESSAGE, connection_hdl(), msg));
  }
  m_action_cond_.Signal();
}

// static long GetNowMs() {
//   return std::chrono::duration_cast<std::chrono::milliseconds>(
//     std::chrono::system_clock::now().time_since_epoch()).count();
// }

void WebsocketServer::ProcessMessages() {
  QSetThreadName("WebProcessMsg");
  while (true) {
    m_action_mutex_.Lock();
    while (m_actions_.empty()) {
      m_action_cond_.Wait(&m_action_mutex_);
    }

    action a = m_actions_.front();
    m_actions_.pop();

    m_action_mutex_.Unlock();

    if (a.type == SUBSCRIBE) {
      absl::MutexLock lock(&m_con_mutex_);
      m_connections_.insert(a.hdl);
    } else if (a.type == UNSUBSCRIBE) {
      absl::MutexLock lock(&m_con_mutex_);
      m_connections_.erase(a.hdl);
    } else if (a.type == MESSAGE) {
      absl::MutexLock lock(&m_con_mutex_);
      for (auto &con : m_connections_) {
        websocketpp::lib::error_code ec;
        m_server_.send(con, a.msg, websocketpp::frame::opcode::text, ec);
        if (ec) {
          QLOG(ERROR) << " WS send error " << ec.value() << " " << ec.message();
        }
      }
    } else if (a.type == STOP) {
      absl::MutexLock lock(&m_con_mutex_);
      for (auto &con : m_connections_) {
        websocketpp::lib::error_code ec;
        m_server_.close(con, websocketpp::close::status::normal, "STOP", ec);
        if (ec) {
          QLOG(ERROR) << " WS send error " << ec.value() << " " << ec.message();
        }
      }
      m_server_.stop();
      break;
    }
  }
}
}  // namespace qcraft::websocket
