#ifndef ONBOARD_LITE_TRANSPORT_TRANSMITTER_GRPC_STREAM_TRANSMITTER_H_
#define ONBOARD_LITE_TRANSPORT_TRANSMITTER_GRPC_STREAM_TRANSMITTER_H_

#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>

#include "onboard/lite/logging.h"
#include "onboard/lite/transport/transmitter/transmitter.h"

namespace qcraft {

class GRPCStreamTransmitter : public Transmitter {
 public:
  explicit GRPCStreamTransmitter(const NodeConfig &node_config,
                                 const RunParamsProtoV2 &);

  virtual ~GRPCStreamTransmitter();

  void Enable() override;

  void Disable() override;

  bool Transmit(const LiteMsgWrapper &lite_msg) override;

  bool Transmit(const ShmMessage &shm_message) override;

 private:
  template <typename Request, typename Response>
  class TimedStreamWriter {
   public:
    TimedStreamWriter(
        std::function<std::unique_ptr<
            grpc::ClientReaderWriter<Request, Response>>(grpc::ClientContext *)>
            call)
        : call_(call) {}

    ~TimedStreamWriter() { Disable(); }

    void Enable();
    void Disable();

    // not thread-safe
    bool Transmit(const Request &request, int64_t timeout_ms);
    bool Transmit(const Request &request);

   private:
    void TimerLoop();

    template <typename Lock, typename Predicate>
    bool WaitFor(Lock *lock, int64_t timeout_ms, Predicate predicate);

   private:
    std::condition_variable cv_;
    std::mutex mutex_;
    std::future<void> timer_loop_;
    std::unique_ptr<grpc::ClientContext> client_context_;
    std::shared_ptr<grpc::ClientReaderWriter<Request, Response>> stream_;
    std::function<std::unique_ptr<grpc::ClientReaderWriter<Request, Response>>(
        grpc::ClientContext *)>
        call_;

    enum State {
      INIT = 1,
      ENABLED,
      WRITING,
      WRITE_COMPLETE,
      SHUTTING_DOWN
    } state_ = INIT;
    int64_t timeout_ms_;
  };

  std::unique_ptr<
      TimedStreamWriter<TransmitLiteMsgRequest, TransmitLiteMsgResponse>>
      lite_msg_writer_;
  std::unique_ptr<
      TimedStreamWriter<TransmitShmMsgRequest, TransmitShmMsgResponse>>
      shm_msg_writer_;
};

template <typename Request, typename Response>
void GRPCStreamTransmitter::TimedStreamWriter<Request, Response>::Enable() {
  std::lock_guard<std::mutex> lock(mutex_);

  client_context_ = std::make_unique<grpc::ClientContext>();
  stream_ = call_(client_context_.get());
  state_ = ENABLED;
  timer_loop_ = std::async(
      std::launch::async,
      std::bind(&GRPCStreamTransmitter::TimedStreamWriter<Request,
                                                          Response>::TimerLoop,
                this));
}

template <typename Request, typename Response>
void GRPCStreamTransmitter::TimedStreamWriter<Request, Response>::Disable() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == INIT || state_ == SHUTTING_DOWN) return;
    state_ = SHUTTING_DOWN;
    cv_.notify_all();
  }
  client_context_->TryCancel();
  timer_loop_.wait();
  stream_.reset();
  client_context_.reset();
}

template <typename Request, typename Response>
template <typename Lock, typename Predicate>
bool GRPCStreamTransmitter::TimedStreamWriter<Request, Response>::WaitFor(
    Lock *lock, int64_t timeout_ms, Predicate predicate) {
  if (timeout_ms > 0) {
    return cv_.wait_for(*lock, std::chrono::milliseconds(timeout_ms),
                        predicate);
  } else {
    cv_.wait(*lock, predicate);
    return true;
  }
}

template <typename Request, typename Response>
bool GRPCStreamTransmitter::TimedStreamWriter<Request, Response>::Transmit(
    const Request &request, int64_t timeout_ms) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == SHUTTING_DOWN) {
      return false;
    }
    state_ = WRITING;
    timeout_ms_ = timeout_ms;
    cv_.notify_all();
  }
  bool success = stream_->Write(request);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = WRITE_COMPLETE;
    cv_.notify_all();
  }
  if (!success) {
    QLOG_EVERY_N_SEC(WARNING, 3.0) << "[Transmit] message write failed";
    return false;
  }
  return true;
}

template <typename Request, typename Response>
bool GRPCStreamTransmitter::TimedStreamWriter<Request, Response>::Transmit(
    const Request &request) {
  return Transmit(request, -1);
}

template <typename Request, typename Response>
void GRPCStreamTransmitter::TimedStreamWriter<Request, Response>::TimerLoop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (state_ != SHUTTING_DOWN) {
    cv_.wait(lock,
             [&]() { return state_ == WRITING || state_ == SHUTTING_DOWN; });
    if (state_ == SHUTTING_DOWN) {
      break;
    }
    bool success = WaitFor(&lock, timeout_ms_, [&]() {
      return state_ == WRITE_COMPLETE || state_ == SHUTTING_DOWN;
    });
    if (!success) {
      QLOG(WARNING) << "[Transmit] message write timeout. try cancel";
      client_context_->TryCancel();
    }
  }
}

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_TRANSMITTER_GRPC_STREAM_TRANSMITTER_H_
