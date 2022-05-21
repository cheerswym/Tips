
#ifndef ONBOARD_LITE_TRANSPORT_RECEIVER_RECEIVER_CONTEXT_H_
#define ONBOARD_LITE_TRANSPORT_RECEIVER_RECEIVER_CONTEXT_H_

#include <grpc++/grpc++.h>

#include <memory>

#include "onboard/global/trace.h"

namespace qcraft {

class ReceiveContext {
 public:
  ReceiveContext() {}

  virtual ~ReceiveContext() {}

  virtual bool NextState(bool) = 0;

  virtual void Reset() = 0;
};

template <typename ServerContextType, typename RequestType,
          typename ResponseType>
class ReceiveContextUnaryImpl final : public ReceiveContext {
 public:
  ReceiveContextUnaryImpl(
      std::function<void(ServerContextType *, RequestType *,
                         grpc::ServerAsyncResponseWriter<ResponseType> *,
                         void *)>
          request_transmit,
      std::function<grpc::Status(RequestType *, ResponseType *)>
          receive_transmit)
      : sever_context_(std::make_unique<ServerContextType>()),
        request_(std::make_unique<RequestType>()),
        response_(std::make_unique<ResponseType>()),
        next_state_(std::bind(&ReceiveContextUnaryImpl::Transimit, this,
                              std::placeholders::_1)),
        request_transmit_(request_transmit),
        receive_transmit_(receive_transmit),
        response_transmit_(sever_context_.get()) {
    request_transmit_(sever_context_.get(), request_.get(), &response_transmit_,
                      static_cast<void *>(this));
  }

  ~ReceiveContextUnaryImpl() override {}

  bool NextState(bool ok) override { return next_state_(ok); }

  void Reset() override {
    sever_context_ = std::make_unique<ServerContextType>();
    request_ = std::make_unique<RequestType>();
    response_transmit_ =
        grpc::ServerAsyncResponseWriter<ResponseType>(sever_context_.get());
    next_state_ = std::bind(&ReceiveContextUnaryImpl::Transimit, this,
                            std::placeholders::_1);
    SCOPED_QTRACE("Request_Transmit");
    request_transmit_(sever_context_.get(), request_.get(), &response_transmit_,
                      static_cast<void *>(this));
  }

 private:
  bool Transimit(bool ok) {
    if (!ok) {
      return false;
    }

    SCOPED_QTRACE("Response_Transmit");
    next_state_ = std::bind(&ReceiveContextUnaryImpl::Complete, this,
                            std::placeholders::_1);
    auto status = receive_transmit_(request_.get(), response_.get());
    response_transmit_.Finish(*response_, status, static_cast<void *>(this));
    return true;
  }

  bool Complete(bool ok) { return false; }

 private:
  std::unique_ptr<ServerContextType> sever_context_;
  std::unique_ptr<RequestType> request_;
  std::unique_ptr<ResponseType> response_;
  std::function<bool(bool)> next_state_;
  std::function<void(ServerContextType *, RequestType *,
                     grpc::ServerAsyncResponseWriter<ResponseType> *, void *)>
      request_transmit_;
  std::function<grpc::Status(RequestType *, ResponseType *)> receive_transmit_;
  grpc::ServerAsyncResponseWriter<ResponseType> response_transmit_;
};

template <typename ServerContextType, typename RequestType,
          typename ResponseType>
class ReceiveContextUnaryStreamImpl final : public ReceiveContext {
 public:
  using StreamType = grpc::ServerAsyncReaderWriter<ResponseType, RequestType>;
  ReceiveContextUnaryStreamImpl(
      std::function<void(ServerContextType *, StreamType *, void *)>
          request_connect,
      std::function<grpc::Status(RequestType *, ResponseType *)>
          receive_transmit)
      : server_context_(std::make_unique<ServerContextType>()),
        stream_(std::make_unique<StreamType>(server_context_.get())),
        next_state_(std::bind(&ReceiveContextUnaryStreamImpl::RequestTransmit,
                              this, std::placeholders::_1)),
        receive_transmit_(receive_transmit),
        request_connect_(request_connect) {
    request_connect_(server_context_.get(), stream_.get(),
                     static_cast<void *>(this));
  }

  ~ReceiveContextUnaryStreamImpl() override {}

  bool NextState(bool ok) override { return next_state_(ok); }

  void Reset() override {
    server_context_ = std::make_unique<ServerContextType>();
    stream_ = std::make_unique<StreamType>(server_context_.get());
    next_state_ = std::bind(&ReceiveContextUnaryStreamImpl::RequestTransmit,
                            this, std::placeholders::_1);
    SCOPED_QTRACE("Request_Transmit");
    request_connect_(server_context_.get(), stream_.get(),
                     static_cast<void *>(this));
  }

 private:
  bool ReceiveTransmit(bool ok) {
    if (!ok) {
      return false;
    }

    SCOPED_QTRACE("Receive_Transmit");
    auto status = receive_transmit_(request_.get(), response_.get());
    return RequestTransmit(ok);
  }

  bool RequestTransmit(bool ok) {
    if (!ok) {
      return false;
    }
    SCOPED_QTRACE("Request_Transmit");
    request_ = std::make_unique<RequestType>();
    response_ = std::make_unique<ResponseType>();
    next_state_ = std::bind(&ReceiveContextUnaryStreamImpl::ReceiveTransmit,
                            this, std::placeholders::_1);
    stream_->Read(request_.get(), static_cast<void *>(this));
    return true;
  }

 private:
  std::unique_ptr<ServerContextType> server_context_;
  std::unique_ptr<StreamType> stream_;
  std::unique_ptr<RequestType> request_;
  std::unique_ptr<ResponseType> response_;
  std::function<bool(bool)> next_state_;
  std::function<grpc::Status(RequestType *, ResponseType *)> receive_transmit_;
  std::function<void(ServerContextType *, StreamType *, void *)>
      request_connect_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_RECEIVER_RECEIVER_CONTEXT_H_
