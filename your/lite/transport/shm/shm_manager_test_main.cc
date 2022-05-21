#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_map>

#include "boost/process.hpp"
#include "glog/logging.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/lite/transport/shm/condition_notifier.h"
#include "onboard/lite/transport/shm/shm_manager.h"

namespace qcraft {
void SignalHandler(int signum, siginfo_t *siginfo, void *context) {
  LOG(INFO) << "recevied signal";
  exit(0);
}

void RunReader(const std::string &name, const int subject) {
  LOG(INFO) << "RunReader :" << name;
  shm::ShmManager::Instance()->AttachShm();
  auto notifier = std::make_unique<shm::ConditionNotifier>();
  shm::ReadableInfo readable_info;
  int index;
  while (true) {
    if (!notifier->Read(100, &readable_info, &index)) {
      continue;
    }
    if (readable_info.domain_channel_hash != subject) {
      continue;
    }
    shm::ShmManager::Instance()->GetAddressFromHandle(readable_info.handle);
    shm::ShmManager::Instance()->DecreaseRefCount(readable_info.handle, 1);
    shm::ShmManager::Instance()->DecreaseIndexRefCount(index, 1);
    LOG(INFO) << "readable_info.handle:" << readable_info.handle << " REF:"
              << shm::ShmManager::Instance()->GetRefCount(readable_info.handle)
              << " index:" << index << " ref:"
              << shm::ShmManager::Instance()->GetIndexRefCount(index);
  }
}

void RunWriter(const std::string &name, const int subject, const int refcount) {
  LOG(INFO) << "RunWriter :" << name;
  shm::ShmManager::Instance()->AttachShm();
  auto notifier = std::make_unique<shm::ConditionNotifier>();
  for (int i = 0; i < 1000; ++i) {
    auto address = qcraft::shm::ShmManager::Instance()->Allocate(100);
    auto handle =
        qcraft::shm::ShmManager::Instance()->GetHandleFromAddress(address);
    qcraft::shm::ShmManager::Instance()->IncreaseRefCount(handle, 2);
    size_t header_size = 0;
    size_t msg_size = 0;
    size_t domain_channel_hash = subject;

    shm::ReadableInfo readable_info{handle, header_size, msg_size,
                                    domain_channel_hash};
    LOG(INFO) << "Writer write handle:" << readable_info.handle;

    uint64_t current_write_seq, current_write_index;
    CHECK(notifier->GetWriteIndex(&current_write_seq, &current_write_index));
    CHECK(notifier->Write(readable_info, 2, current_write_seq,
                          current_write_index));
    usleep(10000);
  }
}
}  // namespace qcraft

int main(int argc, char *argv[]) {
  qcraft::InitQCraft(&argc, &argv);

  // init shm
  qcraft::shm::ShmManager::Instance()->DestroyAndCreateShm();
  size_t mem_before_start =
      qcraft::shm::ShmManager::Instance()->GetFreeMemory();
  int size = 30;
  std::vector<pid_t> pids;
  pids.reserve(size);

  for (int i = 0; i < size; ++i) {
    if ((pids[i] = fork()) < 0) {
      perror("fork");
      abort();
    } else if (pids[i] == 0) {
      // in child process
      if (i % 3 == 0) {
        qcraft::RunWriter(std::to_string(i), i / 3, 2);
      } else {
        qcraft::RunReader(std::to_string(i), i / 3);
      }
      exit(0);
    }
  }
  LOG(INFO) << "PARENT process say hello";
  sleep(20);
  LOG(INFO) << "kill all";
  for (int i = 0; i < size; ++i) {
    kill(pids[i], SIGKILL);
  }
  /* Wait for children to exit. */
  int status;
  for (int i = 0; i < size; ++i) {
    int pid = wait(&status);
    printf("Child with PID %ld exited with status 0x%x.\n", (long)pid, status);
  }
  for (int i = 0; i < size / 3; ++i) {
    LOG(INFO) << i << ":"
              << qcraft::shm::ShmManager::Instance()->GetRefCount(i);
  }
  size_t mem_end = qcraft::shm::ShmManager::Instance()->GetFreeMemory();
  LOG(INFO) << "memory before:" << mem_before_start;
  LOG(INFO) << "memory  after:" << mem_end;
}