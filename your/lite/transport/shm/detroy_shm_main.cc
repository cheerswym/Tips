#include "onboard/global/ftrace.h"
#include "onboard/lite/transport/shm/shm_manager.h"

int main(int argc, char *argv[]) {
  qcraft::shm::ShmManager::Instance()->DestroyShm();
  qcraft::FTrace::Instance()->Uninit();
  return 0;
}