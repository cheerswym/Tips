#ifndef ONBOARD_LITE_TRANSPORT_SHM_SHM_MANAGER_H_
#define ONBOARD_LITE_TRANSPORT_SHM_SHM_MANAGER_H_

#include <sys/types.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>

#include "boost/functional/hash.hpp"
#include "boost/interprocess/allocators/allocator.hpp"
#include "boost/interprocess/managed_shared_memory.hpp"
#include "boost/interprocess/sync/named_mutex.hpp"
#include "boost/unordered_map.hpp"
#include "onboard/base/base.h"
#include "onboard/global/singleton.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/lite/transport/shm/shm_flags.h"

namespace qcraft {
namespace shm {
namespace bip = boost::interprocess;

// 2000 massages per second, 20 seconds a circle.
const uint32_t kNotifierSlotSize = 20 * 2000;

struct ReadableInfo {
  bip::managed_shared_memory::handle_t handle;
  size_t header_size;
  size_t msg_size;
  size_t domain_channel_hash;
};

struct Indicator {
  std::atomic<uint64_t> next_write_seq = {0};
  std::atomic<uint64_t> seqs[kNotifierSlotSize];
  ReadableInfo infos[kNotifierSlotSize];
};

// Ref Count Map that can live in shm.
typedef bip::managed_shared_memory::handle_t RefCountKeyType;
typedef int RefCountValueType;
typedef std::pair<const RefCountKeyType, RefCountValueType> RefCountItemType;
typedef bip::allocator<RefCountItemType,
                       bip::managed_shared_memory::segment_manager>
    RefCountShmAllocator;
typedef boost::unordered_map<
    RefCountKeyType, RefCountValueType, boost::hash<RefCountKeyType>,
    std::equal_to<RefCountKeyType>, RefCountShmAllocator>
    RefCountMap;

// Channel Count Map that can live in shm.
typedef int64 ChannelCountKeyType;
typedef int ChannelCountValueType;
typedef std::pair<const ChannelCountKeyType, ChannelCountValueType>
    ChannelCountItemType;
typedef bip::allocator<ChannelCountItemType,
                       bip::managed_shared_memory::segment_manager>
    ChannelCountShmAllocator;
typedef boost::unordered_map<ChannelCountKeyType, ChannelCountValueType,
                             boost::hash<ChannelCountKeyType>,
                             std::equal_to<ChannelCountKeyType>,
                             ChannelCountShmAllocator>
    ChannelCountMap;

// Subscriber Count Map that can live in shm.
typedef size_t SubCountKeyType;
typedef int SubCountValueType;
typedef std::pair<const SubCountKeyType, SubCountValueType> SubCountItemType;
typedef bip::allocator<SubCountItemType,
                       bip::managed_shared_memory::segment_manager>
    SubCountAllocator;
typedef boost::unordered_map<SubCountKeyType, SubCountValueType,
                             boost::hash<SubCountKeyType>,
                             std::equal_to<SubCountKeyType>, SubCountAllocator>
    SubCountMap;

typedef int IndexRefCountValueType;
typedef bip::allocator<IndexRefCountValueType,
                       bip::managed_shared_memory::segment_manager>
    IndexRefCountAllocator;
typedef std::vector<IndexRefCountValueType, IndexRefCountAllocator>
    IndexRefCountVector;

class ShmManager {
 public:
  Indicator* CreateNotifierIndicator();

  void DestroyAndCreateShm();

  void AttachShm();

  void DestroyShm();

  void ShowDebug();

  void* Allocate(std::size_t size);

  void Deallocate(void* buffer);

  int GetRefCount(bip::managed_shared_memory::handle_t handle);
  int GetIndexRefCount(int index);

  void IncreaseRefCount(bip::managed_shared_memory::handle_t handle, int n);
  void IncreaseIndexRefCount(int index, int n);

  void DecreaseRefCount(bip::managed_shared_memory::handle_t handle, int n);
  void GarbageCollection(int index);
  void DecreaseIndexRefCountOfSameHandle(
      const ShmMessageMetadata& shm_msg_metadata, int n);
  void DecreaseIndexRefCount(int index, int n);

  int64_t Subscribe(const std::string& domain_channel);

  void UnSubscribe(const std::string& domain_channel, int64_t number);

  int GetSubscriberNum(const std::string& domain_channel);

  void* GetAddressFromHandle(bip::managed_shared_memory::handle_t handle);

  bip::managed_shared_memory::handle_t GetHandleFromAddress(void*);

  size_t GetFreeMemory();

  size_t GetUsedMemory();

  size_t GetMaxMemory();

 private:
  std::unique_ptr<bip::managed_shared_memory> segment_;

  IndexRefCountVector* index_ref_count_vector_;

  RefCountMap* ref_count_map_;

  SubCountMap* sub_count_map_;

  ChannelCountMap* channel_count_map_;

  Indicator* indicator_;

  std::unique_ptr<bip::named_mutex> index_ref_count_mutex_;

  std::unique_ptr<bip::named_mutex> ref_count_mutex_;

  std::unique_ptr<bip::named_mutex> sub_count_mutex_;

  std::unique_ptr<bip::named_mutex> channel_count_mutex_;

  std::hash<std::string> str_hash_;

  DECLARE_SINGLETON(ShmManager);
};
}  // namespace shm
}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_SHM_SHM_MANAGER_H_
