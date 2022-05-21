#include "onboard/lite/transport/shm/shm_manager.h"

#include <functional>

#include "boost/interprocess/sync/scoped_lock.hpp"
#include "glog/logging.h"
#include "onboard/global/clock.h"
#include "onboard/global/counter.h"
#include "onboard/lite/transport/shm/lite_shm_header.h"
#include "onboard/lite/transport/shm/shm_debug_util.h"
#include "onboard/lite/transport/shm/shm_flags.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace shm {
namespace {
const char kShmSegmentName[] = "segments";
const char kNotifierIndicatorName[] = "notifier_indicator";
const char kIndexRefCountName[] = "shm_index_ref_count";
const char kRefCountMapName[] = "shm_ref_count_map";
const char kSubCountMapName[] = "shm_sub_count_map";
const char kChannelCountMapName[] = "shm_channel_count_map";
const char kIndexRefCountMutexName[] = "shm_index_count_mutex";
const char kRefcountMapMutexName[] = "shm_ref_count_map_mutex";
const char kSubcountMapMutexName[] = "shm_sub_count_map_mutex";
const char kChannelcountMapMutexName[] = "shm_channel_count_map_mutex";
const int32 kShmTagNumber = 6;

template <typename K, typename V>
void print_map(const std::string& map_name, std::map<K, V> const& m) {
  std::cout << "-----------" << map_name << " ------------\n";
  int counter = 0;
  std::map<std::string, V> temp;
  for (auto const& pair : m) {
    counter += pair.second;
    temp[DebugUtil::Instance()->TryFindChannelNameWithHash(pair.first)] =
        pair.second;
  }
  for (auto const& pair : temp) {
    std::cout << "{" << pair.first << ": " << pair.second << "}\n";
  }
  std::cout << map_name << " key num:" << m.size() << " total:" << counter
            << "\n";
  std::cout << "-----------------------\n";
}

std::string GetShmName(const std::string& name) {
  return FLAGS_transport_managed_shared_memory_name + "_" + name;
}

}  // namespace

ShmManager::ShmManager() {}

void ShmManager::DestroyAndCreateShm() {
  LOG(WARNING) << "Entering DestroyAndCreateShm()";
  // destroy
  DestroyShm();

  // build
  index_ref_count_mutex_ = std::make_unique<bip::named_mutex>(
      bip::open_or_create, GetShmName(kIndexRefCountMutexName).c_str());

  ref_count_mutex_ = std::make_unique<bip::named_mutex>(
      bip::open_or_create, GetShmName(kRefcountMapMutexName).c_str());
  sub_count_mutex_ = std::make_unique<bip::named_mutex>(
      bip::open_or_create, GetShmName(kSubcountMapMutexName).c_str());
  channel_count_mutex_ = std::make_unique<bip::named_mutex>(
      bip::open_or_create, GetShmName(kChannelcountMapMutexName).c_str());

  segment_ = std::make_unique<bip::managed_shared_memory>(
      bip::create_only, GetShmName(kShmSegmentName).c_str(),
      FLAGS_transport_managed_shared_memory_size);

  // construct and init the indicator
  auto* indicator = segment_->construct<Indicator>(
      GetShmName(kNotifierIndicatorName).c_str())();
  indicator->next_write_seq = 1;
  for (int i = 0; i < kNotifierSlotSize; ++i) {
    indicator->seqs[i].store(0);
  }

  {
    bip::scoped_lock<bip::named_mutex> lock(*index_ref_count_mutex_);
    index_ref_count_vector_ = segment_->construct<IndexRefCountVector>(
        GetShmName(kIndexRefCountName).c_str())(
        segment_->get_allocator<IndexRefCountVector>());
    if (index_ref_count_vector_->size() == 0) {
      index_ref_count_vector_->resize(kNotifierSlotSize);
    }
  }
  ref_count_map_ =
      segment_->construct<RefCountMap>(GetShmName(kRefCountMapName).c_str())(
          100, boost::hash<RefCountKeyType>(), std::equal_to<RefCountKeyType>(),
          segment_->get_allocator<RefCountItemType>());

  sub_count_map_ =
      segment_->construct<SubCountMap>(GetShmName(kSubCountMapName).c_str())(
          100, boost::hash<SubCountKeyType>(), std::equal_to<SubCountKeyType>(),
          segment_->get_allocator<SubCountItemType>());

  channel_count_map_ = segment_->construct<ChannelCountMap>(
      GetShmName(kChannelCountMapName).c_str())(
      100, boost::hash<ChannelCountKeyType>(),
      std::equal_to<ChannelCountKeyType>(),
      segment_->get_allocator<ChannelCountItemType>());
}

void ShmManager::AttachShm() {
  VLOG(3) << "AttachShm";

  // build
  index_ref_count_mutex_ = std::make_unique<bip::named_mutex>(
      bip::open_or_create, GetShmName(kIndexRefCountMutexName).c_str());

  ref_count_mutex_ = std::make_unique<bip::named_mutex>(
      bip::open_or_create, GetShmName(kRefcountMapMutexName).c_str());
  sub_count_mutex_ = std::make_unique<bip::named_mutex>(
      bip::open_or_create, GetShmName(kSubcountMapMutexName).c_str());
  channel_count_mutex_ = std::make_unique<bip::named_mutex>(
      bip::open_or_create, GetShmName(kChannelcountMapMutexName).c_str());

  segment_ = std::make_unique<bip::managed_shared_memory>(
      bip::open_or_create, GetShmName(kShmSegmentName).c_str(),
      FLAGS_transport_managed_shared_memory_size);

  {
    bip::scoped_lock<bip::named_mutex> lock(*index_ref_count_mutex_);
    index_ref_count_vector_ = segment_->find_or_construct<IndexRefCountVector>(
        GetShmName(kIndexRefCountName).c_str())(
        segment_->get_allocator<IndexRefCountVector>());
    if (index_ref_count_vector_->size() == 0) {
      index_ref_count_vector_->resize(kNotifierSlotSize);
    }
  }

  ref_count_map_ = segment_->find_or_construct<RefCountMap>(
      GetShmName(kRefCountMapName).c_str())(
      100, boost::hash<RefCountKeyType>(), std::equal_to<RefCountKeyType>(),
      segment_->get_allocator<RefCountItemType>());

  sub_count_map_ = segment_->find_or_construct<SubCountMap>(
      GetShmName(kSubCountMapName).c_str())(
      100, boost::hash<SubCountKeyType>(), std::equal_to<SubCountKeyType>(),
      segment_->get_allocator<SubCountItemType>());

  channel_count_map_ = segment_->find_or_construct<ChannelCountMap>(
      GetShmName(kChannelCountMapName).c_str())(
      100, boost::hash<ChannelCountKeyType>(),
      std::equal_to<ChannelCountKeyType>(),
      segment_->get_allocator<ChannelCountItemType>());
}

Indicator* ShmManager::CreateNotifierIndicator() {
  if (!segment_) {
    segment_ = std::make_unique<bip::managed_shared_memory>(
        bip::open_only, GetShmName(kShmSegmentName).c_str());
  }
  indicator_ = segment_->find_or_construct<Indicator>(
      GetShmName(kNotifierIndicatorName).c_str())();

  VLOG(3) << "sizeof(Indicator)" << sizeof(Indicator);
  VLOG(3) << "managed_shm.get_num_named_objects: "
          << segment_->get_num_named_objects();
  return indicator_;
}

int ShmManager::GetIndexRefCount(int index) {
  {
    bip::scoped_lock<bip::named_mutex> lock(*index_ref_count_mutex_);
    return (*index_ref_count_vector_)[index];
  }
}

int ShmManager::GetRefCount(bip::managed_shared_memory::handle_t handle) {
  {
    bip::scoped_lock<bip::named_mutex> lock(*ref_count_mutex_);
    if (ref_count_map_->find(handle) != ref_count_map_->end()) {
      return ref_count_map_->at(handle);
    }
    return 0;
  }
}

void ShmManager::IncreaseIndexRefCount(int index, int n) {
  if (n == 0) {
    return;
  }

  {
    bip::scoped_lock<bip::named_mutex> lock(*index_ref_count_mutex_);
    (*index_ref_count_vector_)[index] += n;
  }
}

void ShmManager::GarbageCollection(int index) {
  QCOUNTER_SPAN("GarbageCollection");
  int count;
  {
    bip::scoped_lock<bip::named_mutex> lock(*index_ref_count_mutex_);
    count = (*index_ref_count_vector_)[index];
    if (LIKELY(count == 0)) {
      return;
    }
    (*index_ref_count_vector_)[index] -= count;
  }

  size_t before = GetFreeMemory();
  ReadableInfo& old_info = indicator_->infos[index];
  int ref_cnt = GetRefCount(old_info.handle);
  void* addr = GetAddressFromHandle(old_info.handle);
  shm::LiteShmHeader lite_shm_header;

  CHECK(lite_shm_header.DeserializeFrom(reinterpret_cast<char*>(addr)))
      << "Failed to Deserialize header";
  if (lite_shm_header.tag_number() == kShmTagNumber) {
    ShmMessageMetadata msgmeta;
    msgmeta.ParseFromArray(reinterpret_cast<char*>(addr) + old_info.header_size,
                           old_info.msg_size);

    int msgmeta_ref_cnt = GetRefCount(msgmeta.cross_proc().handle());
    LOG(ERROR) << " handle:" << old_info.handle << " domain_channel"
               << old_info.domain_channel_hash << " ref_cnt: " << ref_cnt
               << " msgmeta handle():" << msgmeta.cross_proc().handle()
               << " msgmeta_ref_cnt: " << msgmeta_ref_cnt << " index :" << index
               << " count: " << count;
    // Some msg such as partial_spin will alloc once but send multiple times,
    // the reference count will increase when sending. So i need to subtract
    // the reference count of the index instead of the reference count of the
    // shm handle itself during garbage collection.
    DecreaseRefCount(msgmeta.cross_proc().handle(), count);
  }
  DecreaseRefCount(old_info.handle, ref_cnt);
  QCOUNTER("collect_memory_byte", GetFreeMemory() - before);
}

void ShmManager::DecreaseIndexRefCountOfSameHandle(
    const ShmMessageMetadata& shm_msg_metadata, int n) {
  if (n == 0) {
    return;
  }
  int count;
  int index;
  {
    bip::scoped_lock<bip::named_mutex> lock(*index_ref_count_mutex_);
    index = shm_msg_metadata.cross_proc().index();
    count = n <= (*index_ref_count_vector_)[index]
                ? n
                : (*index_ref_count_vector_)[index];

    // Judge index ref count first and then handle
    if ((count != 0) && (shm_msg_metadata.cross_proc().msg_handle() ==
                         indicator_->infos[index].handle)) {
      (*index_ref_count_vector_)[index] -= count;
    } else {
      return;
    }
  }

  VLOG(1) << "count:" << count << " index:" << index
          << " index ref count:" << GetIndexRefCount(index)
          << " handle():" << shm_msg_metadata.cross_proc().msg_handle()
          << " ref_cnt: "
          << GetRefCount(shm_msg_metadata.cross_proc().msg_handle())
          << " msgmeta handle:" << shm_msg_metadata.cross_proc().handle()
          << "msgmeta ref_cnt: "
          << GetRefCount(shm_msg_metadata.cross_proc().handle());

  DecreaseRefCount(shm_msg_metadata.cross_proc().handle(), n);
  DecreaseRefCount(shm_msg_metadata.cross_proc().msg_handle(), n);
}

void ShmManager::DecreaseIndexRefCount(int index, int n) {
  if (n == 0) {
    return;
  }

  {
    bip::scoped_lock<bip::named_mutex> lock(*index_ref_count_mutex_);
    (*index_ref_count_vector_)[index] -= n;
  }
}

void ShmManager::IncreaseRefCount(bip::managed_shared_memory::handle_t handle,
                                  int n) {
  QCOUNTER_SPAN("IncreaseRefCount");
  if (n == 0) {
    return;
  }

  VLOG(1) << "IncreaseRefCount";
  int count;
  {
    bip::scoped_lock<bip::named_mutex> lock(*ref_count_mutex_);
    (*ref_count_map_)[handle] += n;
    count = ref_count_map_->at(handle);
  }

  VLOG(1) << "IncreaseRefCount:AFTER +" << n << " key:" << handle
          << " refcount:" << count;
}

void ShmManager::DecreaseRefCount(bip::managed_shared_memory::handle_t handle,
                                  int n) {
  QCOUNTER_SPAN("DecreaseRefCount");
  if (n == 0) {
    return;
  }
  int count;

  VLOG(1) << "DecreaseRefCount";
  {
    bip::scoped_lock<bip::named_mutex> lock(*ref_count_mutex_);
    count = n <= (*ref_count_map_)[handle] ? n : (*ref_count_map_)[handle];
    if (count == 0) {
      return;
    }
    (*ref_count_map_)[handle] -= count;
    count = ref_count_map_->at(handle);

    VLOG(1) << "DecreaseRefCount:AFTER -" << n << " key:" << handle
            << " refcount:" << count;

    if (count == 0) {
      VLOG(1) << "TRY DEALLOCATE:" << handle;

      Deallocate(segment_->get_address_from_handle(handle));
    }
    CHECK_GE(count, 0);
  }
}

int64_t ShmManager::Subscribe(const std::string& domain_channel) {
  int count;
  int64_t timestamp = 0;
  {
    bip::scoped_lock<bip::named_mutex> lock(*sub_count_mutex_);
    timestamp = absl::ToUnixMicros(Clock::Now());
    size_t hash = str_hash_(domain_channel);
    (*sub_count_map_)[hash] += 1;
    count = sub_count_map_->at(hash);
  }
  LOG(INFO) << "Subscribe:" << domain_channel << " sub_count:" << count;
  return timestamp;
}

void ShmManager::UnSubscribe(const std::string& domain_channel,
                             int64_t number) {
  int count;
  {
    bip::scoped_lock<bip::named_mutex> lock(*sub_count_mutex_);
    size_t hash = str_hash_(domain_channel);
    (*sub_count_map_)[hash] -= number;
    count = sub_count_map_->at(hash);
  }

  LOG(INFO) << "UnSubscribe:" << domain_channel << " sub_count:" << count;
}

int ShmManager::GetSubscriberNum(const std::string& domain_channel) {
  int num;
  {
    bip::scoped_lock<bip::named_mutex> lock(*sub_count_mutex_);
    size_t domain_channel_hash = str_hash_(domain_channel);

    if (sub_count_map_->count(domain_channel_hash)) {
      num = (*sub_count_map_)[domain_channel_hash];
    } else {
      num = 0;
    }
  }

  VLOG(3) << "GetSubscribeNum:" << domain_channel << " num:" << num;
  return num;
}

void ShmManager::DestroyShm() {
  VLOG(3) << "DestroyShm";
  segment_.reset();
  bip::shared_memory_object::remove(GetShmName(kShmSegmentName).c_str());
  bip::named_mutex::remove(GetShmName(kIndexRefCountMutexName).c_str());
  bip::named_mutex::remove(GetShmName(kRefcountMapMutexName).c_str());
  bip::named_mutex::remove(GetShmName(kSubcountMapMutexName).c_str());
  bip::named_mutex::remove(GetShmName(kChannelcountMapMutexName).c_str());
}

bip::managed_shared_memory::handle_t ShmManager::GetHandleFromAddress(
    void* buffer) {
  QCOUNTER_SPAN("get_handle_from_address");
  if (!segment_) {
    segment_ = std::make_unique<bip::managed_shared_memory>(
        bip::open_only, GetShmName(kShmSegmentName).c_str());
  }
  return segment_->get_handle_from_address(buffer);
}

void* ShmManager::GetAddressFromHandle(
    bip::managed_shared_memory::handle_t handle) {
  if (!segment_) {
    segment_ = std::make_unique<bip::managed_shared_memory>(
        bip::open_only, GetShmName(kShmSegmentName).c_str());
  }
  QCOUNTER_SPAN("GetAddressFromHandle");
  return segment_->get_address_from_handle(handle);
}

void* ShmManager::Allocate(std::size_t size) {
  QCOUNTER_SPAN("shm_allocate");
  if (!segment_) {
    segment_ = std::make_unique<bip::managed_shared_memory>(
        bip::open_only, GetShmName(kShmSegmentName).c_str());
  }

  VLOG(3) << "Allocate:" << size
          << " free mem Before:" << segment_->get_free_memory();
  void* data = segment_->allocate(size, std::nothrow);
  bip::managed_shared_memory::handle_t handle = GetHandleFromAddress(data);

  VLOG(3) << "Allocate:" << size << " , handle:" << handle
          << " free mem After:" << segment_->get_free_memory();
  return data;
}

void ShmManager::Deallocate(void* buffer) {
  QCOUNTER_SPAN("shm_deallocate");
  VLOG(3) << "Deallocate";
  const size_t before = segment_->get_free_memory();
  segment_->deallocate(buffer);
  const size_t after = segment_->get_free_memory();

  VLOG(3) << "Deallocate, free mem after:" << after
          << " freed:" << after - before;
}

void ShmManager::ShowDebug() {
  std::map<RefCountKeyType, RefCountValueType> ref_map;
  std::map<SubCountKeyType, SubCountValueType> sub_map;
  std::map<ChannelCountKeyType, ChannelCountValueType> channel_map;
  {
    bip::scoped_lock<bip::named_mutex> lock(*ref_count_mutex_);
    for (const auto item : *ref_count_map_) {
      ref_map.emplace(item);
    }
  }
  {
    bip::scoped_lock<bip::named_mutex> lock(*sub_count_mutex_);
    for (const auto item : *sub_count_map_) {
      sub_map.emplace(item);
    }
  }
  {
    bip::scoped_lock<bip::named_mutex> lock(*channel_count_mutex_);
    for (const auto item : *channel_count_map_) {
      channel_map.emplace(item);
    }
  }
  print_map("sub_map", sub_map);
  print_map("ref_map", ref_map);
  print_map("channel_map", channel_map);
}

size_t ShmManager::GetFreeMemory() { return segment_->get_free_memory(); }

size_t ShmManager::GetUsedMemory() {
  return segment_->get_size() - segment_->get_free_memory();
}

size_t ShmManager::GetMaxMemory() {
  return FLAGS_transport_managed_shared_memory_size;
}

}  // namespace shm
}  // namespace qcraft
