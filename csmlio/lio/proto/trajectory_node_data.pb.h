// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/trajectory_node_data.proto

#ifndef PROTOBUF_cartographer_2fmapping_2fproto_2ftrajectory_5fnode_5fdata_2eproto__INCLUDED
#define PROTOBUF_cartographer_2fmapping_2fproto_2ftrajectory_5fnode_5fdata_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3004000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "csmlio/sensor/proto/sensor.pb.h"
#include "csmlio/transform/proto/transform.pb.h"
// @@protoc_insertion_point(includes)
namespace csmlio {
namespace mapping {
namespace proto {
class TrajectoryNodeData;
class TrajectoryNodeDataDefaultTypeInternal;
extern TrajectoryNodeDataDefaultTypeInternal _TrajectoryNodeData_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace csmlio

namespace csmlio {
namespace mapping {
namespace proto {

namespace protobuf_cartographer_2fmapping_2fproto_2ftrajectory_5fnode_5fdata_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static void InitDefaultsImpl();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_cartographer_2fmapping_2fproto_2ftrajectory_5fnode_5fdata_2eproto

// ===================================================================

class TrajectoryNodeData : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.proto.TrajectoryNodeData) */ {
 public:
  TrajectoryNodeData();
  virtual ~TrajectoryNodeData();

  TrajectoryNodeData(const TrajectoryNodeData& from);

  inline TrajectoryNodeData& operator=(const TrajectoryNodeData& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  TrajectoryNodeData(TrajectoryNodeData&& from) noexcept
    : TrajectoryNodeData() {
    *this = ::std::move(from);
  }

  inline TrajectoryNodeData& operator=(TrajectoryNodeData&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const TrajectoryNodeData& default_instance();

  static inline const TrajectoryNodeData* internal_default_instance() {
    return reinterpret_cast<const TrajectoryNodeData*>(
               &_TrajectoryNodeData_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(TrajectoryNodeData* other);
  friend void swap(TrajectoryNodeData& a, TrajectoryNodeData& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline TrajectoryNodeData* New() const PROTOBUF_FINAL { return New(NULL); }

  TrajectoryNodeData* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const TrajectoryNodeData& from);
  void MergeFrom(const TrajectoryNodeData& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(TrajectoryNodeData* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated float rotational_scan_matcher_histogram = 6;
  int rotational_scan_matcher_histogram_size() const;
  void clear_rotational_scan_matcher_histogram();
  static const int kRotationalScanMatcherHistogramFieldNumber = 6;
  float rotational_scan_matcher_histogram(int index) const;
  void set_rotational_scan_matcher_histogram(int index, float value);
  void add_rotational_scan_matcher_histogram(float value);
  const ::google::protobuf::RepeatedField< float >&
      rotational_scan_matcher_histogram() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_rotational_scan_matcher_histogram();

  // .cartographer.transform.proto.Quaterniond gravity_alignment = 2;
  bool has_gravity_alignment() const;
  void clear_gravity_alignment();
  static const int kGravityAlignmentFieldNumber = 2;
  const ::csmlio::transform::proto::Quaterniond& gravity_alignment() const;
  ::csmlio::transform::proto::Quaterniond* mutable_gravity_alignment();
  ::csmlio::transform::proto::Quaterniond* release_gravity_alignment();
  void set_allocated_gravity_alignment(::csmlio::transform::proto::Quaterniond* gravity_alignment);

  // .cartographer.sensor.proto.CompressedPointCloud filtered_gravity_aligned_point_cloud = 3;
  bool has_filtered_gravity_aligned_point_cloud() const;
  void clear_filtered_gravity_aligned_point_cloud();
  static const int kFilteredGravityAlignedPointCloudFieldNumber = 3;
  const ::csmlio::sensor::proto::CompressedPointCloud& filtered_gravity_aligned_point_cloud() const;
  ::csmlio::sensor::proto::CompressedPointCloud* mutable_filtered_gravity_aligned_point_cloud();
  ::csmlio::sensor::proto::CompressedPointCloud* release_filtered_gravity_aligned_point_cloud();
  void set_allocated_filtered_gravity_aligned_point_cloud(::csmlio::sensor::proto::CompressedPointCloud* filtered_gravity_aligned_point_cloud);

  // .cartographer.sensor.proto.CompressedPointCloud high_resolution_point_cloud = 4;
  bool has_high_resolution_point_cloud() const;
  void clear_high_resolution_point_cloud();
  static const int kHighResolutionPointCloudFieldNumber = 4;
  const ::csmlio::sensor::proto::CompressedPointCloud& high_resolution_point_cloud() const;
  ::csmlio::sensor::proto::CompressedPointCloud* mutable_high_resolution_point_cloud();
  ::csmlio::sensor::proto::CompressedPointCloud* release_high_resolution_point_cloud();
  void set_allocated_high_resolution_point_cloud(::csmlio::sensor::proto::CompressedPointCloud* high_resolution_point_cloud);

  // .cartographer.sensor.proto.CompressedPointCloud low_resolution_point_cloud = 5;
  bool has_low_resolution_point_cloud() const;
  void clear_low_resolution_point_cloud();
  static const int kLowResolutionPointCloudFieldNumber = 5;
  const ::csmlio::sensor::proto::CompressedPointCloud& low_resolution_point_cloud() const;
  ::csmlio::sensor::proto::CompressedPointCloud* mutable_low_resolution_point_cloud();
  ::csmlio::sensor::proto::CompressedPointCloud* release_low_resolution_point_cloud();
  void set_allocated_low_resolution_point_cloud(::csmlio::sensor::proto::CompressedPointCloud* low_resolution_point_cloud);

  // .cartographer.transform.proto.Rigid3d local_pose = 7;
  bool has_local_pose() const;
  void clear_local_pose();
  static const int kLocalPoseFieldNumber = 7;
  const ::csmlio::transform::proto::Rigid3d& local_pose() const;
  ::csmlio::transform::proto::Rigid3d* mutable_local_pose();
  ::csmlio::transform::proto::Rigid3d* release_local_pose();
  void set_allocated_local_pose(::csmlio::transform::proto::Rigid3d* local_pose);

  // int64 timestamp = 1;
  void clear_timestamp();
  static const int kTimestampFieldNumber = 1;
  ::google::protobuf::int64 timestamp() const;
  void set_timestamp(::google::protobuf::int64 value);

  // @@protoc_insertion_point(class_scope:cartographer.mapping.proto.TrajectoryNodeData)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedField< float > rotational_scan_matcher_histogram_;
  mutable int _rotational_scan_matcher_histogram_cached_byte_size_;
  ::csmlio::transform::proto::Quaterniond* gravity_alignment_;
  ::csmlio::sensor::proto::CompressedPointCloud* filtered_gravity_aligned_point_cloud_;
  ::csmlio::sensor::proto::CompressedPointCloud* high_resolution_point_cloud_;
  ::csmlio::sensor::proto::CompressedPointCloud* low_resolution_point_cloud_;
  ::csmlio::transform::proto::Rigid3d* local_pose_;
  ::google::protobuf::int64 timestamp_;
  mutable int _cached_size_;
  friend struct protobuf_cartographer_2fmapping_2fproto_2ftrajectory_5fnode_5fdata_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TrajectoryNodeData

// int64 timestamp = 1;
inline void TrajectoryNodeData::clear_timestamp() {
  timestamp_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 TrajectoryNodeData::timestamp() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.TrajectoryNodeData.timestamp)
  return timestamp_;
}
inline void TrajectoryNodeData::set_timestamp(::google::protobuf::int64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.TrajectoryNodeData.timestamp)
}

// .cartographer.transform.proto.Quaterniond gravity_alignment = 2;
inline bool TrajectoryNodeData::has_gravity_alignment() const {
  return this != internal_default_instance() && gravity_alignment_ != NULL;
}
inline void TrajectoryNodeData::clear_gravity_alignment() {
  if (GetArenaNoVirtual() == NULL && gravity_alignment_ != NULL) delete gravity_alignment_;
  gravity_alignment_ = NULL;
}
inline const ::csmlio::transform::proto::Quaterniond& TrajectoryNodeData::gravity_alignment() const {
  const ::csmlio::transform::proto::Quaterniond* p = gravity_alignment_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.TrajectoryNodeData.gravity_alignment)
  return p != NULL ? *p : *reinterpret_cast<const ::csmlio::transform::proto::Quaterniond*>(
      &::csmlio::transform::proto::_Quaterniond_default_instance_);
}
inline ::csmlio::transform::proto::Quaterniond* TrajectoryNodeData::mutable_gravity_alignment() {
  
  if (gravity_alignment_ == NULL) {
    gravity_alignment_ = new ::csmlio::transform::proto::Quaterniond;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.TrajectoryNodeData.gravity_alignment)
  return gravity_alignment_;
}
inline ::csmlio::transform::proto::Quaterniond* TrajectoryNodeData::release_gravity_alignment() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.TrajectoryNodeData.gravity_alignment)
  
  ::csmlio::transform::proto::Quaterniond* temp = gravity_alignment_;
  gravity_alignment_ = NULL;
  return temp;
}
inline void TrajectoryNodeData::set_allocated_gravity_alignment(::csmlio::transform::proto::Quaterniond* gravity_alignment) {
  delete gravity_alignment_;
  gravity_alignment_ = gravity_alignment;
  if (gravity_alignment) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.TrajectoryNodeData.gravity_alignment)
}

// .cartographer.sensor.proto.CompressedPointCloud filtered_gravity_aligned_point_cloud = 3;
inline bool TrajectoryNodeData::has_filtered_gravity_aligned_point_cloud() const {
  return this != internal_default_instance() && filtered_gravity_aligned_point_cloud_ != NULL;
}
inline void TrajectoryNodeData::clear_filtered_gravity_aligned_point_cloud() {
  if (GetArenaNoVirtual() == NULL && filtered_gravity_aligned_point_cloud_ != NULL) delete filtered_gravity_aligned_point_cloud_;
  filtered_gravity_aligned_point_cloud_ = NULL;
}
inline const ::csmlio::sensor::proto::CompressedPointCloud& TrajectoryNodeData::filtered_gravity_aligned_point_cloud() const {
  const ::csmlio::sensor::proto::CompressedPointCloud* p = filtered_gravity_aligned_point_cloud_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.TrajectoryNodeData.filtered_gravity_aligned_point_cloud)
  return p != NULL ? *p : *reinterpret_cast<const ::csmlio::sensor::proto::CompressedPointCloud*>(
      &::csmlio::sensor::proto::_CompressedPointCloud_default_instance_);
}
inline ::csmlio::sensor::proto::CompressedPointCloud* TrajectoryNodeData::mutable_filtered_gravity_aligned_point_cloud() {
  
  if (filtered_gravity_aligned_point_cloud_ == NULL) {
    filtered_gravity_aligned_point_cloud_ = new ::csmlio::sensor::proto::CompressedPointCloud;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.TrajectoryNodeData.filtered_gravity_aligned_point_cloud)
  return filtered_gravity_aligned_point_cloud_;
}
inline ::csmlio::sensor::proto::CompressedPointCloud* TrajectoryNodeData::release_filtered_gravity_aligned_point_cloud() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.TrajectoryNodeData.filtered_gravity_aligned_point_cloud)
  
  ::csmlio::sensor::proto::CompressedPointCloud* temp = filtered_gravity_aligned_point_cloud_;
  filtered_gravity_aligned_point_cloud_ = NULL;
  return temp;
}
inline void TrajectoryNodeData::set_allocated_filtered_gravity_aligned_point_cloud(::csmlio::sensor::proto::CompressedPointCloud* filtered_gravity_aligned_point_cloud) {
  delete filtered_gravity_aligned_point_cloud_;
  filtered_gravity_aligned_point_cloud_ = filtered_gravity_aligned_point_cloud;
  if (filtered_gravity_aligned_point_cloud) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.TrajectoryNodeData.filtered_gravity_aligned_point_cloud)
}

// .cartographer.sensor.proto.CompressedPointCloud high_resolution_point_cloud = 4;
inline bool TrajectoryNodeData::has_high_resolution_point_cloud() const {
  return this != internal_default_instance() && high_resolution_point_cloud_ != NULL;
}
inline void TrajectoryNodeData::clear_high_resolution_point_cloud() {
  if (GetArenaNoVirtual() == NULL && high_resolution_point_cloud_ != NULL) delete high_resolution_point_cloud_;
  high_resolution_point_cloud_ = NULL;
}
inline const ::csmlio::sensor::proto::CompressedPointCloud& TrajectoryNodeData::high_resolution_point_cloud() const {
  const ::csmlio::sensor::proto::CompressedPointCloud* p = high_resolution_point_cloud_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.TrajectoryNodeData.high_resolution_point_cloud)
  return p != NULL ? *p : *reinterpret_cast<const ::csmlio::sensor::proto::CompressedPointCloud*>(
      &::csmlio::sensor::proto::_CompressedPointCloud_default_instance_);
}
inline ::csmlio::sensor::proto::CompressedPointCloud* TrajectoryNodeData::mutable_high_resolution_point_cloud() {
  
  if (high_resolution_point_cloud_ == NULL) {
    high_resolution_point_cloud_ = new ::csmlio::sensor::proto::CompressedPointCloud;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.TrajectoryNodeData.high_resolution_point_cloud)
  return high_resolution_point_cloud_;
}
inline ::csmlio::sensor::proto::CompressedPointCloud* TrajectoryNodeData::release_high_resolution_point_cloud() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.TrajectoryNodeData.high_resolution_point_cloud)
  
  ::csmlio::sensor::proto::CompressedPointCloud* temp = high_resolution_point_cloud_;
  high_resolution_point_cloud_ = NULL;
  return temp;
}
inline void TrajectoryNodeData::set_allocated_high_resolution_point_cloud(::csmlio::sensor::proto::CompressedPointCloud* high_resolution_point_cloud) {
  delete high_resolution_point_cloud_;
  high_resolution_point_cloud_ = high_resolution_point_cloud;
  if (high_resolution_point_cloud) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.TrajectoryNodeData.high_resolution_point_cloud)
}

// .cartographer.sensor.proto.CompressedPointCloud low_resolution_point_cloud = 5;
inline bool TrajectoryNodeData::has_low_resolution_point_cloud() const {
  return this != internal_default_instance() && low_resolution_point_cloud_ != NULL;
}
inline void TrajectoryNodeData::clear_low_resolution_point_cloud() {
  if (GetArenaNoVirtual() == NULL && low_resolution_point_cloud_ != NULL) delete low_resolution_point_cloud_;
  low_resolution_point_cloud_ = NULL;
}
inline const ::csmlio::sensor::proto::CompressedPointCloud& TrajectoryNodeData::low_resolution_point_cloud() const {
  const ::csmlio::sensor::proto::CompressedPointCloud* p = low_resolution_point_cloud_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.TrajectoryNodeData.low_resolution_point_cloud)
  return p != NULL ? *p : *reinterpret_cast<const ::csmlio::sensor::proto::CompressedPointCloud*>(
      &::csmlio::sensor::proto::_CompressedPointCloud_default_instance_);
}
inline ::csmlio::sensor::proto::CompressedPointCloud* TrajectoryNodeData::mutable_low_resolution_point_cloud() {
  
  if (low_resolution_point_cloud_ == NULL) {
    low_resolution_point_cloud_ = new ::csmlio::sensor::proto::CompressedPointCloud;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.TrajectoryNodeData.low_resolution_point_cloud)
  return low_resolution_point_cloud_;
}
inline ::csmlio::sensor::proto::CompressedPointCloud* TrajectoryNodeData::release_low_resolution_point_cloud() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.TrajectoryNodeData.low_resolution_point_cloud)
  
  ::csmlio::sensor::proto::CompressedPointCloud* temp = low_resolution_point_cloud_;
  low_resolution_point_cloud_ = NULL;
  return temp;
}
inline void TrajectoryNodeData::set_allocated_low_resolution_point_cloud(::csmlio::sensor::proto::CompressedPointCloud* low_resolution_point_cloud) {
  delete low_resolution_point_cloud_;
  low_resolution_point_cloud_ = low_resolution_point_cloud;
  if (low_resolution_point_cloud) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.TrajectoryNodeData.low_resolution_point_cloud)
}

// repeated float rotational_scan_matcher_histogram = 6;
inline int TrajectoryNodeData::rotational_scan_matcher_histogram_size() const {
  return rotational_scan_matcher_histogram_.size();
}
inline void TrajectoryNodeData::clear_rotational_scan_matcher_histogram() {
  rotational_scan_matcher_histogram_.Clear();
}
inline float TrajectoryNodeData::rotational_scan_matcher_histogram(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.TrajectoryNodeData.rotational_scan_matcher_histogram)
  return rotational_scan_matcher_histogram_.Get(index);
}
inline void TrajectoryNodeData::set_rotational_scan_matcher_histogram(int index, float value) {
  rotational_scan_matcher_histogram_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.TrajectoryNodeData.rotational_scan_matcher_histogram)
}
inline void TrajectoryNodeData::add_rotational_scan_matcher_histogram(float value) {
  rotational_scan_matcher_histogram_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.TrajectoryNodeData.rotational_scan_matcher_histogram)
}
inline const ::google::protobuf::RepeatedField< float >&
TrajectoryNodeData::rotational_scan_matcher_histogram() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.TrajectoryNodeData.rotational_scan_matcher_histogram)
  return rotational_scan_matcher_histogram_;
}
inline ::google::protobuf::RepeatedField< float >*
TrajectoryNodeData::mutable_rotational_scan_matcher_histogram() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.TrajectoryNodeData.rotational_scan_matcher_histogram)
  return &rotational_scan_matcher_histogram_;
}

// .cartographer.transform.proto.Rigid3d local_pose = 7;
inline bool TrajectoryNodeData::has_local_pose() const {
  return this != internal_default_instance() && local_pose_ != NULL;
}
inline void TrajectoryNodeData::clear_local_pose() {
  if (GetArenaNoVirtual() == NULL && local_pose_ != NULL) delete local_pose_;
  local_pose_ = NULL;
}
inline const ::csmlio::transform::proto::Rigid3d& TrajectoryNodeData::local_pose() const {
  const ::csmlio::transform::proto::Rigid3d* p = local_pose_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.TrajectoryNodeData.local_pose)
  return p != NULL ? *p : *reinterpret_cast<const ::csmlio::transform::proto::Rigid3d*>(
      &::csmlio::transform::proto::_Rigid3d_default_instance_);
}
inline ::csmlio::transform::proto::Rigid3d* TrajectoryNodeData::mutable_local_pose() {
  
  if (local_pose_ == NULL) {
    local_pose_ = new ::csmlio::transform::proto::Rigid3d;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.TrajectoryNodeData.local_pose)
  return local_pose_;
}
inline ::csmlio::transform::proto::Rigid3d* TrajectoryNodeData::release_local_pose() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.TrajectoryNodeData.local_pose)
  
  ::csmlio::transform::proto::Rigid3d* temp = local_pose_;
  local_pose_ = NULL;
  return temp;
}
inline void TrajectoryNodeData::set_allocated_local_pose(::csmlio::transform::proto::Rigid3d* local_pose) {
  delete local_pose_;
  local_pose_ = local_pose;
  if (local_pose) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.TrajectoryNodeData.local_pose)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace proto
}  // namespace mapping
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cartographer_2fmapping_2fproto_2ftrajectory_5fnode_5fdata_2eproto__INCLUDED
