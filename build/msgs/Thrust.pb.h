// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Thrust.proto

#ifndef PROTOBUF_INCLUDED_Thrust_2eproto
#define PROTOBUF_INCLUDED_Thrust_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_Thrust_2eproto 

namespace protobuf_Thrust_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_Thrust_2eproto
namespace ercr_msgs {
namespace msgs {
class Thrust;
class ThrustDefaultTypeInternal;
extern ThrustDefaultTypeInternal _Thrust_default_instance_;
}  // namespace msgs
}  // namespace ercr_msgs
namespace google {
namespace protobuf {
template<> ::ercr_msgs::msgs::Thrust* Arena::CreateMaybeMessage<::ercr_msgs::msgs::Thrust>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace ercr_msgs {
namespace msgs {

// ===================================================================

class Thrust : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:ercr_msgs.msgs.Thrust) */ {
 public:
  Thrust();
  virtual ~Thrust();

  Thrust(const Thrust& from);

  inline Thrust& operator=(const Thrust& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Thrust(Thrust&& from) noexcept
    : Thrust() {
    *this = ::std::move(from);
  }

  inline Thrust& operator=(Thrust&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Thrust& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Thrust* internal_default_instance() {
    return reinterpret_cast<const Thrust*>(
               &_Thrust_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Thrust* other);
  friend void swap(Thrust& a, Thrust& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Thrust* New() const final {
    return CreateMaybeMessage<Thrust>(NULL);
  }

  Thrust* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Thrust>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Thrust& from);
  void MergeFrom(const Thrust& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Thrust* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required float left = 1;
  bool has_left() const;
  void clear_left();
  static const int kLeftFieldNumber = 1;
  float left() const;
  void set_left(float value);

  // required float right = 2;
  bool has_right() const;
  void clear_right();
  static const int kRightFieldNumber = 2;
  float right() const;
  void set_right(float value);

  // @@protoc_insertion_point(class_scope:ercr_msgs.msgs.Thrust)
 private:
  void set_has_left();
  void clear_has_left();
  void set_has_right();
  void clear_has_right();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  float left_;
  float right_;
  friend struct ::protobuf_Thrust_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Thrust

// required float left = 1;
inline bool Thrust::has_left() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Thrust::set_has_left() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Thrust::clear_has_left() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Thrust::clear_left() {
  left_ = 0;
  clear_has_left();
}
inline float Thrust::left() const {
  // @@protoc_insertion_point(field_get:ercr_msgs.msgs.Thrust.left)
  return left_;
}
inline void Thrust::set_left(float value) {
  set_has_left();
  left_ = value;
  // @@protoc_insertion_point(field_set:ercr_msgs.msgs.Thrust.left)
}

// required float right = 2;
inline bool Thrust::has_right() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Thrust::set_has_right() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Thrust::clear_has_right() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Thrust::clear_right() {
  right_ = 0;
  clear_has_right();
}
inline float Thrust::right() const {
  // @@protoc_insertion_point(field_get:ercr_msgs.msgs.Thrust.right)
  return right_;
}
inline void Thrust::set_right(float value) {
  set_has_right();
  right_ = value;
  // @@protoc_insertion_point(field_set:ercr_msgs.msgs.Thrust.right)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace ercr_msgs

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_Thrust_2eproto
