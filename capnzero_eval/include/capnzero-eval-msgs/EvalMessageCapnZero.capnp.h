// Generated by Cap'n Proto compiler, DO NOT EDIT
// source: EvalMessageCapnZero.capnp

#ifndef CAPNP_INCLUDED_d9693c54d5510667_
#define CAPNP_INCLUDED_d9693c54d5510667_

#include <capnp/generated-header-support.h>

#if CAPNP_VERSION != 6001
#error "Version mismatch between generated code and library headers.  You must use the same version of the Cap'n Proto compiler and library."
#endif


namespace capnp {
namespace schemas {

CAPNP_DECLARE_SCHEMA(f513b30d5c7ab329);

}  // namespace schemas
}  // namespace capnp

namespace capnzero_eval {

struct EvalMessageCapnZero {
  EvalMessageCapnZero() = delete;

  class Reader;
  class Builder;
  class Pipeline;

  struct _capnpPrivate {
    CAPNP_DECLARE_STRUCT_HEADER(f513b30d5c7ab329, 1, 1)
    #if !CAPNP_LITE
    static constexpr ::capnp::_::RawBrandedSchema const* brand() { return &schema->defaultBrand; }
    #endif  // !CAPNP_LITE
  };
};

// =======================================================================================

class EvalMessageCapnZero::Reader {
public:
  typedef EvalMessageCapnZero Reads;

  Reader() = default;
  inline explicit Reader(::capnp::_::StructReader base): _reader(base) {}

  inline ::capnp::MessageSize totalSize() const {
    return _reader.totalSize().asPublic();
  }

#if !CAPNP_LITE
  inline ::kj::StringTree toString() const {
    return ::capnp::_::structString(_reader, *_capnpPrivate::brand());
  }
#endif  // !CAPNP_LITE

  inline bool hasPayload() const;
  inline  ::capnp::List< ::uint32_t>::Reader getPayload() const;

  inline  ::uint32_t getId() const;

private:
  ::capnp::_::StructReader _reader;
  template <typename, ::capnp::Kind>
  friend struct ::capnp::ToDynamic_;
  template <typename, ::capnp::Kind>
  friend struct ::capnp::_::PointerHelpers;
  template <typename, ::capnp::Kind>
  friend struct ::capnp::List;
  friend class ::capnp::MessageBuilder;
  friend class ::capnp::Orphanage;
};

class EvalMessageCapnZero::Builder {
public:
  typedef EvalMessageCapnZero Builds;

  Builder() = delete;  // Deleted to discourage incorrect usage.
                       // You can explicitly initialize to nullptr instead.
  inline Builder(decltype(nullptr)) {}
  inline explicit Builder(::capnp::_::StructBuilder base): _builder(base) {}
  inline operator Reader() const { return Reader(_builder.asReader()); }
  inline Reader asReader() const { return *this; }

  inline ::capnp::MessageSize totalSize() const { return asReader().totalSize(); }
#if !CAPNP_LITE
  inline ::kj::StringTree toString() const { return asReader().toString(); }
#endif  // !CAPNP_LITE

  inline bool hasPayload();
  inline  ::capnp::List< ::uint32_t>::Builder getPayload();
  inline void setPayload( ::capnp::List< ::uint32_t>::Reader value);
  inline void setPayload(::kj::ArrayPtr<const  ::uint32_t> value);
  inline  ::capnp::List< ::uint32_t>::Builder initPayload(unsigned int size);
  inline void adoptPayload(::capnp::Orphan< ::capnp::List< ::uint32_t>>&& value);
  inline ::capnp::Orphan< ::capnp::List< ::uint32_t>> disownPayload();

  inline  ::uint32_t getId();
  inline void setId( ::uint32_t value);

private:
  ::capnp::_::StructBuilder _builder;
  template <typename, ::capnp::Kind>
  friend struct ::capnp::ToDynamic_;
  friend class ::capnp::Orphanage;
  template <typename, ::capnp::Kind>
  friend struct ::capnp::_::PointerHelpers;
};

#if !CAPNP_LITE
class EvalMessageCapnZero::Pipeline {
public:
  typedef EvalMessageCapnZero Pipelines;

  inline Pipeline(decltype(nullptr)): _typeless(nullptr) {}
  inline explicit Pipeline(::capnp::AnyPointer::Pipeline&& typeless)
      : _typeless(kj::mv(typeless)) {}

private:
  ::capnp::AnyPointer::Pipeline _typeless;
  friend class ::capnp::PipelineHook;
  template <typename, ::capnp::Kind>
  friend struct ::capnp::ToDynamic_;
};
#endif  // !CAPNP_LITE

// =======================================================================================

inline bool EvalMessageCapnZero::Reader::hasPayload() const {
  return !_reader.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS).isNull();
}
inline bool EvalMessageCapnZero::Builder::hasPayload() {
  return !_builder.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS).isNull();
}
inline  ::capnp::List< ::uint32_t>::Reader EvalMessageCapnZero::Reader::getPayload() const {
  return ::capnp::_::PointerHelpers< ::capnp::List< ::uint32_t>>::get(_reader.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS));
}
inline  ::capnp::List< ::uint32_t>::Builder EvalMessageCapnZero::Builder::getPayload() {
  return ::capnp::_::PointerHelpers< ::capnp::List< ::uint32_t>>::get(_builder.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS));
}
inline void EvalMessageCapnZero::Builder::setPayload( ::capnp::List< ::uint32_t>::Reader value) {
  ::capnp::_::PointerHelpers< ::capnp::List< ::uint32_t>>::set(_builder.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS), value);
}
inline void EvalMessageCapnZero::Builder::setPayload(::kj::ArrayPtr<const  ::uint32_t> value) {
  ::capnp::_::PointerHelpers< ::capnp::List< ::uint32_t>>::set(_builder.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS), value);
}
inline  ::capnp::List< ::uint32_t>::Builder EvalMessageCapnZero::Builder::initPayload(unsigned int size) {
  return ::capnp::_::PointerHelpers< ::capnp::List< ::uint32_t>>::init(_builder.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS), size);
}
inline void EvalMessageCapnZero::Builder::adoptPayload(
    ::capnp::Orphan< ::capnp::List< ::uint32_t>>&& value) {
  ::capnp::_::PointerHelpers< ::capnp::List< ::uint32_t>>::adopt(_builder.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS), kj::mv(value));
}
inline ::capnp::Orphan< ::capnp::List< ::uint32_t>> EvalMessageCapnZero::Builder::disownPayload() {
  return ::capnp::_::PointerHelpers< ::capnp::List< ::uint32_t>>::disown(_builder.getPointerField(
      ::capnp::bounded<0>() * ::capnp::POINTERS));
}

inline  ::uint32_t EvalMessageCapnZero::Reader::getId() const {
  return _reader.getDataField< ::uint32_t>(
      ::capnp::bounded<0>() * ::capnp::ELEMENTS);
}

inline  ::uint32_t EvalMessageCapnZero::Builder::getId() {
  return _builder.getDataField< ::uint32_t>(
      ::capnp::bounded<0>() * ::capnp::ELEMENTS);
}
inline void EvalMessageCapnZero::Builder::setId( ::uint32_t value) {
  _builder.setDataField< ::uint32_t>(
      ::capnp::bounded<0>() * ::capnp::ELEMENTS, value);
}

}  // namespace

#endif  // CAPNP_INCLUDED_d9693c54d5510667_
