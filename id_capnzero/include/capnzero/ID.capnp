@0xf1c8e694d7d7fbbe;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("capnzero");

struct ID {
    value @0 :Data;
    type @1 :IDType;
}

enum IDType {
  integer @0;
  wildcard @1;
  uuid @2;
}