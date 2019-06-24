@0xf1c8e694d7d7fbbe;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("capnzero");

struct ID {
    value @0 :Data;
    type @1 :UInt8;

    const wildcard :UInt8 = 0;
    const uuid :UInt8 = 1;
}