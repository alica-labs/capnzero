@0xf1c8e694d7d7fbbe;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("capnzero");

struct ID {
    value @0 :Data;
    type @1 :Int8;

    const wildcard :Int8 = 0;
    const uuid :Int8 = 1;
}