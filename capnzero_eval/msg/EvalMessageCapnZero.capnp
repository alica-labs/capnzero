using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("capnzero_eval");

@0xd9693c54d5510667;

struct EvalMessageCapnZero {
  payload @0 :List(UInt32);
  id @1 :UInt32;
}
