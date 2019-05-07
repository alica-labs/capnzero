using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("capnzero_eval");

@0xd9693c54d5510667;

struct EvalMessage {
  payload @0 :Text;
  id @1 :Int16;
}
