using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("capnzero");

@0xd9693c54d5510667;

struct Testmessage {
  payload @0 :Text;
 id @1 :Int16;
}
