using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("discovery_msgs");

@0x8c1726f48700b030;

struct Beacon {
  uuid @0 :Data;
  ip @1 :Text;
  port @2 :Int16;
}
