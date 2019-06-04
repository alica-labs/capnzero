@0xc0f7398d39bb2416;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("capnzero");

using IDMsg = import "/capnzero/ID.capnp";

struct ProcessCommand {
	command @0 :Cmd;
	receiverID @1 :IDMsg.ID;
	robotIDs @2 :List(IDMsg.ID);


enum Cmd {
	start @0;
	stop @1;
 }
}


