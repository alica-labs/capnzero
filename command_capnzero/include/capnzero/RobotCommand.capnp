@0xc0f7398d39bb2416;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("capnzero");

using IDMsg = import "/capnzero/ID.capnp";

struct RobotCommand {
	receiverID @0 :IDMsg.ID;
	command @1 :Cmd;

enum Cmd {
	start @0;
	stop @1;
 }
}


