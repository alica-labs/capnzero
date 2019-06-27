#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import pathlib

import schemautils
import argparse
import zmq


def list_schema():
    data = json.load(pathlib.Path("~/.local/share/CapnZero/info.json").expanduser().open("r"))
    for item in data["schema"].keys():
        print(item)


def main():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    iface_ok = False
    message_type = None
    skip = False
    parser = argparse.ArgumentParser(description="Echo CapnZero communication.")
    parser.add_argument("interface", metavar="iface", type=str, help="The interface you want to use. (e.g.: udp://224.0.0.2:5555)", nargs="*")
    parser.add_argument("schema", metavar="schema", type=str, help="The schema that the data is formatted in.", nargs="*")
    parser.add_argument("topic", metavar="topic", type=str, help="The topic that schould be listened to.", default="", nargs="*")
    parser.add_argument("-l", action="store_true", help="List the known topics.")
    parser.add_argument("-u", action="store_true", help="Update the schema list.")
    args = parser.parse_args()
    if args.u:
        import update_index
        update_index.update()
        skip = True
    if args.l:
        list_schema()
        skip = True
    if not skip:
        if args.interface != "":
            socket.connect(args.interface)
            socket.setsockopt_string(zmq.SUBSCRIBE, args.topic)
            iface_ok = True
        if args.schema != "":
            data = json.load(pathlib.Path("~/.local/share/CapnZero/info.json").expanduser().open("r"))
            message_type = schemautils.load_schema(data["schema"][args.schema])
            classes = message_type.__dict__.keys()
            message_class = None
            for item in classes:
                if (item.lower() in args.schema.split("-")[-1].lower()) and type(getattr(message_type, item)) is "capnp.lib.capnp._StructModule":
                    message_class = getattr(message_type, item)
                    break
        if not (args.l or args.u):
            while True:
                message = socket.recv()
                msg = message_class.from_bytes(message)
                print(msg)


if __name__ == "__main__":
    main()