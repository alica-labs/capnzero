#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import capnp


def get_schema_files():
    res = []
    for root, dirs, files in os.walk("/".join(os.getcwd().split("/")[:-1])):
        for item in files:
            if (".capnp" in item) and (".capnp.h" not in item) and (".capnp.c++" not in item) and ("/env/" not in root):
                res.append(root + "/" + item)
    return res


def get_include_paths(items=[]):
    tmp = set()
    [tmp.add("/".join(i.split("/")[:-2]) + "/") for i in items]
    return list(tmp)


def gen_compile_command(input, output="", includes=[]):
    if output != "":
        command = "capnp compile -oc++:{} --src-prefix={} ".format(output, "/".join(input.split("/")[:-1]))
    else:
        command = "capnp compile -oc++ "
    for item in includes:
        command += "--import-path={} ".format(item)
    command += input
    return command


# "/home/stefan/stummelws/src/capnzero/capnzero/msg/string.capnp"
def load_schema(path=""):
    capnp.remove_import_hook()
    im_paths = get_include_paths(get_schema_files())
    im_paths.append("/usr/include")
    im_paths.append("/usr/local/include")
    module = capnp.load(path, imports=im_paths)
    return module


def mk_name(data):
    return "-".join(data.split("/")[-2:])[:-6]


def main():
    schemas = get_schema_files()
    print("Schema files: ")
    for item in schemas:
        print(item)
    print("Include dirs:")
    for item in get_include_paths(schemas):
        print(item)
    print('Loading "/home/stefan/stummelws/src/capnzero/capnzero/msg/string.capnp"')
    load_schema("/home/stefan/stummelws/src/capnzero/capnzero/msg/string.capnp")


if __name__ == "__main__":
    main()
