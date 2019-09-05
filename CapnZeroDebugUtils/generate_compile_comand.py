#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from schemautils import *


def main():
    include_dirs = get_include_paths(get_schema_files())
    if len(sys.argv) < 2:
        print("Too view arguments.\nMust specify a file to compile!")
    elif len(sys.argv) < 3:
        command = gen_compile_command(sys.argv[1], includes=include_dirs)
        print("\nResult:")
        print(command)
    else:
        command = gen_compile_command(sys.argv[1], sys.argv[2], include_dirs)
        print("\nResult:")
        print(command)
        print("\n\nCompile:")
        os.system(command)


if __name__ == "__main__":
    main()
