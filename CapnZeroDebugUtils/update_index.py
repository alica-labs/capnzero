#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import schemautils
import json
import pathlib


def update():
    dataset = {}
    schema = schemautils.get_schema_files()
    dataset["schema"] = {}
    for item in schema:
        dataset["schema"][schemautils.mk_name(item)] = item
    with pathlib.Path("~/.local/share/CapnZero/info.json").expanduser().open("w") as file:
        json.dump(dataset, file, indent=2)


if __name__ == "__main__":
    update()
