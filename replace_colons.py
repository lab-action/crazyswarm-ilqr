#!/usr/bin/env python

from pathlib import Path
import os


def main():
    path = Path("src/iLQR/datalog").resolve()
    if not path.is_dir():
        raise ValueError()
    
    for file in path.iterdir():
        file_new = Path(file.parent) / file.name.replace(".", "_", 1).replace(":", ".")
        os.replace(file, file_new)

if __name__ == "__main__":
    main()
