#!/usr/bin/env python3

# Heavily based on this [reference card](http://csci206sp2020.courses.bucknell.edu/files/2020/01/riscv-card.pdf)
# and the official [spec](https://github.com/riscv/riscv-isa-manual/releases/download/Ratified-IMAFDQC/riscv-spec-20191213.pdf)


import argparse
import os
import os.path as path
import re
import sys

import rv32i


class AssemblyProgram:
    def __init__(self, start_address=0, labels=None):
        self.address = start_address
        self.line_number = 0
        self.labels = {}
        if labels:
            for k in labels:
                self.labels[k] = labels[k]
        self.parsed_lines = []

    def parse_line(self, line):
        self.line_number += 1
        parsed = {}
        line = line.strip()
        parsed["original"] = line
        line = re.sub("\s*#.*", "", line)  # Remove comments.
        line = re.sub("\s*//.*", "", line)  # Remove C-style comments.
        match = re.search("^(\w+):", line)
        if match:
            self.labels[match.group(1)] = self.address
            line = re.sub("^(\w+):\s*", "", line)
            parsed["label"] = match.group(1)
        match = re.search("^(\w+)\s*(.*)", line)
        if not match:
            return -1
        parsed["line_number"] = self.line_number
        parsed["instruction"] = match.group(1)
        parsed["args"] = [x.strip() for x in match.group(2).split(",")]
        # Handle psuedo-instructions.
        if parsed["instruction"] == "nop":
            parsed["instruction"] = "addi"
            parsed["args"] = ["x0", "x0", "0"]
        if parsed["instruction"] == "j":
            parsed["instruction"] = "jal"
            parsed["args"].insert(0, "x0")
        if parsed["instruction"] == "jr":
            parsed["instruction"] = "jalr"
            parsed["args"].insert(0, "x0")
            parsed["args"].append("0")
        if parsed["instruction"] == "mv":
            parsed["instruction"] = "addi"
            parsed["args"].append("0")
        if parsed["instruction"] == "not":
            parsed["instruction"] = "xori"
            parsed["args"].append("-1")
        if parsed["instruction"] == "li":

            raise NotImplemented("li is not supported")
        if parsed["instruction"] == "bgt":
            parsed["instruction"] = "blt"
            parsed["args"] = [
                parsed["args"][1],
                parsed["args"][0],
                parsed["args"][2],
            ]
        if parsed["instruction"] == "bgez":
            parsed["instruction"] = "bge"
            parsed["args"].insert(1, "x0")
        if parsed["instruction"] == "call":
            print("Warning: call only works with nearby functions!")
            parsed["instruction"] = "jal"
            parsed["args"].insert(0, "ra")
        if parsed["instruction"] == "ret":
            parsed["instruction"] = "jalr"
            parsed["args"] = ["x0", "ra", "0"]
        self.address += 4
        self.parsed_lines.append(parsed)
        return 0

    def write_mem(self, fn, hex_notbin=True, disable_annotations=False):
        output = []
        address = 0
        for line in self.parsed_lines:
            try:
                bits = rv32i.line_to_bits(line, labels=self.labels, address=address)
            except rv32i.LineException as e:
                print(f"Error on line {line['line_number']} ({line['instruction']})")
                print(f"  {e}")
                print(f"  original line: {line['original']}")
                return -1
            except Exception as e:
                print(f"Unhandled error, possible bug in assembler!!!")
                print(f"Error on line {line['line_number']} ({line['instruction']})")
                print(f"  {e}")
                print(f"  original line: {line['original']}")
                raise e
            address += 4
            output.append((bits, line))
        # Only write the file if the above completes without errors
        with open(fn, "w") as f:
            address = 0
            for bits, line in output:
                annotation = f" // PC={hex(address)} line={line['line_number']}: {line['original']}"
                if disable_annotations:
                    annotation = ""
                if hex_notbin:
                    f.write(f"{bits.hex}{annotation}\n")
                else:
                    f.write(bits.bin + "\n")
                address += 4
        return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="input file name of human readable assembly")
    parser.add_argument(
        "-o",
        "--output",
        help="output file name of hex values in text that can be read from SystemVerilog's readmemh",
    )
    parser.add_argument(
        "--disable_annotations",
        action="store_true",
        default=False,
        help="Prints memh files without any annotations.",
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="increases verbosity of the script",
    )
    args = parser.parse_args()
    if not path.exists(args.input):
        raise Exception(f"input file {args.input} does not exist.")
    ap = AssemblyProgram()
    with open(args.input, "r") as f:
        for line in f:
            ap.parse_line(line)
    if args.verbose:
        print(f"Parsed {len(ap.parsed_lines)} instructions. Label table:")
        print("  " + ",\n  ".join([f"{k} -> {ap.labels[k]}" for k in ap.labels]))
    if args.output:
        sys.exit(
            ap.write_mem(
                args.output,
                hex_notbin=not "memb" in args.output,
                disable_annotations=args.disable_annotations,
            )
        )
    sys.exit(0)


if __name__ == "__main__":
    main()
