#!/usr/bin/env python3
import argparse
from bitstring import BitArray
from io import UnsupportedOperation
import os.path as path
import rv32i


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="input assembled file")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="output file name to store human readable assembly",
    )
    parser.add_argument(
        "-r",
        "--radix",
        default="hex",
        help="Radix of the input (hex or bin). Ignored if this is binary input.",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Treat input as raw binary values (not ascii text).",
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
    if args.raw:
        raise NotImplemented("Haven't implemented raw parsing yet.")
    if args.radix not in ["hex", "bin"]:
        raise ValueError(f"Radix {args.radix} not supported.")
    instructions = []
    with open(args.input, "r") as f:
        for i, line in enumerate(f):
            if "#" in line:
                line = line.split("#")[0]
            if "//" in line:
                line = line.split("//")[0]
            line = line.strip()
            bits = None
            if args.radix == "hex":
                bits = BitArray(hex=line)
            if args.radix == "bin":
                bits = BitArray(bin=line)
            if not bits:
                raise ValueError(f"Error: Couldn't parse line {i+1}")
            if bits.length != 32:
                raise ValueError(
                    f"Error: line {i+1} was {bits.length} bits, not 32."
                )
            instructions.append(bits)
    labels = {}
    with open(args.output, "w") as f:
        for i, bits in enumerate(instructions):
            try:
                line = rv32i.bits_to_line(bits, labels=None)
            except Exception as e:
                print(f"Error on line {i+1}: ")
                raise e
            f.write(line + "\n")
    if args.verbose:
        print(f"Disassembled {args.input} -> {args.output}. Label table: ")
        print("  " + ",\n  ".join([f"{k} <- {labels[k]}" for k in labels]))


if __name__ == "__main__":
    main()
