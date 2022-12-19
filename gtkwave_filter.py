#!/usr/bin/env python3
import asyncio
from bitstring import BitArray
import rv32i
import sys

# Based on Matt Venn's work: https://github.com/mattvenn/gtkwave-python-filter-process
# Modified to be async to avoid delays


async def instruction_filter():
    loop = asyncio.get_event_loop()
    reader = asyncio.StreamReader()
    protocol = asyncio.StreamReaderProtocol(reader)
    await loop.connect_read_pipe(lambda: protocol, sys.stdin)
    w_transport, w_protocol = await loop.connect_write_pipe(
        asyncio.streams.FlowControlMixin, sys.stdout
    )
    writer = asyncio.StreamWriter(w_transport, w_protocol, reader, loop)

    while True:
        try:
            line = await asyncio.wait_for(reader.readline(), 0.1)
        except asyncio.exceptions.TimeoutError:
            continue
        except EOFError:
            # sys.stderr.write(f">>> Translation stopping.\n")
            return 0
        except Exception as e:
            # sys.stderr.write(f">>> Uncaught Exception: {dir(e)} {e}\n")
            return 1
        if not line:
            # sys.stderr.write(f">>> Translation stopping.\n")
            return 0
        line = line.decode()
        line = line.strip()
        # sys.stderr.write(f">>> line: {line}\n")

        if "x" in line.lower():
            writer.write(bytes("< X >\n", "ascii"))
            continue
        # sys.stderr.write(f">>> {line}\n")
        bits = BitArray(hex=line)
        if not bits or bits.length != 32:
            # sys.stderr.write(f">>> bad bit array: {bits} form line {line}\n")
            writer.write(bytes(line + "\n", "ascii"))
            continue
        # TODO(avinash) - generate labels from known assembly file.
        try:
            disassembled = rv32i.bits_to_line(bits)
        except Exception as e:
            writer.write(bytes(" > ??? < \n", "ascii"))
            # sys.stderr.write("f>>> Couldn't parse {line}\n")

        writer.write(bytes(f"{disassembled}\n", "ascii"))


if __name__ == "__main__":
    ret = asyncio.run(instruction_filter())
    sys.exit(ret)
