"""Layer-A unit test for verify_live_bag._header_stamp_ns (docset 05, M8 / F-06).

The witness's CDR byte-offset stamp extractor (verify_live_bag.py, the ROS lane) is fiddly
offset/endianness logic that pytest never collects (not a test_*.py). It's a self-contained pure
function, so pin its offset (+4 CDR encapsulation bytes) and little-endian <iI decode with a canned
buffer here. A wrong offset/endianness would otherwise silently corrupt every stamp.

verify_live_bag imports only ROS-free modules at top (rate_report / replay_assertions /
ingest.bag_reader); rosbag2_py is imported lazily inside _open_reader, so importing the module for
this pure function is safe under the Layer-A (no-ROS) env.
"""

from __future__ import annotations

import struct

from verify_live_bag import _ENCAPSULATION_BYTES, _header_stamp_ns


def _cdr_header(sec: int, nsec: int) -> bytes:
    """A minimal CDR buffer: 4 encapsulation bytes + a little-endian (int32 sec, uint32 nsec)."""
    return b"\x00\x01\x00\x00" + struct.pack("<iI", sec, nsec)


def test_header_stamp_ns_decodes_known_stamp() -> None:
    # 12 s + 500_000_000 ns → 12_500_000_000 ns.
    data = _cdr_header(12, 500_000_000)
    assert _header_stamp_ns(data) == 12 * 1_000_000_000 + 500_000_000


def test_header_stamp_ns_reads_past_the_4_byte_encapsulation() -> None:
    # Garbage in the 4 encapsulation bytes must NOT be read as the stamp (offset correctness).
    data = b"\xde\xad\xbe\xef" + struct.pack("<iI", 3, 7)
    assert _header_stamp_ns(data) == 3 * 1_000_000_000 + 7
    assert _ENCAPSULATION_BYTES == 4


def test_header_stamp_ns_none_when_too_short() -> None:
    # Fewer than 4 + 8 bytes can't hold a stamp → None (not an IndexError).
    assert _header_stamp_ns(b"\x00\x01\x00\x00\x00\x00\x00") is None
