"""The watch-loops' bounded 'already-seen' membership set (M8 / F-09, PR #16 / F-07).

Canonical home for the LRU-capped membership set both watch loops use to track handled bags. It
lived duplicated in ``docker/ingest/__main__.py`` and ``analysis/upload_daemon/__main__.py`` (byte
identical); F-07 consolidated it here. This module is homed inside ``docker/ingest/`` so the ingest
Dockerfile's ``COPY docker/ingest/ /opt/ingest/ingest/`` ships it in the container unchanged; the
dev-host upload daemon reaches it via a ``sys.path`` bootstrap (see its ``__main__``).
"""

from __future__ import annotations

from collections import OrderedDict
from pathlib import Path


class _BoundedSeen:
    """A membership set with an LRU cap: tracks 'already handled' bags without growing unbounded.

    The watch loops only need 'have I already handled this bag this run?'. An unbounded set grows one
    entry per bag for the process lifetime (F-09); this caps it at ``maxlen``, evicting the
    oldest-added key when full. Eviction is safe because both handlers are idempotent (rsync -a /
    INSERT OR REPLACE) — a re-seen evicted bag is just a cheap redundant re-handle, never data loss.
    """

    def __init__(self, maxlen: int = 4096) -> None:
        self._seen: OrderedDict[Path, None] = OrderedDict()
        self._maxlen = maxlen

    def __contains__(self, bag: Path) -> bool:
        return bag in self._seen

    def add(self, bag: Path) -> None:
        self._seen[bag] = None
        self._seen.move_to_end(bag)
        while len(self._seen) > self._maxlen:
            self._seen.popitem(last=False)  # evict the oldest-added

    def __len__(self) -> int:
        return len(self._seen)
