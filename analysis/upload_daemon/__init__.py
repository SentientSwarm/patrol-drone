"""Dev-host upload daemon for the logging & replay pipeline (docset 05, M8).

Watches the recorder's bag-output directory and dumbly copies each completed bag (+ its sidecar)
to the DGX (or a CI stand-in) via a pluggable :class:`~upload_daemon.transport.Transport`. All
fact derivation / indexing happens on the DGX side (the dumb-producer invariant, design §3.4) —
this package only transfers.
"""
