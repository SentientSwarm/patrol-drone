"""DGX-side ingest + manifest for the logging & replay pipeline (docset 05, M8).

Indexes each bag the upload daemon lands into a queryable SQLite manifest (design §4.2.4): the
:class:`~ingest.ingest_service.IngestService` derives authoritative facts from the bag itself and
upserts one row into the :class:`~ingest.manifest_store.ManifestStore`, which the ``manifest_query``
CLI reads. This is the "smart" half of the dumb-producer / smart-ingestion split (design §3.4).
"""
