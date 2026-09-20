"""On-disk audit trail.

MQTT events are the live feed; this is the record that survives. If the broker
dies, or the console was not running, or the demo needs forensics afterwards,
the JSONL file on disk is the only account of what the node decided and why.

Guardrails calls the audit trail a deliverable. A deliverable that exists only
in flight is not a deliverable.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class AuditLog:
    """Append-only JSONL event log.

    Thread-safety: `write` is safe from any thread; it holds a lock and flushes
    on every record so a hard kill cannot lose the last decision.
    """

    def __init__(self, directory: str = "logs", node_id: str = "killswitch") -> None:
        self._lock = threading.Lock()
        self._handle = None
        self._path: Optional[Path] = None
        try:
            folder = Path(directory)
            folder.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            self._path = folder / f"{node_id}-{stamp}.jsonl"
            self._handle = self._path.open("a", encoding="utf-8")
            logger.info("Audit log: %s", self._path)
        except OSError as exc:
            # A missing audit log must not prevent the node from running, but it
            # must be loud — this is the forensic record.
            logger.error("Could not open audit log in %s: %s", directory, exc)

    @property
    def path(self) -> Optional[Path]:
        return self._path

    def write(self, event: str, detail: Dict[str, Any]) -> None:
        """Append one event. Never raises."""
        if self._handle is None:
            return
        record = {"ts": time.time(), "monotonic": time.monotonic(), "event": event, **detail}
        try:
            with self._lock:
                self._handle.write(json.dumps(record, default=str) + "\n")
                self._handle.flush()
        except (OSError, TypeError, ValueError) as exc:
            logger.error("Audit write failed for %s: %s", event, exc)

    def close(self) -> None:
        with self._lock:
            if self._handle is not None:
                try:
                    self._handle.close()
                finally:
                    self._handle = None
