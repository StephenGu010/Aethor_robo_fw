"""Consume one fixed host application Save claim before any AA-producing commit.

The caller must always use the same pre-provisioned, trusted directory for this
maintenance operation. A successful return permits the caller to proceed only
once; an exception forbids commit. Existing tokens are never read as approval,
cleared, repaired, renamed, or retried, even after a new request, boot, or image.

This is a host application once guard, not bus exactly-once delivery or an
absolute guarantee against physical power loss. It does not resist a malicious
local administrator, replacement of parent directories, or deliberate token
removal. The controller USB serial does not identify unique drive hardware and
cannot establish the drive's 24 V supply state. No hardware access occurs here.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import errno
import json
import os
from pathlib import Path
import re
import stat

TOKEN_FILENAME = "id7_pmax512_pos2.once"


@dataclass(frozen=True, slots=True)
class SaveOnceBinding:
    """Bind a claim to exact evidence hashes and one configured controller session."""

    firmware_hex_sha256: str
    source_manifest_sha256: str
    cold_baseline_sha256: str
    controller_usb_serial: str
    boot_epoch: int
    request_id: int
    owner_generation: int
    cdc_link_state: int

    def __post_init__(self) -> None:
        """Reject invalid fields when constructing the immutable binding."""
        _validate_binding(self)


def _validate_binding(binding: SaveOnceBinding) -> None:
    """Require exact fixed-identity strings and non-coerced uint32 session values."""
    if type(binding) is not SaveOnceBinding:
        raise ValueError("binding must be an exact SaveOnceBinding")
    for field_name in ("firmware_hex_sha256", "source_manifest_sha256", "cold_baseline_sha256"):
        digest = getattr(binding, field_name)
        if type(digest) is not str or re.fullmatch(r"[0-9a-f]{64}", digest) is None or digest == "0" * 64:
            raise ValueError(f"{field_name} must be a nonzero lowercase SHA-256")
    if type(binding.controller_usb_serial) is not str or binding.controller_usb_serial != "386B336F3134":
        raise ValueError("expected controller USB serial 386B336F3134")
    for field_name in ("boot_epoch", "request_id", "owner_generation", "cdc_link_state"):
        value = getattr(binding, field_name)
        if type(value) is not int or not 1 <= value <= 0xFFFFFFFF:
            raise ValueError(f"{field_name} must be a nonzero exact uint32")
    if not binding.cdc_link_state & 0x80000000 or not binding.cdc_link_state & 0x7FFFFFFF:
        raise ValueError("cdc_link_state must be configured with a nonzero generation")


def claim_save_once(directory: str | os.PathLike[str], binding: SaveOnceBinding) -> Path:
    """Exclusively consume the fixed token, returning only after write, fsync and close.

    Validate the binding before touching the filesystem. Require an existing real
    directory and refuse a final symlink or Windows reparse point. O_EXCL rejects
    every existing token entry, including empty files, directories and symlinks.
    All write/sync/close failures propagate while the created entry remains
    consumed; the caller must not send commit or attempt claim recovery.
    """
    _validate_binding(binding)
    claim_directory = Path(directory)
    directory_status = claim_directory.lstat()
    reparse_attribute = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    if stat.S_ISLNK(directory_status.st_mode) or getattr(directory_status, "st_file_attributes", 0) & reparse_attribute:
        raise OSError(errno.ELOOP, "claim directory must not be a link or reparse point", str(claim_directory))
    if not stat.S_ISDIR(directory_status.st_mode):
        raise NotADirectoryError(errno.ENOTDIR, "claim directory must already exist", str(claim_directory))

    document = {
        "schema_version": 1,
        "target": {
            "can_id": 7,
            "mst": 23,
            "firmware_main": 808597046,
            "firmware_sub": 49,
            "pmax_raw": 0x44000000,
            "mode": 2,
        },
        "binding": asdict(binding),
    }
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    token_path = claim_directory / TOKEN_FILENAME
    descriptor = os.open(token_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0), 0o600)
    try:
        written_bytes = 0
        while written_bytes < len(payload):
            write_count = os.write(descriptor, payload[written_bytes:])
            if type(write_count) is not int or not 0 < write_count <= len(payload) - written_bytes:
                raise OSError(errno.EIO, "token write made invalid progress", str(token_path))
            written_bytes += write_count
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return token_path
