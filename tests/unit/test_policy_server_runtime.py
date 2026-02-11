#!/usr/bin/env python3
"""Runtime-focused unit tests for policy_server internals."""

import errno
import sys
import types
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "manipulation_policy"))

from manipulation_policy import policy_server  # noqa: E402


class _FakeModel:
    def __init__(self):
        self.to_calls = []
        self.eval_called = False

    def to(self, device):
        self.to_calls.append(device)
        return self

    def eval(self):
        self.eval_called = True


class _FakeAutoModelForVision2Seq:
    calls = []

    @classmethod
    def from_pretrained(cls, model_id, **kwargs):
        cls.calls.append((model_id, kwargs))
        return _FakeModel()


class _FakeAutoProcessor:
    calls = []

    @classmethod
    def from_pretrained(cls, model_id, trust_remote_code=True):
        cls.calls.append((model_id, trust_remote_code))
        return object()


def _reset_openvla_globals(monkeypatch):
    monkeypatch.setattr(policy_server, "_OPENVLA_MODEL", None)
    monkeypatch.setattr(policy_server, "_OPENVLA_PROCESSOR", None)
    monkeypatch.setattr(policy_server, "_OPENVLA_DEVICE", None)
    monkeypatch.setattr(policy_server, "_OPENVLA_DTYPE", None)


def test_send_json_ignores_broken_pipe():
    handler = policy_server.PolicyRequestHandler.__new__(policy_server.PolicyRequestHandler)
    handler.close_connection = False
    handler.send_response = lambda status: None
    handler.send_header = lambda name, value: None
    handler.end_headers = lambda: None

    class _BrokenPipeWriter:
        def write(self, _):
            raise BrokenPipeError(errno.EPIPE, "broken pipe")

    handler.wfile = _BrokenPipeWriter()

    handler._send_json({"ok": True}, status_code=200)
    assert handler.close_connection is True


def test_send_json_reraises_non_disconnect_oserror():
    handler = policy_server.PolicyRequestHandler.__new__(policy_server.PolicyRequestHandler)
    handler.close_connection = False
    handler.send_response = lambda status: None
    handler.send_header = lambda name, value: None
    handler.end_headers = lambda: None

    class _GenericOSErrorWriter:
        def write(self, _):
            raise OSError(errno.ENOSPC, "disk full")

    handler.wfile = _GenericOSErrorWriter()

    with pytest.raises(OSError):
        handler._send_json({"ok": True}, status_code=200)


def test_load_openvla_uses_device_map_for_flash_attention(monkeypatch):
    _FakeAutoModelForVision2Seq.calls = []
    _FakeAutoProcessor.calls = []
    _reset_openvla_globals(monkeypatch)

    fake_torch = types.SimpleNamespace(
        cuda=types.SimpleNamespace(
            is_available=lambda: True,
            is_bf16_supported=lambda: False,
        ),
        bfloat16="bf16",
        float16="fp16",
        float32="fp32",
    )

    monkeypatch.setattr(policy_server, "torch", fake_torch)
    monkeypatch.setattr(policy_server, "AutoProcessor", _FakeAutoProcessor)
    monkeypatch.setattr(policy_server, "AutoModelForVision2Seq", _FakeAutoModelForVision2Seq)
    monkeypatch.setattr(policy_server, "Image", object())

    monkeypatch.setenv("OPENVLA_MODEL_ID", "dummy/openvla")
    monkeypatch.setenv("OPENVLA_DEVICE", "cuda")
    monkeypatch.setenv("OPENVLA_ATTENTION_IMPL", "flash_attention_2")

    model, processor, device, dtype = policy_server._load_openvla()

    assert processor is not None
    assert model.eval_called is True
    assert model.to_calls == []
    assert device == "cuda"
    assert dtype == "fp16"
    assert len(_FakeAutoModelForVision2Seq.calls) == 1
    _, load_kwargs = _FakeAutoModelForVision2Seq.calls[0]
    assert load_kwargs["attn_implementation"] == "flash_attention_2"
    assert load_kwargs["torch_dtype"] == "fp16"
    assert load_kwargs["device_map"] == {"": "cuda"}
