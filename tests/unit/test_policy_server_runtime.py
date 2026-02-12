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


def test_resolve_openvla_action_scalings_prefers_request_values(monkeypatch):
    monkeypatch.setenv("OPENVLA_XYZ_SCALING", "0.3")
    monkeypatch.setenv("OPENVLA_ROTATION_SCALING", "0.4")
    xyz_scaling, rot_scaling = policy_server._resolve_openvla_action_scalings(
        {"openvla_xyz_scaling": 0.6, "openvla_rotation_scaling": 0.7}
    )
    assert xyz_scaling == pytest.approx(0.6)
    assert rot_scaling == pytest.approx(0.7)


def test_resolve_openvla_action_scalings_use_component_env(monkeypatch):
    monkeypatch.setenv("OPENVLA_XYZ_SCALING", "0.4")
    monkeypatch.setenv("OPENVLA_ROTATION_SCALING", "0.8")
    xyz_scaling, rot_scaling = policy_server._resolve_openvla_action_scalings({})
    assert xyz_scaling == pytest.approx(0.4)
    assert rot_scaling == pytest.approx(0.8)


def test_resolve_openvla_action_scalings_default_to_one(monkeypatch):
    monkeypatch.delenv("OPENVLA_XYZ_SCALING", raising=False)
    monkeypatch.delenv("OPENVLA_ROTATION_SCALING", raising=False)
    xyz_scaling, rot_scaling = policy_server._resolve_openvla_action_scalings({})
    assert xyz_scaling == pytest.approx(1.0)
    assert rot_scaling == pytest.approx(1.0)


def test_resolve_openvla_action_scalings_invalid_values_fall_back(monkeypatch):
    monkeypatch.delenv("OPENVLA_XYZ_SCALING", raising=False)
    monkeypatch.delenv("OPENVLA_ROTATION_SCALING", raising=False)
    xyz_scaling, rot_scaling = policy_server._resolve_openvla_action_scalings(
        {"openvla_xyz_scaling": "bad", "openvla_rotation_scaling": "worse"}
    )
    assert xyz_scaling == pytest.approx(1.0)
    assert rot_scaling == pytest.approx(1.0)


def test_resolve_openvla_action_clamps_prefers_request_values(monkeypatch):
    monkeypatch.setenv("OPENVLA_CLIP_ACTIONS", "false")
    monkeypatch.setenv("OPENVLA_POSITION_BOUNDS", "-0.9,0.9")
    monkeypatch.setenv("OPENVLA_ROTATION_BOUNDS", "-0.8,0.8")
    monkeypatch.setenv("OPENVLA_GRIPPER_BOUNDS", "0.1,0.9")
    clip_actions, position_bounds, rotation_bounds, gripper_bounds = (
        policy_server._resolve_openvla_action_clamps(
            {
                "openvla_clip_actions": True,
                "openvla_position_bounds": [-0.3, 0.3],
                "openvla_rotation_bounds": [-0.2, 0.2],
                "openvla_gripper_bounds": [0.0, 1.0],
            }
        )
    )
    assert clip_actions is True
    assert position_bounds == pytest.approx((-0.3, 0.3))
    assert rotation_bounds == pytest.approx((-0.2, 0.2))
    assert gripper_bounds == pytest.approx((0.0, 1.0))


def test_resolve_openvla_action_clamps_use_env(monkeypatch):
    monkeypatch.setenv("OPENVLA_CLIP_ACTIONS", "true")
    monkeypatch.setenv("OPENVLA_POSITION_BOUNDS", "-0.6,0.6")
    monkeypatch.setenv("OPENVLA_ROTATION_BOUNDS", "-0.4,0.4")
    monkeypatch.setenv("OPENVLA_GRIPPER_BOUNDS", "0.2,0.8")
    clip_actions, position_bounds, rotation_bounds, gripper_bounds = (
        policy_server._resolve_openvla_action_clamps({})
    )
    assert clip_actions is True
    assert position_bounds == pytest.approx((-0.6, 0.6))
    assert rotation_bounds == pytest.approx((-0.4, 0.4))
    assert gripper_bounds == pytest.approx((0.2, 0.8))


def test_build_stub_response_applies_openvla_component_scalings(monkeypatch):
    class _NoOpContextManager:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    class _FakeTorchForInference:
        @staticmethod
        def inference_mode():
            return _NoOpContextManager()

        @staticmethod
        def is_tensor(_):
            return False

    class _FakeProcessorForInference:
        def __call__(self, **kwargs):
            return {"input_ids": [1, 2, 3]}

    class _FakeModelForInference:
        def predict_action(self, **kwargs):
            return [0.4, -0.2, 0.1, 0.2, -0.4, 0.6, 1.0]

    monkeypatch.setattr(policy_server, "torch", _FakeTorchForInference())
    monkeypatch.setattr(policy_server, "_decode_image", lambda request: object())
    monkeypatch.setattr(
        policy_server,
        "_load_openvla",
        lambda: (_FakeModelForInference(), _FakeProcessorForInference(), "cpu", "fp32"),
    )

    response = policy_server.build_stub_response(
        {
            "reference_frame": "piper_base_link",
            "task": "pick up the bottle",
            "openvla_xyz_scaling": 0.5,
            "openvla_rotation_scaling": 0.25,
        }
    )

    expected_qx, expected_qy, expected_qz, expected_qw = policy_server._euler_to_quaternion(
        0.05, -0.1, 0.15
    )
    assert response["has_eef_target"] is True
    assert response["eef_target_pose"]["position"]["x"] == pytest.approx(0.2)
    assert response["eef_target_pose"]["position"]["y"] == pytest.approx(-0.1)
    assert response["eef_target_pose"]["position"]["z"] == pytest.approx(0.05)
    assert response["eef_target_pose"]["orientation"]["x"] == pytest.approx(expected_qx)
    assert response["eef_target_pose"]["orientation"]["y"] == pytest.approx(expected_qy)
    assert response["eef_target_pose"]["orientation"]["z"] == pytest.approx(expected_qz)
    assert response["eef_target_pose"]["orientation"]["w"] == pytest.approx(expected_qw)
    assert response["reference_frame"] == "piper_base_link"


def test_build_stub_response_applies_openvla_component_clamps(monkeypatch):
    class _NoOpContextManager:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    class _FakeTorchForInference:
        @staticmethod
        def inference_mode():
            return _NoOpContextManager()

        @staticmethod
        def is_tensor(_):
            return False

    class _FakeProcessorForInference:
        def __call__(self, **kwargs):
            return {"input_ids": [1, 2, 3]}

    class _FakeModelForInference:
        def predict_action(self, **kwargs):
            return [0.9, -0.8, 0.7, 0.6, -0.5, 0.4, 1.3]

    monkeypatch.setattr(policy_server, "torch", _FakeTorchForInference())
    monkeypatch.setattr(policy_server, "_decode_image", lambda request: object())
    monkeypatch.setattr(
        policy_server,
        "_load_openvla",
        lambda: (_FakeModelForInference(), _FakeProcessorForInference(), "cpu", "fp32"),
    )

    response = policy_server.build_stub_response(
        {
            "reference_frame": "piper_base_link",
            "task": "pick up the bottle",
            "openvla_clip_actions": True,
            "openvla_position_bounds": [-0.2, 0.2],
            "openvla_rotation_bounds": [-0.1, 0.1],
            "openvla_gripper_bounds": [0.0, 1.0],
        }
    )

    expected_qx, expected_qy, expected_qz, expected_qw = policy_server._euler_to_quaternion(
        0.1, -0.1, 0.1
    )
    assert response["has_eef_target"] is True
    assert response["eef_target_pose"]["position"]["x"] == pytest.approx(0.2)
    assert response["eef_target_pose"]["position"]["y"] == pytest.approx(-0.2)
    assert response["eef_target_pose"]["position"]["z"] == pytest.approx(0.2)
    assert response["eef_target_pose"]["orientation"]["x"] == pytest.approx(expected_qx)
    assert response["eef_target_pose"]["orientation"]["y"] == pytest.approx(expected_qy)
    assert response["eef_target_pose"]["orientation"]["z"] == pytest.approx(expected_qz)
    assert response["eef_target_pose"]["orientation"]["w"] == pytest.approx(expected_qw)
    assert response["gripper_command"] == pytest.approx(1.0)
    assert response["reference_frame"] == "piper_base_link"


def test_build_stub_response_does_not_pass_unnorm_key_by_default(monkeypatch):
    class _NoOpContextManager:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    class _FakeTorchForInference:
        @staticmethod
        def inference_mode():
            return _NoOpContextManager()

        @staticmethod
        def is_tensor(_):
            return False

    class _FakeProcessorForInference:
        def __call__(self, **kwargs):
            return {"input_ids": [1, 2, 3]}

    class _FakeModelForInference:
        def __init__(self):
            self.predict_kwargs = None

        def predict_action(self, **kwargs):
            self.predict_kwargs = kwargs
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    fake_model = _FakeModelForInference()
    monkeypatch.delenv("OPENVLA_UNNORM_KEY", raising=False)
    monkeypatch.setattr(policy_server, "torch", _FakeTorchForInference())
    monkeypatch.setattr(policy_server, "_decode_image", lambda request: object())
    monkeypatch.setattr(
        policy_server,
        "_load_openvla",
        lambda: (fake_model, _FakeProcessorForInference(), "cpu", "fp32"),
    )

    policy_server.build_stub_response({"task": "pick up the bottle"})

    assert fake_model.predict_kwargs is not None
    assert "unnorm_key" not in fake_model.predict_kwargs
    assert fake_model.predict_kwargs["do_sample"] is False


def test_build_stub_response_passes_unnorm_key_when_env_set(monkeypatch):
    class _NoOpContextManager:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    class _FakeTorchForInference:
        @staticmethod
        def inference_mode():
            return _NoOpContextManager()

        @staticmethod
        def is_tensor(_):
            return False

    class _FakeProcessorForInference:
        def __call__(self, **kwargs):
            return {"input_ids": [1, 2, 3]}

    class _FakeModelForInference:
        def __init__(self):
            self.predict_kwargs = None

        def predict_action(self, **kwargs):
            self.predict_kwargs = kwargs
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    fake_model = _FakeModelForInference()
    monkeypatch.setenv("OPENVLA_UNNORM_KEY", " custom_key ")
    monkeypatch.setattr(policy_server, "torch", _FakeTorchForInference())
    monkeypatch.setattr(policy_server, "_decode_image", lambda request: object())
    monkeypatch.setattr(
        policy_server,
        "_load_openvla",
        lambda: (fake_model, _FakeProcessorForInference(), "cpu", "fp32"),
    )

    policy_server.build_stub_response({"task": "pick up the bottle"})

    assert fake_model.predict_kwargs is not None
    assert fake_model.predict_kwargs["unnorm_key"] == "custom_key"
    assert fake_model.predict_kwargs["do_sample"] is False
