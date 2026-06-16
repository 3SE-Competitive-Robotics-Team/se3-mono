from pathlib import Path

from sim_loop.runtime import _unlink_socket_path


def test_unlink_socket_path_refuses_regular_file(tmp_path: Path) -> None:
    path = tmp_path / "not-a-socket"
    path.write_text("user data")
    try:
        _unlink_socket_path(path, require_socket=True)
    except RuntimeError as exc:
        assert "non-socket" in str(exc)
    else:
        raise AssertionError("regular files must not be unlinked")
    assert path.exists()
