from pathlib import Path
import tomllib


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYPROJECT = PROJECT_ROOT / "pyproject.toml"


def test_runtime_dependencies_include_uvicorn_websocket_backend():
    project = tomllib.loads(PYPROJECT.read_text(encoding="utf-8"))["project"]
    dependencies = project["dependencies"]
    assert any(requirement.startswith("websockets") for requirement in dependencies)
