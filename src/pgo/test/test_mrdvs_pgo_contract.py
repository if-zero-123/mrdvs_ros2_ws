from pathlib import Path

import pytest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize(
    'relative_path',
    [
        'src/interface/package.xml',
        'src/interface/srv/SaveMaps.srv',
        'src/pgo/package.xml',
        'src/pgo/LICENSE',
        'src/pgo/src/pgo_node.cpp',
        'src/pgo/src/pgos/simple_pgo.cpp',
    ],
)
def test_upstream_pgo_package_layout_is_present(relative_path):
    path = WORKSPACE_ROOT / relative_path
    assert path.is_file(), f'missing upstream PGO file: {relative_path}'
