import unicodedata
from pathlib import Path


class BagNameError(ValueError):
    """Raised when a bag name cannot safely map to one child directory."""


def validate_bag_name(raw: str) -> str:
    name = unicodedata.normalize("NFC", raw.strip())
    if not name or len(name) > 80:
        raise BagNameError("数据包名称长度必须为 1 到 80 个字符")
    if name in {".", ".."} or any(character in name for character in "/\\"):
        raise BagNameError("数据包名称不能包含路径字符")
    if not all(character.isalnum() or character in "-_" for character in name):
        raise BagNameError("数据包名称只能包含中文、字母、数字、短横线和下划线")
    return name


def resolve_bag_path(root: Path, raw_name: str) -> Path:
    root_resolved = root.expanduser().resolve()
    candidate = (root_resolved / validate_bag_name(raw_name)).resolve()
    if candidate.parent != root_resolved:
        raise BagNameError("数据包路径超出保存目录")
    return candidate
