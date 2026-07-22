from pathlib import Path
import re


PROJECT_ROOT = Path(__file__).resolve().parents[1]
STATIC = PROJECT_ROOT / "src" / "mrdvs_web_console" / "static"


def read(path: str) -> str:
    return (STATIC / path).read_text(encoding="utf-8")


def test_static_assets_never_reference_external_urls():
    paths = [
        *STATIC.rglob("*.html"),
        *STATIC.rglob("*.css"),
        *(STATIC / "js").rglob("*.js"),
    ]
    assert paths
    text = "\n".join(path.read_text(encoding="utf-8", errors="ignore") for path in paths)
    assert "https://" not in text
    assert "http://" not in text


def test_index_contains_required_mobile_sections_and_controls():
    html = read("index.html")
    for element_id in [
        "system-status",
        "bag-name",
        "record-with-driver",
        "driver-control",
        "pointcloud-view",
        "imu-view",
        "bag-list",
        "settings-form",
        "log-view",
    ]:
        assert f'id="{element_id}"' in html
    assert 'name="viewport"' in html
    assert 'maxlength="80"' in html


def test_frontend_uses_only_local_vendor_assets():
    html = read("index.html")
    assert "/static/vendor/chart.umd.js" in html
    assert "/static/js/app.js" in html
    assert (STATIC / "vendor" / "three.module.min.js").stat().st_size > 100_000
    assert (STATIC / "vendor" / "chart.umd.js").stat().st_size > 100_000


def test_vendored_modules_include_all_relative_imports():
    vendor = STATIC / "vendor"
    for module in vendor.glob("*.js"):
        source = module.read_text(encoding="utf-8", errors="ignore")
        for relative_path in re.findall(r'from["\'](\./[^"\']+)["\']', source):
            assert (module.parent / relative_path).is_file(), (
                f"{module.name} 引用了缺失的离线模块 {relative_path}"
            )


def test_javascript_matches_api_and_binary_contracts():
    api = read("js/api.js")
    pointcloud = read("js/pointcloud.js")
    app = read("js/app.js")
    for endpoint in [
        "/api/status",
        "/api/driver/start",
        "/api/recording/start",
        "/api/bags",
        "/api/settings/autostart",
        "/ws/pointcloud",
        "/ws/imu",
    ]:
        assert endpoint in api + app
    assert "MPC1" in pointcloud
    assert "8 + count * 16" in pointcloud
    assert "geometry.dispose()" in pointcloud


def test_mobile_css_has_touch_targets_and_single_column_breakpoint():
    css = read("styles.css")
    assert "min-height: 44px" in css
    assert "@media (max-width: 760px)" in css
    assert "grid-template-columns: 1fr" in css


def test_settings_page_warns_before_disabling_autostart():
    app = read("js/app.js")
    assert "sudo systemctl enable --now mrdvs-collector.target" in app
    assert "下次开机" in app
