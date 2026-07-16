from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_lidar_launch_and_driver_require_optical_xyz_coordinates():
    launch_source = (PACKAGE_ROOT / 'launch/lx_lidar_ros.launch.py').read_text()
    header_source = (PACKAGE_ROOT / 'src/lx_camera/lx_camera.h').read_text()
    driver_source = (PACKAGE_ROOT / 'src/lx_camera/lx_camera.cpp').read_text()

    assert '{"LX_INT_XYZ_COORDINATE": 0}' in launch_source
    assert 'int expected_xyz_coordinate_ = -1;' in header_source
    assert 'set_critical_int(LX_INT_XYZ_COORDINATE,' in driver_source
    assert 'VerifyCriticalIntParameter(LX_INT_XYZ_COORDINATE,' in driver_source
