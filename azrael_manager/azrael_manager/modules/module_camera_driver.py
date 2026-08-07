from azrael_manager.interface import Interface
from azrael_manager.parameter_types import ParameterTypes
from azrael_manager_msgs.srv import InvokeService
import subprocess
import ipaddress

class ActivateCameraDriver(Interface):
    def __init__(self):
        self._name_service = "activate_camera_driver"
        self._parameter_definitions = {}

    def on_configure(self) -> None:
        pass

    def on_validate_parameters(self, pars: dict) -> tuple[int, str]:
        return (InvokeService.Response.SUCCESS, "No problem")

    def run(self) -> subprocess.Popen:
        proc = subprocess.Popen(
            ['ssh', 'azrael_raspy_camera', '"ros2 launch realsense2_camera rs_launch.py   enable_depth:=false   enable_color:=true   enable_infra1:=false   enable_infra2:=false   enable_gyro:=true   enable_accel:=true   unite_imu_method:=2 rgb_camera.color_profile:=640x480x15"']
        )

        return proc
