from azrael_manager.interface import Interface
from azrael_manager.parameter_types import ParameterTypes
from azrael_manager_msgs.srv import InvokeService
import subprocess
import ipaddress

class ActivateMotorDriver(Interface):
    def __init__(self):
        self._name_service = "activate_motor_driver"
        self._parameter_definitions = {}

    def on_configure(self) -> None:
        pass

    def on_validate_parameters(self, pars: dict) -> tuple[int, str]:
        return (InvokeService.Response.SUCCESS, "No problem")

    def run(self) -> subprocess.Popen:
        proc = subprocess.Popen(
            ['ssh', 'azrael_raspy_motor', 'sudo azrael_base_driver/build/azrael_mobile_driver']
        )

        return proc
