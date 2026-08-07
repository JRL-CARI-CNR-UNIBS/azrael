from azrael_manager.parameter_types import  ParameterTypes
from azrael_manager.interface import Interface
from azrael_manager_msgs.srv import InvokeService
import subprocess

class BaseBringup(Interface):
    def __init__(self):
        super().__init__()

        self._name_service = "base_bringup"

        self._parameter_definitions = {
        }


    def on_configure(self) -> None:
        pass

    def run(self) -> subprocess.Popen:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'azrael_app', 'base_bringup.launch.yml']
        )

        return proc

    def on_validate_parameters(self, pars: dict) -> tuple[int, str]:
        return (InvokeService.Response.SUCCESS, "No problem")
