from azrael_manager.parameter_types import  ParameterTypes
from azrael_manager.interface import Interface
from azrael_manager_msgs.srv import InvokeService
import subprocess

class UrBringup(Interface):
    def __init__(self):
        super().__init__()

        self._name_service = "ur_bringup"

        self._parameter_definitions = {
            "fake"      : ParameterTypes.BOOL,
            "gripper"   : ParameterTypes.STRING
        }

        self.AVAILABLE_GRIPPERS = ['None', 'robotiq-2f-85', 'robotiq-2f-140']

    def on_configure(self) -> None:
        pass

    def run(self) -> subprocess.Popen:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'azrael_app', 'ur_bringup.launch.py', f"fake_ur:={self._last_call_parameters['fake']}", f"gripper:={self._last_call_parameters['gripper']}"]
        )

        return proc

    def on_validate_parameters(self, pars: dict) -> tuple[int, str]:
        self.logger.debug(f"pars fed: {[pars]}")
        if 'gripper' not in pars:
            pars['gripper'] = 'None'
        elif pars['gripper'] not in self.AVAILABLE_GRIPPERS:
            return (InvokeService.Response.FAILED, f"Gripper [{pars['gripper']}] not available")

        if 'fake' not in pars:
            pars['fake'] = True


        self.logger.debug(f"pars after validation: {[pars]}")

        return (InvokeService.Response.SUCCESS, "No problem")
