from azrael_manager_msgs.srv import InvokeService, GetParameterList
from azrael_manager.parameter_types import ParameterTypes
from abc import ABC, abstractmethod
import subprocess
import rclpy
import rclpy.logging
import ast

class Interface(ABC):

    def __init__(self):
        self._name_service: str = ""
        self._parameter_definitions: dict[str, dict] = {}
        self._last_call_parameters = {}
        self.logger = None

    def configure(self, logger) -> None:
        self.logger = logger
        self.on_configure()

    @abstractmethod
    def on_configure(self) -> None:
        pass

    @abstractmethod
    def on_validate_parameters(self, pars: dict) -> tuple[int, str]:
        pass

    @abstractmethod
    def run(self) -> subprocess.Popen:
        pass

    def list_parameters(self) -> tuple[list[str], list[str], list[str], int, str]:
        names = []
        types = []
        descs = []
        for p, d in self._parameter_definitions.items():
            names.append(p)
            types.append(ParameterTypes.as_string(d['type']))
            descs.append(d['description'])
        return names, types, descs, GetParameterList.Response.SUCCESS, "No problem"

    def get_name(self) -> str:
        return self._name_service

    def get_parameters_definition(self) -> dict[str, dict]:
        return self._parameter_definitions

    def validate_parameters(self, param_list: list[str]) -> tuple[int, str]:
        pars = {}
        p = ""
        for cnt, token in enumerate(param_list):
            if cnt % 2 == 0:
                # Is param name
                if token not in self._parameter_definitions:
                    return (InvokeService.Response.FAILED, f"Parameter {token} does not exist in service {self._name_service}")
                p = token
            else:
                # Is value
                t = self._parameter_definitions[p]['type']
                v = None
                if t == ParameterTypes.STRING:
                    v = token
                else:
                    try:
                        val = ast.literal_eval(token)
                    except SyntaxError:
                        return (InvokeService.Response.FAILED, f"Value [{token}] is not a suitable value")
                    except Exception as ex:
                        return (InvokeService.Response.FAILED, f"Error while evaluating [{token}]: {ex}")

                    if not (t == ParameterTypes.BOOL and type(val) is bool) and \
                       not (t == ParameterTypes.INT and type(val) is int) and \
                       not (t == ParameterTypes.FLOAT and type(val) is float) and \
                       not (t == ParameterTypes.BOOL_ARRAY and all(isinstance(k, bool) for k in val)) and \
                       not (t == ParameterTypes.INT_ARRAY and all(isinstance(k, int) for k in val)) and \
                       not (t == ParameterTypes.FLOAT_ARRAY and all(isinstance(k, float) for k in val)) and \
                       not (t == ParameterTypes.STRING_ARRAY and all(isinstance(k, str) for k in val)):
                           return (InvokeService.Response.FAILED, f"Value [{token}] of parameter [{p}] is of wrong type: {t}")
                    v = val

                pars[p] = v
                self.logger.debug(f"parameter: {'p'}, value: {'v'}")
                p = ""

        local_err = self.on_validate_parameters(pars)
        if local_err[0] != InvokeService.Response.SUCCESS:
            return local_err

        self._last_call_parameters = pars
        return (InvokeService.Response.SUCCESS, "No problem")
