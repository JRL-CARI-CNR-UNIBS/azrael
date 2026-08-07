from rclpy.node import Node
import rclpy

from azrael_manager.interface import Interface
from azrael_manager_msgs.srv import InvokeService

import subprocess
import importlib
import pathlib
import yaml
import argparse

from ament_index_python import get_package_share_path

class AzraelMasterNode(Node):
    def __init__(self, fconfig):
        super().__init__("__azrael_manager__")

        self.invoke_launcher = self.create_service(srv_type=InvokeService, srv_name="/azrael/system/invoke_service", callback=self.handle_service)
        self._defined_modules_and_classes: dict[str, list[str]] = {}
        self._available_services: dict[str, Interface] = {}
        self._procs: dict[str, subprocess.Popen] = {}

        self.setup(fconfig)

    def handle_service(self, req: InvokeService.Request, res: InvokeService.Response):
        if req.service not in self._available_services:
            self.get_logger().error(f'Service requested [{req.service}] is not available')
        selected = self._available_services[req.service]
        selected.validate_parameters(req.parameters)
        self._procs[req.service] = selected.run()

    def add_service(self, module_name: str, class_name: str) -> None:
        modl = importlib.import_module(module_name)
        clss = getattr(modl, class_name)

        if not issubclass(clss, Interface):
            self.get_logger().error(f'Class {clss} of module {modl} is not a valid Interface implementation')
            return

        srv = clss()
        srv.configure()
        self._available_services[srv.get_name()] = srv

        return

    def setup(self, fconfig) -> bool:
        try:
            pathlib.Path(fconfig).resolve()
        except (OSError, RuntimeError):
            self.get_logger().fatal(f'Config file path is not valid!')
            return False

        # Read required modules and classes from yaml
        with open(fconfig, 'r') as f:
            yf = yaml.safe_load(f)
            if yf['services']:
                for lt in yf['services']:
                    if lt['module'] and lt['classes']:
                        self._defined_modules_and_classes[lt['module']] = lt['classes']
                        self.get_logger().info(f"Defined: module {lt['module']}, classes \n{lt['classes']}")

        for k,v in self._defined_modules_and_classes.items():
            for c in v:
                self.add_service(k, c)

        return True


def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    args = parser.parse_args()

    config_path = get_package_share_path('azrael_manager') / 'config' / 'modules.yaml'
    if args.config:
        config_path = args.config

    node = AzraelMasterNode(config_path)

    rclpy.spin(node)

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
