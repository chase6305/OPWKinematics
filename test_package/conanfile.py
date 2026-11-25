import os
import shutil
import sys

from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class OPWKinematicsTestConan(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    test_type = "explicit"

    def requirements(self):
        self.requires(self.tested_reference_str)

    def layout(self):
        cmake_layout(self)

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def test(self):
        if can_run(self):
            executable = "opw_consumer.exe" if self.settings.os == "Windows" else "opw_consumer"
            self.run(f'"{os.path.join(self.cpp.build.bindir, executable)}"', env="conanrun")
            package = self.dependencies["opw-kinematics"]
            if package.options.with_urdf_tool:
                tool = "opw_from_urdf.exe" if self.settings.os == "Windows" else "opw_from_urdf"
                self.run(f'"{os.path.join(package.package_folder, "bin", tool)}" --help', env="conanrun")
            if package.options.with_python:
                python = os.environ.get("OPW_PYTHON_EXECUTABLE") or shutil.which("python3") or sys.executable
                self.run(f'"{python}" -c "import opw_kinematics"', env="conanrun")
