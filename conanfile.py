import os
import shutil
import subprocess
import sys

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class OPWKinematicsConan(ConanFile):
    name = "opw-kinematics"
    version = "0.1.0"
    package_type = "library"
    description = "Analytic forward and inverse kinematics for six-axis OPW robots"
    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "with_urdf_tool": [True, False],
        "with_python": [True, False],
        "python_abi": ["ANY"],
    }
    default_options = {
        "shared": False, "fPIC": True, "with_urdf_tool": False,
        "with_python": False, "python_abi": "auto",
    }
    exports_sources = (
        "CMakeLists.txt", "cmake/*", "include/*", "src/*", "tests/*", "tools/*", "python/*",
    )

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared or self.options.with_python:
            self.options.rm_safe("fPIC")
        if not self.options.with_python:
            self.options.rm_safe("python_abi")

    def _python_target(self):
        if not hasattr(self, "_opw_python"):
            executable = os.environ.get("OPW_PYTHON_EXECUTABLE") or shutil.which("python3") or sys.executable
            try:
                version, abi = subprocess.check_output(
                    [executable, "-c", "import sys, sysconfig; "
                     "print('%d.%d' % sys.version_info[:2]); "
                     "print(sysconfig.get_config_var('SOABI') or sys.implementation.cache_tag)"],
                    text=True, timeout=10,
                ).splitlines()
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                raise ConanInvalidConfiguration(f"Cannot inspect Python interpreter {executable}") from error
            if not abi.startswith("cpython-") or tuple(map(int, version.split("."))) < (3, 8):
                raise ConanInvalidConfiguration("Python bindings require CPython 3.8 or newer")
            self._opw_python = executable, abi
        return self._opw_python

    def validate(self):
        if self.settings.get_safe("compiler.cppstd"):
            check_min_cppstd(self, "17")
        if self.options.with_python:
            _, abi = self._python_target()
            if str(self.options.python_abi) not in ("auto", abi):
                raise ConanInvalidConfiguration(f"python_abi must match the selected interpreter ({abi})")

    def requirements(self):
        self.requires("eigen/3.4.0", transitive_headers=True)
        if self.options.with_urdf_tool:
            self.requires("urdfdom/4.0.0", transitive_headers=False, transitive_libs=False)
        if self.options.with_python:
            self.requires("pybind11/3.0.1", transitive_headers=False)

    def package_id(self):
        if self.info.options.with_python:
            self.info.options.python_abi = self._python_target()[1]

    def layout(self):
        cmake_layout(self)

    def generate(self):
        CMakeDeps(self).generate()
        toolchain = CMakeToolchain(self)
        toolchain.cache_variables["BUILD_TESTING"] = False
        toolchain.cache_variables["OPW_BUILD_URDF_TOOL"] = self.options.with_urdf_tool
        toolchain.cache_variables["OPW_BUILD_PYTHON"] = self.options.with_python
        if self.options.with_python:
            toolchain.cache_variables["Python3_EXECUTABLE"] = self._python_target()[0]
        toolchain.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        CMake(self).install()

    def package_info(self):
        self.cpp_info.libs = ["OPWKinematics"]
        self.cpp_info.set_property("cmake_file_name", "OPWKinematics")
        self.cpp_info.set_property("cmake_target_name", "OPWKinematics::OPWKinematics")
        if self.options.with_python:
            python_dir = os.path.join(self.package_folder, self.cpp_info.libdirs[0], "python")
            self.runenv_info.prepend_path("PYTHONPATH", python_dir)
