"""
Setup script for DSF Python bindings
This script uses setuptools to build the C++ core of DSF with Python bindings
using pybind11 and CMake.
It extracts the version from the C++ header file and configures the build
process accordingly.
"""

import os
from pathlib import Path
import platform
import re
import subprocess
import sys

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


def get_version_from_header():
    """Extract version from C++ header file.

    Raises RuntimeError if version cannot be extracted, unless
    DSF_PACKAGE_VERSION env var is set (CI builds) or
    DSF_ALLOW_MISSING_VERSION is set to allow local dev fallback.
    """
    header_path = Path(__file__).parent / "src" / "dsf" / "dsf.hpp"
    try:
        with open(header_path, "r", encoding="UTF-8") as header_file:
            content = header_file.read()

        major_match = re.search(r"DSF_VERSION_MAJOR = (\d+)", content)
        minor_match = re.search(r"DSF_VERSION_MINOR = (\d+)", content)
        patch_match = re.search(r"DSF_VERSION_PATCH = (\d+)", content)

        if major_match and minor_match and patch_match:
            return (
                f"{major_match.group(1)}.{minor_match.group(1)}.{patch_match.group(1)}"
            )

        # Version regex failed to match
        error_msg = (
            f"Failed to extract version from {header_path}. "
            "Expected DSF_VERSION_MAJOR, DSF_VERSION_MINOR, DSF_VERSION_PATCH defines."
        )
        if os.environ.get("DSF_ALLOW_MISSING_VERSION"):
            print(f"WARNING: {error_msg}. Using 0.0.0.dev0 for local build.")
            return "0.0.0.dev0"
        raise RuntimeError(error_msg)
    except FileNotFoundError as e:
        error_msg = f"Version header file not found: {header_path}"
        if os.environ.get("DSF_ALLOW_MISSING_VERSION"):
            print(f"WARNING: {error_msg}. Using 0.0.0.dev0 for local build.")
            return "0.0.0.dev0"
        raise RuntimeError(error_msg) from e


class CMakeExtension(Extension):  # pylint: disable=too-few-public-methods
    """Custom CMake extension class for setuptools"""

    def __init__(self, name: str, sourcedir: str = ""):
        super().__init__(name, sources=[])
        self.sourcedir = Path(sourcedir).resolve()


class CMakeBuild(build_ext):
    """Custom build_ext command to handle CMake extensions"""

    def run(self):
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError as exc:
            raise RuntimeError(
                "CMake must be installed to build the extensions"
            ) from exc

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext: CMakeExtension):
        extdir = Path(self.get_ext_fullpath(ext.name)).parent.resolve()
        cfg = "Release"
        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            "-DBUILD_PYTHON_BINDINGS=ON",
        ]

        if "CMAKE_ARGS" in os.environ:
            cmake_args += os.environ["CMAKE_ARGS"].split()

        if platform.system() == "Windows":
            cmake_args += [f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}"]
            if "CMAKE_TOOLCHAIN_FILE" in os.environ:
                cmake_args.append(
                    f"-DCMAKE_TOOLCHAIN_FILE={os.environ['CMAKE_TOOLCHAIN_FILE']}"
                )
            if "VCPKG_TARGET_TRIPLET" in os.environ:
                cmake_args.append(
                    f"-DVCPKG_TARGET_TRIPLET={os.environ['VCPKG_TARGET_TRIPLET']}"
                )

        # Add macOS-specific CMake prefix paths for Homebrew dependencies
        if platform.system() == "Darwin":  # macOS
            try:
                fmt_prefix = subprocess.check_output(
                    ["brew", "--prefix", "fmt"], text=True
                ).strip()
                spdlog_prefix = subprocess.check_output(
                    ["brew", "--prefix", "spdlog"], text=True
                ).strip()

                cmake_prefix_path = f"{fmt_prefix};{spdlog_prefix}"
                cmake_args.append(f"-DCMAKE_PREFIX_PATH={cmake_prefix_path}")
                print(f"Added macOS Homebrew prefix paths: {cmake_prefix_path}")

            except (subprocess.CalledProcessError, FileNotFoundError):
                print(
                    "Warning: Could not determine Homebrew prefix paths. Make sure Homebrew is installed and dependencies are available."
                )
                # Fallback to common Homebrew paths
                cmake_args.append("-DCMAKE_PREFIX_PATH=/opt/homebrew;/usr/local")

        build_args = []

        # Use Ninja if available in the current environment, otherwise use Unix Makefiles
        use_ninja = False
        try:
            subprocess.check_output(["ninja", "--version"])
            use_ninja = True
        except (OSError, subprocess.CalledProcessError):
            use_ninja = False
        if platform.system() != "Windows":
            if use_ninja:
                cmake_args += ["-G", "Ninja"]
            else:
                cmake_args += ["-G", "Unix Makefiles"]

        subprocess.check_call(["cmake", ext.sourcedir] + cmake_args, cwd=build_temp)
        subprocess.check_call(
            ["cmake", "--build", ".", "--config", cfg, "--verbose"] + build_args,
            cwd=build_temp,
        )


# Get version from header file, unless explicitly overridden for CI pre-releases.
PROJECT_VERSION = os.environ.get("DSF_PACKAGE_VERSION", get_version_from_header())


setup(
    version=PROJECT_VERSION,
    ext_modules=[CMakeExtension("dsf_cpp")],
    cmdclass={"build_ext": CMakeBuild},
)
