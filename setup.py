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
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

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
        self.pre_build()
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError as exc:
            raise RuntimeError(
                "CMake must be installed to build the extensions"
            ) from exc

        for ext in self.extensions:
            self.build_extension(ext)

        self.run_stubgen()

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

        # Pass DSF_HPC_BUILD environment variable to CMake for HPC-compatible builds
        hpc_build = os.environ.get("DSF_HPC_BUILD", "0")
        if hpc_build in ("1", "true", "TRUE", "on", "ON"):
            cmake_args.append("-DDSF_HPC_BUILD=ON")
            print("HPC Build Mode enabled: using conservative -O3 optimization")

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

    def pre_build(self):
        """Extracts doxygen documentation from XML files and creates a C++ unordered_map"""

        try:
            subprocess.run(["doxygen", "Doxyfile"], check=True)
        except FileNotFoundError as exc:
            raise RuntimeError(
                "Doxygen is not installed or not found in PATH. Please install Doxygen to build documentation."
            ) from exc
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(
                "Doxygen failed to run. Ensure that 'Doxyfile' exists and is valid."
            ) from exc
        docs = {}
        DOXYGEN_XML_DIR = "xml"

        def extract_param_info(member):
            """Extract parameter information from a memberdef element."""
            params = []
            for param in member.findall(".//param"):
                param_type = param.find("type")
                param_name = param.find("declname")

                type_text = ""
                if param_type is not None:
                    # Handle complex types with references
                    type_parts = []
                    if param_type.text:
                        type_parts.append(param_type.text)
                    for ref in param_type.findall("ref"):
                        if ref.text:
                            type_parts.append(ref.text)
                    if param_type.tail:
                        type_parts.append(param_type.tail)
                    type_text = "".join(type_parts).strip()

                name_text = param_name.text if param_name is not None else ""

                if type_text or name_text:
                    params.append(f"{type_text} {name_text}".strip())

            return params

        def extract_param_docs(member):
            """Extract parameter documentation from detailed description."""
            param_docs = {}
            detailed_desc = member.find("detaileddescription")
            if detailed_desc is not None:
                for param_list in detailed_desc.findall(
                    ".//parameterlist[@kind='param']"
                ):
                    for param_item in param_list.findall("parameteritem"):
                        param_name_list = param_item.find("parameternamelist")
                        param_desc = param_item.find("parameterdescription")

                        if param_name_list is not None and param_desc is not None:
                            param_name = param_name_list.find("parametername")
                            if param_name is not None and param_name.text:
                                desc_para = param_desc.find("para")
                                desc_text = (
                                    desc_para.text
                                    if desc_para is not None and desc_para.text
                                    else ""
                                )
                                param_docs[param_name.text] = desc_text

            return param_docs

        def extract_return_info(member):
            """Extract return type and documentation."""
            return_type = ""
            return_doc = ""

            # Extract return type
            type_elem = member.find("type")
            if type_elem is not None:
                type_parts = []
                if type_elem.text:
                    type_parts.append(type_elem.text)
                for ref in type_elem.findall("ref"):
                    if ref.text:
                        type_parts.append(ref.text)
                if type_elem.tail:
                    type_parts.append(type_elem.tail)
                return_type = "".join(type_parts).strip()

            # Extract return documentation
            detailed_desc = member.find("detaileddescription")
            if detailed_desc is not None:
                for return_elem in detailed_desc.findall(
                    ".//simplesect[@kind='return']"
                ):
                    para = return_elem.find("para")
                    if para is not None and para.text:
                        return_doc = para.text
                        break

            return return_type, return_doc

        def format_documentation_entry(
            name,
            brief,
            detailed,
            params=None,
            param_docs=None,
            return_type="",
            return_doc="",
        ):
            """Format a documentation entry with Description, Args, and Returns sections."""
            # Description section
            description = []
            if brief:
                description.append(brief)
            if detailed and detailed != brief:
                description.append(detailed)

            doc_parts = []

            # Description
            desc_text = "\n".join(description).strip()
            if desc_text:
                doc_parts.append(f"Description\n{desc_text}")
            else:
                doc_parts.append("Description\nNo description available.")

            # Args section
            if params:
                args_section = ["Args"]
                if param_docs:
                    for param in params:
                        param_name = param.split()[-1] if param else ""
                        param_doc = param_docs.get(param_name, "No description")
                        args_section.append(f"  {param}: {param_doc}")
                else:
                    for param in params:
                        args_section.append(f"  {param}: No description")
                doc_parts.append("\n".join(args_section))
            else:
                doc_parts.append("Args\n  None")

            # Returns section
            returns_section = ["Returns"]
            if return_type:
                if return_doc:
                    returns_section.append(f"  {return_type}: {return_doc}")
                else:
                    returns_section.append(f"  {return_type}: No description")
            else:
                returns_section.append("  void: No return value")

            doc_parts.append("\n".join(returns_section))

            return "\n\n".join(doc_parts)

        # Main parsing function
        for file_path in Path(DOXYGEN_XML_DIR).iterdir():
            if (
                file_path.name.startswith("class")
                or file_path.name.startswith("namespace")
                or file_path.name.startswith("struct")
            ):
                tree = ET.parse(file_path)
                root = tree.getroot()

                for compound in root.findall(".//compounddef"):
                    name = compound.find("compoundname").text
                    brief = compound.find("briefdescription").findtext(
                        "para", default=""
                    )
                    detailed = compound.find("detaileddescription").findtext(
                        "para", default=""
                    )

                    # Format compound documentation
                    docs[name] = format_documentation_entry(name, brief, detailed)

                    # Process member functions/variables
                    for member in compound.findall(".//memberdef"):
                        member_name = member.find("name").text
                        member_brief = member.find("briefdescription").findtext(
                            "para", default=""
                        )
                        member_detailed = member.find("detaileddescription").findtext(
                            "para", default=""
                        )

                        # Extract function-specific information
                        if member.get("kind") == "function":
                            # Extract parameters
                            params = extract_param_info(member)
                            param_docs = extract_param_docs(member)

                            # Extract return information
                            return_type, return_doc = extract_return_info(member)

                            # Format with full documentation structure
                            docs[f"{name}::{member_name}"] = format_documentation_entry(
                                f"{name}::{member_name}",
                                member_brief,
                                member_detailed,
                                params,
                                param_docs,
                                return_type,
                                return_doc,
                            )
                        else:
                            # For non-function members (variables, etc.)
                            docs[f"{name}::{member_name}"] = format_documentation_entry(
                                f"{name}::{member_name}", member_brief, member_detailed
                            )

        with open("./src/dsf/.docstrings.hpp", "w", encoding="UTF-8") as f:
            f.write("#pragma once\n\n#include <unordered_map>\n#include <string>\n\n")
            f.write("namespace dsf {\n")
            f.write(
                "    const std::unordered_map<std::string, std::string> g_docstrings = {\n"
            )
            for k, v in docs.items():
                f.write(f'        {{"{k}", R"""({v})"""}},\n')
            f.write("    };\n")
            f.write("}\n")

    def run_stubgen(self):
        """Generate stub files for the Python bindings"""
        print("Starting stub generation...")

        # Find the built extension module
        ext_path = None
        for ext in self.extensions:
            ext_path = self.get_ext_fullpath(ext.name)
            print(f"Extension path: {ext_path}")
            break

        if not ext_path:
            print("Warning: No extension path found, skipping stub generation")
            return

        # Check both the full path and build lib location
        module_dir = Path(ext_path).parent
        build_lib_path = Path(self.build_lib) / "dsf_cpp.so"

        print(f"Checking extension at: {ext_path}")
        print(f"Checking build lib at: {build_lib_path}")
        print(f"Module directory: {module_dir}")

        # Use build lib directory for stub generation
        stub_output_dir = self.build_lib

        # Set up environment with proper Python path
        env = os.environ.copy()
        env["PYTHONPATH"] = self.build_lib + os.pathsep + env.get("PYTHONPATH", "")
        print(f"PYTHONPATH: {env['PYTHONPATH']}")

        try:
            # Generate stub files
            cmd = [
                "pybind11-stubgen",
                "dsf_cpp",
                "--ignore-invalid-expressions",
                "std::function|dsf::RoadDynamics",
                "--enum-class-locations",
                "TrafficLightOptimization:dsf_cpp",
                "--output-dir",
                stub_output_dir,
            ]
            print(f"Running command: {' '.join(cmd)}")

            result = subprocess.run(
                cmd, check=True, env=env, capture_output=True, text=True
            )
            print("Stub generation completed successfully")
            print(f"stdout: {result.stdout}")

            # Check if stub file or package directory was created
            stub_file = Path(stub_output_dir) / "dsf_cpp.pyi"
            package_stub_dir = Path(stub_output_dir) / "dsf_cpp"

            source_pkg_dir = Path(__file__).parent / "src" / "dsf"
            source_stub = source_pkg_dir / "__init__.pyi"

            if stub_file.exists():
                print(f"Stub file successfully created at {stub_file}")
                # For editable installs, also copy to source directory for development
                if source_stub != stub_file:
                    print(f"Copying stub file to package: {source_stub}")
                    shutil.copy2(stub_file, source_stub)
            elif package_stub_dir.exists():
                # pybind11-stubgen may emit a package directory with multiple .pyi files
                init_stub = package_stub_dir / "__init__.pyi"
                if init_stub.exists():
                    print(
                        f"Stub package directory found at {package_stub_dir}, copying __init__.pyi to {source_stub}"
                    )
                    source_pkg_dir.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(init_stub, source_stub)
                else:
                    print(
                        f"Stub package directory found at {package_stub_dir} but no __init__.pyi present"
                    )

                # Also copy any other .pyi files (module-level stubs) into the package source dir
                for pyi in package_stub_dir.glob("*.pyi"):
                    dest = source_pkg_dir / pyi.name
                    if pyi.name == "__init__.pyi":
                        continue
                    print(f"Copying {pyi} -> {dest}")
                    shutil.copy2(pyi, dest)
            else:
                print(
                    f"Warning: Stub file not found at {stub_file} and no package dir at {package_stub_dir}"
                )

            # Copy all .pyi files from source to build_lib so they're included in the wheel
            build_lib_dsf = Path(self.build_lib) / "dsf"
            build_lib_dsf.mkdir(parents=True, exist_ok=True)
            for pyi_file in source_pkg_dir.glob("**/*.pyi"):
                rel_path = pyi_file.relative_to(source_pkg_dir)
                dest_file = build_lib_dsf / rel_path
                dest_file.parent.mkdir(parents=True, exist_ok=True)
                print(f"Copying stub to build_lib: {pyi_file} -> {dest_file}")
                shutil.copy2(pyi_file, dest_file)

        except subprocess.CalledProcessError as e:
            print(f"Warning: Stub generation failed: {e}")
            print(f"stdout: {e.stdout}")
            print(f"stderr: {e.stderr}")
            # Don't fail the build if stub generation fails

        # Always copy existing .pyi files from source to build_lib (even if stubgen failed)
        # This ensures pre-existing stubs like mdt.pyi and mobility.pyi are included in the wheel
        source_pkg_dir = Path(__file__).parent / "src" / "dsf"
        build_lib_dsf = Path(self.build_lib) / "dsf"
        build_lib_dsf.mkdir(parents=True, exist_ok=True)
        for pyi_file in source_pkg_dir.glob("**/*.pyi"):
            rel_path = pyi_file.relative_to(source_pkg_dir)
            dest_file = build_lib_dsf / rel_path
            dest_file.parent.mkdir(parents=True, exist_ok=True)
            if not dest_file.exists():
                print(f"Copying existing stub to build_lib: {pyi_file} -> {dest_file}")
                shutil.copy2(pyi_file, dest_file)

        # Copy py.typed marker for PEP 561 compliance
        py_typed_src = source_pkg_dir / "py.typed"
        py_typed_dest = build_lib_dsf / "py.typed"
        if py_typed_src.exists() and not py_typed_dest.exists():
            print(f"Copying py.typed marker to build_lib: {py_typed_dest}")
            shutil.copy2(py_typed_src, py_typed_dest)


# Get version from header file, unless explicitly overridden for CI pre-releases.
PROJECT_VERSION = os.environ.get("DSF_PACKAGE_VERSION", get_version_from_header())


setup(
    version=PROJECT_VERSION,
    ext_modules=[CMakeExtension("dsf_cpp")],
    cmdclass={"build_ext": CMakeBuild},
)
