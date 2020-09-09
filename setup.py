import platform
import subprocess
from typing import List
from pathlib import Path
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    """
    Custom setuptools extension that configures a CMake project.
    """

    def __init__(self,
                 name: str,
                 cmake_configure_options: List[str] = (),
                 source_dir: str = str(Path(".").absolute()),
                 cmake_build_type: str = "Release"):

        super().__init__(name=name, sources=[])

        if not Path(source_dir).absolute().is_dir():
            raise ValueError(f"Directory '{source_dir}' does not exist")

        self.cmake_build_type = cmake_build_type
        self.source_dir = str(Path(source_dir).absolute())
        self.cmake_configure_options = cmake_configure_options


class BuildExtension(build_ext):
    """
    Setuptools build extension handler.
    It processes all the extensions listed in the 'ext_modules' entry.
    """

    def run(self) -> None:
        """
        Process all the registered extensions executing only the CMakeExtension objects.
        """

        # Filter the CMakeExtension objects
        cmake_extensions = [e for e in self.extensions if isinstance(e, CMakeExtension)]

        if len(cmake_extensions) == 0:
            raise ValueError("No CMakeExtension objects found")

        # Check that CMake is installed
        self._raise_if_not_installed(command="cmake")

        # Check that Ninja is installed
        self._raise_if_not_installed(command="ninja")

        if platform.system() != "Linux":
            raise RuntimeError("Only Linux is currently supported")

        for ext in cmake_extensions:
            self.build_extension(ext)

    @staticmethod
    def _raise_if_not_installed(command: str) -> None:

        try:
            _ = subprocess.check_output(['which', command])
        except subprocess.CalledProcessError:
            raise RuntimeError(f"Required command '{command}' not found")

    def build_extension(self, ext: CMakeExtension) -> None:
        """
        Build a CMakeExtension object.

        Args:
            ext: The CMakeExtension object to build.
        """

        # Disable editable installation (pip install -e .)
        if self.inplace:
            raise RuntimeError("Editable installation is not supported")

        # In normal installation, the ext_dir is the folder that is then compressed to
        # make the archive. Once installed, it represents the site-package directory.
        # Instead, in editable installation, the ext_dir is the directory that contains
        # the in-source Python package. In this case, the CMake project would be installed
        # in-source, and this is the reason why the editable installation is disabled.
        ext_dir = Path(self.get_ext_fullpath(ext.name)).parent.absolute()
        cmake_install_prefix = ext_dir / "idyntree"

        # CMake configure arguments
        configure_args = [
            f"-GNinja",
            f"-DCMAKE_INSTALL_PREFIX:PATH={cmake_install_prefix}",
        ]

        # Extend the configure arguments with those passed from the extension
        configure_args += ext.cmake_configure_options

        # CMake build arguments
        build_args = [
            '--config', ext.cmake_build_type
        ]

        # TODO: Handle differences in OS and CMake generators
        # Refer to https://github.com/pybind/cmake_example/blob/master/setup.py
        if platform.system() == "Windows":
            raise NotImplementedError("Platform not yet supported")

        elif platform.system() == "Darwin":
            raise NotImplementedError("Platform not yet supported")

        elif platform.system() == "Linux":

            configure_args += [
                f"-DCMAKE_BUILD_TYPE={ext.cmake_build_type}",
            ]

            install_target = "install"

        else:
            raise RuntimeError(f"Unsupported '{platform.system()}' platform")

        # Get the absolute path to the build folder
        build_folder = str(Path('.').absolute() / self.build_temp)

        # Make sure that the build folder exists
        Path(build_folder).mkdir(exist_ok=True, parents=True)

        # Commands to be executed
        configure_command = ['cmake', '-S', ext.source_dir, '-B', build_folder] \
                            + configure_args
        build_command = ['cmake', '--build', build_folder] + build_args
        install_command = ['cmake', '--build', build_folder, '--target', install_target]

        print(f"")
        print(f"==> Configuring:")
        print(f"$ {' '.join(configure_command)}")
        print(f"")
        print(f"==> Building:")
        print(f"$ {' '.join(build_command)}")
        print(f"")
        print(f"==> Installing:")
        print(f"$ {' '.join(install_command)}")
        print("")

        # Call CMake
        subprocess.check_call(configure_command)
        subprocess.check_call(build_command)
        subprocess.check_call(install_command)


# Read the contents of your README file
with open(Path(__file__).parent.absolute() / 'README.md', encoding='utf-8') as f:
    long_description = f.read()


setup(
    name='idyntree',
    description="Multibody Dynamics Library designed for Free Floating Robots",
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/robotology/idyntree',
    keywords="robotics dynamics free-floating floating-base robot robotic-library multibody-dynamics",
    license="LGPL",
    platforms=['linux'],
    classifiers=[
        "Development Status :: 6 - Mature",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering",
        "Framework :: Robot Framework",
        "Intended Audience :: Education",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3 :: Only",
        "License :: OSI Approved :: GNU Lesser General Public License v2 or later (LGPLv2+)",
    ],
    use_scm_version=dict(local_scheme="dirty-tag"),
    python_requires='>=3.6',
    install_requires=[
        "numpy",
    ],
    ext_modules=[
        CMakeExtension(name='idyntree',
                       source_dir=str(Path(".").absolute()),
                       cmake_configure_options=[
                           "-DBUILD_SHARED_LIBS:BOOL=OFF",
                           "-DIDYNTREE_USES_PYTHON:BOOL=ON",
                           "-DIDYNTREE_COMPILE_TESTS:BOOL=OFF",
                           "-DIDYNTREE_PACKAGE_FOR_PYPI:BOOL=ON",
                           "-DIDYNTREE_USES_IPOPT:BOOL=ON",
                           "-DIDYNTREE_USES_ASSIMP:BOOL=ON",
                           "-DIDYNTREE_USES_IRRLICHT:BOOL=OFF",
                           "-DIDYNTREE_USES_QT5:BOOL=OFF",
                           "-DIDYNTREE_USES_OSQPEIGEN:BOOL=OFF",
                           "-DIDYNTREE_USES_ALGLIB:BOOL=OFF",
                           "-DIDYNTREE_USES_WORHP:BOOL=OFF",
                           "-DIDYNTREE_USES_YARP:BOOL=OFF",
                           "-DIDYNTREE_USES_ICUB_MAIN:BOOL=OFF",
                       ]),
    ],
    cmdclass=dict(build_ext=BuildExtension),
    zip_safe=False,
)
