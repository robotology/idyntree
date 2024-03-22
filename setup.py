import os

import cmake_build_extension
import setuptools

if "CIBUILDWHEEL" in os.environ and os.environ["CIBUILDWHEEL"] == "1":
    CIBW_CMAKE_OPTIONS = ["-DCMAKE_INSTALL_LIBDIR=lib"]
else:
    CIBW_CMAKE_OPTIONS = []

setuptools.setup(
    cmdclass=dict(build_ext=cmake_build_extension.BuildExtension),
    ext_modules=[
        cmake_build_extension.CMakeExtension(
            name="BuildAndInstall",
            install_prefix="idyntree",
            expose_binaries=["bin/idyntree-model-info"],
            cmake_configure_options=[
                "-DBUILD_SHARED_LIBS:BOOL=OFF",
                "-DIDYNTREE_USES_PYTHON:BOOL=ON",
                "-DBUILD_TESTING:BOOL=OFF",
                "-DIDYNTREE_PACKAGE_FOR_PYPI:BOOL=ON",
                "-DIDYNTREE_USES_IPOPT:BOOL=OFF",
                "-DIDYNTREE_USES_ASSIMP:BOOL=OFF",
                "-DIDYNTREE_USES_IRRLICHT:BOOL=OFF",
                "-DIDYNTREE_USES_OSQPEIGEN:BOOL=OFF",
                "-DIDYNTREE_USES_ALGLIB:BOOL=OFF",
                "-DIDYNTREE_USES_WORHP:BOOL=OFF",
                "-DIDYNTREE_USES_YARP:BOOL=OFF",
                "-DIDYNTREE_USES_ICUB_MAIN:BOOL=OFF",
                "-DIDYNTREE_PYTHON_PIP_METADATA_INSTALL:BOOL=OFF",
            ]
            + CIBW_CMAKE_OPTIONS,
        ),
    ],
)
