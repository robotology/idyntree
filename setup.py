import os
from pathlib import Path
from setuptools import setup
from cmake_build_extension import BuildExtension, CMakeExtension

# Read the contents of your README file
with open(Path(__file__).parent.absolute() / 'README.md', encoding='utf-8') as f:
    long_description = f.read()

if "CIBUILDWHEEL" in os.environ and os.environ["CIBUILDWHEEL"] == "1":
    CIBW_CMAKE_OPTIONS = ["-DCMAKE_INSTALL_LIBDIR=lib"]
else:
    CIBW_CMAKE_OPTIONS = []

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
    setup_requires=[
        "setuptools_scm",
        "ninja",
        "cmake",
        "numpy",
        "cmake-build-extension",
    ],
    ext_modules=[
        CMakeExtension(name='BuildAndInstall',
                       install_prefix="idyntree",
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
                       ] + CIBW_CMAKE_OPTIONS),
    ],
    cmdclass=dict(build_ext=BuildExtension),
    zip_safe=False,
)
