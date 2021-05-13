### Build

* Set build type (Debug/Release)
```shell
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release/Debug
```

* Set build python version
```shell
mkdir build && cd build
cmake .. -DPYBIND11_PYTHON_VERSION=3.8
```