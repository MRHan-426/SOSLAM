## 1.Check your dependencies, mine are as follows:
---
```shell
cmake 3.26.0
libboost 1.71.0  # make sure to compile C++ version for source code.
eigen 3.3.7
```

```shell
sudo apt install libeigen3-dev
sudo apt install libtbb-dev
```

## 2.Download GTSAM 4.1.1

Note that higher version may bring unexpected errors.

```shell
git clone --branch 4.1.1 https://github.com/borglab/gtsam.git
```

## 3.Modify Eigen cmake config file: cmake/HandleEigen.cmake

add below line just after "option(GTSAM_USE_SYSTEM_EIGEN...)"

```shell
set(GTSAM_USE_SYSTEM_EIGEN ON)
```
Then:

```shell

mkdir build
cmake ..
make check
sudo make install
```


## 4.Notes for variables and functions

(1) quardicKey is initialized with zero, zero represents None.

So if you want to create a new quadric, start from long unsigned int 1.
