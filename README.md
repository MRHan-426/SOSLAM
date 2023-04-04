## 1.Check your dependencies, mine are as follows:
---
```shell
cmake 3.26.0
libboost 1.71.0  # make sure to compile C++ version from source code.
eigen 3.3.7
```

```shell
sudo apt install libeigen3-dev
sudo apt install libtbb-dev
sudo apt-get install libpugixml-dev # a light library to deal with xml file
```

## 2.Download GTSAM 4.1.1

Note that higher version may bring unexpected errors.

```shell
git clone --branch 4.1.1 https://github.com/borglab/gtsam.git
```

## 3.Modify Eigen cmake config file

Open: cmake/HandleEigen.cmake

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

(1) quardicKey is initialized with 66666, 66666 represents None.

So if you want to create a new quadric, try to avoid it from being 66666.

(2) optimizer_batch = true  ->  Dummy，initialize_quadric_ray_intersection, batch

optimizer_batch = false  ->  SOSLAM，initialize_with_ssc_psc_bbs, step(LM )

(3) if you want to run dummy example, run ./soslam_exe --dummy.

By default, we will use hand labelled data.

