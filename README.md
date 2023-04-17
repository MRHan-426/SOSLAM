# SO-SLAM



![](SO-SLAM.assets/3 00_00_00-00_00_30.gif)



This is Team 6's final project git repository for ROB530: Mobile Robotics. 

The title of our project is **Implementation and Evaluation of Semantic-Object SLAM Algorithm**

The team members include: Ziqi Han, Zhewei Ye, Tien-Li Lin, Yi-Cheng Liu, Shubh Agrawal.

**Related Paper:**  

+ Liao Z, Hu Y, Zhang J, et al. So-slam: Semantic object slam with scale proportional and symmetrical texture constraints[J]. IEEE Robotics and Automation Letters, 2022, 7(2): 4008-4015. [**[PDF]**](https://arxiv.org/abs/2109.04884)

---

## 1. Prerequisites

```shell
cmake 3.26.0
eigen 3.3.7
libboost 1.71.0  # make sure to compile C++ version from source code.
```

```shell
sudo apt install libeigen3-dev
sudo apt install libtbb-dev
sudo apt-get install libpugixml-dev # a light library to deal with xml file
sudo apt install libflann-dev  # dependency of PCL
sudo apt-get install libpcl-dev
sudo apt-get install libusb-1.0-0-dev
```



## 2. Compile GTSAM

**Note that higher version may bring unexpected errors, we do not test other version so far.**

```shell
git clone --branch 4.1.1 https://github.com/borglab/gtsam.git
```

Modify Eigen cmake config file: cmake/HandleEigen.cmake

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



## 3. Compile SO-SLAM

Branch Master contains point cloud visualization, so you have some more prerequisites.

```shell
git clone --branch Master https://github.com/MRHan-426/SOSLAM_plus.git
```

Branch 4.1.1 doesnot contain point cloud visualization, so you don't have to compile PCL, VTK.

```shell
git clone --branch 4.1.1 https://github.com/MRHan-426/SOSLAM_plus.git
```

Then:

```shell
mkdir build
cmake ..
make
```



## 4. Examples

+ **Dummy Example**: We provide a demo to visualize for debugging. It will shows two hand designed ellipsoid.

```shell
./soslam_exe --dummy --3d
```

![image-20230417154558047](SO-SLAM.assets/image-20230417154558047.png)



+ **Fr2 Desk:** We provide a demo running dataset TUM RGBD. [Download hand labeled dataset]

```shell
./soslam_exe --Fr2_Desk --3d
```

<img src="SO-SLAM.assets/3.png" alt="3" style="zoom:25%;" />



+ **Fr1 Desk2:** We provide a demo running dataset TUM RGBD. [Download hand labeled dataset]

```shell
./soslam_exe --Fr1_Desk2 --3d
```

<img src="SO-SLAM.assets/4.png" alt="4" style="zoom:25%;" />



+ **Fr2 Dishes:** We provide a demo running dataset TUM RGBD. [Download hand labeled dataset]

```shell
./soslam_exe --Fr2_Dishes --3d
```

<img src="https://github.com/MRHan-426/SOSLAM_plus/blob/master/gtsam_quardic/1.png" alt="2" style="zoom:25%;" />



There are detailed configurations in **config.yaml**, please change if you need.

It costs us a lot of time to label and associate these datasets, so please star if they do help you.



## 4. Videos and Documentation

+ Our project presentation video is on [**YouTube**](https://youtu.be/_yUy5nOtfMM).

  

+ Project Document: [**[PDF]**](TODO)

  

## 5. Note

+ If you want to use it in your work or with other datasets, you should prepare the dataset containing:

  - RGB image

  - Label xml (contain "objectKey" key to store the data association information)
  - Odom txt
  - Depth image (if you do not need point cloud visualization, just ignore)

​		Be aware that you should rename your images and xmls as number 1,2,3,...

​		Be aware that RGB, Depth, Label, Odom must match.

+ This is an incomplete version of our project. We have a lot of experiments to be done.

  

## 6. Acknowledgement

Thanks for the great work: **SO-SLAM**, **Quadric-SLAM**, **EAO-SLAM**, **ORB-SLAM2**, **YOLO-v8**.



## 7. Contact

+ Ziqi Han, Email: ziqihan@umich.edu
+ Zhewei Ye, Email: yezhewei@umich.edu
+ Tien-Li Lin, Email: tienli@umich.edu
+ Yi-Cheng Liu, Email: liuyiche@umich.edu
+ Shubh Agrawal, Email: shbhgrwl@umich.edu 



Please cite the author's paper if you use the code in your work.

```
Liao, Z., Hu, Y., Zhang, J., Qi, X., Zhang, X. & Wang, W. (2021). SO-SLAM: Semantic Object SLAM with Scale Proportional and Symmetrical Texture Constraints.
```



## TODO

- [x] 1. Get dataset ready. Label and associate data by hand.

- [x] 2. Rewrite quardicslam in C++, called "c++ core".

    - [x] 2.1 Finish compiling.
    - [x] 2.2 Get correct answer.

- [ ] 3.New Constraints and quadric Initialization.

  - [x] 3.1 Finish writing constraints.
    - [x] 3.1.1 Semantic Scale constraint.
    - [x] 3.1.2 Plane Support constraint.
    - [ ] 3.1.3 Symmetry Texture constraint.
    - [x] 3.1.4 Bounding Box constraint.    
  - [x] 3.2 Finish writing Quadric Initialization.
  - [x] 3.3 Add Constraints into "c++ core".
    - [x] 3.3.1 Debug Semantic Scale constraint.
    - [x] 3.3.2 Debug Plane Support constraint.
    - [x] 3.3.3 Debug Symmetry Texture constraint.
    - [x] 3.3.4 Debug Bounding Box Constraint.  
  - [x] 3.4 Add Quadric Initialization part into "c++ core".

- [x] 4. Add visualization part.

    - [x] dependency problems.
    - [x] adding quadrics drawing(MapDrawer).
      - [x] Map object that can return all objects to draw.
      - [x] camera pose drawing.
      - [x] graph drawing.
      - [x] verify drawing correctness.
    - [x] adding frame drawer(FrameDrawer).
      - [x] state == Tracking::NOT_INITIALIZED.
      - [x] quadricimage is in Tracking, move to here. 
      - [x] It also use a Update to update images, find a way to place Update.
      - [x] verify drawing correctness.
    - [x] point cloud
    - [x] multithread part
    - [x] world axis
    - [x] usability: control visualization

===========================================================================

4.10 ~ 4.13 TODO list

- [x] 5. Finish debugging.

    - [x] 5.1 Debug for the main optimization loop.
    - [x] 5.2 Debug for symmetry factor.

- [x] 6. Evaluation our algrithm. [3D point cloud to get ground truth]

    - [x] 6.1 Rot.
      - [x] basic Rot.
      - [x] check association
      - [x] check semantic table.
      - [x] other dataset(annotation and binding)
    - [x] 6.2 IOU.

- [x] 7. Add visual odometry to take place of odom ground truth. [Orb Slam] [Optional]

===========================================================================

4.14 ~ 4.16 TODO list

- [x] 8. Doing experiments, Record Video. 
- [ ] 9. Write final report.
