# SOSLAM_plus

## TODO

- [x] 1. Get dataset ready. Label and associate data by hand.
- [x] 2. Rewrite quardicslam in C++, called "c++ core".
    - [x] 2.1 Finish compiling.
    - [x] 2.2 Get correct answer.
- [ ] 3.New Constraints and quadric Initialization.
    - [x] 3.1 Finish writing constraints.
         - [x] 3.1.1 Semantic Scale constraint.
         - [X] 3.1.2 Plane Support constraint.
         - [ ] 3.1.3 Symmetry Texture constraint.
         - [x] 3.1.4 Bounding Box constraint.    
    - [x] 3.2 Finish writing Quadric Initialization.
    - [ ] 3.3 Add Constraints into "c++ core".
         - [x] 3.3.1 Debug Semantic Scale constraint.
         - [X] 3.3.2 Debug Plane Support constraint.
         - [ ] 3.3.3 Debug Symmetry Texture constraint.
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
    - [x] multithread part
    - [x] world axis
 
===========================================================================

4.10 ~ 4.13 TODO list
- [ ] 5. Finish debugging.
    - [ ] 5.1 Debug for the main optimization loop.
    - [ ] 5.2 Debug for symmetry factor.
- [ ] 6. Evaluation our algrithm. [3D point cloud to get ground truth]
    - [ ] 6.1 Rot.
    - [ ] 6.2 IOU.
- [ ] 7. Add visual odometry to take place of odom ground truth. [Orb Slam]

===========================================================================

4.14 ~ 4.16 TODO list
- [ ] 8. Doing experiments, Record Video. 
- [ ] 9. Write final report.
 


## log  

### 20230323  Zhewei Ye  
Got TUM rgbd example work with different python methods.  
```python
associator=QuadricIouAssociator(), # use IOU
quadric_initialiser=utils.initialise_quadric_from_depth) #use depth init
```
In data_source=TumRgbd(path=dataset_path, rgb_calib=camera_calib), we need to read depth information:
```python
def next(
    self, state: QuadricSlamState
) -> Tuple[Optional[SE3], Optional[np.ndarray], Optional[np.ndarray]]:
    i = self.data_i
    self.data_i += 1
    return (SE3() if i == 0 else self._gt_to_SE3(i) *
            self._gt_to_SE3(i - 1).inv(),
            cv2.imread(os.path.join(self.path,
                             cast(str, self.data['rgb'][i][1]))), 
            cv2.imread(os.path.join(self.path,
                             cast(str, self.data['depth'][i][1])))) # return depth information read in file
```
initialise_quadric_ray_intersection will not work since it just init quad in the center of pose

### 20230322  Ziqi Han

Finish semantic scale constraint, TODO: design full semantic table.

```shell
SemanticTable.h
SemanticScaleFactor.cpp
SemanticScaleFactor.h
```

Finish initialization, which create a new graph and add factors to optimize.

```shell
ConstrainedDualQuadric ConstrainedDualQuadric::initialize(
    const AlignedBox2 &measured,
    const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
    const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
    const gtsam::SharedNoiseModel &model,
    const gtsam::Pose3 &pose,
    const MeasurementModel &errorType = STANDARD)
```

TODO: How to get camera pose in a easy way?

### 20230322  Zhewei Ye  

Found a possible trick in adding function in gtsam_quadric:  
Firstly adding function in cpp and h, then adding function declairarion in gtsam_quadrics.i  
Be sure if you are goint to use gtsam::Vector, no matter Vector2 or 3, remember declair  

```c
  gtsam::Vector getRad()const;
```
in gtsam_quadrics.i  

### 20230321  Ziqi Han

If you want to modify gtsam_quadric code (C++), you can run:

```shell
pip install .
```

Be aware to delete the following code in "setup.py". Otherwise it will be relatively slow, because you will recompile gtsam.

```shell
shutil.rmtree(os.path.join(build_lib_dir, 'gtsam'), ignore_errors=True)

```

If you fail, please check and set virtual memory:


```shell
c++: fatal error: Killed signal terminated program cc1plus
  compilation terminated.
```


If you want to modify quadricslam code (Python), you can run:

```shell
pip install .
```

### 20230320  Ziqi Han

please name as PlaneSupportingFactor, SemanticScaleFactor. I need Factors between Gtsam::Pose3() 
and Gtsam_quadric::ConstrainDualQuadric.

```shell
def initialise_quadric_single_frame(
    new_BoundingBoxFactor: gtsam_quadrics.BoundingBoxFactor,
    new_PlaneSupportingFactor: gtsam_quadrics.PlaneSupportingFactor,
    new_SemanticScaleFactor: gtsam_quadrics.SemanticScaleFactor,
) -> gtsam_quadrics.ConstrainedDualQuadric:
```

Done some part of initialization and optimization. Not completed.

Add new branch "ziqihan" to save current changes. Note that the code may not work.

### 20230313  Zhewei Ye

python 3.8.10 is recommended, pyton 3.10 will encounter pytorch incompability issue  

### 20230218  Zhewei Ye


```
TypeError: gca() got an unexpected keyword argument 'projection'
```

For this problem, change the python code from  

```
ax = plt.gca(projection='3d')
```

to  

```
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
```

### 20230217  Zhewei Ye

please install gtsam_quadrics using below, or it will result in unexpected error 

```shell
pip install gtsam_quadrics --no-binary :all:  
```

