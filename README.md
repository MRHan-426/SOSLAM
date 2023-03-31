# SOSLAM_plus

## TODO

- [ ] Get dataset ready. Label and associate data by hand.
- [ ] Rewrite quardicslam in C++, called "c++ core".
    - [x] Finish compiling.
    - [ ] Get correct answer.
- [ ] New Constraints and quadric Initialization.
    - [ ] Finish writing constraints.
          - [ ] Semantic Scale constraint.
          - [ ] Plane Support constraint.
          - [ ] Symmetry Texture constraint.
          - [ ] Bounding Box Constraint.    
    - [ ] Finish writing Quadric Initialization.
    - [ ] Add Constraints into "c++ core".
          - [ ] Debug Semantic Scale constraint.
          - [ ] Debug Plane Support constraint.
          - [ ] Debug Symmetry Texture constraint.
          - [ ] Debug Bounding Box Constraint.  
    - [ ] Add Quadric Initialization part into "c++ core".
- [ ] Add visualization part.
- [ ] Doing experiments, Record Video.
- [ ] Write final report.
 


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

