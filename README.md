# SOSLAM_plus

## log  


### 20230321  Ziqi Han

If you want to modify gtsam_quadric code (C++), you can run:

```shell
pip install .
```

Be aware to delete the following code in "setup.py". Otherwise it will be relatively slow, because you will recompile gtsam.

```shell
shutil.rmtree(os.path.join(build_lib_dir, 'gtsam'), ignore_errors=True)
shutil.copytree(os.path.join(build_dir, 'gtsam', 'python', 'gtsam'),
                os.path.join(build_lib_dir, 'gtsam'))
shutil.copy(os.path.join(build_dir, gtsam_quadrics_so),
            os.path.join(build_lib_dir, gtsam_quadrics_so))
shutil.copy(os.path.join(build_lib_dir, "gtsam", gtsam_so),
            os.path.join(source_dir, gtsam_so))
shutil.copy(os.path.join(build_lib_dir, gtsam_quadrics_so),
            os.path.join(source_dir, gtsam_quadrics_so))

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

