"""
定义了几个函数。
QuadricInitialiser:用来初始化对偶二次曲面
initialise_quadric_from_depth:这个函数用于从深度图像中初始化一个对偶二次曲面。(对我们没有用)
initialise_quadric_ray_intersection:由物体在多个视图中的观测点和边界框信息生成对偶二次曲面。(oriention和size暂时不管)
new_factors:用于比较两个因子图current和previous之间的差异
new_values:用于比较两个变量集current和previous之间的差异。
ps_and_qs_from_values:用于将一个变量集values中的变量按照它们的类型进行分类。
"""
from typing import Callable, List
import gtsam
import gtsam_quadrics
import numpy as np
from .quadricslam_states import QuadricSlamState


"""
函数QuadricInitialiser:用来初始化对偶二次曲面
接受三个参数:
一个 List[gtsam.Pose3] 类型的列表,表示位姿序列;
一个 List[gtsam_quadrics.AlignedBox2] 类型的列表,表示bounding box;
一个 QuadricSlamState 类型的实例,表示当前的 Quadric SLAM 状态。
返回一个 gtsam_quadrics.ConstrainedDualQuadric 类型的实例,表示初始化得到的二次曲面。

List[gtsam.Pose3]: list representing sequencial pose
List[gtsam_quadrics.AlignedBox2]: Lise of b-box
QuadricSlamState: current state of quadric slam

return gtsam_quadrics.ConstrainedDualQuadric containing dual quadrics obtained be initialization
"""
QuadricInitialiser = Callable[
    [List[gtsam.Pose3], List[gtsam_quadrics.AlignedBox2], QuadricSlamState],
    gtsam_quadrics.ConstrainedDualQuadric]



def initialise_quadric_from_depth(
        obs_poses: List[gtsam.Pose3],
        boxes: List[gtsam_quadrics.AlignedBox2],
        state: QuadricSlamState,
        object_depth=0.1) -> gtsam_quadrics.ConstrainedDualQuadric:
    """
    这个函数用于从深度图像中初始化一个对偶二次曲面。
    输入参数包括:
    包含多个视图中物体的姿态obs_poses、包含多个视图中物体的边界框的列表、QuadricSlamState类型的state和物体的深度值object_depth。
    返回 ConstrainedDualQuadric 类型的对偶二次曲面对象。

    useless for us
    """
    # Uses the depth image to initialise a quadric from a single view 
    # (note: this assumes there is only a single view, and will discard all extra views)
    # 函数首先检查输入参数是否合法。
    s = state.system
    assert s.calib_rgb is not None
    assert state.this_step is not None
    n = state.this_step
    assert n.depth is not None

    # 用列表中的第一个物体位姿和bounding box来进行初始化
    box = boxes[0]
    pose = obs_poses[0]
    calib = gtsam.Cal3_S2(s.calib_rgb)

    # get average box depth
    # 然后计算bounding box的深度值。
    dbox = box.vector().astype('int')  # get discrete box bounds
    box_depth = n.depth[dbox[1]:dbox[3], dbox[0]:dbox[2]].mean()

    # compute the 3D point corrosponding to the box center
    # 计算bounding box的中心点在三维空间中的坐标(quadric_center)。
    center = box.center()
    x = (center[0] - calib.px()) * box_depth / calib.fx()
    y = (center[1] - calib.py()) * box_depth / calib.fy()
    relative_point = gtsam.Point3(x, y, box_depth)
    quadric_center = pose.transformFrom(relative_point)

    # compute quadric rotation using .Lookat
    # 根据物体姿态、bounding box中心点和视角向上的向量来计算对偶二次曲面的旋转矩阵和位姿
    up_vector = pose.transformFrom(gtsam.Point3(0, -1, 0))
    quadric_rotation = gtsam.PinholeCameraCal3_S2.Lookat(
        pose.translation(), quadric_center, up_vector,
        calib).pose().rotation()
    quadric_pose = gtsam.Pose3(quadric_rotation, quadric_center)

    # compute the quadric radii from the box shape
    # 根据bounding box的形状和深度值计算对偶二次曲面的半轴长度
    tx = (box.xmin() - calib.px()) * box_depth / calib.fx()
    ty = (box.ymin() - calib.py()) * box_depth / calib.fy()
    radii = np.array([np.abs(tx - x), np.abs(ty - y), object_depth])

    return gtsam_quadrics.ConstrainedDualQuadric(quadric_pose, radii)



def initialise_quadric_ray_intersection(
        obs_poses: List[gtsam.Pose3], boxes: List[gtsam_quadrics.AlignedBox2],
        state: QuadricSlamState) -> gtsam_quadrics.ConstrainedDualQuadric:
    """
    由物体在多个视图中的观测点和边界框信息生成对偶二次曲面。
    TODO:我们要将多帧修改为单帧。
    obs_poses: 一个包含多个视图中物体的位姿的列表
    boxes: 一个包含多个视图中物体的边界框的列表
    state: 一个QuadricSlamState对象,包含当前状态信息。
    对偶二次曲面的大小和方向直接捏造一个定值。

    generate dual quadrics from observation points in multiple frames and b-boxes
    TODO we need to upgrade multiple frame to one frame
    obs_poses: a list of objects in several views
    boxes: a list of b-boxes in several views
    state: a QuadricSlamState object containing current state and information
    fudge the size and direcion for dual quadric
    """
    # Takes all observations of a quadric, projects rays into 3D space, and
    # uses their closest convergence point to place the quadric. Initial
    # orientation and size are currently just dumb guesses.

    # Get each observation point
    # 从obs_poses参数中获取每个视图中的物体位置。
    ps = np.array([op.translation() for op in obs_poses])

    # Get each observation direction
    # TODO actually use bounding box rather than assuming middle...
    # 从obs_poses参数中获取每个视图中的物体方向。
    vs = np.array([op.rotation().matrix()[:, 0] for op in obs_poses])

    # Apply this to compute point closet to where all rays converge: https://stackoverflow.com/a/52089698/1386784
    # 应用以上信息计算出在3D空间中所有光线汇聚的最近点,即曲面的质心。
    i_minus_vs = np.eye(3) - (vs[:, :, np.newaxis] @ vs[:, np.newaxis, :])
    quadric_centroid = np.linalg.lstsq(
        i_minus_vs.sum(axis=0),
        (i_minus_vs @ ps[:, :, np.newaxis]).sum(axis=0),
        rcond=None)[0].squeeze()

    # Fudge the rest for now  # TODO do better...
    # 对偶二次曲面的大小和方向直接捏造一个定值。
    return gtsam_quadrics.ConstrainedDualQuadric(
        gtsam.Rot3(), gtsam.Point3(quadric_centroid), [1, 1, 0.1])



def new_factors(current: gtsam.NonlinearFactorGraph,
                previous: gtsam.NonlinearFactorGraph):
    """
    用于比较两个因子图current和previous之间的差异。  
    该函数会找到在current中但不在previous中的所有因子,然后将这些因子构建成一个新的因子图并返回。  

    compare the difference between current factor graph and previous factor graph.  
    find all factors in current graph but not in previous, and use them to form a new factor graph.  
    return the newly formed graph.  
    """
    # Figure out the new factors
    fs = (set([current.at(i) for i in range(0, current.size())]) -
          set([previous.at(i) for i in range(0, previous.size())]))

    # Return a NEW graph with the factors
    out = gtsam.NonlinearFactorGraph()
    for f in fs:
        out.add(f)
    return out



def new_values(current: gtsam.Values, previous: gtsam.Values):
    """
    用于比较两个变量集current和previous之间的差异。
    该函数会找到在current中但不在previous中的所有变量,然后将这些变量构建成一个新的变量集并返回。
    需要注意的是,current和previous中的变量类型可以是gtsam.Quadric或者gtsam.Pose3。

    compare the difference between current values and previous values.
    find all variables in current values but not in previous, and use them to form a new variable set
    To be noticed, type of variable can be gtsam.Quadric or gtsam.Pose3
    """
    # Figure out new values
    cps, cqs = ps_and_qs_from_values(current)
    pps, pqs = ps_and_qs_from_values(previous)
    vs = {
        **{k: cps[k] for k in list(set(cps.keys()) - set(pps.keys()))},
        **{k: cqs[k] for k in list(set(cqs.keys()) - set(pqs.keys()))}
    }

    # Return NEW values with each of our estimates
    out = gtsam.Values()
    for k, v in vs.items():
        if type(v) == gtsam_quadrics.ConstrainedDualQuadric:
            v.addToValues(out, k)
        else:
            out.insert(k, v)
    return out


def ps_and_qs_from_values(values: gtsam.Values):
    """
    用于将一个变量集values中的变量按照它们的类型进行分类。
    函数返回一个包含两个字典的元组(cps, cqs),其中cps是所有类型为gtsam.Pose3的变量构成的字典,
    cqs是所有类型为gtsam_quadrics.ConstrainedDualQuadric的变量构成的字典。

    classify variables in values according to their type.
    return a tuple containing two dicts: (cps,cqs), 
    cps contains gtsam.Pose3 
    cqs contains gtsam_quadrics.ConstrainedDualQuadric
    """
    # TODO there's got to be a better way to access the typed values...
    
    # x是位姿,q是对偶二次曲线。
    return ({
        k: values.atPose3(k)
        for k in values.keys()
        if gtsam.Symbol(k).string()[0] == 'x'
    }, {
        k: gtsam_quadrics.ConstrainedDualQuadric.getFromValues(values, k)
        for k in values.keys()
        if gtsam.Symbol(k).string()[0] == 'q'
    })