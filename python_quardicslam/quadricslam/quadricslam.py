'''
定义了QuadricSlam类
step：迭代优化
spin：批优化
guess_initial_values：猜测初始值，进行初始化
class QuadricSlam
step: iterative optimization
spin: batch optimization
guess_initial_values: guess inital values and do initialization

目前只有bounding box的因子。
Only have b-box factor temporary
'''

from itertools import groupby
from types import FunctionType
from typing import Callable, Dict, List, Optional, Union

import gtsam
import gtsam_quadrics
import numpy as np
from spatialmath import SE3

from .data_associator import DataAssociator
from .data_source import DataSource
from .detector import Detector
from .quadricslam_states import QuadricSlamState, StepState, SystemState
from .utils import (
    QuadricInitialiser,
    initialise_quadric_ray_intersection,
    new_factors,
    new_values,
)
from .visual_odometry import VisualOdometry


class QuadricSlam:
    """初始化，定义了系统初始状态等等 init all system variables"""
    def __init__(
        self,
        data_source: DataSource,
        visual_odometry: Optional[VisualOdometry] = None,
        detector: Optional[Detector] = None,
        associator: Optional[DataAssociator] = None,
        initial_pose: Optional[SE3] = None,
        noise_prior: np.ndarray = np.array([0] * 6, dtype=np.float64),
        noise_odom: np.ndarray = np.array([0.01] * 6, dtype=np.float64),
        noise_boxes: np.ndarray = np.array([3] * 4, dtype=np.float64),
        optimiser_batch: Optional[bool] = None,
        optimiser_params: Optional[Union[gtsam.ISAM2Params,
                                         gtsam.LevenbergMarquardtParams,
                                         gtsam.GaussNewtonParams]] = None,
        on_new_estimate: Optional[Callable[[QuadricSlamState], None]] = None,
        quadric_initialiser:
        QuadricInitialiser = initialise_quadric_ray_intersection
    ) -> None:
        # TODO this needs a default data associator, we can't do anything
        # meaningful if this is None...
        if associator is None:
            raise NotImplementedError('No default data associator yet exists, '
                                      'so you must provide one.')
        self.associator = associator
        self.data_source = data_source
        self.detector = detector
        self.visual_odometry = visual_odometry

        self.on_new_estimate = on_new_estimate # 当系统有新的估计值时会调用该函数。 call to handle new estimates
        self.quadric_initialiser = quadric_initialiser # 初始化对偶二次曲面 init dual quadric

        # Bail if optimiser settings and modes aren't compatible
        if (optimiser_batch == True and
                type(optimiser_params) == gtsam.ISAM2Params):
            raise ValueError("ERROR: Can't run batch mode with '%s' params." %
                             type(optimiser_params))
        elif (optimiser_batch == False and optimiser_params is not None and
              type(optimiser_params) != gtsam.ISAM2Params):
            raise ValueError(
                "ERROR: Can't run incremental mode with '%s' params." %
                type(optimiser_params))
        if optimiser_params is None:
            optimiser_params = (gtsam.LevenbergMarquardtParams()
                                if optimiser_batch is True else
                                gtsam.ISAM2Params())

        # Setup the system state, and perform a reset
        self.state = QuadricSlamState(
            SystemState(
                initial_pose=SE3() if initial_pose is None else initial_pose,
                noise_prior=noise_prior,
                noise_odom=noise_odom,
                noise_boxes=noise_boxes,
                optimiser_batch=type(optimiser_params) != gtsam.ISAM2Params,
                optimiser_params=optimiser_params))
        self.reset()


    def guess_initial_values(self) -> None:
        '''
        用来猜测系统状态的初始值的。
        在进行图优化之前，需要估计一些参数的初值，而这些参数可能在初始状态下是未知的，因此需要猜测。
        这个函数的主要作用是通过一些启发式方法猜测初始值。

        Use some heuristic method to guess init values.
        before graph optimization, we need to estimate some init value. 
        However, some params might be unknown in init state, and we need to guess.
        '''
        # Guessing approach (only guess values that don't already have an estimate):
        # - guess poses using dead reckoning
        # - guess quadrics using Euclidean mean of all observations
        s = self.state.system

        # fs:当前系统状态的因子列表,通过从系统状态的图中提取所有因子来构建的。
        fs = [s.graph.at(i) for i in range(0, s.graph.nrFactors())]

        # Start with prior factors
        # 使用先验因子对机器人的位姿进行初始化,先验因子是已知的机器人位姿，可以用来约束状态估计值。
        # 因此这里的目的是将先验因子的信息插入到状态估计值中。
        for pf in [
                f for f in fs if type(f) == gtsam.PriorFactorPose3 and
                not s.estimates.exists(f.keys()[0])
        ]:
            s.estimates.insert(pf.keys()[0], pf.prior())

        # 从图优化问题的所有因子中筛选出类型为gtsam.BetweenFactorPose3的因子，并将其存储在bfs中。
        bfs = [f for f in fs if type(f) == gtsam.BetweenFactorPose3]
        done = False
        # 根据bf的第一个位姿的估计值以及bf的测量值推测出bf的第二个位姿的估计值，并将其插入s.estimates中
        while not done:
            bf = next((f for f in bfs if s.estimates.exists(f.keys()[0]) and
                       not s.estimates.exists(f.keys()[1])), None)
            if bf is None:
                done = True
                continue
            s.estimates.insert(
                bf.keys()[1],
                s.estimates.atPose3(bf.keys()[0]) * bf.measured())
            bfs.remove(bf)
        # 对于所有剩余的bf，将bf的第二个位姿的估计值设置为原点。
        for bf in [
                f for f in bfs if not all([
                    s.estimates.exists(f.keys()[i])
                    for i in range(0, len(f.keys()))
                ])
        ]:
            s.estimates.insert(bf.keys()[1], gtsam.Pose3())

        # Add all quadric factors、
        # 添加所有的quadric因子。
        _ok = lambda x: x.objectKey() # 表示将输入的x对象的objectKey()方法返回的结果作为输出
        # 筛选出图中类型为 gtsam_quadrics.BoundingBoxFactor 的因子，生成一个列表 bbs 。
        # 然后将其按照 objectKey 进行排序，并使用 groupby 按照 objectKey 将其分组，将同一对象的因子分为一组。
        bbs = sorted([
            f for f in fs if type(f) == gtsam_quadrics.BoundingBoxFactor and
            not s.estimates.exists(f.objectKey())
        ],
                     key=_ok)

        # 对于每一组因子 qbbs ，使用 quadric_initialiser 函数对其进行初始化，
        # 传入参数为包含每个因子的 姿态、测量值 以及状态 state 。
        # 初始化完成后，将结果使用 addToValues 函数添加到优化问题的估计值中。
        for qbbs in [list(v) for k, v in groupby(bbs, _ok)]:
            self.quadric_initialiser(
                [s.estimates.atPose3(bb.poseKey()) for bb in qbbs],
                [bb.measurement() for bb in qbbs],
                self.state).addToValues(s.estimates, qbbs[0].objectKey())


    
    def spin(self) -> None:
        '''
        等待数据源完成，进行批优化(batch)模式。
        猜测初始值，进行优化，最后将优化后的结果存储在系统估计值中。
        如果定义了 on_new_estimate，则该函数将使用当前状态调用 on_new_estimate。
        Conduct batch optimization after data source is done.
        Guess the initial values and optimize them. Store the optimized outputs in system.estimates.
        if on_new_estimate is true, call on_new_estimate on current state. 
        '''
        while not self.data_source.done():
            self.step()

        if self.state.system.optimiser_batch:
            self.guess_initial_values()
            s = self.state.system
            s.optimiser = s.optimiser_type(s.graph, s.estimates,
                                           s.optimiser_params)
            s.estimates = s.optimiser.optimize()
            if self.on_new_estimate:
                self.on_new_estimate(self.state)



    def step(self) -> None:
        '''
        计算当前机器人的运动，利用机器人传感器读取的数据（如视觉、激光雷达等）以及之前保存的信息，
        生成机器人的位姿和场景的三维结构（建图）。
        Calculate current robot movement using data read by sensors (cameras, Lidars, etc.) and previously stored information.
        Generate robot pose and 3D structure of scene (mapping).
        '''
        # Setup state for the current step
        # 将当前状态分配给变量s，并创建一个新的状态变量n
        s = self.state.system
        p = self.state.prev_step
        n = StepState(
            0 if self.state.prev_step is None else self.state.prev_step.i + 1)
        self.state.this_step = n

        # Get latest data from the scene (odom, images, and detections)
        # 获取最新的测量数据，包括里程计、图像、深度和目标检测结果。
        n.odom, n.rgb, n.depth = (self.data_source.next(self.state))
        if self.visual_odometry is not None:
            n.odom = self.visual_odometry.odom(self.state)
        n.detections = (self.detector.detect(self.state)
                        if self.detector else [])
        # 将最新的测量数据关联到之前的数据中
        # associate newest measure data into previous data.
        n.new_associated, s.associated, s.unassociated = (
            self.associator.associate(self.state))

        # Extract some labels
        # TODO handle cases where different labels used for a single quadric???
        # 将标签信息提取出来。
        s.labels = {
            d.quadric_key: d.label
            for d in s.associated
            if d.quadric_key is not None
        }

        # Add new pose to the factor graph
        # 将最新的测量数据添加到图中
        # if this is the first step, add robot pose using prior information.
        # 如果这是第一步，则使用先验信息添加机器人的位姿
        if p is None:
            s.graph.add(
                gtsam.PriorFactorPose3(n.pose_key, s.initial_pose,
                                       s.noise_prior))
        # 构造gtsam.BetweenFactorPose3因子，表示上一帧和当前帧之间的运动，加入到优化图中。
        # construct gtsam.BetweenFactorPose3 which represent the motion bewteen last frame and current one 
        # and add to the factor graph.
        else:
            s.graph.add(
                gtsam.BetweenFactorPose3(
                    p.pose_key, n.pose_key,
                    gtsam.Pose3(((SE3() if p.odom is None else p.odom).inv() *
                                 (SE3() if n.odom is None else n.odom)).A),
                    s.noise_odom))

        # Add any newly associated detections to the factor graph
        # 对于每个新关联的检测结果，将其添加到图中。
        # 将新检测到的物体的边界框信息以及物体在图像上的投影位置转换为因子，并将其加入到图优化的因子图中
        # 详见BoundingBoxFactor.h
        # convert newly detected object's b-box and object's projection location on image to factor and add to the factor graph
        # look up BoundingBoxFactor.h for detail 
        for d in n.new_associated:
            if d.quadric_key is None:
                print("WARN: skipping associated detection with "
                      "quadric_key == None")
                continue
            s.graph.add(
                gtsam_quadrics.BoundingBoxFactor(
                    gtsam_quadrics.AlignedBox2(d.bounds),
                    gtsam.Cal3_S2(s.calib_rgb), d.pose_key, d.quadric_key,
                    s.noise_boxes))

        # Optimise if we're in iterative mode
        # 批处理模式(Batch)是将所有的因子同时添加到因子图中，然后一次性优化所有的因子，
        # 而迭代模式(Iterative)是逐个添加因子，然后每次添加一个因子之后就立即进行一次优化。
        # 如果是迭代模式，则进行优化以得到更好的状态估计。
        # Batch mode is adding all factors into the factor graph then optimize them.
        # iterative mode is iteratively adding one factor and optimize the graph once.
        # If in iterative mode, optimize to get a better state estimation. 
        if not s.optimiser_batch:
            # 猜测一组初始值。
            # guess a set of initial values
            self.guess_initial_values()
            
            # 创建优化器 construct optimiser
            if s.optimiser is None:
                s.optimiser = s.optimiser_type(s.optimiser_params)
            try:
                # pu.db
                # 更新当前优化器的因子和值。 update current optimiser's factors and values
                s.optimiser.update(
                    new_factors(s.graph, s.optimiser.getFactorsUnsafe()),
                    new_values(s.estimates,
                               s.optimiser.getLinearizationPoint()))

                # 计算得到最新的状态估计。 calculate new estimation
                s.estimates = s.optimiser.calculateEstimate()
            except RuntimeError as e:
                # For handling gtsam::InderminantLinearSystemException:
                #   https://gtsam.org/doxygen/a03816.html
                pass
            if self.on_new_estimate:
                self.on_new_estimate(self.state)

        # 向后推进一个步骤。 step to next state
        self.state.prev_step = n


    # 归零，没什么好说的 reset to zero.
    def reset(self) -> None:
        self.data_source.restart()

        s = self.state.system
        s.associated = []
        s.unassociated = []
        s.labels = {}
        s.graph = gtsam.NonlinearFactorGraph()
        s.estimates = gtsam.Values()
        s.optimiser = (None if s.optimiser_batch else s.optimiser_type(
            s.optimiser_params))

        s.calib_depth = self.data_source.calib_depth()
        s.calib_rgb = self.data_source.calib_rgb()

        self.state.prev_step = None
        self.state.this_step = None
