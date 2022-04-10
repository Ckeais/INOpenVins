# Data Evaluation

Data evaluation is done with MATLAB and the [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation). MATLAB is used to convert the rosbag file to txt file in order to use [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation).

## MATLAB
`rosbag_to_mat.m` is used to converted the rosbag file to `.mat` file. `save_to_txt.m` is used to convert the `.mat` file to txt to be processed in `rpg_trajectory_evaluation`. For example, for the Indoor_5 dataset. the final txt file is produced are `indoor_5_gt.txt`, `indoor_5_openvins.txt` and `indoor_5_ours`.


## rpg_trajectory_evaluation
This toolbox is used to generate the trajectories comparsion and error plot.


### Install
The package is written in python and tested in Ubuntu 16.04 and 18.04.
Currently only `python2` is supported.
The package can be used as a ROS package as well as a standalone tool.
To use it as a ROS package, simply clone it into your workspace.
It only depends on [`catkin_simple`](https://github.com/catkin/catkin_simple) to build.

**Dependencies**: You will need install the following:

* `numpy` and `matplotlib` for the analysis/plotting
* `colorama` for colored console output
* `ruamel.yaml` ([install](https://pypi.org/project/ruamel.yaml/)) for [preserving the order in yaml configurations](https://stackoverflow.com/questions/5121931/in-python-how-can-you-load-yaml-mappings-as-ordereddicts)

### Usage Example
For the Indoor_5 dataset processed by MATLAB. the dataset file is stored and renamed in `laptop/laptop` following the naming convection of the toolbox. Our INOpenVINS result is stored in the `laptop/laptop/Modified`. The ground truth txt file it renamed to `stamped_groundtruth.txt` and the estimated result is renamed to `stamped_traj_estimate.txt`. Similarily, results generated with the original OpenVINS is stored in the `laptop/laptop/OpenVINS` file.\
To regenerate the result for Indoor_5, run:
```
python2 scripts/analyze_trajectories.py \
  OpenVINS.yaml --output_dir=./laptop/ --results_dir=./laptop/ --platform laptop --odometry_error_per_dataset --plot_trajectories --rmse_table --rmse_boxplot 
```

The results will then stored in the `laptop/laptop_IN_5_result` for our Indoor_5 dataset.

### Credit
Zichao Zhang, Davide Scaramuzza: A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry, IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS), 2018.

```
@InProceedings{Zhang18iros,
  author = {Zhang, Zichao and Scaramuzza, Davide},
  title = {A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry},
  booktitle = {IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS)},
  year = {2018}
}
```