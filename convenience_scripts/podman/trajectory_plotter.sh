sequence_path=sequences/evaluation_2023_07_v1.json

obvi_xlim_min=-30
obvi_xlim_max=30
obvi_ylim_min=-30
obvi_ylim_max=70
obvi_figsize_x=3.6
obvi_figsize_y=6
obvi_datadir=/root/LTOV-SLAM-Evaluation/data/ut_vslam_results/evaluation_2023_07_v1/base7a_1_fallback_a_2
obvi_savepath=figs/obvi.png
python src/evaluation/trajectory_sequence_topdown_plotter.py \
    --datadir $obvi_datadir \
    --sequence_path $sequence_path \
    --savepath $obvi_savepath \
    --xlim_min $obvi_xlim_min \
    --xlim_max $obvi_xlim_max \
    --ylim_min $obvi_ylim_min \
    --ylim_max $obvi_ylim_max \
    --figsize_x $obvi_figsize_x \
    --figsize_y $obvi_figsize_y \
    --is_obvi_slam

orb_xlim_min=-30
orb_xlim_max=80
orb_ylim_min=-30
orb_ylim_max=70
orb_figsize_x=6.6
orb_figsize_y=6
orb_datadir=/root/LTOV-SLAM-Evaluation/data/orb_slam_3_out/evaluation_2023_07_v1_orb_finally_probably/
orb_savepath=figs/orb.png
python src/evaluation/trajectory_sequence_topdown_plotter.py \
    --datadir $orb_datadir \
    --sequence_path $sequence_path \
    --savepath $orb_savepath \
    --xlim_min $orb_xlim_min \
    --xlim_max $orb_xlim_max \
    --ylim_min $orb_ylim_min \
    --ylim_max $orb_ylim_max \
    --figsize_x $orb_figsize_x \
    --figsize_y $orb_figsize_y

oa_xlim_min=-30
oa_xlim_max=30
oa_ylim_min=-30
oa_ylim_max=70
oa_figsize_x=3.6
oa_figsize_y=6
oa_datadir=/root/LTOV-SLAM-Evaluation/data/oa_slam_out/evaluation_2023_07_v1_0821_traj_with_waypoints
oa_savepath=figs/oa.png
python src/evaluation/trajectory_sequence_topdown_plotter.py \
    --datadir $oa_datadir \
    --sequence_path $sequence_path \
    --savepath $oa_savepath \
    --xlim_min $oa_xlim_min \
    --xlim_max $oa_xlim_max \
    --ylim_min $oa_ylim_min \
    --ylim_max $oa_ylim_max \
    --figsize_x $oa_figsize_x \
    --figsize_y $oa_figsize_y

droid_xlim_min=-30
droid_xlim_max=30
droid_ylim_min=-30
droid_ylim_max=70
droid_figsize_x=3.6
droid_figsize_y=6
droid_datadir=/root/LTOV-SLAM-Evaluation/data/droid_slam_out/evaluation_2023_07_v1
droid_savepath=figs/droid.png
python src/evaluation/trajectory_sequence_topdown_plotter.py \
    --datadir $droid_datadir \
    --sequence_path $sequence_path \
    --savepath $droid_savepath \
    --xlim_min $droid_xlim_min \
    --xlim_max $droid_xlim_max \
    --ylim_min $droid_ylim_min \
    --ylim_max $droid_ylim_max \
    --figsize_x $droid_figsize_x \
    --figsize_y $droid_figsize_y