sequence_path=sequences/evaluation_2023_07_v1.json
# root_datadir=/robodata/taijing/object-slam/oslam/
root_datadir=/root/LTOV-SLAM-Evaluation/data/

obvi_xlim_min=-30
obvi_xlim_max=30
obvi_ylim_min=-30
obvi_ylim_max=70
obvi_figsize_x=3.6
obvi_figsize_y=6
# obvi_datadir=/root/LTOV-SLAM-Evaluation/data/ut_vslam_results/evaluation_2023_07_v1/base7a_1_fallback_a_2
obvi_datadir=${root_datadir}ut_vslam_results/evaluation_2023_07_v1/base7a_1_fallback_a_2
obvi_savepath=figs/TopdownTrajObVi.png
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
orb_datadir=${root_datadir}orb_slam_3_out/evaluation_2023_07_v1_orb_finally_probably/
orb_savepath=figs/TopdownTrajORB.png
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
oa_datadir=${root_datadir}oa_slam_out/evaluation_2023_07_v1_0821_traj_with_waypoints
oa_savepath=figs/TopdownTrajOA.png
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
droid_datadir=${root_datadir}droid_slam_out/evaluation_2023_07_v1
droid_savepath=figs/TopdownTrajDroid.png
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

legoloam_xlim_min=-30
legoloam_xlim_max=30
legoloam_ylim_min=-30
legoloam_ylim_max=70
legoloam_figsize_x=3.6
legoloam_figsize_y=6
legoloam_datadir=${root_datadir}lego_loam_formatted_out/evaluation_2023_07_v1
legoloam_savepath=figs/TopdownTrajLeGOLOAMNoWps.png
python src/evaluation/trajectory_sequence_topdown_plotter.py \
    --datadir $legoloam_datadir \
    --sequence_path $sequence_path \
    --savepath $legoloam_savepath \
    --xlim_min $legoloam_xlim_min \
    --xlim_max $legoloam_xlim_max \
    --ylim_min $legoloam_ylim_min \
    --ylim_max $legoloam_ylim_max \
    --figsize_x $legoloam_figsize_x \
    --figsize_y $legoloam_figsize_y \
    --no_duplicate_waypoints