#!/bin/bash
# usage: bash run_rule_agents.sh {vehicle_type} {num_blue} {formation_mode}
python rule_leader.py $1 $2 $3&
python avoid.py $1 $2 vel &
# python red_random_vel.py $1 $2 &
uav_id=1
while(( $uav_id< $2 )) 
do
    python follower.py $1 $uav_id $2 &
    python red_random_vel.py $1 $2 $uav_id &
    let "uav_id++"
done
