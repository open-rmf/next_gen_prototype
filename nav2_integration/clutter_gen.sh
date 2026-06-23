#!/bin/bash
set -e

# Spawns a given number of cubes in random positions in the warehouse world.
# Usage:
#   ./clutter_gen.sh <number_of_cubes>

if [[ $# -eq 0 ]] ; then
    echo "Error: Number of cubes to spawn not provided."
    echo "Usage: $0 <number_of_cubes>"
    exit 1
fi

N_CUBES=$1

echo "Spawning ${N_CUBES} cubes..."
touch clutter.csv
for i in $(seq 1 $N_CUBES);
do
    MODEL_NAME="random_box_${i}"

    # Generate a random position between (-10, -10) and (10, 10)
    X_POS=$(awk -v seed=$RANDOM 'BEGIN{srand(seed); printf "%.2f", -10+rand()*20}')
    Y_POS=$(awk -v seed=$RANDOM 'BEGIN{srand(seed); printf "%.2f", -10+rand()*20}')
    POSE="${X_POS} ${Y_POS} 0 0 0 0"

    # Using single quotes for attributes is valid in XML and simplifies quoting in bash
    SDF_CONTENT="<sdf version='1.6'><model name='${MODEL_NAME}'><pose>${POSE}</pose><static>true</static><link name='link'><visual name='v'><geometry><box><size>0.2 0.2 1</size></box></geometry></visual><collision name='c'><geometry><box><size>0.2 0.2 1</size></box></geometry></collision></link></model></sdf>"

    echo $POSE >> clutter.csv
    # Spawn the model in Gazebo using the /world/warehouse/create service
    gz service -s /world/warehouse/create \
      --reqtype gz.msgs.EntityFactory \
      --reptype gz.msgs.Boolean \
      --timeout 300 \
      --req "sdf: \"${SDF_CONTENT}\""
done

echo "Successfully spawned ${N_CUBES} cubes."
