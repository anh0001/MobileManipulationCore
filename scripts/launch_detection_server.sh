#!/bin/bash
source ${HOME}/miniconda3/etc/profile.d/conda.sh
conda activate mobile_manipulation_vla

export PYTHONPATH=/srv/data/users/anhar/codes/MobileManipulationCore/build/manipulation_detection:$PYTHONPATH

cd /srv/data/users/anhar/codes/MobileManipulationCore

echo "[$(date)] Starting detection server..." >> ${HOME}/detection_server.log
exec python3 -m manipulation_detection.detection_server --host 0.0.0.0 --port 30543 >> ${HOME}/detection_server.log 2>&1
