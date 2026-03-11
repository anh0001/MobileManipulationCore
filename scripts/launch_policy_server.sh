#!/bin/bash
source ${HOME}/miniconda3/etc/profile.d/conda.sh
conda activate mobile_manipulation_vla

export OPENVLA_MODEL_ID="openvla/openvla-7b"
export OPENVLA_DEVICE="cuda"
export OPENVLA_ATTENTION_IMPL="flash_attention_2"
export OPENVLA_UNNORM_KEY="bridge_orig"

cd /srv/data/users/anhar/codes/MobileManipulationCore

echo "[$(date)] Starting policy server..." >> ${HOME}/policy_server.log
exec python3 -m manipulation_policy.policy_server --host 0.0.0.0 --port 30542 >> ${HOME}/policy_server.log 2>&1
