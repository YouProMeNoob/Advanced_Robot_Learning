#!/bin/bash

BASHRC="$HOME/.bashrc"

# Define the block to add
read -r -d '' CONDA_BLOCK <<'EOF'
# >>> Conda initialize >>>
__conda_setup="$('/opt/conda/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    export PATH="/opt/conda/bin:$PATH"
fi
unset __conda_setup

conda activate llm_env
source /root/catkin_ws/devel/setup.bash
EOF

# Only add if not already present
if ! grep -q 'conda activate llm_env' "$BASHRC"; then
    echo "Appending Conda and ROS setup to $BASHRC"
    printf "\n%s\n" "$CONDA_BLOCK" >> "$BASHRC"
else
    echo "Conda/ROS block already in $BASHRC — skipping"
fi

apt update
apt install -y terminator
terminator

