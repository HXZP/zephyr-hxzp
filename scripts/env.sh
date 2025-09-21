#!/bin/bash

source .venv/bin/activate
source ./zephyr-sdk/zephyr/zephyr-env.sh

CUR_PATH=$(dirname "$(readlink -f \"$0\")")
export ZEPHYR_SDK_INSTALL_DIR="$CUR_PATH/zephyr-sdk/toolchain"
