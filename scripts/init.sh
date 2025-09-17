#!/bin/bash

west init -m https://github.com/HXZP/zephyr-hxzp.git --mf hxzp.yml

west update
source zephyr-sdk/zephyr/zephyr-env.sh

# install pip package dependencies
west packages pip --install

# install toolchain
west sdk install --install-dir zephyr-sdk/toolchain \
     --toolchains arm-zephyr-eabi

cp -s zephyr-hxzp/scripts/env.sh env.sh
