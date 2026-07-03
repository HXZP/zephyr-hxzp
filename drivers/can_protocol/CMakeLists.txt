# Copyright (c) 2026 HXZP
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_HXZP_CAN_PROTOCOL can_protocol.c)
