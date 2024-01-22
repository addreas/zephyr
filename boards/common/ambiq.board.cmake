# SPDX-License-Identifier: Apache-2.0

board_set_flasher_ifnset(ambiq)

board_finalize_runner_args(ambiq "--asb=${ZEPHYR_HAL_AMBIQ_MODULE_DIR}/scripts/asb.py")
