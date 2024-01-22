# Copyright (c) 2017 Linaro Limited.
# Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
#
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing Ambiq Apollo devices with using the Ambiq bootloader.'''

from os import path

from runners.core import ZephyrBinaryRunner, RunnerCaps

import os
import sys

class AmbiqBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for Ambiq bootloader.'''

    def __init__(self, cfg, port, reset, asb, baud):
        super().__init__(cfg)
        self.app_bin = cfg.bin_file
        self.port = port
        self.reset = bool(reset)
        self.asb = asb
        self.baud = baud

    @classmethod
    def name(cls):
        return 'ambiq'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        # Required
        parser.add_argument('--asb', required=True,
                            help='path to asb.py')
        parser.add_argument('--port', default='/dev/ttyUSB0',
                            help='serial port to use')
        parser.add_argument('--baud', default='115200',
                            help='baud rate to use')
        # Optional
        parser.set_defaults(reset=True)

    @classmethod
    def do_create(cls, cfg, args):
        return AmbiqBinaryRunner(
            cfg,
            port=args.port,
            reset=args.reset,
            asb=args.asb,
            baud=args.baud,
        )

    def do_run(self, command, **kwargs):
        # self.require(self.asb)

        cmd_flash = [sys.executable, self.asb,
            '-port', self.port,
            '-b', self.baud,
            '--bin', self.app_bin,
            '-o', 'build/zephyr/application',
            '--load-address-blob', '0x20000',
            '--load-address-wired', '0xC000',
            '--magic-num', '0xCB',
            '-i', '6'
            ]

        if not self.reset:
            cmd_flash.extend(['-r', '0'])

        self.logger.info("Flashing ambiq chip on {} ({}bps)".format(self.port, self.baud))
        self.check_call(cmd_flash)
