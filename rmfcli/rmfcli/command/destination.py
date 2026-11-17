# Copyright 2026 Chen Bainian
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from argparse import ArgumentParser
from typing import Any

from rmfcli.extensions import CommandExtension, VerbExtension
from rmfcli.extensions.utils import add_subparsers
from rmfcli.verb.destination import extensions as verb_extensions


class DestinationCommand(CommandExtension):
    """Run Destination CLI sub-commands"""

    def add_arguments(self, parser: ArgumentParser, cli_name: str, *, argv: Any = None) -> None:
        self._subparser = parser
        add_subparsers(parser, cli_name, "_verb", verb_extensions, required=False)

    def main(self, *, parser: ArgumentParser, args: Any) -> None | int:
        if not hasattr(args, "_verb"):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension: VerbExtension = args._verb

        # call the verb's main method
        rc: None | int = extension.main(parser=parser, args=args)
        return rc
