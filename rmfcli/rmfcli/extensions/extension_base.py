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
# limitations under the License.from argparse import ArgumentParser

from argparse import ArgumentParser
from typing import Any


class ExtensionBase:
    """
    The extension point for 'command' extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods must be defined:
    * `main`

    The following methods can be defined:
    * `add_arguments`
    """

    NAME = None

    def __init__(self) -> None:
        super(ExtensionBase, self).__init__()

    def add_arguments(self, parser: ArgumentParser, cli_name: str, *, argv: Any = None) -> None:
        pass

    def main(self, *, parser: ArgumentParser, args: Any) -> int | None:
        raise NotImplementedError()
        pass
