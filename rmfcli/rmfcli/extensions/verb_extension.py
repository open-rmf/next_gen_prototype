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

from .extension_base import ExtensionBase


class VerbExtension(ExtensionBase):
    NAME = None

    def __init__(self) -> None:
        super(VerbExtension, self).__init__()

    def main(self, *, parser: ArgumentParser, args: Any) -> int | None:
        raise NotImplementedError()
        pass
