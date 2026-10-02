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

from argparse import ArgumentParser, RawDescriptionHelpFormatter
from signal import SIGINT
from typing import Any, Optional

from .command import extensions as command_extensions
from .extensions import ExtensionBase
from .extensions.utils import add_subparsers


def main(
    *, script_name: str = "rmf", argv: Any = None, description: Optional[str] = None
) -> int | str | None:
    if description is None:
        description = f"{script_name} is an extensible command-line tool for RMF"

    # top level parser
    parser = ArgumentParser(description=description, formatter_class=RawDescriptionHelpFormatter)
    extension_key = "_command"
    add_subparsers(
        parser, script_name, extension_key, command_extensions, required=False, argv=argv
    )

    # register argcomplete hook if available
    try:
        from argcomplete import autocomplete
    except ImportError:
        pass
    else:
        autocomplete(parser, exclude=["-h", "--help"], always_complete_options=False)

    args = parser.parse_args(args=argv)

    extension: Optional[ExtensionBase] = getattr(args, extension_key, None)

    # handle the case that no command was passed
    if extension is None:
        parser.print_help()
        return 0

    # call the main method of the extension
    try:
        rc = extension.main(parser=parser, args=args)
    except KeyboardInterrupt:
        rc = SIGINT
    except RuntimeError as e:
        rc = str(e)
    return rc
