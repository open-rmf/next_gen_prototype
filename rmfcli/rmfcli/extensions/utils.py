from argparse import ArgumentParser, RawDescriptionHelpFormatter, _SubParsersAction
from collections import OrderedDict
from typing import Any, List

from .extension_base import ExtensionBase


def get_first_line_doc(any_type: Any) -> str:
    if not any_type.__doc__:
        return ""
    lines: List[str] = any_type.__doc__.splitlines()
    if not lines:
        return ""
    if lines[0]:
        line = lines[0]
    elif len(lines) > 1:
        line = lines[1]
    return line.strip().rstrip(".")


def add_subparsers(
    parser: ArgumentParser,
    cli_name: str,
    dest: str,
    extensions: OrderedDict[str, ExtensionBase],
    required: bool = True,
    argv: Any = None,
) -> _SubParsersAction:
    # Generate description
    description = ""
    max_length = max(len(name) for name in extensions.keys())
    for name, extension in extensions.items():
        description += "%s  %s\n" % (name.ljust(max_length), get_first_line_doc(extension))

    # Create subparser
    subparsers = parser.add_subparsers(
        title="Commands",
        description=description,
        metavar=f"Call `{cli_name} <command> -h` for more detailed usage.",
    )

    subparsers.dest = " " + dest.lstrip("_")
    subparsers.required = required
    for name, extension in extensions.items():
        command_parser = subparsers.add_parser(name, formatter_class=RawDescriptionHelpFormatter)
        extension.add_arguments(command_parser, name, argv=argv)
        command_parser.set_defaults(**{dest: extension})

    return subparsers
