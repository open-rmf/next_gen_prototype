from .extension_base import ExtensionBase


class CommandExtension(ExtensionBase):
    NAME = None

    def __init__(self) -> None:
        super(CommandExtension, self).__init__()
