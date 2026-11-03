from typing import OrderedDict

from rmfcli.extensions import ExtensionBase

from .send import SendVerb
from .dummy import DummyVerb

extensions: OrderedDict[str, ExtensionBase] = OrderedDict()
extensions["send"] = SendVerb()
extensions["dummy"] = DummyVerb()
