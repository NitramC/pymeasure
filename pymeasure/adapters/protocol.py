#
# This file is part of the PyMeasure package.
#
# Copyright (c) 2013-2022 PyMeasure Developers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

import logging

from .adapter import Adapter

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())


class ProtocolAdapter(Adapter):
    """ Adapter class testing command exchange without instrument hardware.

    :param kwargs: TBD key-word arguments
    """

    def __init__(self, preprocess_reply=None, **kwargs):
        super().__init__(preprocess_reply=preprocess_reply)
        # TODO: Make this skeleton implementation workable

    # TODO: Harmonise ask being write+read (i.e., remove from VISAAdapter),
    #   use protocol tests to confirm it works correctly
    # TODO: Remove all now-unnecessary ask() implementations -
    #   read and write implementations should have all (test this)
    # TODO: Implement over-the-wire message-traffic logging here, first
    # TODO: Check timing impact of non-firing logging messages
    # TODO: Roll out message-traffic logging to all Adapters, try to DRY
