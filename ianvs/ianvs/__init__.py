from ianvs._ianvs_bindings import parse_encoding
import sensor_msgs.msg
from typing import Optional
import numpy as np

import imageio.v3


def get_image(msg: sensor_msgs.msg.Image) -> Optional[np.ndarray]:
    """Return appropriate image from message if possible."""
    info = parse_encoding(msg.encoding)
    if info is None:
        return None

    shape = (msg.height, msg.width, info.channels)
    return np.squeeze(np.frombuffer(msg.data, dtype=info.dtype_str).reshape(shape))


def _parse_image(msg):
    if msg is None:
        return None

    if "CompressedImage" in type(msg).__name__:
        return imageio.v3.imread(msg.data.tobytes())

    return get_image(msg)
