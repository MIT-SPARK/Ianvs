import contextlib

import ianvs._ianvs_bindings as _bindings

import numpy as np
from sensor_msgs.msg import Image, CompressedImage


def get_image(msg: Image | CompressedImage) -> np.ndarray | None:
    """Return appropriate image from message if possible."""
    if isinstance(msg, CompressedImage):
        import imageio.v3

        return imageio.v3.imread(msg.data.tobytes())

    info = _bindings.parse_encoding(msg.encoding)
    if info is None:
        return None

    shape = (msg.height, msg.width, info.channels)
    return np.squeeze(np.frombuffer(msg.data, dtype=info.dtype_str).reshape(shape))


@contextlib.contextmanager
def init_node_handle(node_name: str):
    guard = _bindings._NodeInitHolder(node_name)
    yield guard.name
    del guard
