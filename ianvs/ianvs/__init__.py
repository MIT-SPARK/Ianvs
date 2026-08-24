from ianvs._ianvs_bindings import parse_encoding
from sensor_msgs.msg import Image, CompressedImage
import numpy as np


def get_image(msg: Image | CompressedImage) -> np.ndarray | None:
    """Return appropriate image from message if possible."""
    if isinstance(msg, CompressedImage):
        import imageio.v3

        return imageio.v3.imread(msg.data.tobytes())

    info = parse_encoding(msg.encoding)
    if info is None:
        return None

    shape = (msg.height, msg.width, info.channels)
    return np.squeeze(np.frombuffer(msg.data, dtype=info.dtype_str).reshape(shape))
