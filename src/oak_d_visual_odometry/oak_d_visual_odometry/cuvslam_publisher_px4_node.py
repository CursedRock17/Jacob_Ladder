"""PX4 entry point for the OAK-D PyCuVSLAM publisher."""

from .cuvslam_publisher_node import main as _main


def main(args=None):
    _main(args=args, publish_px4=True)


if __name__ == "__main__":
    main()
