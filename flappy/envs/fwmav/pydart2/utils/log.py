import sys
import logging


def setup(filename=None):
    logging.basicConfig()

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.handlers = []
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.DEBUG)
    logfmt = '[%(levelname)s][%(asctime)s][%(name)s:%(lineno)d] %(message)s'
    formatter = logging.Formatter(logfmt)
    ch.setFormatter(formatter)
    root.addHandler(ch)

    # set up logging to console
    if filename is not None:
        # filename = "output.log"
        console = logging.FileHandler(filename, "w+")
        console.setLevel(logging.DEBUG)
        console.setFormatter(formatter)
        root.addHandler(console)

    logging.addLevelName(logging.FATAL,
                         "\033[1;41m%s\033[1;0m" %
                         logging.getLevelName(logging.FATAL))
    logging.addLevelName(logging.ERROR,
                         "\033[1;41m%s\033[1;0m" %
                         logging.getLevelName(logging.ERROR))
    logging.addLevelName(logging.WARNING,
                         "\033[1;43m%s\033[1;0m" %
                         logging.getLevelName(logging.WARNING))
    logging.addLevelName(logging.INFO,
                         "\033[1;42m%s\033[1;0m" %
                         logging.getLevelName(logging.INFO))
    logging.addLevelName(logging.DEBUG,
                         "\033[1;44m%s\033[1;0m" %
                         logging.getLevelName(logging.DEBUG))
