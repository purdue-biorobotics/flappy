import time
from functools import partial


def decorator_time_measure(func, text_decor=None):
    def time_func_wrapper(self, *args, **kwargs):
        import time
        tic = time.time()
        ret = func(self, *args, **kwargs)
        toc = time.time()
        txt = "[%s] toc-tic = %.8fs" % (func.__name__, toc - tic)
        if text_decor is not None:
            txt = text_decor(txt)
        if hasattr(self, "logger"):
            self.logger.info(txt)
        else:
            print(txt)
        return ret
    return time_func_wrapper


def decorator_time_measure_colored(text_decor):
    return partial(decorator_time_measure, text_decor=text_decor)


class TimeMeasure(object):
    def __init__(self, logger=None, name=None, color=None):
        self.logger = logger
        self.name = name
        self.color = color

    def __enter__(self, ):
        self.tic = time.time()

    def __exit__(self, type, value, traceback):
        self.toc = time.time()
        txt = "" if self.name is None else self.name
        txt += "toc-tic = %.8f" % (self.toc - self.tic)
        if self.color:
            txt = self.color(txt)
        if self.logger is not None:
            self.logger.info(txt)
        else:
            print(txt)
