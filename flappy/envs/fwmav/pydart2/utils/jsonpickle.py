import sys


def init_numpy_handler():
    print("register jsonpickle.ext.numpy handlers...")
    try:
        import jsonpickle.ext.numpy
        jsonpickle.ext.numpy.register_handlers()
    except Exception:
        e = sys.exc_info()[1]
        print("exception = " + str(e))
        print("register jsonpickle.ext.numpy handlers... NG")
        return
    print("register jsonpickle.ext.numpy handlers... OK")
