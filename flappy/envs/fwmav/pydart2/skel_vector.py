import numpy as np


class SkelVector(np.ndarray):
    def __new__(cls, data=None, skel=None):
        if data is None:
            data = np.zeros(skel.ndofs)
        obj = np.asarray(data).view(cls)
        obj.skel = skel
        obj.keys = [d.name for d in skel.dofs]
        return obj

    def to_index(self, key):
        if isinstance(key, int):
            if key < 0:
                return len(self) + key
            else:
                return key
        elif isinstance(key, str):
            return self.keys.index(key)
        else:
            return None

    def __getitem__(self, key):
        if isinstance(key, str):
            index = self.to_index(key)
            return self.__getitem__(index)
        elif isinstance(key, list) or isinstance(key, tuple):
            indices = [self.to_index(x) for x in key]
            ret = self.__getitem__(np.array(indices))
            ret.keys = [self.keys[i] for i in indices]
            return ret
        else:
            return np.ndarray.__getitem__(self, key)

    def __getslice__(self, start, end):
        ret = np.ndarray.__getslice__(self, start, end)
        ret.keys = self.keys[start:end]
        return ret

    def __setitem__(self, key, value):
        if isinstance(key, str):
            index = self.to_index(key)
            return self.__setitem__(index, value)
        elif isinstance(key, list) or isinstance(key, tuple):
            indices = [self.to_index(x) for x in key]
            return self.__setitem__(np.array(indices), value)
        else:
            return np.ndarray.__setitem__(self, key, value)

    def __contains__(self, key):
        return key in self.keys

    def __array_finalize__(self, obj):
        if obj is None:
            return
        self.skel = getattr(obj, 'skel', None)
        self.keys = getattr(obj, 'keys', None)
