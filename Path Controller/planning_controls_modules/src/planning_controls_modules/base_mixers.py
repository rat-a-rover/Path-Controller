#!/usr/bin/env python

import functools
import pdb

REDUCER = 'reducer'
COMPACT = 'compact'

class BaseMixer(object):
    __type__ = COMPACT

    @classmethod
    def __reducer__(cls, args):
        #pdb.set_trace()
        return functools.reduce(getattr(cls, '__reduce__'), args)

    @classmethod
    def __compact__(cls, args):
        return args[0]

    @classmethod
    def mix(cls, istream):
        if type(istream) is list:
            ostream = getattr(cls, '__'+cls.__type__+'__')(istream)
        else:
            ostream = istream

        return ostream

class AdderMixer(BaseMixer):
    __type__ = REDUCER

    @classmethod
    def __reduce__(cls, arg_1, arg_2):
        arg_3 = dict()
        arg_3.update({k: None for k in arg_1.keys()})
        arg_3.update({k: None for k in arg_2.keys()})

        for k in arg_3.keys():
            arg_3[k] = str(float(arg_1.setdefault(k, 0)) + \
                           float(arg_2.setdefault(k, 0)))

        return arg_3
