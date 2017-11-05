from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.string cimport string

cdef extern from "vl_python.h":
    cdef cppclass ViveLibre:
        ViveLibre()
        map[int,vector[float]] pollAngles()
        string get_config()

cdef class PyViveLibre:
    cdef ViveLibre* thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self):
        self.thisptr = new ViveLibre()
    def __dealloc__(self):
        del self.thisptr

    def pollAngles(self):
        return self.thisptr.pollAngles()

    def get_config(self):
        return self.thisptr.get_config()
