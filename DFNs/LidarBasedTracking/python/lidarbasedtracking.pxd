from libcpp cimport bool
from cython.operator cimport dereference as deref
cimport _lidarbasedtracking


cdef class LidarBasedTracking:
    cdef _lidarbasedtracking.LidarBasedTracking* thisptr
    cdef bool delete_thisptr
