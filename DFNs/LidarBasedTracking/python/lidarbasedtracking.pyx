# distutils: language=c++
cimport _lidarbasedtracking
cimport lidarbasedtracking
from cython.operator cimport dereference as deref
from libcpp cimport bool
from libcpp.string cimport string
cimport cdff_types
cimport _cdff_types


cdef class LidarBasedTracking:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.delete_thisptr and self.thisptr != NULL:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _lidarbasedtracking.LidarBasedTracking()
        self.delete_thisptr = True

    def process(self):
        self.thisptr.process()

    def set_configuration_file(self, str configuration_file_path):
        cdef string path = configuration_file_path.encode()
        self.thisptr.setConfigurationFile(path)

    def configure(self):
        self.thisptr.configure()

    def sourceCloudInput(self, cdff_types.Pointcloud data):
        cdef _cdff_types.asn1SccPointcloud * cpp_data = data.thisptr
        self.thisptr.sourceCloudInput(deref(cpp_data))

    def stateOutput(self):
        cdef cdff_types.RigidBodyState out = cdff_types.RigidBodyState()
        out.thisptr[0] = self.thisptr.stateOutput()
        return out

