# distutils: language=c++
cimport _stereomotionestimation
cimport stereomotionestimation
from cython.operator cimport dereference as deref
from libcpp cimport bool
from libcpp.string cimport string
cimport cdff_types
cimport _cdff_types


cdef class StereoMotionEstimationEdres:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.delete_thisptr and self.thisptr != NULL:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _stereomotionestimation.StereoMotionEstimationEdres()
        self.delete_thisptr = True

    def process(self):
        self.thisptr.process()

    def set_configuration_file(self, str configuration_file_path):
        cdef string path = configuration_file_path.encode()
        self.thisptr.setConfigurationFile(path)

    def configure(self):
        self.thisptr.configure()

    def framePairInput(self, cdff_types.FramePair data):
        cdef _cdff_types.asn1SccFramePair * cpp_data = data.thisptr
        self.thisptr.framePairInput(deref(cpp_data))

    def disparityInput(self, cdff_types.Frame data):
        cdef _cdff_types.asn1SccFrame * cpp_data = data.thisptr
        self.thisptr.disparityInput(deref(cpp_data))

    def poseOutput(self):
        cdef cdff_types.TransformWithCovariance out = cdff_types.TransformWithCovariance()
        out.thisptr[0] = self.thisptr.poseOutput()
        return out

