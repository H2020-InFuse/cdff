# distutils: language=c++
cimport _imagedegradation
cimport imagedegradation
from cython.operator cimport dereference as deref
from libcpp cimport bool
from libcpp.string cimport string
cimport cdff_types
cimport _cdff_types


cdef class ImageDegradation:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.delete_thisptr and self.thisptr != NULL:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _imagedegradation.ImageDegradation()
        self.delete_thisptr = True

    def process(self):
        self.thisptr.process()

    def set_configuration_file(self, str configuration_file_path):
        cdef string path = configuration_file_path.encode()
        self.thisptr.setConfigurationFile(path)

    def configure(self):
        self.thisptr.configure()

    def originalImageInput(self, cdff_types.Frame data):
        cdef _cdff_types.asn1SccFrame * cpp_data = data.thisptr
        self.thisptr.originalImageInput(deref(cpp_data))

    def degradedImageOutput(self):
        cdef cdff_types.Frame out = cdff_types.Frame()
        out.thisptr[0] = self.thisptr.degradedImageOutput()
        return out



cdef class ImageDegradationEdres:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.delete_thisptr and self.thisptr != NULL:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _imagedegradation.ImageDegradationEdres()
        self.delete_thisptr = True

    def process(self):
        self.thisptr.process()

    def set_configuration_file(self, str configuration_file_path):
        cdef string path = configuration_file_path.encode()
        self.thisptr.setConfigurationFile(path)

    def configure(self):
        self.thisptr.configure()

    def originalImageInput(self, cdff_types.Frame data):
        cdef _cdff_types.asn1SccFrame * cpp_data = data.thisptr
        self.thisptr.originalImageInput(deref(cpp_data))

    def degradedImageOutput(self):
        cdef cdff_types.Frame out = cdff_types.Frame()
        out.thisptr[0] = self.thisptr.degradedImageOutput()
        return out

