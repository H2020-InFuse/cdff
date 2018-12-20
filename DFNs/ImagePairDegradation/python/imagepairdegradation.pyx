# distutils: language=c++
cimport _imagepairdegradation
cimport imagepairdegradation
from cython.operator cimport dereference as deref
from libcpp cimport bool
from libcpp.string cimport string
cimport cdff_types
cimport _cdff_types


cdef class ImagePairDegradation:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.delete_thisptr and self.thisptr != NULL:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _imagepairdegradation.ImagePairDegradation()
        self.delete_thisptr = True

    def process(self):
        self.thisptr.process()

    def set_configuration_file(self, str configuration_file_path):
        cdef string path = configuration_file_path.encode()
        self.thisptr.setConfigurationFile(path)

    def configure(self):
        self.thisptr.configure()

    def originalImagePairInput(self, cdff_types.FramePair data):
        cdef _cdff_types.asn1SccFramePair * cpp_data = data.thisptr
        self.thisptr.originalImagePairInput(deref(cpp_data))

    def degradedImagePairOutput(self):
        cdef cdff_types.FramePair out = cdff_types.FramePair()
        out.thisptr[0] = self.thisptr.degradedImagePairOutput()
        return out



cdef class ImagePairDegradationEdres:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.delete_thisptr and self.thisptr != NULL:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _imagepairdegradation.ImagePairDegradationEdres()
        self.delete_thisptr = True

    def process(self):
        self.thisptr.process()

    def set_configuration_file(self, str configuration_file_path):
        cdef string path = configuration_file_path.encode()
        self.thisptr.setConfigurationFile(path)

    def configure(self):
        self.thisptr.configure()

    def originalImagePairInput(self, cdff_types.FramePair data):
        cdef _cdff_types.asn1SccFramePair * cpp_data = data.thisptr
        self.thisptr.originalImagePairInput(deref(cpp_data))

    def degradedImagePairOutput(self):
        cdef cdff_types.FramePair out = cdff_types.FramePair()
        out.thisptr[0] = self.thisptr.degradedImagePairOutput()
        return out

