from libcpp cimport bool
from libcpp.string cimport string
cimport _cdff_types


cdef extern from "StereoMotionEstimationInterface.hpp" namespace "CDFF::DFN":
    cdef cppclass StereoMotionEstimationInterface:
        StereoMotionEstimationInterface()
        void process() except +
        void setConfigurationFile(string) except +
        void configure() except +

        void framePairInput(_cdff_types.asn1SccFramePair& data) except +
        void disparityInput(_cdff_types.asn1SccFrame& data) except +

        _cdff_types.asn1SccTransformWithCovariance& poseOutput() except +


cdef extern from "StereoMotionEstimationEdres.hpp" namespace "CDFF::DFN::StereoMotionEstimation":
    cdef cppclass StereoMotionEstimationEdres(StereoMotionEstimationInterface):
        pass
