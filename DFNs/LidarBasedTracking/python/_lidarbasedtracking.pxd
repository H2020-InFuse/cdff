from libcpp cimport bool
from libcpp.string cimport string
cimport _cdff_types


cdef extern from "LidarBasedTrackingInterface.hpp" namespace "CDFF::DFN":
    cdef cppclass LidarBasedTrackingInterface:
        LidarBasedTrackingInterface()
        void process() except +
        void setConfigurationFile(string) except +
        void configure() except +

        void sourceCloudInput(_cdff_types.asn1SccPointcloud& data) except +

        _cdff_types.asn1SccRigidBodyState& stateOutput() except +


cdef extern from "LidarBasedTracking.hpp" namespace "CDFF::DFN::LidarBasedTracking":
    cdef cppclass LidarBasedTracking(LidarBasedTrackingInterface):
        pass
