/**
 * @addtogroup DFNs
 * @{
 */

#ifndef TRANSFORM3DESTIMATION_INTERFACE_HPP
#define TRANSFORM3DESTIMATION_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Pose.h>
#include <CorrespondenceMap3D.h>
#include <Sequences.h>
#include <Eigen.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that estimates the geometric transformation between two sets of matched 3d points
     */
    class Transform3DEstimationInterface : public CDFF::DFN::DFNCommonInterface
    {
        public:

            Transform3DEstimationInterface();
            virtual ~Transform3DEstimationInterface();

            /**
             * Send value to input port "matches"
             * @param matches: keypoint matches between the 3d points in n cartesian systems O1, O2, ... On. 
	     *        Each correspondence map contains pairs of 3d points map(A-B) ={ ((x^A_1, y^A_1, z^A_1), 
	     *        (x^B_1, y^B_1, z^B_1)), ..., ((x^A_n, y^A_n, z^A_n), (x^B_n, y^B_n, z^B_n)) }. 
	     *        Each pair contains the coordinates of the same 3d points in
	     *        two coordinate systems. The correspondence maps appear in the following order,
	     *        map(O1-O2), map(01-03), ..., map(01-0n), map(O2-O3), ..., map(O2-On), ..., map(On-1, On)
             */
            virtual void matchesInput(const asn1SccCorrespondenceMaps3DSequence& data);

            /**
             * Query value from output port "transforms"
             * @return transforms: this is an estimation of the poses of O2 in O1, O3 in O1, ..., On in O, 
	     *         O2 in O3, O2 in O4, ..., O2 in On, ..., On-1 in On.
             *         A pose has null quaternion if it was impossible to compute, as computation
	     *         may fail if matches are not good enough.
             */
            virtual const asn1SccPosesSequence& transformsOutput() const;
            /**
             * Query value from output port "success"
             * @return success: boolean flag indicating successful computation of the pose of O2 in O1, 
	     *         only when the input matches contain a single map, map(01-02). If the
	     *         matches contain multiple maps, this output is always true.
             */
            virtual bool successOutput() const;
            /**
             * Query value from output port "error"
             * @return error: The square error of the optimal transform estimation
             */
            virtual float errorOutput() const;

        protected:

            asn1SccCorrespondenceMaps3DSequence inMatches;
            asn1SccPosesSequence outTransforms;
            bool outSuccess;
	    float outError;
    };
}
}

#endif // TRANSFORM3DESTIMATION_INTERFACE_HPP

/** @} */
