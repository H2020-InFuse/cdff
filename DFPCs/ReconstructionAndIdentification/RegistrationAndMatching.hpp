/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef REGISTRATIONANDIDENTIFICATION_REGISTRATIONANDMATCHING_HPP
#define REGISTRATIONANDIDENTIFICATION_REGISTRATIONANDMATCHING_HPP

#include "ReconstructionAndIdentificationInterface.hpp"
#include <Reconstruction3D/RegistrationFromStereo.hpp>
#include <PointCloudModelLocalisation/FeaturesMatching3D.hpp>

namespace CDFF
{
namespace DFPC
{
namespace ReconstructionAndIdentification
{
	/**
	 * This DFN Chains reconstruct a 3D scene using the RegistrationFromStereo DFPC implementation and 
	 * then it finds the position of the model within the scene using the FeaturesMatching3D DFPC Implementation
	 */
	class RegistrationAndMatching : public ReconstructionAndIdentificationInterface
	{
	public:
		RegistrationAndMatching();
		~RegistrationAndMatching();
		void run();
		void setup();
		void modelInput(const asn1SccPointcloud& data);

	private:
		bool modelFeaturesAvailable;
		CDFF::DFPC::Reconstruction3D::RegistrationFromStereo* registrationFromStereo;
		CDFF::DFPC::PointCloudModelLocalisation::FeaturesMatching3D* featuresMatching3d;
	};
}
}
}

#endif // REGISTRATIONANDIDENTIFICATION_REGISTRATIONANDMATCHING_HPP

/** @} */
