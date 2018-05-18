#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

class Utility{
	public:
		Utility(){}
		static float GPSDist(float lat1, float lon1, float lat2, float lon2);
		static float GPSDistFast(float lat1, float lon1, float lat2, float lon2);
		static float GPSBearing(float lat1, float lon1, float lat2, float lon2);
		static float GPStoCartesian(float lat, float gpslong);
		static float CartesiantoGPS(float x, float y);
		static glm::vec3 QuaternionToEuler(float x, float y, float z, float w);
		static glm::quat EulerToQuaternion(float x, float y, float z);
		static float** ReadGPSCoordinates(std::string filepath);
};
#endif
