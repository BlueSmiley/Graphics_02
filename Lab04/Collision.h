#ifndef _COLLISION_HEADER_
#define _COLLISION_HEADER_
#include "Model.h"
#include "maths_funcs.h"

struct Ray;
struct RayInfo;
struct RayHitModel;

struct Ray {
	vec3 origin;
	vec3 direction;
	Ray(vec3 rayOrigin, vec3 rayDirection) {
		origin = rayOrigin;
		direction = rayDirection;
	}
};

struct RayInfo {
	float tmin;
	float tmax;
};

struct RayHitModel {
	Model* model;
	RayInfo hitInfo;
};

//simple rollback based
std::vector<Model*> checkForCollisions(Model moving, std::vector<Model*> background);
bool checkBoxCollision(BoundingBox one, BoundingBox two);
std::vector<BoundingBox> calculateTransformedHitboxes(Model moving);
BoundingBox autoCalculateBoundingBox(Model model);

BoundingBox OBBToABB(BoundingBox original);
bool intersection(Ray ray, BoundingBox box);
std::vector<Model*> checkForRayCasts(Ray ray, std::vector<Model*> background);

#endif
