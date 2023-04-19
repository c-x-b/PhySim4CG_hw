#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

struct RigidBody {
	Vec3 pos;
	Quat ori;
	Vec3 linearV;
	Vec3 angularV;
	Vec3 size;
	float mass;
	Mat4 inertia;
	bool fixed;
	RigidBody(Vec3 _pos, Vec3 _size, float _mass):
		pos(_pos), ori(Quat(0,0,0,1)), linearV(Vec3(0,0,0)), angularV(Vec3(0,0,0)), size(_size), mass(_mass),inertia(Mat4()), fixed(false) {
		float sqrx = size.x * size.x * mass / 12.0f, sqry = size.y * size.y * mass / 12.0f, sqrz = size.z * size.z * mass / 12.0f;
		inertia.initScaling(sqry + sqrz, sqrx + sqrz, sqrx + sqry);
		cout << "Inertia: " << inertia << endl;
	}
};

struct Force {
	int index;
	Vec3 loc;
	Vec3 force;
	Force(int _index, Vec3 _loc, Vec3 _force):
		index(_index), loc(_loc), force(_force) {}
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	
	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, float mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void setFixed(int i);
	
	void setUp1BodyScene();
	void setUp2BodyScene();
	void setUpComplexScene();
	Mat4 calcObject2WorldMat(const RigidBody& body);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	vector<RigidBody> rigidBodyList;
	vector<Force> forceList;

	const float uN = 0.8;
	const float uT = 0.8;
	};
#endif