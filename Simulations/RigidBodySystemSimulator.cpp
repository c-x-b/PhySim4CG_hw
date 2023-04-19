#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	rigidBodyList.clear();
	forceList.clear();
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "1 Body, 2 Body, Complex Scene";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (auto& rigidBody : rigidBodyList) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawRigidBody(calcObject2WorldMat(rigidBody));
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	printf("\n");
	switch (m_iTestCase) {
	case 0:
		cout << "1 Body!" << endl;
		setUp1BodyScene();
		break;
	case 1:
		cout << "2 Body!" << endl;
		setUp2BodyScene();
		break;
	case 2:
		cout << "Complex Scene!" << endl;
		setUpComplexScene();
		break;
	}
	printf("\n");
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if ((mouseDiff.x != 0 || mouseDiff.y != 0) && m_iTestCase==0) {
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		float inputScale = 0.01f;
		inputWorld = inputWorld * inputScale;
		applyForceOnBody(0, Vec3(0, 0, 0), inputWorld);
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	int bodyNum = rigidBodyList.size();
	vector<Vec3> bodyForceList(bodyNum);
	vector<Vec3> bodyTorqueList(bodyNum);
	vector<Mat4> bodyRotMatList(bodyNum);
	vector<Mat4> body2WorldMatList(bodyNum);
	for (int i = 0; i < bodyNum; i++) {
		bodyRotMatList[i] = rigidBodyList[i].ori.getRotMat();
		body2WorldMatList[i] = calcObject2WorldMat(rigidBodyList[i]);
	}
	// collision
	const float c = 1.0;
	for (int i=0;i<bodyNum-1;i++)
		for (int j = i+1; j < bodyNum; j++) {
			if ((rigidBodyList[i].fixed && rigidBodyList[j].fixed) ) continue;
			CollisionInfo info = checkCollisionSAT(body2WorldMatList[i], body2WorldMatList[j]);
			if (!info.isValid) continue;
			Vec3& n = info.normalWorld;
			Vec3 xi = info.collisionPointWorld - rigidBodyList[i].pos, xj = info.collisionPointWorld - rigidBodyList[j].pos;
			Vec3 vi = rigidBodyList[i].linearV + cross(rigidBodyList[i].angularV, xi);
			Vec3 vj = rigidBodyList[j].linearV + cross(rigidBodyList[j].angularV, xj);
			Vec3 tmpi = cross(rigidBodyList[i].inertia.inverse().transformVector(cross(xi, n)), xi);
			Vec3 tmpj = cross(rigidBodyList[j].inertia.inverse().transformVector(cross(xj, n)), xj);
			float vRel = dot(n, (vi - vj));
			if (vRel > 0) {
				cout << "collision but seperating found!\n";
				cout << i << "," << j << endl;
				cout << "normal: " << n << " vRel: " << vi - vj << endl;
				cout << "vi: " << vi << " vj: " << vj << endl;
				cout << "collisionPoint: " << info.collisionPointWorld << endl;
				cout << endl;
				continue;
			}
			float J = (-(1 + c) * vRel) / (1 / rigidBodyList[i].mass + 1 / rigidBodyList[j].mass + dot(tmpi + tmpj, n));
			cout << "collision found!\n";
			cout << i << "," << j << ": " << J << endl;
			cout << "collisionPoint: " << info.collisionPointWorld << endl;
			if (!rigidBodyList[i].fixed) {
				rigidBodyList[i].linearV += J * n / rigidBodyList[i].mass;
				rigidBodyList[i].angularV += rigidBodyList[i].inertia.inverse().transformVector(cross(xi, J * n));
			}
			if (!rigidBodyList[j].fixed) {
				rigidBodyList[j].linearV -= J * n / rigidBodyList[j].mass;
				rigidBodyList[j].angularV -= rigidBodyList[j].inertia.inverse().transformVector(cross(xj, J * n));
			}
		}
	// apply force
	int tmpI;
	for (auto& force : forceList) {
		tmpI = force.index;
		bodyForceList[tmpI] += force.force;
		Vec3 rotLoc = force.loc - rigidBodyList[tmpI].pos;
		bodyTorqueList[tmpI] += cross(rotLoc, force.force);
	}

	for (int i = 0; i < bodyNum; i++) {
		if (rigidBodyList[i].fixed) continue;
		Vec3 a = bodyForceList[i] / rigidBodyList[i].mass;
		rigidBodyList[i].pos += rigidBodyList[i].linearV * timeStep;
		rigidBodyList[i].linearV += a * timeStep;

		Mat4 rotMatTran = bodyRotMatList[i];
		rotMatTran.transpose();
		Mat4 actualI = rotMatTran * rigidBodyList[i].inertia * bodyRotMatList[i];
		actualI = actualI.inverse();
		Vec3 tmp = rigidBodyList[i].angularV * timeStep / 2.0f;
		Quat deltaOri = Quat(tmp.x, tmp.y, tmp.z, 0) * rigidBodyList[i].ori;
		rigidBodyList[i].ori += deltaOri;
		rigidBodyList[i].ori = rigidBodyList[i].ori.unit();
		rigidBodyList[i].angularV += actualI.transformVector(bodyTorqueList[i]) * timeStep;
	}

	forceList.clear();
}

void RigidBodySystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void RigidBodySystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return rigidBodyList.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return rigidBodyList[i].pos;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return rigidBodyList[i].linearV;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return rigidBodyList[i].angularV;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	forceList.push_back(Force(i, loc, force));
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, float mass) {
	rigidBodyList.push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	rigidBodyList[i].ori = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	rigidBodyList[i].linearV = velocity;
}

void RigidBodySystemSimulator::setFixed(int i) {
	rigidBodyList[i].fixed = true;
}

void RigidBodySystemSimulator::setUp1BodyScene() {
	rigidBodyList.clear();
	forceList.clear();
	addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
	applyForceOnBody(0, Vec3(0.0, 0.0f, 0.0), Vec3(0, 0, 200));
}

void RigidBodySystemSimulator::setUp2BodyScene() {
	rigidBodyList.clear();
	forceList.clear();
	addRigidBody(Vec3(-0.1f, 0.0f, -0.5f), Vec3(0.4f, 0.2f, 0.2f), 1.0f);
	addRigidBody(Vec3(0.2f, 0.0f, 0.5f), Vec3(0.4f, 0.2f, 0.2f), 1.0f);
	setOrientationOf(0, Quat(Vec3(1.0f, 1.0f, 0), (float)(M_PI) * 0.25f));
	setVelocityOf(0, Vec3(0.0f, 0.0f, 0.1f));
	setVelocityOf(1, Vec3(0.0f, 0.0f, -0.1f));
}

void RigidBodySystemSimulator::setUpComplexScene() {
	rigidBodyList.clear();
	forceList.clear();
	addRigidBody(Vec3(-0.1f, 0.0f, -0.5f), Vec3(0.4f, 0.2f, 0.2f), 1.0f);
	addRigidBody(Vec3(0.2f, 0.0f, 0.5f), Vec3(0.4f, 0.2f, 0.2f), 1.0f);

	addRigidBody(Vec3(0, 0, 1.0f), Vec3(1.9f, 2.0f, 0.1f), 100.0f);
	addRigidBody(Vec3(0, 0, -1.0f), Vec3(1.9f, 2.0f, 0.1f), 100.0f);
	addRigidBody(Vec3(1.0f, 0, 0), Vec3(0.1f, 2.0f, 1.9f), 100.0f);
	addRigidBody(Vec3(-1.0f, 0, 0), Vec3(0.1f, 2.0f, 1.9f), 100.0f);
	setFixed(2);
	setFixed(3);
	setFixed(4);
	setFixed(5);

	setOrientationOf(0, Quat(Vec3(1.0f, 1.0f, 0), (float)(M_PI) * 0.25f));
	setVelocityOf(0, Vec3(0.0f, 0.0f, 0.1f));
	setVelocityOf(1, Vec3(0.0f, 0.0f, -0.1f));
}

Mat4 RigidBodySystemSimulator::calcObject2WorldMat(const RigidBody& body) {
	Mat4 Obj2WorldMatrix;
	Obj2WorldMatrix.initScaling(body.size.x, body.size.y, body.size.z);
	Obj2WorldMatrix *= body.ori.getRotMat();
	Obj2WorldMatrix *= Mat4(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		body.pos.x, body.pos.y, body.pos.z, 1);
	return Obj2WorldMatrix;
}