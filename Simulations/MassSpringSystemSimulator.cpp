#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() 
{
	m_iIntegrator = EULER;
	gravity = 0.0f;
	complexSceneSize = 5;
	pointList.clear();
	springList.clear();
}

const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "2Point,ComplexScene";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "step=0.1 min=0");
	TwType TW_TYPE_METHOD = TwDefineEnumFromString("Method", "Euler,LeapFrog(NotDone),MidPoint");
	TwAddVarRW(DUC->g_pTweakBar, "Method", TW_TYPE_METHOD, &m_iIntegrator, "");
	switch (m_iTestCase) {
	case 0:break;
	case 1:
		TwAddVarRW(DUC->g_pTweakBar, "Size", TW_TYPE_INT32, &complexSceneSize, "min=3");
		break;
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (m_iTestCase == 1 && pointList.size() != complexSceneSize * complexSceneSize)
		setUpComplexScene();
	for (auto& spring : springList) {
		Vec3 pos1 = pointList[spring.p1].pos;
		Vec3 pos2 = pointList[spring.p2].pos;
		DUC->beginLine();
		DUC->drawLine(pos1, Vec3(1, 0, 0), pos2, Vec3(0, 1, 0));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	printf("\n");
	switch (m_iTestCase) {
	case 0:
		cout << "2-Point !\n";
		setUp2PointScene();
		break;
	case 1:
		cout << "Complex Scene !\n";
		setUpComplexScene();
		break;
	}
	printf("\n");
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{

}

void MassSpringSystemSimulator::calcForces(vector<Vec3> &forceList, const vector<Point> &pointList) {
	forceList.clear();
	m_externalForce.y = -gravity;
	for (int i = 0; i < pointList.size(); i++) {
		forceList.push_back(m_externalForce);
	}
	for (auto &spring : springList) {
		Vec3 tmpX = pointList[spring.p2].pos - pointList[spring.p1].pos;
		Vec3 tmpF = m_fStiffness * (norm(tmpX) - spring.initLen) * tmpX / norm(tmpX);
		forceList[spring.p1] += tmpF;
		forceList[spring.p2] -= tmpF;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	vector<Vec3> forceList;
	calcForces(forceList, pointList);
	if (m_iIntegrator == EULER) {
		for (int i = 0; i < pointList.size();i++) {
			auto& point = pointList[i];
			if (!point.isFixed) {
				point.pos += point.velocity * timeStep;
				point.velocity += forceList[i] / m_fMass * timeStep;
			}
		}
	}
	else if (m_iIntegrator == LEAPFROG) {

	}
	else if (m_iIntegrator == MIDPOINT) {
		vector<Point> midPointList;
		for (int i = 0; i < pointList.size(); i++) {
			auto point = pointList[i];
			if (!point.isFixed) {
				point.pos += point.velocity * timeStep / 2;
				point.velocity += forceList[i] / m_fMass * timeStep / 2;
			}
			midPointList.push_back(point);
		}
		calcForces(forceList, midPointList);
		for (int i = 0; i < pointList.size(); i++) {
			auto& point = pointList[i];
			if (!point.isFixed) {
				point.pos += midPointList[i].velocity * timeStep;
				point.velocity += forceList[i] / m_fMass * timeStep;
			}
		}
	}
	for (int i = 0; i < pointList.size(); i++) {
		Point& point = pointList[i];
		if (point.pos.y < -1) {
			point.pos.y = -1;
			point.velocity.y = -point.velocity.y * 0.8;
		}
		printf("Point %d: Pos( %.2f, %.2f, %.2f), v( %.2f, %.2f, %.2f)\n",
			i, point.pos[0], point.pos[1], point.pos[2], point.velocity[0], point.velocity[1], point.velocity[2]);
	}
	printf("\n");
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int index = pointList.size();
	Point tmp = Point(position, Velocity, isFixed);
	pointList.push_back(tmp);
	return index;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	springList.push_back(Spring(masspoint1, masspoint2, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return pointList.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springList.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	if (index >= pointList.size()) {
		cout << "Index Error !\n";
		return Vec3();
	}
	return pointList[index].pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	if (index >= pointList.size()) {
		cout << "Index Error !\n";
		return Vec3();
	}
	return pointList[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
	gravity = -m_externalForce.y;
}

void MassSpringSystemSimulator::setUp2PointScene()
{
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	applyExternalForce(Vec3(0, 0, 0));
	pointList.clear();
	springList.clear();
	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	addSpring(p0, p1, 1.0);

	cout << "Scene Setup Successfully !\n";
}

void MassSpringSystemSimulator::setUpComplexScene()
{
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	applyExternalForce(Vec3(0, -1, 0));
	pointList.clear();
	springList.clear();

	int n = complexSceneSize;
	int y = 2;

	for (int i = 0; i < n * n; i++) {
		float x = i % n - float(n - 1) / 2;
		float z = (i / n) % n - float(n - 1) / 2;
		if (i==0 || i==n-1 || i==n*(n-1) || i==n*n-1)
			addMassPoint(Vec3(x, y, z), Vec3(), true);
		else 
			addMassPoint(Vec3(x, y, z), Vec3(), false);
	}
	for (int i=0;i<n-1;i++)
		for (int j = 0; j < n; j++) {
			int p1 = j * n + i;
			addSpring(p1, p1 + 1, 1.0f);
			int p2 = i * n + j;
			addSpring(p2, p2 + n, 1.0f);
		}

	cout << "Scene Setup Successfully !\n";
}