#include "flip.h"

FlipSimulator::FlipSimulator() {
	m_fRatio = 0.95f;
	m_fOverRelaxation = 1.9f;
	m_bSeparateParticles = true;
	m_bCompensateDrift = true;
	m_externalForce = Vec3(0.0f, -9.8f, 0.0f);
}

// UI functions
const char* FlipSimulator::getTestCasesStr() {
	return "PIC/FLIP";
}

void FlipSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "fRatio", TW_TYPE_FLOAT, &m_fRatio, "step=0.01 min=0 max=1.0");
	TwAddVarRW(DUC->g_pTweakBar, "overRelaxation", TW_TYPE_FLOAT, &m_fOverRelaxation, "step=0.1 min=1.0 max=2.0");
	TwAddVarRW(DUC->g_pTweakBar, "SeparateParticles", TW_TYPE_BOOLCPP, &m_bSeparateParticles, "");
	TwAddVarRW(DUC->g_pTweakBar, "CompensateDrift", TW_TYPE_BOOLCPP, &m_bCompensateDrift, "");
}

void FlipSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void FlipSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (int i = 0; i < m_iNumSpheres; i++) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100.0, m_particleColor[i]);
		DUC->drawSphere(m_particlePos[i], Vec3(m_particleRadius * 2, m_particleRadius * 2, m_particleRadius * 2));
	}
	DUC->beginLine();
	DUC->drawLine(Vec3(-0.5f, -0.5f + m_h, -0.5f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.5f, -0.5f + m_h, -0.5f), Vec3(1.0f, 0.0f, 0.0f));
	DUC->drawLine(Vec3(-0.5f, -0.5f + m_h * 2,-0.5f),Vec3(1.0f,0.0f,0.0f),Vec3(0.5f, -0.5f + m_h * 2,-0.5f),Vec3(1.0f,0.0f,0.0f));
	DUC->endLine();
}

void FlipSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	printf("\n");
	switch (m_iTestCase) {
	case 0:
		cout << "Test !\n";
		setupScene(25);
		break;
	}
	printf("\n");
}

void FlipSimulator::externalForcesCalculations(float timeElapsed) {
	
}

void FlipSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void FlipSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Simulation Functions
void FlipSimulator::integrateParticles(float timeStep) {
	for(int i = 0; i < m_iNumSpheres; i++) {
		m_particleVel[i] += timeStep * m_externalForce;
		m_particlePos[i] += timeStep * m_particleVel[i];
		
	}
}

void FlipSimulator::pushParticlesApart(int numIters) {
	float hCell = 2.2f * m_particleRadius;
	float hinfCell = 1.0f / hCell;
	int hashCellX = floor(1.0f * hinfCell) + 1;
	int hashCellY = hashCellX;
	int hashCellZ = hashCellX;
	int offX = hashCellY * hashCellZ;
	int offY = hashCellZ;
	float minDistance = 2.0f * m_particleRadius;

	std::vector<std::vector<int>> hashTable;
	hashTable.resize(hashCellX * hashCellY * hashCellZ);

	for (int i = 0; i < m_iNumSpheres; i++) {
		int x = floor((m_particlePos[i].x + 0.5f) * hinfCell);
		x = max(0, min(x, hashCellX - 1));
		int y = floor((m_particlePos[i].y + 0.5f) * hinfCell);
		y = max(0, min(y, hashCellY - 1));
		int z = floor((m_particlePos[i].z + 0.5f) * hinfCell);
		z = max(0, min(z, hashCellZ - 1));
		int index = x * offX + y * offY + z;

		hashTable[index].push_back(i);
	}

	for (int iter = 0; iter < numIters; iter++) {
		for (int i = 0; i < m_iNumSpheres; i++) {
			int x = floor((m_particlePos[i].x + 0.5f) * hinfCell);
			x = max(0, min(x, hashCellX - 1));
			int x0 = max(x - 1, 1);
			int x1 = min(x + 1, hashCellX - 2);
			int y = floor((m_particlePos[i].y + 0.5f) * hinfCell);
			x = max(0, min(x, hashCellX - 1));
			int y0 = max(y - 1, 1);
			int y1 = min(y + 1, hashCellY - 2);
			int z = floor((m_particlePos[i].z + 0.5f) * hinfCell);
			x = max(0, min(x, hashCellX - 1));
			int z0 = max(z - 1, 0);
			int z1 = min(z + 0, hashCellZ - 1);

			for (int xi = x0; xi <= x1; xi++) {
				for (int yi = y0; yi <= y1; yi++) {
					for (int zi = z0; zi <= z1; zi++) {
						int index = xi * offX + yi * offY + zi;

						for (auto ptr = hashTable[index].begin(); ptr != hashTable[index].end(); ptr++) {
							int j = *ptr;
							if (j == i) continue;
							Vec3 vd = m_particlePos[i] - m_particlePos[j];
							float distance = norm(vd);
							if (distance >= minDistance || distance == 0.0f) continue;

							Vec3 dpos = 0.5f * (minDistance - distance) * vd / distance;
							m_particlePos[i] += dpos;
							m_particlePos[j] -= dpos;

						}
					}
				}
			}

		}
	}
}

void FlipSimulator::handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel) {
	for (int i = 0; i < m_iNumSpheres; i++) {
		bool noCollision = false;
		int cnt = 0;
		Vec3 tmp;
		while (!noCollision && cnt<5)
		{
			noCollision = true;
			cnt++;
			if (m_particlePos[i].x < -0.5f + m_h + m_particleRadius) {
				m_particlePos[i].x = -0.5f + m_h + m_particleRadius;
				m_particleVel[i].x = 0;
			}
			else if (m_particlePos[i].x > 0.5f - m_particleRadius) {
				m_particlePos[i].x = 0.5f -m_particleRadius;
				m_particleVel[i].x = 0;
			}
			if (m_particlePos[i].z < -0.5f + m_h + m_particleRadius) {
				m_particlePos[i].z = -0.5f + m_h + m_particleRadius;
				m_particleVel[i].z = 0;
			}
			else if (m_particlePos[i].z > 0.5f - m_particleRadius) {
				m_particlePos[i].z = 0.5f - m_particleRadius;
				m_particleVel[i].z = 0;
			}
			if (m_particlePos[i].y < -0.5f + m_h + m_particleRadius) {
				m_particlePos[i].y = -0.5f + m_h + m_particleRadius;
				m_particleVel[i].y = 0;
			}
			else if (m_particlePos[i].y > 0.5f - m_particleRadius) {
				m_particlePos[i].y = 0.5f - m_particleRadius;
				m_particleVel[i].y = 0;
			}
			/*
			tmp = m_particlePos[i] - obstaclePos;
			if (normalize(tmp) < obstacleRadius + m_particleRadius) {
				tmp *= sqrt(obstacleRadius + m_particleRadius);
				m_particlePos[i] = obstaclePos + tmp;
				m_particleVel[i] = Vec3(0.0f, 0.0f, 0.0f); // TODO
			}
			*/
		}
	}
}

void FlipSimulator::updateParticleDensity() {
	m_particleDensity.clear(); m_particleDensity.resize(m_iNumCells, 0.0f);
	int offset = 0.5f * m_h;

	for (int i = 0; i < m_iNumSpheres; i++) {
		float x = max((float)m_particlePos[i].x + 0.5f, m_h + m_particleRadius);
		int x0 = floor((x - offset) * m_fInvSpacing);
		float dx = (x - offset - x0 * m_h) * m_fInvSpacing;
		int x1 = min(x0 + 1, m_iCellX - 1);
		float y = max((float)m_particlePos[i].y + 0.5f, m_h + m_particleRadius);
		int y0 = floor((y - offset) * m_fInvSpacing);
		float dy = (y - offset - y0 * m_h) * m_fInvSpacing;
		int y1 = min(y0 + 1, m_iCellY - 1);
		float z = max((float)m_particlePos[i].z + 0.5f, m_h + m_particleRadius);
		int z0 = floor((z - offset) * m_fInvSpacing);
		float dz = (z - offset - z0 * m_h) * m_fInvSpacing;
		int z1 = min(z0 + 1, m_iCellZ - 1);

		m_particleDensity[calcIndex(x0, y0, z0)] += (1 - dx) * (1 - dy) * (1 - dz);
		m_particleDensity[calcIndex(x0, y0, z1)] += (1 - dx) * (1 - dy) * dz;
		m_particleDensity[calcIndex(x0, y1, z0)] += (1 - dx) * dy * (1 - dz);
		m_particleDensity[calcIndex(x0, y1, z1)] += (1 - dx) * dy * dz;
		m_particleDensity[calcIndex(x1, y0, z0)] += dx * (1 - dy) * (1 - dz);
		m_particleDensity[calcIndex(x1, y0, z1)] += dx * (1 - dy) * dz;
		m_particleDensity[calcIndex(x1, y1, z0)] += dx * dy * (1 - dz);
		m_particleDensity[calcIndex(x1, y1, z1)] += dx * dy * dz;
	}

	if (m_particleRestDensity == 0.0f) {
		float sum = 0.0f;
		int num = 0;
		for (int i = 0; i < m_iNumCells; i++) {
			if (m_type[i] == FLUID_CELL) {
				sum += m_particleDensity[i];
				num++;
			}
		}
		if (num > 0)
			m_particleRestDensity = sum / num;
	}
}

void FlipSimulator::transferVelocities(bool toGrid, float flipRatio) {
	const float off_x[3] = { 0.0f, 0.5f * m_h, 0.5f * m_h };
	const float off_y[3] = { 0.5f * m_h, 0.0f, 0.5f * m_h };
	const float off_z[3] = { 0.5f * m_h, 0.5f * m_h, 0.0f };
	std::vector<float> w; 
	if (toGrid) {
		m_pre_vel.assign(m_vel.begin(), m_vel.end());
		m_vel.clear(); m_vel.resize(m_iNumCells, Vec3(0.0f));
		for (int i = 0; i < m_iNumCells; i++)
			m_type[i] = (m_s[i] == 0) ? SOLID_CELL : EMPTY_CELL;
		for (int i = 0; i < m_iNumSpheres; i++) {
			int x = floor((m_particlePos[i].x + 0.5f) * m_fInvSpacing);
			int y = floor((m_particlePos[i].y + 0.5f) * m_fInvSpacing);
			int z = floor((m_particlePos[i].z + 0.5f) * m_fInvSpacing);
			int index = calcIndex(x, y, z);
			if (m_type[index] == EMPTY_CELL)
				m_type[index] = FLUID_CELL;
		}
		for (int dimension = 0; dimension < 3; dimension++) {
			w.clear();
			w.resize(m_iNumCells, 0.0f);
			for (int i = 0; i < m_iNumSpheres; i++) {
				int x0 = floor((m_particlePos[i].x + 0.5f - off_x[dimension]) * m_fInvSpacing);
				float dx = (m_particlePos[i].x + 0.5f -off_x[dimension] - x0 * m_h) * m_fInvSpacing;
				int x1 = min(x0 + 1, m_iCellX - 1);
				int y0 = floor((m_particlePos[i].y + 0.5f - off_y[dimension]) * m_fInvSpacing);
				float dy = (m_particlePos[i].y + 0.5f - off_y[dimension] - y0 * m_h) * m_fInvSpacing;
				int y1 = min(y0 + 1, m_iCellY - 1);
				int z0 = floor((m_particlePos[i].z + 0.5f - off_z[dimension]) * m_fInvSpacing);
				float dz = (m_particlePos[i].z + 0.5f - off_z[dimension] - z0 * m_h) * m_fInvSpacing;
				int z1 = min(z0 + 1, m_iCellZ - 1);

				float tmpV = m_particleVel[i][dimension];
				m_vel[calcIndex(x0, y0, z0)][dimension] += tmpV * (1 - dx) * (1 - dy) * (1 - dz); w[calcIndex(x0, y0, z0)] += (1 - dx) * (1 - dy) * (1 - dz);
				m_vel[calcIndex(x0, y0, z1)][dimension] += tmpV * (1 - dx) * (1 - dy) * dz; w[calcIndex(x0, y0, z1)] += (1 - dx) * (1 - dy) * dz;
				m_vel[calcIndex(x0, y1, z0)][dimension] += tmpV * (1 - dx) * dy * (1 - dz); w[calcIndex(x0, y1, z0)] += (1 - dx) * dy * (1 - dz);
				m_vel[calcIndex(x0, y1, z1)][dimension] += tmpV * (1 - dx) * dy * dz; w[calcIndex(x0, y1, z1)] += (1 - dx) * dy * dz;
				m_vel[calcIndex(x1, y0, z0)][dimension] += tmpV * dx * (1 - dy) * (1 - dz); w[calcIndex(x1, y0, z0)] += dx * (1 - dy) * (1 - dz);
				m_vel[calcIndex(x1, y0, z1)][dimension] += tmpV * dx * (1 - dy) * dz; w[calcIndex(x1, y0, z1)] += dx * (1 - dy) * dz;
				m_vel[calcIndex(x1, y1, z0)][dimension] += tmpV * dx * dy * (1 - dz); w[calcIndex(x1, y1, z0)] += dx * dy * (1 - dz);
				m_vel[calcIndex(x1, y1, z1)][dimension] += tmpV * dx * dy * dz; w[calcIndex(x1, y1, z1)] += dx * dy * dz;
			}
			for (int x = 0; x < m_iCellX; x++) 
				for (int y = 0;y < m_iCellY;y++)
					for (int z = 0;z < m_iCellZ;z++) {
						int i = calcIndex(x, y, z);
						if (w[i] > 0.0f)
							m_vel[i][dimension] /= w[i];
						if (m_type[i] == SOLID_CELL || (dimension==0 && x > 0 && m_type[calcIndex(x - 1, y, z)] == SOLID_CELL)
							|| (dimension == 1 && y > 0 && m_type[calcIndex(x, y - 1, z)] == SOLID_CELL)
							|| (dimension == 2 && z > 0 && m_type[calcIndex(x, y, z - 1)] == SOLID_CELL)) {
							m_vel[i][dimension] = m_pre_vel[i][dimension];
						}
			}
		}
		
	}
	else {
		for (int dimension = 0; dimension < 3; dimension++) {
			for (int i = 0; i < m_iNumSpheres; i++) {
				int x0 = floor((m_particlePos[i].x + 0.5f - off_x[dimension]) * m_fInvSpacing);
				//x0 = max(1, x0);
				float dx = (m_particlePos[i].x + 0.5f - off_x[dimension] - x0 * m_h) * m_fInvSpacing;
				int x1 = min(x0 + 1, m_iCellX - 1);
				int y0 = floor((m_particlePos[i].y + 0.5f - off_y[dimension]) * m_fInvSpacing);
				//y0 = max(1, y0);
				float dy = (m_particlePos[i].y + 0.5f - off_y[dimension] - y0 * m_h) * m_fInvSpacing;
				int y1 = min(y0 + 1, m_iCellY - 1);
				int z0 = floor((m_particlePos[i].z + 0.5f - off_z[dimension]) * m_fInvSpacing);
				//z0 = max(1, z0);
				float dz = (m_particlePos[i].z + 0.5f - off_z[dimension] - z0 * m_h) * m_fInvSpacing;
				int z1 = min(z0 + 1, m_iCellZ - 1);

				float oldV = m_particleVel[i][dimension];
				float d = 0.0f;
				float picV = 0.0f; float dV = 0.0f;
				int tmp = 0;
				int offset = (dimension == 0) ? m_iCellY * m_iCellZ : ((dimension == 1) ? m_iCellZ : 1);

				tmp = calcIndex(x0, y0, z0);
				if (m_type[tmp] != EMPTY_CELL || m_type[tmp - offset] != EMPTY_CELL) {
					picV += m_vel[tmp][dimension] * (1 - dx) * (1 - dy) * (1 - dz);
					dV += (m_vel[tmp][dimension] - m_pre_vel[tmp][dimension]) * (1 - dx) * (1 - dy) * (1 - dz);
					d += (1 - dx) * (1 - dy) * (1 - dz);
				}
				tmp = calcIndex(x0, y0, z1);
				if (m_type[tmp] != EMPTY_CELL || (m_type[tmp - offset] != EMPTY_CELL)) {
					picV += m_vel[tmp][dimension] * (1 - dx) * (1 - dy) * dz;
					dV += (m_vel[tmp][dimension] - m_pre_vel[tmp][dimension]) * (1 - dx) * (1 - dy) * dz;
					d += (1 - dx) * (1 - dy) * dz;
				}
				tmp = calcIndex(x0, y1, z0);
				if (m_type[tmp] != EMPTY_CELL || (m_type[tmp - offset] != EMPTY_CELL)) {
					picV += m_vel[tmp][dimension] * (1 - dx) * dy * (1 - dz);
					dV += (m_vel[tmp][dimension] - m_pre_vel[tmp][dimension]) * (1 - dx) * dy * (1 - dz);
					d += (1 - dx) * dy * (1 - dz);
				}
				tmp = calcIndex(x0, y1, z1);
				if (m_type[tmp] != EMPTY_CELL || (m_type[tmp - offset] != EMPTY_CELL)) {
					picV += m_vel[tmp][dimension] * (1 - dx) * dy * dz;
					dV += (m_vel[tmp][dimension] - m_pre_vel[tmp][dimension]) * (1 - dx) * dy * dz;
					d += (1 - dx) * dy * dz;
				}
				tmp = calcIndex(x1, y0, z0);
				if (m_type[tmp] != EMPTY_CELL || (m_type[tmp - offset] != EMPTY_CELL)) {
					picV += m_vel[tmp][dimension] * dx * (1 - dy) * (1 - dz);
					dV += (m_vel[tmp][dimension] - m_pre_vel[tmp][dimension]) * dx * (1 - dy) * (1 - dz);
					d += dx * (1 - dy) * (1 - dz);
				}
				tmp = calcIndex(x1, y0, z1);
				if (m_type[tmp] != EMPTY_CELL || (m_type[tmp - offset] != EMPTY_CELL)) {
					picV += m_vel[tmp][dimension] * dx * (1 - dy) * dz;
					dV += (m_vel[tmp][dimension] - m_pre_vel[tmp][dimension]) * dx * (1 - dy) * dz;
					d += dx * (1 - dy) * dz;
				}
				tmp = calcIndex(x1, y1, z0);
				if (m_type[tmp] != EMPTY_CELL && (m_type[tmp - offset] != EMPTY_CELL)) {
					picV += m_vel[tmp][dimension] * dx * dy * (1 - dz);
					dV += (m_vel[tmp][dimension] - m_pre_vel[tmp][dimension]) * dx * dy * (1 - dz);
					d += dx * dy * (1 - dz);
				}
				tmp = calcIndex(x1, y1, z1);
				if (m_type[tmp] != EMPTY_CELL || (m_type[tmp - offset] != EMPTY_CELL)) {
					picV += m_vel[tmp][dimension] * dx * dy * dz;
					dV += (m_vel[tmp][dimension] - m_pre_vel[tmp][dimension]) * dx * dy * dz;
					d += dx * dy * dz;
				}

				if (d > 0.0f) {
					m_particleVel[i][dimension] = (1.0f - flipRatio) * picV / d + flipRatio * (oldV + dV / d);
				}
			}
		}
	}
}

void FlipSimulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
	m_pre_vel.assign(m_vel.begin(), m_vel.end());
	for (int iter = 0; iter < numIters; iter++) {

		for (int x = 1; x < m_iCellX - 1; x++) {
			for (int y = 1; y < m_iCellY - 1; y++) {
				for (int z = 1; z < m_iCellZ - 1; z++) {
					int center = calcIndex(x, y, z);
					if (m_type[center] != FLUID_CELL)
						continue;
					int left = calcIndex(x - 1, y, z);
					int right = calcIndex(x + 1, y, z);
					int top = calcIndex(x, y + 1, z);
					int bottom = calcIndex(x, y - 1, z);
					int front = calcIndex(x, y, z + 1);
					int back = calcIndex(x, y, z - 1);
					int s = m_s[left] + m_s[right] + m_s[top] + m_s[bottom] + m_s[front] + m_s[back];
					if (s == 0) continue;

					float d = m_vel[right].x - m_vel[center].x + m_vel[top].y - m_vel[center].y + m_vel[front].z - m_vel[center].z;
					if (m_particleRestDensity > 0.0f && compensateDrift) {
						float compression = m_particleDensity[center] - m_particleRestDensity;
						if (compression > 0.0f)
							d -= 1.0f * compression;
					}

					float p = -(d / (float)s) * overRelaxation;

					m_vel[center].x -= m_s[left] * p;
					m_vel[right].x += m_s[right] * p;
					m_vel[center].y -= m_s[bottom] * p;
					m_vel[top].y += m_s[top] * p;
					m_vel[center].z -= m_s[back] * p;
					m_vel[front].z += m_s[front] * p;
				}
			}
		}
	}
}

void FlipSimulator::updateParticleColors() {
	/*for (int i = 0; i < m_iNumSpheres; i++) {
		m_particleColor[i] = Vec3(0.5f);

		int x = floor((m_particlePos[i].x + 0.5f) * m_fInvSpacing);
		x = max(1, min(x, m_iCellX - 1));
		int y = floor((m_particlePos[i].y + 0.5f) * m_fInvSpacing);
		y = max(1, min(y, m_iCellY - 1));
		int z = floor((m_particlePos[i].z + 0.5f) * m_fInvSpacing);
		z = max(1, min(z, m_iCellZ - 1));

		if (m_particleRestDensity > 0.0f) {
			float relDentisy = m_particleDensity[calcIndex(x, y, z)] / m_particleRestDensity;
			if (relDentisy > 0.7) {
				m_particleColor[i] = Vec3(0.2f, 0.2f, 1.0f);
			}
		}
	}*/
}
