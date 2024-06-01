#pragma once
#ifndef PARTICLE_RENDER
#include <vector>

#include "PxPhysicsAPI.h"

#include "snippetrender/SnippetRender.h"
#include "snippetrender/Camera.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, MathLib::HTransform3& camera);

namespace
{
	MathLib::Camera* sCamera;

	void renderCallback()
	{
		stepPhysics(true);

		Snippets::startRender(sCamera);

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbDynamicActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC );
		if (nbDynamicActors)
		{
			std::vector<PxRigidActor*> dynamicActors(nbDynamicActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, reinterpret_cast<PxActor**>(&dynamicActors[0]), nbDynamicActors);
			Snippets::renderActors(&dynamicActors[0], static_cast<PxU32>(dynamicActors.size()), true);
		}

		PxU32 nbStaticActors = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC);
		if (nbStaticActors)
		{
			std::vector<PxRigidActor*> staticActors(nbStaticActors);
			scene->getActors(PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&staticActors[0]), nbStaticActors);
			Snippets::renderActors(&staticActors[0], static_cast<PxU32>(staticActors.size()), true,PxVec3(0.7,0,0));
		}
		Snippets::showFPS();
		Snippets::finishRender();
	}

	void exitCallback(void)
	{
		delete sCamera;
		cleanupPhysics(true);
	}
}


void renderLoop()
{
	sCamera = new MathLib::Camera(MathLib::HVector3(50.0f, 50.0f, 50.0f), MathLib::HVector3(-0.6f, -0.2f, -0.7f));

	Snippets::setupDefault("PhysX Snippet HelloWorld", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	Snippets::initFPS();
	glutMainLoop();
}
#else
#include <vector>

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "snippetrender/SnippetRender.h"
#include "snippetrender/Camera.h"
#define CUDA_SUCCESS 0
#define SHOW_SOLID_SDF_SLICE 0
#define IDX(i, j, k, offset) ((i) + dimX * ((j) + dimY * ((k) + dimZ * (offset))))
using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern PxPBDParticleSystem* getParticleSystem();
extern PxParticleAndDiffuseBuffer* getParticleBuffer();

extern int getNumDiffuseParticles();


namespace
{
	MathLib::Camera* sCamera;

	Snippets::SharedGLBuffer sPosBuffer;
	Snippets::SharedGLBuffer sDiffusePosLifeBuffer;

	void onBeforeRenderParticles()
	{
	}

	void renderParticles()
	{

		PxPBDParticleSystem* particleSystem = getParticleSystem();
		if (particleSystem)
		{
			PxParticleAndDiffuseBuffer* userBuffer = getParticleBuffer();
			PxVec4* positions = userBuffer->getPositionInvMasses();
			PxVec4* diffusePositions = userBuffer->getDiffusePositionLifeTime();

			const PxU32 numParticles = userBuffer->getNbActiveParticles();
			const PxU32 numDiffuseParticles = userBuffer->getNbActiveDiffuseParticles();

			PxScene* scene;
			PxGetPhysics().getScenes(&scene, 1);
			PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();

			cudaContextManager->acquireContext();

			PxCudaContext* cudaContext = cudaContextManager->getCudaContext();
			cudaContext->memcpyDtoH(sPosBuffer.map(), CUdeviceptr(positions), sizeof(PxVec4) * numParticles);
			cudaContext->memcpyDtoH(sDiffusePosLifeBuffer.map(), CUdeviceptr(diffusePositions), sizeof(PxVec4) * numDiffuseParticles);

			cudaContextManager->releaseContext();

#if SHOW_SOLID_SDF_SLICE
			particleSystem->copySparseGridData(sSparseGridSolidSDFBufferD, PxSparseGridDataFlag::eGRIDCELL_SOLID_GRADIENT_AND_SDF);
#endif
		}

		sPosBuffer.unmap();
		sDiffusePosLifeBuffer.unmap();
		PxVec3 color(0.5f, 0.5f, 1);
		Snippets::DrawPoints(sPosBuffer.vbo, sPosBuffer.size / sizeof(PxVec4), color, 2.f);

		PxParticleAndDiffuseBuffer* userBuffer = getParticleBuffer();

		const PxU32 numActiveDiffuseParticles = userBuffer->getNbActiveDiffuseParticles();

		//printf("NumActiveDiffuse = %i\n", numActiveDiffuseParticles);

		if (numActiveDiffuseParticles > 0)
		{
			PxVec3 colorDiffuseParticles(1, 1, 1);
			Snippets::DrawPoints(sDiffusePosLifeBuffer.vbo, numActiveDiffuseParticles, colorDiffuseParticles, 2.f);
		}

		Snippets::DrawFrame(PxVec3(0, 0, 0));
	}

	void allocParticleBuffers()
	{
		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();

		PxParticleAndDiffuseBuffer* userBuffer = getParticleBuffer();

		const PxU32 maxParticles = userBuffer->getMaxParticles();
		const PxU32 maxDiffuseParticles = userBuffer->getMaxDiffuseParticles();

		sDiffusePosLifeBuffer.initialize(cudaContextManager);
		sDiffusePosLifeBuffer.allocate(maxDiffuseParticles * sizeof(PxVec4));

		sPosBuffer.initialize(cudaContextManager);
		sPosBuffer.allocate(maxParticles * sizeof(PxVec4));
	}

	void clearupParticleBuffers()
	{
		sPosBuffer.release();
		sDiffusePosLifeBuffer.release();
	}

	void renderCallback()
	{
		onBeforeRenderParticles();

		stepPhysics(true);

		Snippets::startRender(sCamera);

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			std::vector<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
		}

		renderParticles();

		Snippets::showFPS();

		Snippets::finishRender();
	}

	void cleanup()
	{
		delete sCamera;
		clearupParticleBuffers();
		cleanupPhysics(true);
	}

	void exitCallback(void)
	{
	}
}

void renderLoop()
{
	sCamera = new MathLib::Camera(MathLib::HVector3(15.0f, 10.0f, 15.0f), MathLib::HVector3(-0.6f, -0.2f, -0.6f));

	Snippets::setupDefault("PhysX Snippet PBFFluid", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	Snippets::initFPS();

	allocParticleBuffers();

	glutMainLoop();

	cleanup();
}
#endif