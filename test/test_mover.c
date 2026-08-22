// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

// b3CollideMoverAndSphere / Capsule / Hull are internal
#include "shape.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

static int ParallelPlanes( void )
{
	b3CollisionPlane planes[3] = { 0 };
	planes[0].plane.normal = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	planes[0].plane.offset = 0.5f;
	planes[0].pushLimit = FLT_MAX;
	planes[1].plane.normal = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	planes[1].plane.offset = 1.0f;
	planes[1].pushLimit = FLT_MAX;
	// planes[2].plane.normal = b3Normalize((b3Vec3){ 0.2f, 0.0f, 0.9f });
	// planes[2].plane.offset = 0.25f;
	// planes[2].pushLimit = FLT_MAX;

	b3Vec3 target = { 0.0f, 0.0f, 0.0f };
	b3PlaneSolverResult result = b3SolvePlanes( target, planes, 2 );

	ENSURE( result.iterationCount == 2 );
	ENSURE_SMALL( result.delta.z - 1.0f, 0.0055f );

	return 0;
}

static int GamePlanes( void )
{
	// This scenario takes many iterations because the target is deep into the plane.
	b3CollisionPlane planes[3] = { 0 };
	planes[0].plane.normal = (b3Vec3){ 0.0f, -0.23941046f, 0.970918416f };
	planes[0].plane.offset = 0.390724182f;
	planes[0].pushLimit = FLT_MAX;
	planes[1].plane.normal = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	planes[1].plane.offset = 1.49998093f;
	planes[1].pushLimit = FLT_MAX;

	b3Vec3 target = { -2.5390625f, 0.0f, -73.6880798f };

	planes[0].plane.offset -= b3Dot( planes[0].plane.normal, target );
	planes[1].plane.offset -= b3Dot( planes[1].plane.normal, target );
	target = b3Vec3_zero;

	b3PlaneSolverResult result = b3SolvePlanes( target, planes, 2 );

	ENSURE( result.iterationCount == 20 );

	return 0;
}

// ---------------------------------------------------------------------------
// Mover-collide overlap handling
//
// b3CollideMoverAndSphere / Capsule / Hull must never emit a plane with a
// degenerate (zero) normal, even when the mover deeply penetrates the shape.
// On deep overlap the GJK path returns a {0,0,0} normal; these tests guard the
// fix that replaces it with an analytic (sphere/capsule) or dropped (hull) result.
// ---------------------------------------------------------------------------

static int MoverSphereSeparated( void )
{
	b3Sphere shape = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3Capsule mover = { { 4.0f, 3.0f, 0.0f }, { 6.0f, 3.0f, 0.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndSphere( &result, &shape, &mover );
	ENSURE( count == 0 );

	return 0;
}

static int MoverSphereTouching( void )
{
	b3Sphere shape = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	// Mover core segment runs along X at y = 0.6, leaving it 0.1 inside the
	// 0.7 combined radius.
	b3Capsule mover = { { -1.0f, 0.6f, 0.0f }, { 1.0f, 0.6f, 0.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndSphere( &result, &shape, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );

	// Push-out points from the sphere straight up toward the mover.
	ENSURE( result.plane.normal.y > 0.99f );
	ENSURE_SMALL( result.plane.offset - 0.1f, 1e-5f );

	return 0;
}

static int MoverSphereDeepOverlap( void )
{
	b3Sphere shape = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	// Mover axis runs straight through the sphere center: the bug case where
	// GJK reports a zero normal.
	b3Capsule mover = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndSphere( &result, &shape, &mover );
	ENSURE( count == 1 );

	// The normal must still be a valid unit vector.
	ENSURE( b3IsNormalized( result.plane.normal ) );

	// The fallback axis is perpendicular to the mover axis (X).
	ENSURE_SMALL( result.plane.normal.x, 1e-5f );

	// Deepest possible penetration: the full combined radius.
	ENSURE_SMALL( result.plane.offset - 0.7f, 1e-5f );

	return 0;
}

static int MoverCapsuleSeparated( void )
{
	b3Capsule shape = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.3f };
	b3Capsule mover = { { -1.0f, 5.0f, 0.0f }, { 1.0f, 5.0f, 0.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndCapsule( &result, &shape, &mover );
	ENSURE( count == 0 );

	return 0;
}

static int MoverCapsuleTouching( void )
{
	b3Capsule shape = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.3f };

	// Parallel mover 0.4 above, leaving it 0.1 inside the 0.5 combined radius.
	b3Capsule mover = { { -1.0f, 0.4f, 0.0f }, { 1.0f, 0.4f, 0.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndCapsule( &result, &shape, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );
	ENSURE( result.plane.normal.y > 0.99f );
	ENSURE_SMALL( result.plane.offset - 0.1f, 1e-5f );

	return 0;
}

static int MoverCapsuleDeepOverlap( void )
{
	// Shape capsule along X, mover capsule along Z; their core segments cross
	// exactly at the origin, so GJK reports a zero normal.
	b3Capsule shape = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.3f };
	b3Capsule mover = { { 0.0f, 0.0f, -1.0f }, { 0.0f, 0.0f, 1.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndCapsule( &result, &shape, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );

	// The separating axis of two crossing segments is perpendicular to both.
	ENSURE_SMALL( result.plane.normal.x, 1e-5f );
	ENSURE_SMALL( result.plane.normal.z, 1e-5f );
	ENSURE_SMALL( result.plane.offset - 0.5f, 1e-5f );

	return 0;
}

static int MoverCapsuleParallelOverlap( void )
{
	// Mover core segment coincides with the shape core segment: the cross-product
	// axis degenerates, so a perpendicular of the mover axis is used instead.
	b3Capsule shape = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.3f };
	b3Capsule mover = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndCapsule( &result, &shape, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );

	// The fallback axis is perpendicular to the mover axis (X).
	ENSURE_SMALL( result.plane.normal.x, 1e-5f );
	ENSURE_SMALL( result.plane.offset - 0.5f, 1e-5f );

	return 0;
}

static int MoverHullSeparated( void )
{
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3Capsule mover = { { -0.3f, 5.0f, 0.0f }, { 0.3f, 5.0f, 0.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndHull( &result, &box.base, &mover );
	ENSURE( count == 0 );

	return 0;
}

static int MoverHullTouching( void )
{
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	// Mover core segment above the +Y face; the 0.2 radius reaches 0.1 into it.
	b3Capsule mover = { { -0.3f, 0.6f, 0.0f }, { 0.3f, 0.6f, 0.0f }, 0.2f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndHull( &result, &box.base, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );
	ENSURE( result.plane.normal.y > 0.99f );
	ENSURE_SMALL( result.plane.offset - 0.1f, 1e-4f );

	return 0;
}

static int MoverHullDeepOverlap( void )
{
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	// Mover core segment lies entirely inside the box, so GJK reports overlap.
	b3Capsule mover = { { -0.2f, 0.0f, 0.0f }, { 0.2f, 0.0f, 0.0f }, 0.1f };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndHull( &result, &box.base, &mover );

	// The overlap guard drops the plane rather than emit a zero normal.
	// todo replace with SAT once b3CollideMoverAndHull resolves overlaps.
	ENSURE( count == 0 );

	return 0;
}

static b3BodyId CreateMoverWall( b3WorldId worldId, b3Pos position, b3BodyType type, uint64_t categoryBits )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = type;
	bodyDef.position = position;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.categoryBits = categoryBits;
	b3BoxHull box = b3MakeBoxHull( 0.1f, 1.0f, 1.0f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
	return bodyId;
}

static int MoverCacheStaticHit( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateMoverWall( worldId, (b3Pos){ 2.0f, 0.0f, 0.0f }, b3_staticBody, 1 );

	b3Capsule mover = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.25f };
	b3Vec3 translation = { 4.0f, 0.0f, 0.0f };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 1;
	b3MoverCache* cache = b3CreateMoverCache();

	float expected = b3World_CastMover( worldId, b3Pos_zero, &mover, translation, filter, NULL, NULL );
	float first = b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 2.0f, filter, NULL, NULL );
	b3MoverCacheStats stats = b3MoverCache_GetStats( cache );
	ENSURE_SMALL( first - expected, 1e-6f );
	ENSURE( stats.missCount == 1 && stats.hitCount == 0 && stats.candidateCount == 1 );

	float second = b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 2.0f, filter, NULL, NULL );
	stats = b3MoverCache_GetStats( cache );
	ENSURE_SMALL( second - expected, 1e-6f );
	ENSURE( stats.missCount == 1 && stats.hitCount == 1 );

	// Leaving the retained region refreshes rather than incorrectly trusting the old candidates.
	b3Pos farOrigin = { 20.0f, 0.0f, 0.0f };
	ENSURE( b3World_CastMoverCached( worldId, farOrigin, &mover, translation, cache, 2.0f, filter, NULL, NULL ) == 1.0f );
	stats = b3MoverCache_GetStats( cache );
	ENSURE( stats.missCount == 2 );

	b3MoverCache_Clear( cache );
	stats = b3MoverCache_GetStats( cache );
	ENSURE( stats.missCount == 0 && stats.hitCount == 0 && stats.candidateCount == 0 );
	b3DestroyMoverCache( cache );
	b3DestroyWorld( worldId );
	return 0;
}

static int MoverCacheInvalidation( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyId wallId = CreateMoverWall( worldId, (b3Pos){ 4.0f, 0.0f, 0.0f }, b3_staticBody, 1 );

	b3Capsule mover = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.25f };
	b3Vec3 translation = { 2.0f, 0.0f, 0.0f };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 1;
	b3MoverCache* cache = b3CreateMoverCache();

	ENSURE( b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 4.0f, filter, NULL, NULL ) == 1.0f );
	ENSURE( b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 4.0f, filter, NULL, NULL ) == 1.0f );
	b3MoverCacheStats stats = b3MoverCache_GetStats( cache );
	ENSURE( stats.missCount == 1 && stats.hitCount == 1 );

	// A static proxy move invalidates the candidate region before the next cast.
	b3Body_SetTransform( wallId, (b3Pos){ 1.0f, 0.0f, 0.0f }, b3Quat_identity );
	float fraction = b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 4.0f, filter, NULL, NULL );
	stats = b3MoverCache_GetStats( cache );
	ENSURE( fraction < 1.0f );
	ENSURE( stats.missCount == 2 && stats.hitCount == 1 );

	// Origin shifting changes every cached world-space bound and also invalidates the cache.
	b3World_ShiftOrigin( worldId, (b3Vec3){ 100.0f, 0.0f, 0.0f } );
	b3Pos shiftedOrigin = { 100.0f, 0.0f, 0.0f };
	fraction = b3World_CastMoverCached( worldId, shiftedOrigin, &mover, translation, cache, 4.0f, filter, NULL, NULL );
	stats = b3MoverCache_GetStats( cache );
	ENSURE( fraction < 1.0f );
	ENSURE( stats.missCount == 3 );

	b3DestroyMoverCache( cache );
	b3DestroyWorld( worldId );
	return 0;
}

static int MoverCacheFilterInvalidation( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyId wallId = CreateMoverWall( worldId, (b3Pos){ 1.0f, 0.0f, 0.0f }, b3_staticBody, 2 );

	b3Capsule mover = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.25f };
	b3Vec3 translation = { 2.0f, 0.0f, 0.0f };
	b3QueryFilter queryFilter = b3DefaultQueryFilter();
	queryFilter.maskBits = 1;
	b3MoverCache* cache = b3CreateMoverCache();

	// The initial category is excluded, so the first cache contains no candidates.
	ENSURE( b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 2.0f, queryFilter, NULL, NULL ) ==
			1.0f );
	ENSURE( b3MoverCache_GetStats( cache ).candidateCount == 0 );

	b3ShapeId shapeId;
	ENSURE( b3Body_GetShapes( wallId, &shapeId, 1 ) == 1 );
	b3Filter shapeFilter = b3Shape_GetFilter( shapeId );
	shapeFilter.categoryBits = 1;
	b3Shape_SetFilter( shapeId, shapeFilter, true );

	// Changing static proxy categories invalidates and rebuilds the cache before use.
	float expected = b3World_CastMover( worldId, b3Pos_zero, &mover, translation, queryFilter, NULL, NULL );
	float fraction =
		b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 2.0f, queryFilter, NULL, NULL );
	b3MoverCacheStats stats = b3MoverCache_GetStats( cache );
	ENSURE_SMALL( fraction - expected, 1e-6f );
	ENSURE( fraction < 1.0f );
	ENSURE( stats.missCount == 2 && stats.candidateCount == 1 );

	b3DestroyMoverCache( cache );
	b3DestroyWorld( worldId );
	return 0;
}

#if defined( BOX3D_DOUBLE_PRECISION )
static int MoverCacheLargeWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3Pos origin = { 100000000.0, -200000000.0, 300000000.0 };
	CreateMoverWall( worldId, (b3Pos){ origin.x + 2.0, origin.y, origin.z }, b3_staticBody, 1 );

	b3Capsule mover = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.25f };
	b3Vec3 translation = { 4.0f, 0.0f, 0.0f };
	b3QueryFilter filter = b3DefaultQueryFilter();
	b3MoverCache* cache = b3CreateMoverCache();
	float expected = b3World_CastMover( worldId, origin, &mover, translation, filter, NULL, NULL );
	b3World_CastMoverCached( worldId, origin, &mover, translation, cache, 2.0f, filter, NULL, NULL );
	float actual = b3World_CastMoverCached( worldId, origin, &mover, translation, cache, 2.0f, filter, NULL, NULL );
	ENSURE_SMALL( actual - expected, 1e-6f );
	ENSURE( actual < 1.0f );
	ENSURE( b3MoverCache_GetStats( cache ).hitCount == 1 );

	b3DestroyMoverCache( cache );
	b3DestroyWorld( worldId );
	return 0;
}
#endif

static int MoverCacheDynamicBody( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateMoverWall( worldId, (b3Pos){ 3.0f, 0.0f, 0.0f }, b3_staticBody, 1 );

	b3Capsule mover = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.25f };
	b3Vec3 translation = { 4.0f, 0.0f, 0.0f };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 3;
	b3MoverCache* cache = b3CreateMoverCache();
	b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 2.0f, filter, NULL, NULL );

	// Dynamic creation does not invalidate the static cache. It must still be found by the per-call dynamic tree cast.
	CreateMoverWall( worldId, (b3Pos){ 1.5f, 0.0f, 0.0f }, b3_dynamicBody, 2 );
	float expected = b3World_CastMover( worldId, b3Pos_zero, &mover, translation, filter, NULL, NULL );
	float fraction = b3World_CastMoverCached( worldId, b3Pos_zero, &mover, translation, cache, 2.0f, filter, NULL, NULL );
	b3MoverCacheStats stats = b3MoverCache_GetStats( cache );
	ENSURE_SMALL( fraction - expected, 1e-6f );
	ENSURE( stats.missCount == 1 && stats.hitCount == 1 );

	b3DestroyMoverCache( cache );
	b3DestroyWorld( worldId );
	return 0;
}

static int MoverCacheDifferential( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyId movingStatic = b3_nullBodyId;
	for ( int i = 0; i < 12; ++i )
	{
		float x = 1.0f + 0.75f * (float)( i % 4 );
		float z = -1.5f + (float)( i / 4 );
		b3BodyId bodyId = CreateMoverWall( worldId, (b3Pos){ x, 0.0f, z }, b3_staticBody, 1 );
		if ( i == 0 )
		{
			movingStatic = bodyId;
		}
	}
	b3BodyId dynamicId = CreateMoverWall( worldId, (b3Pos){ 2.0f, 0.0f, 1.5f }, b3_dynamicBody, 2 );

	b3Capsule mover = { { 0.0f, -0.35f, 0.0f }, { 0.0f, 0.35f, 0.0f }, 0.2f };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 3;
	b3MoverCache* cache = b3CreateMoverCache();

	for ( int i = 0; i < 160; ++i )
	{
		if ( i > 0 && i % 37 == 0 )
		{
			float z = ( i / 37 ) % 2 == 0 ? -0.8f : 0.8f;
			b3Body_SetTransform( movingStatic, (b3Pos){ 1.0f, 0.0f, z }, b3Quat_identity );
		}

		float dynamicZ = -1.4f + 0.02f * (float)( i % 120 );
		b3Body_SetTransform( dynamicId, (b3Pos){ 2.0f, 0.0f, dynamicZ }, b3Quat_identity );

		b3Pos origin = { -0.5f + 0.015f * (float)( i % 80 ), 0.0f, -0.9f + 0.03f * (float)( i % 60 ) };
		b3Vec3 translation = { 2.5f + 0.01f * (float)( i % 7 ), 0.0f, 0.2f - 0.05f * (float)( i % 9 ) };
		float expected = b3World_CastMover( worldId, origin, &mover, translation, filter, NULL, NULL );
		float actual = b3World_CastMoverCached( worldId, origin, &mover, translation, cache, 1.5f, filter, NULL, NULL );
		ENSURE_SMALL( actual - expected, 1e-6f );
	}

	b3MoverCacheStats stats = b3MoverCache_GetStats( cache );
	ENSURE( stats.hitCount > stats.missCount );
	ENSURE( stats.missCount >= 4 );
	b3DestroyMoverCache( cache );
	b3DestroyWorld( worldId );
	return 0;
}

int MoverTest( void )
{
	RUN_SUBTEST( GamePlanes );
	RUN_SUBTEST( ParallelPlanes );

	RUN_SUBTEST( MoverSphereSeparated );
	RUN_SUBTEST( MoverSphereTouching );
	RUN_SUBTEST( MoverSphereDeepOverlap );

	RUN_SUBTEST( MoverCapsuleSeparated );
	RUN_SUBTEST( MoverCapsuleTouching );
	RUN_SUBTEST( MoverCapsuleDeepOverlap );
	RUN_SUBTEST( MoverCapsuleParallelOverlap );

	RUN_SUBTEST( MoverHullSeparated );
	RUN_SUBTEST( MoverHullTouching );
	RUN_SUBTEST( MoverHullDeepOverlap );

	RUN_SUBTEST( MoverCacheStaticHit );
	RUN_SUBTEST( MoverCacheInvalidation );
	RUN_SUBTEST( MoverCacheFilterInvalidation );
	RUN_SUBTEST( MoverCacheDynamicBody );
	RUN_SUBTEST( MoverCacheDifferential );
#if defined( BOX3D_DOUBLE_PRECISION )
	RUN_SUBTEST( MoverCacheLargeWorld );
#endif

	return 0;
}
