// Character mover cache analysis and regression benchmark.

#include "box3d/box3d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct Scenario
{
	b3WorldId worldId;
	b3BodyId nearbyStatic[64];
	int nearbyStaticCount;
	b3Pos origin;
	b3Capsule mover;
	b3Vec3 translation;
	b3AABB cachedBounds[64];
	int cachedBoundCount;
	b3MoverCache* cache;
} Scenario;

static volatile float g_sink;
static float g_cacheExtent = 2.0f;

static bool CacheBoundsCallback( b3ShapeId shapeId, void* context )
{
	Scenario* scenario = context;
	if ( scenario->cachedBoundCount < 64 )
	{
		scenario->cachedBounds[scenario->cachedBoundCount++] = b3Shape_GetAABB( shapeId );
	}
	return true;
}

static void RebuildBoundsCache( Scenario* scenario )
{
	b3Vec3 points[2] = { scenario->mover.center1, scenario->mover.center2 };
	b3AABB local = b3MakeAABB( points, 2, scenario->mover.radius );
	b3AABB bounds = b3OffsetAABB( b3AABB_Inflate( local, 2.0f ), scenario->origin );
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 1;
	scenario->cachedBoundCount = 0;
	b3World_OverlapAABB( scenario->worldId, bounds, filter, CacheBoundsCallback, scenario );
}

static b3AABB MakeSweepBounds( const Scenario* scenario )
{
	b3Vec3 points[2] = { scenario->mover.center1, scenario->mover.center2 };
	b3AABB local = b3MakeAABB( points, 2, scenario->mover.radius );
	b3AABB start = b3OffsetAABB( local, scenario->origin );
	b3AABB finish = { b3Add( start.lowerBound, scenario->translation ), b3Add( start.upperBound, scenario->translation ) };
	return b3AABB_Union( start, finish );
}

static float CastBodies( const Scenario* scenario, uint64_t maskBits )
{
	b3ShapeProxy proxy = { &scenario->mover.center1, 2, scenario->mover.radius };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = maskBits;
	float fraction = 1.0f;

	for ( int i = 0; i < scenario->nearbyStaticCount; ++i )
	{
		b3BodyId bodyId = scenario->nearbyStatic[i];
		b3BodyCastResult result = b3Body_CastShape( bodyId, scenario->origin, &proxy, scenario->translation, filter, fraction,
											   scenario->mover.radius > 0.0f, b3Body_GetTransform( bodyId ) );
		if ( result.hit && result.fraction > 0.0f && result.fraction < fraction )
		{
			fraction = result.fraction;
		}
	}

	return fraction;
}

static float CastCachedRegion( const Scenario* scenario )
{
	b3AABB sweep = MakeSweepBounds( scenario );
	b3ShapeProxy proxy = { &scenario->mover.center1, 2, scenario->mover.radius };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 1;
	float fraction = 1.0f;
	for ( int i = 0; i < scenario->nearbyStaticCount; ++i )
	{
		if ( b3AABB_Overlaps( sweep, scenario->cachedBounds[i] ) == false )
		{
			continue;
		}
		b3BodyId bodyId = scenario->nearbyStatic[i];
		b3BodyCastResult result = b3Body_CastShape( bodyId, scenario->origin, &proxy, scenario->translation, filter, fraction, true,
											   b3Body_GetTransform( bodyId ) );
		if ( result.hit && result.fraction > 0.0f && result.fraction < fraction )
		{
			fraction = result.fraction;
		}
	}
	return fraction;
}

static float CastWorld( const Scenario* scenario, uint64_t maskBits )
{
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = maskBits;
	return b3World_CastMover( scenario->worldId, scenario->origin, &scenario->mover, scenario->translation, filter, NULL, NULL );
}

static float CastSplit( const Scenario* scenario )
{
	float staticFraction = CastBodies( scenario, 1 );
	float movingFraction = CastWorld( scenario, 2 );
	return b3MinFloat( staticFraction, movingFraction );
}

static float CastConservativeCache( const Scenario* scenario )
{
	b3AABB sweep = MakeSweepBounds( scenario );
	for ( int i = 0; i < scenario->cachedBoundCount; ++i )
	{
		if ( b3AABB_Overlaps( sweep, scenario->cachedBounds[i] ) )
		{
			return CastWorld( scenario, 3 );
		}
	}
	return 1.0f;
}

typedef float BenchFcn( const Scenario* scenario );

static float BenchWorld( const Scenario* scenario )
{
	return CastWorld( scenario, 3 );
}

static float BenchStaticWorld( const Scenario* scenario )
{
	return CastWorld( scenario, 1 );
}

static float BenchCachedBodies( const Scenario* scenario )
{
	return CastBodies( scenario, 1 );
}

static float BenchMoverCache( const Scenario* scenario )
{
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 3;
	return b3World_CastMoverCached( scenario->worldId, scenario->origin, &scenario->mover, scenario->translation, scenario->cache,
								  g_cacheExtent, filter, NULL, NULL );
}

static bool PlaneCallback( b3ShapeId shapeId, const b3PlaneResult* planes, int count, void* context )
{
	(void)shapeId;
	(void)planes;
	int* planeCount = context;
	*planeCount += count;
	return true;
}

static float BenchCollideWorld( const Scenario* scenario )
{
	int planeCount = 0;
	b3World_CollideMover( scenario->worldId, scenario->origin, &scenario->mover, b3DefaultQueryFilter(), PlaneCallback,
						  &planeCount );
	return (float)planeCount;
}

static double Run( BenchFcn* fcn, const Scenario* scenario, int iterations )
{
	float sum = 0.0f;
	for ( int i = 0; i < 2000; ++i )
	{
		sum += fcn( scenario );
	}

	uint64_t ticks = b3GetTicks();
	for ( int i = 0; i < iterations; ++i )
	{
		sum += fcn( scenario );
	}
	double nanoseconds = 1.0e6 * b3GetMilliseconds( ticks ) / iterations;
	g_sink = sum;
	return nanoseconds;
}

static b3BodyId AddBox( b3WorldId worldId, b3Vec3 position, b3Vec3 halfExtents, b3BodyType type, uint64_t categoryBits )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = type;
	bodyDef.position = b3ToPos( position );
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.categoryBits = categoryBits;
	b3BoxHull box = b3MakeBoxHull( halfExtents.x, halfExtents.y, halfExtents.z );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
	return bodyId;
}

static Scenario MakeScenario( int width, bool grounded, bool wall, int dynamicCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.capacity.staticShapeCount = width * width + 8;
	worldDef.capacity.dynamicShapeCount = dynamicCount + 8;
	worldDef.capacity.staticBodyCount = width * width + 8;
	worldDef.capacity.dynamicBodyCount = dynamicCount + 8;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	Scenario scenario = { 0 };
	scenario.worldId = worldId;
	scenario.cache = b3CreateMoverCache();
	scenario.origin = b3Pos_zero;
	scenario.mover = (b3Capsule){ { 0.0f, 0.35f, 0.0f }, { 0.0f, 1.45f, 0.0f }, 0.35f };
	scenario.translation = (b3Vec3){ 0.08f, 0.0f, 0.02f };

	float offset = 0.5f * (float)( width - 1 );
	for ( int iz = 0; iz < width; ++iz )
	{
		for ( int ix = 0; ix < width; ++ix )
		{
			b3Vec3 p = { (float)ix - offset, -0.05f, (float)iz - offset };
			b3BodyId bodyId = AddBox( worldId, p, (b3Vec3){ 0.49f, 0.05f, 0.49f }, b3_staticBody, 1 );
			if ( fabsf( p.x ) <= 1.0f && fabsf( p.z ) <= 1.0f && scenario.nearbyStaticCount < 16 )
			{
				scenario.nearbyStatic[scenario.nearbyStaticCount++] = bodyId;
			}
		}
	}

	if ( grounded == false )
	{
		scenario.origin.y = 5.0f;
		scenario.nearbyStaticCount = 0;
	}

	if ( wall )
	{
		b3BodyId wallId = AddBox( worldId, (b3Vec3){ 0.7f, 0.9f, 0.0f }, (b3Vec3){ 0.05f, 0.9f, 0.6f }, b3_staticBody, 1 );
		scenario.nearbyStatic[scenario.nearbyStaticCount++] = wallId;
		scenario.translation = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	}

	for ( int i = 0; i < dynamicCount; ++i )
	{
		float x = 3.0f + 0.75f * (float)( i % 8 );
		float z = -3.0f + 0.75f * (float)( i / 8 );
		AddBox( worldId, (b3Vec3){ x, 0.5f, z }, (b3Vec3){ 0.25f, 0.25f, 0.25f }, b3_dynamicBody, 2 );
	}

	b3World_Step( worldId, 0.0f, 1 );
	return scenario;
}

static void Report( const char* name, Scenario* scenario, int iterations )
{
	RebuildBoundsCache( scenario );
	double world = Run( BenchWorld, scenario, iterations );
	double staticWorld = Run( BenchStaticWorld, scenario, iterations );
	double cached = Run( BenchCachedBodies, scenario, iterations );
	double split = Run( CastSplit, scenario, iterations );
	double conservative = Run( CastConservativeCache, scenario, iterations );
	double collide = Run( BenchCollideWorld, scenario, iterations );
	double apiCache = Run( BenchMoverCache, scenario, iterations );
	uint64_t rebuildTicks = b3GetTicks();
	for ( int i = 0; i < iterations / 10; ++i )
	{
		RebuildBoundsCache( scenario );
	}
	double rebuild = 10.0e6 * b3GetMilliseconds( rebuildTicks ) / iterations;
	float reference = BenchWorld( scenario );
	float candidate = CastSplit( scenario );

	printf( "%-24s cast %7.1f ns  collide %7.1f ns  api-cache %7.1f ns  manual-shapes %7.1f ns  split %7.1f ns  aabb+fallback %7.1f ns  rebuild %7.1f ns  bodies %2d bounds %2d f %.4f/%.4f\n",
			name, world, collide, apiCache, cached, split, conservative, rebuild, scenario->nearbyStaticCount,
			scenario->cachedBoundCount, reference, candidate );
	(void)staticWorld;
}

static void ReportCrowd( int width, int characterCount, int iterations )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.capacity.staticShapeCount = width * width;
	worldDef.capacity.staticBodyCount = width * width;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyId* tiles = malloc( (size_t)width * width * sizeof( b3BodyId ) );
	float offset = 0.5f * (float)( width - 1 );
	for ( int iz = 0; iz < width; ++iz )
	{
		for ( int ix = 0; ix < width; ++ix )
		{
			b3Vec3 p = { (float)ix - offset, -0.05f, (float)iz - offset };
			tiles[iz * width + ix] = AddBox( worldId, p, (b3Vec3){ 0.49f, 0.05f, 0.49f }, b3_staticBody, 1 );
		}
	}
	b3World_Step( worldId, 0.0f, 1 );

	Scenario* characters = calloc( (size_t)characterCount, sizeof( Scenario ) );
	b3Pos* starts = malloc( (size_t)characterCount * sizeof( b3Pos ) );
	for ( int i = 0; i < characterCount; ++i )
	{
		int index = ( i * 7919 ) % ( width * width );
		int ix = index % width;
		int iz = index / width;
		Scenario* s = characters + i;
		s->worldId = worldId;
		s->cache = b3CreateMoverCache();
		s->origin = (b3Pos){ (float)ix - offset, 0.0f, (float)iz - offset };
		starts[i] = s->origin;
		s->mover = (b3Capsule){ { 0.0f, 0.35f, 0.0f }, { 0.0f, 1.45f, 0.0f }, 0.35f };
		s->translation = (b3Vec3){ 0.08f, 0.0f, 0.02f };
		for ( int dz = -2; dz <= 2; ++dz )
		{
			for ( int dx = -2; dx <= 2; ++dx )
			{
				int tx = ix + dx;
				int tz = iz + dz;
				if ( tx < 0 || tx >= width || tz < 0 || tz >= width )
				{
					continue;
				}
				int tileIndex = tz * width + tx;
				int candidateIndex = s->nearbyStaticCount++;
				s->nearbyStatic[candidateIndex] = tiles[tileIndex];
				s->cachedBounds[candidateIndex] = (b3AABB){
					{ (float)tx - offset - 0.49f, -0.1f, (float)tz - offset - 0.49f },
					{ (float)tx - offset + 0.49f, 0.0f, (float)tz - offset + 0.49f },
				};
			}
		}
		BenchMoverCache( s );
	}

	float sum = 0.0f;
	uint64_t ticks = b3GetTicks();
	for ( int i = 0; i < iterations; ++i )
	{
		sum += BenchWorld( characters + i % characterCount );
	}
	double world = 1.0e6 * b3GetMilliseconds( ticks ) / iterations;

	ticks = b3GetTicks();
	for ( int i = 0; i < iterations; ++i )
	{
		sum += CastCachedRegion( characters + i % characterCount );
	}
	double cached = 1.0e6 * b3GetMilliseconds( ticks ) / iterations;

	ticks = b3GetTicks();
	for ( int i = 0; i < iterations; ++i )
	{
		sum += BenchMoverCache( characters + i % characterCount );
	}
	double apiCache = 1.0e6 * b3GetMilliseconds( ticks ) / iterations;

	ticks = b3GetTicks();
	for ( int i = 0; i < iterations; ++i )
	{
		sum += BenchCollideWorld( characters + i % characterCount );
	}
	double collide = 1.0e6 * b3GetMilliseconds( ticks ) / iterations;

	for ( int i = 0; i < characterCount; ++i )
	{
		characters[i].origin = starts[i];
		b3MoverCache_Clear( characters[i].cache );
	}
	ticks = b3GetTicks();
	for ( int i = 0; i < iterations; ++i )
	{
		Scenario* s = characters + i % characterCount;
		sum += BenchMoverCache( s );
		s->origin = b3OffsetPos( s->origin, s->translation );
	}
	double movingCache = 1.0e6 * b3GetMilliseconds( ticks ) / iterations;

	for ( int i = 0; i < characterCount; ++i )
	{
		characters[i].origin = starts[i];
	}
	ticks = b3GetTicks();
	for ( int i = 0; i < iterations; ++i )
	{
		Scenario* s = characters + i % characterCount;
		sum += BenchWorld( s );
		s->origin = b3OffsetPos( s->origin, s->translation );
	}
	double movingWorld = 1.0e6 * b3GetMilliseconds( ticks ) / iterations;
	g_sink = sum;

	printf( "crowd %d characters/%d tiles: cast %.1f ns  collide %.1f ns  api-cache %.1f ns  manual-5x5 %.1f ns  cast delta %.1f%%  mover delta %.1f%%  moving full/cache %.1f/%.1f ns %.1f%%\n",
			characterCount, width * width, world, collide, apiCache, cached, 100.0 * ( world - apiCache ) / world,
			100.0 * ( world - apiCache ) / ( world + collide ), movingWorld, movingCache,
			100.0 * ( movingWorld - movingCache ) / movingWorld );

	for ( int i = 0; i < characterCount; ++i )
	{
		b3DestroyMoverCache( characters[i].cache );
	}
	free( characters );
	free( starts );
	free( tiles );
	b3DestroyWorld( worldId );
}

int main( int argc, char** argv )
{
	int width = argc > 1 ? atoi( argv[1] ) : 64;
	int iterations = argc > 2 ? atoi( argv[2] ) : 200000;
	g_cacheExtent = argc > 3 ? strtof( argv[3], NULL ) : 2.0f;
	printf( "cache extent: %.2f\n", g_cacheExtent );

	Scenario airborne = MakeScenario( width, false, false, 0 );
	Report( "airborne static", &airborne, iterations );
	b3DestroyMoverCache( airborne.cache );
	b3DestroyWorld( airborne.worldId );

	Scenario grounded = MakeScenario( width, true, false, 0 );
	Report( "grounded slide", &grounded, iterations );
	b3DestroyMoverCache( grounded.cache );
	b3DestroyWorld( grounded.worldId );

	Scenario wall = MakeScenario( width, true, true, 0 );
	Report( "wall hit", &wall, iterations );
	b3DestroyMoverCache( wall.cache );
	b3DestroyWorld( wall.worldId );

	Scenario moving = MakeScenario( width, true, false, 64 );
	Report( "ground + 64 dynamic", &moving, iterations );
	b3DestroyMoverCache( moving.cache );
	b3DestroyWorld( moving.worldId );

	ReportCrowd( width, b3MinInt( 4096, width * width ), iterations );

	return g_sink == -1.0f;
}
