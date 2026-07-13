using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

# you can rename the struct, but don't change the identifier
struct CarrotMan @0x81c2f05a394cf4af {
	activeCarrot @0 : Int32;
	nRoadLimitSpeed @1 : Int32;
	remote @2 : Text;
	xSpdType @3 : Int32;
	xSpdLimit @4 : Int32;
	xSpdDist @5 : Int32;
	xSpdCountDown @6 : Int32;
	xTurnInfo @7 : Int32;
	xDistToTurn @8 : Int32;
	xTurnCountDown @9 : Int32;
	atcType @10 : Text;
	vTurnSpeed @11 : Int32;
	szPosRoadName @12 : Text;
	szTBTMainText @13 : Text;
	desiredSpeed @14 : Int32;
	desiredSource @15 : Text;
	carrotCmdIndex @16 : Int32;
	carrotCmd @17 : Text;
	carrotArg @18 : Text;
	xPosLat @19 : Float32;
	xPosLon @20 : Float32;
	xPosAngle @21 : Float32;
	xPosSpeed @22 : Float32;
	trafficState @23 : Int32;
	nGoPosDist @24 : Int32;
	nGoPosTime @25 : Int32;
	szSdiDescr @26 : Text;
	naviPaths @27 : Text;
	leftSec @28 : Int32;
}

struct CarrotNaviState @0xaedffd8f31e7b55d {
	schemaVersion @0 :UInt16;
	generation @1 :UInt64;
	sessionId @2 :Text;
	publishMonoTimeNanos @3 :UInt64;
	connected @4 :Bool;
	vehicle @5 :Vehicle;
	guidanceCurrent @6 :Guidance;
	guidanceNext @7 :Guidance;
	laneCurrent @8 :Lane;
	laneAhead @9 :List(Lane);
	speed @10 :Speed;
	trafficSignal @11 :TrafficSignal;
	crossroad @12 :Crossroad;
	route @13 :Route;
	navigationStatus @14 :NavigationStatus;

	struct ItemMeta {
		present @0 :Bool;
		sequence @1 :UInt64;
		sourceTimestampMillis @2 :UInt64;
		receivedMonoTimeNanos @3 :UInt64;
	}

	struct Vehicle {
		meta @0 :ItemMeta;
		latitude @1 :Float64;
		longitude @2 :Float64;
		headingDeg @3 :Float32;
		speedKph @4 :Float32;
		roadName @5 :Text;
		virtualGps @6 :Bool;
	}

	struct Guidance {
		meta @0 :ItemMeta;
		distanceM @1 :Int32;
		timeSec @2 :Int32;
		turnType @3 :Int32;
		roadName @4 :Text;
		mainText @5 :Text;
		nearDirection @6 :Text;
		midDirection @7 :Text;
		farDirection @8 :Text;
		pointValid @9 :Bool;
		latitude @10 :Float64;
		longitude @11 :Float64;
	}

	struct Lane {
		meta @0 :ItemMeta;
		count @1 :Int16;
		distanceM @2 :Int32;
		visible @3 :Bool;
		lanePlay @4 :Bool;
		currentLane @5 :Int16;
		turnCode @6 :Int32;
		turnInfo @7 :List(Int16);
		etcInfo @8 :List(Int16);
		available @9 :List(Int16);
		guideLineColor @10 :Int16;
		roadCategory @11 :Int16;
		voiceCode @12 :Int16;
	}

	struct Speed {
		meta @0 :ItemMeta;
		currentKph @1 :Float32;
		roadLimitValid @2 :Bool;
		roadLimitKph @3 :Int16;
		sdiPresent @4 :Bool;
		sdiType @5 :Int32;
		sdiDistanceM @6 :Int32;
		sdiSpeedLimitKph @7 :Int16;
		sectionPresent @8 :Bool;
		sectionActive @9 :Bool;
		sectionSpeedLimitKph @10 :Int16;
		sectionAverageKph @11 :Float32;
		sectionOverallAverageKph @12 :Float32;
		sectionRemainingDistanceM @13 :Float32;
		sectionRemainingTimeSec @14 :Int32;
		sectionProgress @15 :Float32;
		sectionSuspended @16 :Bool;
		sectionOffRoute @17 :Bool;
	}

	struct TrafficSignal {
		meta @0 :ItemMeta;
		visible @1 :Bool;
		distanceM @2 :Int32;
		source @3 :Text;
		redValid @4 :Bool;
		redOn @5 :Bool;
		redRemainSec @6 :Int16;
		leftValid @7 :Bool;
		leftOn @8 :Bool;
		leftRemainSec @9 :Int16;
		greenValid @10 :Bool;
		greenOn @11 :Bool;
		greenRemainSec @12 :Int16;
		rightValid @13 :Bool;
		rightOn @14 :Bool;
		rightRemainSec @15 :Int16;
		uturnValid @16 :Bool;
		uturnOn @17 :Bool;
		uturnRemainSec @18 :Int16;
		uiCounterValid @19 :Bool;
		uiCounterRemainSec @20 :Int16;
	}

	struct Crossroad {
		meta @0 :ItemMeta;
		visible @1 :Bool;
		distanceM @2 :Int32;
		imageCode @3 :Int32;
		imageUrl @4 :Text;
	}

	struct Coordinate {
		latitude @0 :Float64;
		longitude @1 :Float64;
	}

	struct Route {
		meta @0 :ItemMeta;
		remainingDistanceM @1 :Int32;
		remainingTimeSec @2 :Int32;
		movedDistanceM @3 :Int32;
		movedTimeSec @4 :Int32;
		totalDistanceM @5 :Int32;
		polyline @6 :List(Coordinate);
	}

	struct NavigationStatus {
		meta @0 :ItemMeta;
		mode @1 :Text;
		guidanceActive @2 :Bool;
		offRoute @3 :Bool;
		routePresent @4 :Bool;
	}
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
