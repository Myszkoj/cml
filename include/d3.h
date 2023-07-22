#pragma once


#include <dpl_EntityManager.h>
#include <array>
#include "cml_Plane.h"
#include "cml_AABB.h"


// components
namespace cml
{
	template<uint32_t N>
	using	Indices = std::array<uint32_t, N>;


	struct	Position3D : public glm::vec3
	{
		using glm::vec3::vec3;
		using glm::vec3::operator%=;
		using glm::vec3::operator&=;
		using glm::vec3::operator+=;
		using glm::vec3::operator-=;
		using glm::vec3::operator*=;
		using glm::vec3::operator/=;
		using glm::vec3::operator=;
		using glm::vec3::operator[];
	};


	struct	Normal3D : public glm::vec3
	{
		using glm::vec3::vec3;
		using glm::vec3::operator%=;
		using glm::vec3::operator&=;
		using glm::vec3::operator+=;
		using glm::vec3::operator-=;
		using glm::vec3::operator*=;
		using glm::vec3::operator/=;
		using glm::vec3::operator=;
		using glm::vec3::operator[];
	};


	struct	TriTopology
	{
		// TODO: should be max value of 30bit unsigned integer  
		static const uint32_t INVALID_INDEX = std::numeric_limits<uint32_t>::max();

		struct	Base
		{
			enum	Type
			{
				AB,
				BC,
				CA,
				UNKNOWN
			};

			// TODO: initialize members
			struct	Adjacent
			{
				uint32_t triangleID : 30;
				uint32_t edgeType	: 2; // AB, BC, or CA
			};

			uint32_t	edgeID = INVALID_INDEX;
			Adjacent	adjacent;
		};

		uint32_t	a = INVALID_INDEX;
		uint32_t	b = INVALID_INDEX;
		uint32_t	c = INVALID_INDEX;
		Base		ab;
		Base		bc;
		Base		ca;
	};
}

// declaration of entities
namespace cml
{
	class d3Point;
	class d3Begin;
	class d3Vector;
	class d3Edge;
	class d3Polygon;
	class d3Triangle;
	class d3Mesh;
}

// description of entities
namespace dpl
{
	template<> struct Description_of<cml::d3Point>
	{
		using BaseType			= cml::d3Point;
		using ParentTypes		= dpl::TypeList<>;
		using ChildTypes		= dpl::TypeList<cml::d3Begin>;
		using PartnerTypes		= dpl::TypeList<>;
		using ComponentTypes	= dpl::TypeList<cml::Position3D>;
	};

	template<> struct Description_of<cml::d3Begin>
	{
		using BaseType			= cml::d3Begin;
		using ParentTypes		= dpl::TypeList<cml::d3Point>;
		using ChildTypes		= dpl::TypeList<>;
		using PartnerTypes		= dpl::TypeList<>;
		using ComponentTypes	= dpl::TypeList<>;
	};

	template<> struct Description_of<cml::d3Vector>
	{
		using BaseType			= cml::d3Begin;
		using ParentTypes		= dpl::TypeList<>;
		using ChildTypes		= dpl::TypeList<>;
		using PartnerTypes		= dpl::TypeList<>;
		using ComponentTypes	= dpl::TypeList<cml::Indices<2>>;
	};

	template<> struct Description_of<cml::d3Edge>
	{
		using BaseType			= cml::d3Vector;
		using ParentTypes		= dpl::TypeList<cml::d3Polygon>;
		using ChildTypes		= dpl::TypeList<>;
		using PartnerTypes		= dpl::TypeList<cml::d3Edge>; //<-- adjacent edge
		using ComponentTypes	= dpl::TypeList<cml::Normal3D>; //<-- points inwards
	};

	template<> struct Description_of<cml::d3Polygon>
	{
		using BaseType			= cml::d3Polygon;
		using ParentTypes		= dpl::TypeList<>;
		using ChildTypes		= dpl::TypeList<cml::d3Edge>;
		using PartnerTypes		= dpl::TypeList<>;
		using ComponentTypes	= dpl::TypeList<cml::Plane, cml::AABB>;
	};

	template<> struct Description_of<cml::d3Triangle>
	{
		using BaseType			= cml::d3Polygon;
		using ParentTypes		= dpl::TypeList<cml::d3Mesh>;
		using ChildTypes		= dpl::TypeList<>;
		using PartnerTypes		= dpl::TypeList<>;
		using ComponentTypes	= dpl::TypeList<cml::TriTopology>;
	};

	template<> struct Description_of<cml::d3Mesh>
	{
		using BaseType			= cml::d3Mesh;
		using ParentTypes		= dpl::TypeList<>;
		using ChildTypes		= dpl::TypeList<cml::d3Triangle>;
		using PartnerTypes		= dpl::TypeList<>;
		using ComponentTypes	= dpl::TypeList<cml::AABB>;
	};
}

// definition of entities
namespace cml
{
	DEFINE_SIMPLE_ENTITY(d3Point);
	DEFINE_SIMPLE_ENTITY(d3Begin);
	DEFINE_SIMPLE_ENTITY(d3Vector);
	DEFINE_SIMPLE_ENTITY(d3Edge);
	DEFINE_SIMPLE_ENTITY(d3Polygon);
	DEFINE_SIMPLE_ENTITY(d3Triangle);
}