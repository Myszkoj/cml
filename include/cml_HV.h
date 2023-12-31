#pragma once


#include "cml_utilities.h"


namespace cml
{
	class	HVScale
	{
	public:		// [DATA]
		float horizontal;
		float vertical;

	public:		// [LIFECYCLE]
		CLASS_CTOR	HVScale(	const float		HORIZONTAL,
								const float		VERTICAL)
			: horizontal(HORIZONTAL)
			, vertical(VERTICAL)
		{

		}

		HVScale&	operator+=(	const HVScale&	OTHER)
		{
			horizontal	+= OTHER.horizontal;
			vertical	+= OTHER.vertical;
			return *this;
		}

		HVScale&	operator*=(	const float		FACTOR)
		{
			horizontal *= FACTOR;
			vertical *= FACTOR;
			return *this;
		}

		HVScale&	operator/=(	const float		FACTOR)
		{
			horizontal /= FACTOR;
			vertical /= FACTOR;
			return *this;
		}

		HVScale		operator*(	const float		FACTOR) const
		{
			return HVScale(*this) *= FACTOR;
		}

		HVScale		 operator/(	const float		FACTOR) const
		{
			return HVScale(*this) /= FACTOR;
		}

	public:		// [FUNCTIONS]
		void reset()
		{
			horizontal	= 0.f;
			vertical	= 0.f;
		}

		void invalidate()
		{
			horizontal	= std::numeric_limits<float>::infinity();
			vertical	= std::numeric_limits<float>::infinity();
		}
	};


	using	HVSize = HVScale;


	// 3D coordinates converted into position on the horizontal(XZ plane) at altitude = VPos.
	class	HVPoint
	{
	public:		// [SUBTYPES]
		enum AxisSystem
		{
			RIGHT_HANDED_Y_UP,
			RIGHT_HANDED_Z_UP
		};

	public:		// [DATA]
		glm::vec2	h; // Position on the horizontal plane.
		float		v; // Vertical offset from the horizontal plane.

	public:		// [LIFECYCLE]
		CLASS_CTOR		HVPoint()
			: h(0.f, 0.f)
			, v(0.f)
		{
			
		}

		CLASS_CTOR		HVPoint(			const glm::vec3&		COORDS_3D,
											const AxisSystem		SYSTEM = RIGHT_HANDED_Y_UP)
			: h(COORDS_3D.x, (SYSTEM == RIGHT_HANDED_Y_UP) ? -COORDS_3D.z : COORDS_3D.y)
			, v((SYSTEM == RIGHT_HANDED_Y_UP) ? COORDS_3D.y : COORDS_3D.z)
		{

		}

		CLASS_CTOR		HVPoint(			const glm::vec2&		H_POS,
											const float				V_POS)
			: h(H_POS)
			, v(V_POS)
		{

		}

		CLASS_CTOR		HVPoint(			const float				HX_POS,
											const float				HY_POS,
											const float				V_POS)
			: h(HX_POS, HY_POS)
			, v(V_POS)
		{

		}

	public:		// [OPERATORS]
		bool			operator==(			const HVPoint&			OTHER) const
		{
			return h == OTHER.h && v == OTHER.v;
		}
		
		bool			operator!=(			const HVPoint&			OTHER) const
		{
			return h != OTHER.h || v != OTHER.v;
		}

		HVPoint			operator+(			const HVPoint&			OTHER) const
		{
			return HVPoint(h + OTHER.h, v + OTHER.v);
		}

		HVPoint			operator-(			const HVPoint&			OTHER) const
		{
			return HVPoint(h - OTHER.h, v - OTHER.v);
		}

		HVPoint&		operator+=(			const HVPoint&			OTHER)
		{
			this->h += OTHER.h;
			this->v += OTHER.v;
			return *this;
		}

		HVPoint&		operator-=(			const HVPoint&			OTHER)
		{
			this->h -= OTHER.h;
			this->v -= OTHER.v;
			return *this;
		}

		HVPoint			operator*(			const HVScale&			SCALE) const
		{
			return HVPoint(h * SCALE.horizontal, v * SCALE.vertical);
		}

		HVPoint&		operator*=(			const HVScale&			SCALE)
		{
			this->h *= SCALE.horizontal;
			this->v *= SCALE.vertical;
			return *this;
		}

	public:		// [FUNCTIONS]
		HVPoint			mod(				const HVScale&			HV) const
		{
			return HVPoint(glm::vec2(fmod(h.x, HV.horizontal), fmod(h.y, HV.horizontal)), fmod(v, HV.vertical));
		}

		void			swap(				HVPoint&				other)
		{
			std::swap(h, other.h);
			std::swap(v, other.v);
		}

		void			minimize(			const HVPoint&			OTHER)
		{
			if(OTHER.h.x < h.x) h.x = OTHER.h.x;
			if(OTHER.h.y < h.y) h.y = OTHER.h.y;
			if(OTHER.v < v) v = OTHER.v;
		}

		void			maximize(			const HVPoint&			OTHER)
		{
			if(OTHER.h.x > h.x) h.x = OTHER.h.x;
			if(OTHER.h.y > h.y) h.y = OTHER.h.y;
			if(OTHER.v > v) v = OTHER.v;
		}

		static HVPoint	min_of(				const HVPoint*			POINTS,
											const uint32_t			NUM_POINTS)
		{
			HVPoint output = POINTS[0];
			for(uint32_t index = 1; index < NUM_POINTS; ++index)
			{
				output.minimize(POINTS[index]);
			}
			return output;
		}

		static HVPoint	max_of(				const HVPoint*			POINTS,
											const uint32_t			NUM_POINTS)
		{
			HVPoint output = POINTS[0];
			for(uint32_t index = 1; index < NUM_POINTS; ++index)
			{
				output.maximize(POINTS[index]);
			}
			return output;
		}

	public:		// [CONVERSIONS]
		//operator const glm::vec3() const
		//{
		//	return this->to_xyz();
		//}

		glm::vec3		to_xyz(				const AxisSystem		SYSTEM = RIGHT_HANDED_Y_UP) const
		{
			return (SYSTEM == RIGHT_HANDED_Y_UP) ? glm::vec3(h.x, v, -h.y) : glm::vec3(h.x, h.y, v);
		}

		float			horizontal_distance(const HVPoint&			OTHER) const
		{
			return glm::distance(this->h, OTHER.h);
		}

		float			vertical_distance(	const HVPoint&			OTHER) const
		{
			return glm::abs(this->v - OTHER.v);
		}

		//void			convert(			const glm::vec3&	NEW_COORDINATES)
		//{
		//	h.x	= NEW_COORDINATES.x;
		//	v	= NEW_COORDINATES.y;
		//	h.y	= -NEW_COORDINATES.z;
		//}

		void			invalidate_horizontal()
		{
			h.x = std::numeric_limits<float>::quiet_NaN();
			h.y = std::numeric_limits<float>::quiet_NaN();
		}
			
		void			invalidate_vertical()
		{
			v = std::numeric_limits<float>::quiet_NaN();
		}

		void			invalidate()
		{
			invalidate_horizontal();
			invalidate_vertical();
		}
	};


	class	HVDistance : public HVScale
	{
	public:		// [LIFECYCLE]
		using HVScale::HVScale;

		CLASS_CTOR	HVDistance(): HVScale(0.f, 0.f){}

		CLASS_CTOR	HVDistance(	const HVPoint& A,
								const HVPoint& B)
			: HVScale(glm::distance(A.h, B.h), glm::abs(A.v - B.v))
		{

		}

	public:		// [FUNCTIONS]
		using HVScale::reset;

		inline void reset(		const HVPoint& A,
								const HVPoint& B)
		{
			horizontal	= glm::distance(A.h, B.h);
			vertical	= glm::abs(A.v - B.v);
		}
	};
}