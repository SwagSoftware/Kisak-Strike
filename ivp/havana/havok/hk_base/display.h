#include <hk_base/array/array.h>

#if 0 && defined(HK_DEBUG)
	class hk_Vector3;
	extern void HK_DISPLAY_POINT(const hk_Vector3& point, int rgb);
	extern void HK_DISPLAY_RAY(const hk_Vector3& point, const hk_Vector3& dir, int rgb);
	extern hk_Array<hk_Vector3> hk_display_points;
	extern hk_Array<hk_Vector3> hk_display_lines;
#else	/* !HK_DEBUG */
#	define HK_DISPLAY_POINT(p,c) /* nothing */
#	define HK_DISPLAY_RAY(p,d,c) /* nothing */
#endif	/* HK_DEBUG */

