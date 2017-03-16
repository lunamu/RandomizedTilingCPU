#ifndef __RANDOMIZED_TILING__
#define __RANDOMIZED_TILING__

#include <UtilsDefs.h>

inline int dimensionCheck(BBox bbox, double r)//consider leave a minimum leaf node.
{
	int x = 0; int y = 0;
	if (bbox.xmax - bbox.xmin > LEAFLEN)
	{
		x = 1;
	}
	if (bbox.ymax - bbox.ymin > LEAFLEN)
	{
		y = 1;
	}
	if (x == 1 && y == 1)return ALL;
	else if (x == 1)return XAXIS;
	else if (y == 1) return YAXIS;
	else return 9;
}


#endif
