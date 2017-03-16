//A 2D DNC Poisson Disk Sampling Method on CPU.
//Make it fast


#include <UtilsDefs.h>
#include <KD_tree.h>
#include <UniformGrids.h>
#include <RandomizedTiling.h>

using namespace std;

vector<Point2D> primaryPoints;
vector<Point2D> secondaryPoints;
BBox primary_right;
BBox primary_down;
BBox secondary_left;
BBox secondary_up;
BBox mid_NE;
BBox mid_SW;

vector<Point2D> centerPoints;
vector<Point2D> triPoints;

//This is storing blockingBox for visulization.
vector<BBox> blockingBox;
KDnode* kd_tree;
//argv explained:
//argv[0]: excutable
//argv[1]: filename of the pattern input
//argv[2]: filename of the maximal poisson disk sampling result



void tilePoints(BBox bbox, vector<Point2D>& result, vector<int>& priority,  Point2D offset,float ratio)
{
	traverse_and_classify(kd_tree, result, priority, bbox, offset,ratio);
}

void DNCPoisson(BBox bbox, vector<Point2D>& points, vector<int>& priority, double r, int axis,float ratio)
{

	if (dimensionCheck(bbox, r) == ALL || dimensionCheck(bbox, r) == axis)
	{
		BBox reduced_bbox_left;
		BBox reduced_bbox_right;

		if (axis == XAXIS)
		{
			//Normal box
			reduced_bbox_left.xmin = bbox.xmin;
			reduced_bbox_left.xmax = IntervalNormal(bbox.xmin + 2 * r, bbox.xmax - 4 * r);
			//reduced_bbox_left.xmax = IntervalUniform(bbox.xmin + 2 * r, bbox.xmax - 4 * r);
			reduced_bbox_left.ymin = bbox.ymin; reduced_bbox_left.ymax = bbox.ymax;


			//!Modified for test
			//reduced_bbox_right.xmin = reduced_bbox_left.xmax + 2 * r;
			reduced_bbox_right.xmin = reduced_bbox_left.xmax + 2 * r;
			reduced_bbox_right.xmax = bbox.xmax;
			reduced_bbox_right.ymin = bbox.ymin; reduced_bbox_right.ymax = bbox.ymax;



			blockingBox.push_back(BBox(reduced_bbox_left.xmax, bbox.ymin, 2 * r, bbox.ymax - bbox.ymin));
		}
		else
		{
			reduced_bbox_left.ymin = bbox.ymin;
			//reduced_bbox_left.ymax = IntervalUniform(bbox.ymin + 2 * r, bbox.ymax - 4 * r);
			reduced_bbox_left.ymax = IntervalNormal(bbox.ymin + 2 * r, bbox.ymax - 4 * r);
			reduced_bbox_left.xmin = bbox.xmin; reduced_bbox_left.xmax = bbox.xmax;

			//!Modified for test
			//reduced_bbox_right.ymin = reduced_bbox_left.ymax + 2 * r;
			reduced_bbox_right.ymin = reduced_bbox_left.ymax + 2 * r;
			reduced_bbox_right.ymax = bbox.ymax;
			reduced_bbox_right.xmin = bbox.xmin; reduced_bbox_right.xmax = bbox.xmax;


			blockingBox.push_back(BBox(bbox.xmin, reduced_bbox_left.ymax, bbox.xmax - bbox.xmin, 2 * r));
		}

		//testing code
		//blockingBox

		DNCPoisson(reduced_bbox_left, points, priority, r, -axis,ratio);
		DNCPoisson(reduced_bbox_right, points, priority, r, -axis,ratio);

	}
	else if (dimensionCheck(bbox, r) == -axis)
	{
		DNCPoisson(bbox, points, priority, r, -axis,ratio);
	}
	else
	{
		double area = (bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin);
		//genPoints(points, bbox, area, r);

		//rest is to use tiling
		double len = bbox.xmax - bbox.xmin + 4 * r;
		double height = bbox.ymax - bbox.ymin + 4 * r;
		double new_xmin = IntervalUniform(0.0, 1.0 - ratio * len);
		double new_ymin = IntervalUniform(0.0, 1.0 - ratio * height);
		BBox ratioBbox(new_xmin, new_xmin + ratio * len, new_ymin, new_ymin + ratio * height);

		//Primary & Secondary box
		primary_right = BBox(bbox.xmax, bbox.xmax + 2 * r, bbox.ymin - 2 * r, bbox.ymax - 2 * r);
		primary_down = BBox(bbox.xmin + 2 * r, bbox.xmax, bbox.ymin - 2 * r, bbox.ymin);
		secondary_left = BBox(bbox.xmin - 2 * r, bbox.xmin + 2 * r, bbox.ymin, bbox.ymax + 2 * r);
		secondary_up = BBox(bbox.xmin + 2 * r, bbox.xmax, bbox.ymax - 2 * r, bbox.ymax + 2 * r);
		mid_NE = BBox(bbox.xmax, bbox.xmax + 2 * r, bbox.ymax - 2 * r, bbox.ymax + 2 * r);
		mid_SW = BBox(bbox.xmin - 2 * r, bbox.xmin + 2 * r, bbox.ymin - 2 * r, bbox.ymin);
		tilePoints(ratioBbox, points, priority, Point2D(bbox.xmin - 2 * r, bbox.ymin - 2 * r),ratio);


		//rest is to use regular tiling
		//double len = bbox.xmax - bbox.xmin;
		//double height = bbox.ymax - bbox.ymin;
		//double new_xmin = 0.0;// IntervalUniform(0.0, 1.0 - ratio * len);
		//double new_ymin = 0.0;
		//BBox ratioBbox(new_xmin, new_xmin + ratio * len, new_ymin, new_ymin + ratio * height);
		//tilePoints(ratioBbox, points, Point2D(bbox.xmin, bbox.ymin));

		//use tiling with distinguish of primary and secondary



	}
}


int main(int argc, char* argv[])
{
	float ratio = atof(argv[3]);
	double radius = 0.005 / ratio;
	//srand((unsigned)time(NULL));
	string pattern_filename = argv[1];
	string result_filename = argv[2];
	ifstream fin_pattern(pattern_filename);
	ofstream fout_result(result_filename);

	//build kd-tree for pattern
	vector<Point2D> pattern;
	Point2D tmp; while ((fin_pattern >> tmp.x, fin_pattern >> tmp.y))pattern.push_back(tmp);
	kd_tree = build(&pattern[0], pattern.size(), 0);

	
	//First, traverse and record all priority.
	BBox original_bbox;
	original_bbox.xmin = 0.0; original_bbox.xmax = 1.0; original_bbox.ymin = 0.0; original_bbox.ymax = 1.0;
	vector<Point2D> points;
	vector<int> priority;
	
	

	//Tile them all in one UniformGrid
	clock_t start = clock(), diff;
	//Timing
	DNCPoisson(original_bbox, points, priority, radius, XAXIS,ratio);
	
	diff = clock() - start;
	int msec = diff * 1000 / CLOCKS_PER_SEC;
	printf("Ratio: %f\nDNC taken %d seconds %d milliseconds\n", ratio, msec / 1000, msec % 1000);
	//printf("%f %d ", ratio, msec);

	UniformGrids points_in_grid(1.0 / (2 * radius));
	for (int i = 0; i < points.size(); i++)
	{
		points_in_grid.insert(points[i], priority[i]);
	}


	start = clock(), diff;
	//Timing
	points_in_grid.eliminate_for_maximal(radius);
	//points_in_grid.eliminate_for_maximal_batch(radius);

	int count = 0;
	for (int i = 0; i < points_in_grid.dimension_edge* points_in_grid.dimension_edge; i++)
	{
		for (int j = 0; j < points_in_grid.grids[i].num; j++)
		{
			if (points_in_grid.grids[i].valid[j])count++;
		}
	}

	diff = clock() - start;
	msec = diff * 1000 / CLOCKS_PER_SEC;
	//printf("%d %d\n", count, msec);
	printf("Points:%d\nEliminate for Maximal take %d seconds %d milliseconds\n",count, msec / 1000, msec % 1000);

	
	for (int i = 0; i < points_in_grid.dimension_edge * points_in_grid.dimension_edge; i++)
	{
		if (points_in_grid.grids[i].num >= 1)
		{
			for (int j = 0; j < points_in_grid.grids[i].num; j++)
			{
				if (points_in_grid.grids[i].valid[j])
				{
					fout_result << points_in_grid.grids[i].points[j].x << " " << points_in_grid.grids[i].points[j].y << endl;
					
				}

			}
		}
	}

	//points_in_grid.test_maximal(radius);

	UniformGrids ugrids(1.0 / (2 * 0.005));
	for (int i = 0; i < pattern.size(); i++)
	{
		ugrids.insert(pattern[i]);
	}
	ugrids.test_maximal(0.005);


	cin >> diff;

}