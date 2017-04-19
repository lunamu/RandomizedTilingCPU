#ifndef _UNIFORMGRIDS_
#define _UNIFORMGRIDS_

#include <UtilsDefs.h>
#include <queue>
const bool VALID = true;
const bool INVALID = false;

const int MAX_PER_GRID = 8;
using namespace std;

struct Grid
{
	int morton;
	int num;
	Point2D points[MAX_PER_GRID];
	bool valid[MAX_PER_GRID];
	int priority[MAX_PER_GRID];
	Grid(){
		morton = 0;
		num = 0;
	}
};
const Float EPS = 0.0000001;


struct PointRecurStruct
{
	Point2D p;
	int gidx;
	int igidx;
	int pri;
};


//2D UniformGrids class
class UniformGrids
{
public:
	UniformGrids(){}
	~UniformGrids();
	//Construct empty UniformGrids based on dimension
	UniformGrids(int dimension);

	//create with width and height
	UniformGrids(int w, int h, BBox _bbox);

	inline unsigned int SpatialToIdx(Point2D point){return (unsigned int) (((int)((point.y - gridBbox.ymin)*height)) * width + (point.x - gridBbox.xmin) * width);}
	
	//insert a point with priority
	void insert(Point2D point, int priority = 0);

	//return false if there are some points within range.otherwise return true;
	bool dartSearch(Point2D point, Float range);
	bool dartSearch(Point2D point, Float range, bool& same);
	bool dartSearch_other(Point2D point, Float range);
	//push all points withing the ring (from inner range to outer range) to the buffer buf
	void ringSearch_buffer(Point2D point, Float range_inner, Float range_outer, vector<Point2D>& buf);	
	void dartSearch_buffer(Point2D point, Float range, vector<Point2D>& buf);

	//push all conflict points to conflictbuffer, record their priority, grididx and in_grid idx;
	void dartSearch_buffer_pri(Point2D point, Float range, vector<Point2D>& conflictBuffer,
		vector<int>& conflictPri, vector<int>& conflictGrididx, vector<int>& conflictIngrid_idx);

	void dartSearch_buffer_pri_array(Point2D point, Float range, PointRecurStruct* conflictBuffer, int& conflict_num);
	void dartSearch_buffer_array(Point2D point, Float range, Point2D* conflictBuffer, int& conflict_num);
	void ringSearch_array(Point2D point, Float range_inner, Float range_outer, Point2D* buf, int& conflict_num);
	//insert in the gap, for pivot point and its corresponding conflict points
	void insert_in_gap(Point2D pivotPoint, Point2D conflictPoint, Float radius);

	//for each pivot_point, do process in the paper
	//attention: parameter is radius
	void process_pivot_point(Point2D& pivotPoint, int pri, Float radius, int grid_idx, int ingrid_idx);
	
	//wrapper function, call this to eliminate all conflict points, and insert points for maximal coverage.
	//attention: parameter is radius
	void eliminate_for_maximal(Float radius);

	//wrapper function, climinate all conflict points, insert points for maximal coverage.
	//This function will insert points on corners.
	void eliminate_for_maximal_corner(Float radius);

	void process_pivot_point_corner(Point2D& pivotPoint, int pri, Float radius, int grid_idx, int ingrid_idx);
	void insert_in_gap_corner(Point2D pivotPoint, Point2D conflictPoint, Float radius);


	//wrapper function, this function eliminate all conflict points, insert points at once.
	void eliminate_for_maximal_batch(Float radius);
	void process_pivot_point_batch(Point2D& pivotPoint, int pri, Float radius, int grid_idx, int ingrid_idx);
	void insert_in_gap_batch(Point2D pivotPoint, vector<Point2D> conflictPoint, Float radius);

	//attention: parameter is radius
	void test_maximal(Float radius);

	int getCount(){int count = 0;
		for (int i = 0; i < dimension_edge*dimension_edge; i++){
			for (int j = 0; j < grids[i].num; j++){
				if (grids[i].valid[j])count++;
			}
		}return count;
	}
	void printToFile(string output_file_name)
	{
		ofstream fout_result(output_file_name);
		for (int i = 0; i <dimension_edge *dimension_edge; i++)
		{
			if (grids[i].num >= 1)
			{
				for (int j = 0; j < grids[i].num; j++)
				{
					if (grids[i].valid[j])
					{
						fout_result << grids[i].points[j].x << " " << grids[i].points[j].y << endl;

					}

				}
			}
		}
	}

	//print all grid information
	void print();
	Grid* grids;
	int dimension_edge;
	int width;
	int height;

	//bbox with actual coordinate
	BBox gridBbox;
	Float w_f;
	Float h_f;

	Float itval;
	Float itval_w;
	Float itval_h;
};

#endif