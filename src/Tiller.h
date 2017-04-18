#pragma once

#include <UtilsDefs.h>
#include <UniformGrids.h>
#include <KD_tree.h>
//This is a class in charge of tilling.
//Main task: input point pattern, tile point sampling, and 


class Tiller {

public:
	Tiller() {};
	Tiller(BBox bbox_, KDnode* kd_, Float radius, string rlt): bbox(bbox_),kd_tree(kd_), points_in_grid(1.0 / (2 * radius) ), result_dir(rlt){ }
	
	~Tiller() {};
	void insert_in_gap(Point2D pivotPoint, Point2D conflictPoint, Float radius, int depth);
	void eliminate_for_maximal(Float radius);
	void process_pivot_point(Point2D& pivotPoint, int pri, Float radius, int grid_idx, int ingrid_idx, int chain);
	void DivideConquerTiling(BBox bbox, Float r, int axis, Float ratio);
	void tilePoints(BBox bbox,Point2D offset, Float ratio){
		traverse_and_classify(kd_tree,bbox, offset, ratio);
	}
	void traverse_and_classify(KDnode* root, BBox query, Point2D offset, Float ratio);

	int getGridCount() { return points_in_grid.getCount(); }

	void printToFile(std::string output_file_name)
	{
		points_in_grid.printToFile(output_file_name);
	}
	void test_maximal(Float radius);
	bool is_maximal(Float radius);
	void test_conflict(Float radius);
	void global_filling(Float radius, vector<Point2D>& query_points, vector<Point2D>& next_batch);

	BBox bbox;
	KDnode* kd_tree;
	UniformGrids points_in_grid;
	vector<Point2D> result;
	vector<int> priority;
	vector<Point2D> gap_points;
	vector<Point2D> conflict_points;
	
	vector<Point2D> chain_points;

	//point buffer to test for gaps;
	vector<Point2D> pointTestBuffer;
	string result_dir;


	//Used in process pivot points
	vector<Point2D> conflictBuffer;
	vector<int> conflictPri;
	vector<int> conflictGrididx;
	vector<int> conflictIngrid_idx;
	//vector<Point2D> pattern;
};