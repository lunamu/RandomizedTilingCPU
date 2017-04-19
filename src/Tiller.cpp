#include<tiller.h>

inline int dimensionCheck(BBox bbox, Float r)//consider leave a minimum leaf node.
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
void Tiller::test_conflict(Float radius)
{
	ofstream gap_file(result_dir + "conf");
	for (int grid_idx = 0; grid_idx < points_in_grid.width * points_in_grid.height; grid_idx++)
	{
		for (int in_idx = 0; in_idx < points_in_grid.grids[grid_idx].num; in_idx++)//iteration of all points
		{
			if (points_in_grid.grids[grid_idx].valid[in_idx])
			{
				Point2D& cur_point = points_in_grid.grids[grid_idx].points[in_idx];
				if (!points_in_grid.dartSearch(cur_point, 2 * radius))conflict_points.push_back(cur_point);
			}
		}
	}
	for (int i = 0; i < conflict_points.size(); i++)
	{
		gap_file << conflict_points[i].x << " " << conflict_points[i].y << endl;
	}


}
void Tiller::test_maximal(Float radius)
{
	//for each valid point in the grid, search range 4*r
	//test if circum center within 2*r from any points.
	//for each valid point in the grid, search range 4*r
	//test if circum center within 2*r from any points.
	ofstream gap_file(result_dir + "gap_points");
	for (int grid_idx = 0; grid_idx < points_in_grid.width * points_in_grid.height; grid_idx++)
	{
		for (int in_idx = 0; in_idx < points_in_grid.grids[grid_idx].num; in_idx++)//iteration of all points
		{
			if (points_in_grid.grids[grid_idx].valid[in_idx])
			{
				Point2D& cur_point = points_in_grid.grids[grid_idx].points[in_idx];
				vector<Point2D> conflictBuffer;
				vector<int> conflictPri;
				vector<int> conflictGrididx;
				vector<int> conflictIngrid_idx;
				points_in_grid.dartSearch_buffer_pri(cur_point, 4 * radius, conflictBuffer, conflictPri, conflictGrididx, conflictIngrid_idx);

				for (auto first_point = conflictBuffer.begin(); first_point != conflictBuffer.end(); first_point++)
				{
					for (auto second_point = first_point + 1; second_point != conflictBuffer.end(); second_point++)
					{
						if (first_point == second_point) continue;
						auto fp = *first_point;
						auto sp = *second_point;
						Point2D center;
						Float cir_r2;
						Circumcenter(cur_point, fp, sp, center, cir_r2);
						//if (cir_r2 > 16 * range * range)continue;
						bool same;
						if (center.x > points_in_grid.gridBbox.xmax || center.x < points_in_grid.gridBbox.xmin || center.y > points_in_grid.gridBbox.ymax || center.y < points_in_grid.gridBbox.ymin)continue;
						else if (points_in_grid.dartSearch(center, 2 * radius, same))
						{
							if (!same)gap_file << center.x << " " << center.y << endl;
						}

					}
				}
			}

		}

	}
	printf("done, gaps above\n");
}


bool Tiller::is_maximal(Float radius)
{
	//for each valid point in the grid, search range 4*r
	//test if circum center within 2*r from any points.
	//for each valid point in the grid, search range 4*r
	//test if circum center within 2*r from any points.
	ofstream gap_file(result_dir + "gap_points");
	for (int grid_idx = 0; grid_idx < points_in_grid.width * points_in_grid.height; grid_idx++)
	{
		for (int in_idx = 0; in_idx < points_in_grid.grids[grid_idx].num; in_idx++)//iteration of all points
		{
			if (points_in_grid.grids[grid_idx].valid[in_idx])
			{
				Point2D& cur_point = points_in_grid.grids[grid_idx].points[in_idx];
				vector<Point2D> conflictBuffer;
				vector<int> conflictPri;
				vector<int> conflictGrididx;
				vector<int> conflictIngrid_idx;
				points_in_grid.dartSearch_buffer_pri(cur_point, 4 * radius, conflictBuffer, conflictPri, conflictGrididx, conflictIngrid_idx);

				for (auto first_point = conflictBuffer.begin(); first_point != conflictBuffer.end(); first_point++)
				{
					for (auto second_point = first_point + 1; second_point != conflictBuffer.end(); second_point++)
					{
						if (first_point == second_point) continue;
						auto fp = *first_point;
						auto sp = *second_point;
						Point2D center;
						Float cir_r2;
						Circumcenter(cur_point, fp, sp, center, cir_r2);
						//if (cir_r2 > 16 * range * range)continue;
						bool same;
						if (center.x > points_in_grid.gridBbox.xmax || center.x < points_in_grid.gridBbox.xmin || center.y > points_in_grid.gridBbox.ymax || center.y < points_in_grid.gridBbox.ymin)continue;
						else if (points_in_grid.dartSearch(center, 2 * radius, same))
						{
							if (!same)return false;
						}

					}
				}
			}

		}

	}
	return true;
}
void Tiller::insert_in_gap(Point2D pivotPoint, Point2D conflictPoint, Float radius, int depth)
{
	Point2D ptBuffer[20];
	int conflict_num;
	points_in_grid.dartSearch_buffer_array(conflictPoint, 4 * radius, ptBuffer, conflict_num);
	
	//if (conflict_num < 2) return;
	for (int fp_idx = 0; fp_idx < conflict_num - 1; fp_idx++)
	{
		for (int sp_idx = fp_idx + 1; sp_idx < conflict_num; sp_idx++)
		{
			if (fp_idx == sp_idx)continue;
			Point2D fp = ptBuffer[fp_idx]; Point2D sp = ptBuffer[sp_idx];
			Point2D center; Float cir_r2;
			if (Circumcenter(pivotPoint, fp, sp, center, cir_r2))
			{
				if (center.x > points_in_grid.gridBbox.xmax || center.x < points_in_grid.gridBbox.xmin || center.y > points_in_grid.gridBbox.ymax || center.y < points_in_grid.gridBbox.ymin)continue;
				if (points_in_grid.dartSearch_other(center, 2 * radius))
				{
					points_in_grid.insert(center);
					return; 
				}
			}
			else continue;

		}
	}
	
}

void Tiller::process_pivot_point(PointRecurStruct& pivotPointStruct, Float radius)
{
	PointRecurStruct conBuf[20];
	int conflict_num;
	/*
	conflictBuffer.clear();
	conflictPri.clear();
	conflictGrididx.clear();
	conflictIngrid_idx.clear();*/
	points_in_grid.dartSearch_buffer_pri_array(pivotPointStruct.p, 2 * radius, conBuf, conflict_num);
	for (int i = 0; i < conflict_num; i++)
	{
		if (conBuf[i].pri > pivotPointStruct.pri)
		{
			process_pivot_point(conBuf[i], radius);
			return;
		}
	}
	//cout << "Pivot point: " << pivotPoint.x << " " << pivotPoint.y << endl;
	//cout << "Conflict point: " << endl;
	/*for (int i = 0; i < conflictBuffer.size(); i++)
	{
	cout << conflictBuffer[i].x << " " << conflictBuffer[i].y << endl;
	}*/
	for (int i = 0; i < conflict_num; i++)
	{
		int gidx = conBuf[i].gidx;
		int iidx = conBuf[i].igidx;
		points_in_grid.grids[gidx].valid[iidx] = INVALID;	//eliminate;
		
		int depth = 0;
		insert_in_gap(pivotPointStruct.p, conBuf[i].p, radius, depth);
	}
}

void Tiller::thread_ops(float radius,int per_thread_ops, int start_offset, int size)
{
	for (int grid_idx = start_offset; (grid_idx <start_offset+ per_thread_ops) && (grid_idx <size); grid_idx++)
	{
		int num = points_in_grid.grids[grid_idx].num;
		if (num > 0)
		{
			for (int in_idx = 0; in_idx < num; in_idx++)//iteration of all points
			{
				if (!points_in_grid.grids[grid_idx].valid[in_idx])continue;
				PointRecurStruct cur_p;
				cur_p.p = points_in_grid.grids[grid_idx].points[in_idx];
				cur_p.pri = points_in_grid.grids[grid_idx].priority[in_idx];
				cur_p.gidx = grid_idx;
				cur_p.igidx = in_idx;
				process_pivot_point(cur_p, radius);
			}
		}
	}
}

void Tiller::eliminate_for_maximal(Float radius)
{
	int thread_num = 24;
	int size = points_in_grid.width * points_in_grid.height;
	int per_thread_ops = (size + thread_num) / thread_num;
	int offsets[24];
	for (int i = 0; i < thread_num; i++)
	{
		offsets[i] = per_thread_ops*i;
	}
	thread all_threads[24];
	for (int i = 0; i < thread_num; i++)
	{
		all_threads[i] = thread(&Tiller::thread_ops, this, radius, per_thread_ops, offsets[i], size);
	}
	for (int i = 0; i < thread_num; i++)
	{
		all_threads[i].join();
	}
	/*
	ofstream chain(result_dir + "chain");
	for (int i = 0; i < chain_points.size(); i++)
	{
		chain << chain_points[i].x << " " << chain_points[i].y << endl;
	}*/
}

void Tiller::traverse_and_classify(KDnode*root, BBox query, Point2D offset, Float ratio)
{
	if (withinBox(query, root->cell_bbox))
	{
		for (int i = 0; i < root->num; i++)
		{
			Point2D tmp_p = root->pts[i];
			//the query is the ratio box, map point in ratio box back.
			tmp_p.x -= query.xmin;
			tmp_p.y -= query.ymin;
			tmp_p.x /= ratio;
			tmp_p.y /= ratio;
			tmp_p.x += offset.x;
			tmp_p.y += offset.y;

			//if (within(primary_right, tmp_p) || within(primary_down, tmp_p))result.push_back(tmp_p);
			if (!within(ide, tmp_p))continue;
			result.push_back(tmp_p);
			if (within(secondary_left, tmp_p) || within(secondary_up, tmp_p)) {
				priority.push_back(0);
			}
			else if (within(primary_right, tmp_p) || within(primary_down, tmp_p))
			{
				priority.push_back(2);
			}
			else if (within(mid_NE, tmp_p) || within(mid_SW, tmp_p))
			{
				priority.push_back(1);
			}
			else
			{
				priority.push_back(-1);
			}
		}
	}
	else if (root->leaf == true)
	{
		for (int i = 0; i < root->num; i++)
		{
			if (within(query, root->pts[i]))
			{
				Point2D tmp_p = root->pts[i];
				tmp_p.x -= query.xmin;
				tmp_p.y -= query.ymin;
				tmp_p.x /= ratio;
				tmp_p.y /= ratio;
				tmp_p.x += offset.x;
				tmp_p.y += offset.y;

				if (!within(ide, tmp_p))continue;

				result.push_back(tmp_p);
				if (within(secondary_left, tmp_p) || within(secondary_up, tmp_p)) {
					priority.push_back(0);
				}
				else if (within(primary_right, tmp_p) || within(primary_down, tmp_p))
				{
					priority.push_back(2);
				}
				else if (within(mid_NE, tmp_p) || within(mid_SW, tmp_p))
				{
					priority.push_back(1);
				}
				else
				{
					priority.push_back(-1);
				}
			}
		}
	}
	else
	{
		if (root->split_axis == X_AXIS)
		{
			if (root->split_position >= query.xmin && root->split_position <= query.xmax)
			{
				traverse_and_classify(root->left, query, offset, ratio);
				traverse_and_classify(root->right, query, offset, ratio);
			}
			else if (query.xmax < root->split_position)
			{
				traverse_and_classify(root->left, query, offset, ratio);
			}
			else
			{
				traverse_and_classify(root->right, query, offset, ratio);
			}
		}
		else
		{
			if (root->split_position >= query.ymin && root->split_position <= query.ymax)
			{
				traverse_and_classify(root->left, query, offset, ratio);
				traverse_and_classify(root->right, query, offset, ratio);
			}
			else if (query.ymax < root->split_position)
			{
				traverse_and_classify(root->left, query, offset, ratio);
			}
			else
			{
				traverse_and_classify(root->right, query, offset, ratio);
			}
		}
	}
}


void Tiller::DivideConquerTiling(BBox bbox, Float r, int axis, Float ratio)
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



			//blockingBox.push_back(BBox(reduced_bbox_left.xmax, bbox.ymin, 2 * r, bbox.ymax - bbox.ymin));
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


			//blockingBox.push_back(BBox(bbox.xmin, reduced_bbox_left.ymax, bbox.xmax - bbox.xmin, 2 * r));
		}

		//testing code
		//blockingBox

		DivideConquerTiling(reduced_bbox_left, r, -axis, ratio);
		DivideConquerTiling(reduced_bbox_right, r, -axis, ratio);

	}
	else if (dimensionCheck(bbox, r) == -axis)
	{
		DivideConquerTiling(bbox, r, -axis, ratio);
	}
	else
	{
		Float area = (bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin);
		//genPoints(points, bbox, area, r);

		//rest is to use tiling
		Float len = bbox.xmax - bbox.xmin + 4 * r;
		Float height = bbox.ymax - bbox.ymin + 4 * r;
		Float new_xmin = IntervalUniform(0.0, 1.0 - ratio * len);
		Float new_ymin = IntervalUniform(0.0, 1.0 - ratio * height);
		BBox ratioBbox(new_xmin, new_xmin + ratio * len, new_ymin, new_ymin + ratio * height);

		//Primary & Secondary box
		primary_right = BBox(bbox.xmax, bbox.xmax + 2 * r, bbox.ymin - 2 * r, bbox.ymax - 2 * r);
		primary_down = BBox(bbox.xmin + 2 * r, bbox.xmax, bbox.ymin - 2 * r, bbox.ymin);
		secondary_left = BBox(bbox.xmin - 2 * r, bbox.xmin + 2 * r, bbox.ymin, bbox.ymax + 2 * r);
		secondary_up = BBox(bbox.xmin + 2 * r, bbox.xmax, bbox.ymax - 2 * r, bbox.ymax + 2 * r);
		mid_NE = BBox(bbox.xmax, bbox.xmax + 2 * r, bbox.ymax - 2 * r, bbox.ymax + 2 * r);
		mid_SW = BBox(bbox.xmin - 2 * r, bbox.xmin + 2 * r, bbox.ymin - 2 * r, bbox.ymin);
		tilePoints(ratioBbox, Point2D(bbox.xmin - 2 * r, bbox.ymin - 2 * r), ratio);


		//rest is to use regular tiling
		//Float len = bbox.xmax - bbox.xmin;
		//Float height = bbox.ymax - bbox.ymin;
		//Float new_xmin = 0.0;// IntervalUniform(0.0, 1.0 - ratio * len);
		//Float new_ymin = 0.0;
		//BBox ratioBbox(new_xmin, new_xmin + ratio * len, new_ymin, new_ymin + ratio * height);
		//tilePoints(ratioBbox, points, Point2D(bbox.xmin, bbox.ymin));

		//use tiling with distinguish of primary and secondary
	}
}


void Tiller::global_filling(Float radius, vector<Point2D>& query_points, vector<Point2D>& next_batch)
{
	//for each valid point in the grid, search range 4*r
	//test if circum center within 2*r from any points.
	//if not, insert the point.
	ofstream gap_file(result_dir + "gap_points");
	for (int i = 0; i < query_points.size(); i++)
	{
		Point2D query = query_points[i];
		Point2D conflictBuffer[20];
		int num_conflict;
		points_in_grid.dartSearch_buffer_array(query, 4 * radius, conflictBuffer, num_conflict);

		for (int fp_idx = 0; fp_idx < num_conflict - 1; fp_idx++)
		{
			for (int sp_idx = i + 1; sp_idx < num_conflict; sp_idx++)
			{
				if (sp_idx == fp_idx) continue;
				Point2D fp = conflictBuffer[fp_idx];
				Point2D sp = conflictBuffer[sp_idx];
				Point2D center;
				Float cir_r2;
				Circumcenter(query, fp, sp, center, cir_r2);

				bool same;
				if (center.x > points_in_grid.gridBbox.xmax || center.x < points_in_grid.gridBbox.xmin || center.y > points_in_grid.gridBbox.ymax || center.y < points_in_grid.gridBbox.ymin)continue;
				else if (points_in_grid.dartSearch(center, 2 * radius, same))
				{
					if (!same)
					{
						points_in_grid.insert(center);
						next_batch.push_back(center);
						//gap_file << center.x << " " << center.y << endl;
					}
				}
			}
		}
	}
	//}
	//for (int grid_idx = 0; grid_idx < points_in_grid.width * points_in_grid.height; grid_idx++)
	//{
	//	for (int in_idx = 0; in_idx < points_in_grid.grids[grid_idx].num; in_idx++)//iteration of all points
	//	{
	//		if (points_in_grid.grids[grid_idx].priority[in_idx] < 0)continue;
	//		if (points_in_grid.grids[grid_idx].valid[in_idx])
	//		{
	//			point_checked++;
	//			Point2D& cur_point = points_in_grid.grids[grid_idx].points[in_idx];
	//			
	//		}

	//	}

	//}
	//printf("Points Checked: %d\n", point_checked);
}