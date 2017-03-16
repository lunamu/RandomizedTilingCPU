#include <KD_tree.h>


BBox ide(0.0, 1.0, 0.0, 1.0);

KDnode* build(Point2D* points, int num, int depth)
{
	if (num == 0)
	{
		return NULL;
	}
	else if (num == 1)
	{
		KDnode* rlt = new KDnode();
		rlt->leaf = true;
		rlt->num = 1;
		rlt->pts = points;
		//printf("leaf created:\n\n " );
		return rlt;
	}
	else
	{
		KDnode* root = new KDnode();
		root->leaf = false;
		root->num = num;
		root->pts = points;
		char split_axis = (depth % 2) ? Y_AXIS : X_AXIS;
		root->split_axis = split_axis;
		root->cell_bbox.xmax = 0.0; root->cell_bbox.ymax = 0.0;
		root->cell_bbox.xmin = 1.0; root->cell_bbox.ymin = 1.0;

		int num_pos, num_neg;
		if (split_axis == X_AXIS)
		{
			sort(points, points + num, cmp_x);
			//qsort(ps, num, sizeof(Point2D), cmp_x);
			int mid = num >> 1;
			root->split_position = points[mid].x;
			num_pos = mid;
			num_neg = num - mid;
			for (int i = 0; i < num; i++)
			{
				root->cell_bbox.xmax = max(root->cell_bbox.xmax, points[i].x);
				root->cell_bbox.ymax = max(root->cell_bbox.ymax, points[i].y);
				root->cell_bbox.xmin = min(root->cell_bbox.xmin, points[i].x);
				root->cell_bbox.ymin = min(root->cell_bbox.ymin, points[i].y);
			}
			//printf("Node created: %f,%f,%f,%f,%d,%d\n", root->cell_bbox.xmin, root->cell_bbox.ymin, root->cell_bbox.xmax, root->cell_bbox.ymax, num, depth);
			root->left = build(points, num_pos, depth + 1);
			root->right = build(points + num_pos, num_neg, depth + 1);
		}
		else
		{
			sort(points, points + num, cmp_y);
			int mid = num >> 1;
			root->split_position = points[mid].y;
			num_pos = mid;
			num_neg = num - mid;
			for (int i = 0; i < num; i++)
			{
				root->cell_bbox.xmax = max(root->cell_bbox.xmax, points[i].x);
				root->cell_bbox.ymax = max(root->cell_bbox.ymax, points[i].y);
				root->cell_bbox.xmin = min(root->cell_bbox.xmin, points[i].x);
				root->cell_bbox.ymin = min(root->cell_bbox.ymin, points[i].y);
			}

			root->left = build(points, num_pos, depth + 1);
			root->right = build(points + num_pos, num_neg, depth + 1);
		}
		return root;
	}
}



void traverse_and_classify(KDnode*root, vector<Point2D>& result, vector<int>& priority, BBox query, Point2D offset,float ratio)
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
			if (within(secondary_left, tmp_p) || within(secondary_up, tmp_p)){
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
				if (within(secondary_left, tmp_p) || within(secondary_up, tmp_p)){
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
				traverse_and_classify(root->left, result, priority, query,offset,ratio);
				traverse_and_classify(root->right, result, priority, query, offset,ratio);
			}
			else if (query.xmax < root->split_position)
			{
				traverse_and_classify(root->left, result, priority, query, offset,ratio);
			}
			else
			{
				traverse_and_classify(root->right, result, priority, query, offset, ratio);
			}
		}
		else
		{
			if (root->split_position >= query.ymin && root->split_position <= query.ymax)
			{
				traverse_and_classify(root->left, result, priority, query, offset, ratio);
				traverse_and_classify(root->right, result, priority, query, offset, ratio);
			}
			else if (query.ymax < root->split_position)
			{
				traverse_and_classify(root->left, result, priority, query, offset, ratio);
			}
			else
			{
				traverse_and_classify(root->right, result, priority, query, offset, ratio);
			}
		}
	}
}
