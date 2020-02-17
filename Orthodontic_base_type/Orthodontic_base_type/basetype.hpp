#ifndef ORTH_BASETYPE_H
#define ORTH_BASETYPE_H

#include <iostream>
#include <vector>
#include "Point.hpp"
#include <iomanip> 
#include <memory>

using std::vector;

#ifndef M_PI
#define M_PI 3.14159265358979323846   // pi
#endif


#define SharedPtr(ClassName) typedef std::shared_ptr<ClassName> p##ClassName;


// Orthodontics library
namespace orth
{
	//***************************************//
	//			    ������������              //
	//***************************************//

	typedef Point3f Vectorf;
	typedef Point3d Vectord;
	typedef Point3f Normal;
	typedef Point3ui Face;
	typedef Point3uc Color;
	typedef double Curvature;
	typedef short Label;
	typedef unsigned char Mask;
	typedef long Index_l;
	typedef unsigned int Index_ui;
	typedef vector<Index_ui> Point2Edge;

	struct HalfEdge_Parallel
	{
		Index_ui CurrentPoint;
		Index_ui EndPoint;
		Index_ui OppoEdge;
		Index_ui CurrentFace;
		Index_ui NextEdge;
		bool SearchLabel;
	};

	struct HalfEdge_Serial
	{
		Vectord* CurrentPoint;
		Vectord* EndPoint;
		HalfEdge_Serial* OppoEdge;
		Face* CurrentFace;
		//HalfEdge_Serial* NextEdge;
		std::shared_ptr<HalfEdge_Serial> NextEdge;
		bool SearchLabel;
	};

	//����ͼ������
	typedef vector<Vectord> PointCloudD;
	typedef vector<Vectorf> PointCloudF;
	typedef vector<Normal> PointNormal;
	typedef vector<Color> PointColor;
	typedef vector<Face> Faces;
	typedef vector<Normal> FacesNormal;
	typedef vector<Label> PointLabel;


	typedef vector<Mask> PointMask;
	typedef vector<Mask> FaceMask;
	typedef vector<Mask> EdgeMask;


	typedef vector<Index_ui> SamplePoints;
	typedef vector<Curvature> PointCurs;
	typedef vector<HalfEdge_Parallel> HalfEdgeCloud_P;
	typedef std::shared_ptr<HalfEdge_Serial> HalfEdgeCloud_S;
	typedef vector<Point2Edge> HalfPointCloud_P;
	typedef vector<Point2Edge> HalfPointCloud_S;

	//��Ӱ�Χ��
	struct Box {
		Vectorf u_0;
		Vectorf u_1;
		Vectorf u_2;
		Vectorf u_3;
		Vectorf d_0;
		Vectorf d_1;
		Vectorf d_2;
		Vectorf d_3;
	};

	struct Plane
	{
		double A;
		double B;
		double C;
		double D;

		Point3d Center;
	};

#define ACROSS 0
#define COPLANE 1
#define AEDGE 2
#define AVERTEX 3
#define NONINTERSECT 4


	struct SpliteKeyPointSt {
		Vectorf firstKeyPointPos;   //当前关键单坐标
									//Vectorf lastKeyPointPos;   //第二个关键点坐标
		PointCloudF pointsPos;    //关键点 以及之间的点坐标
								  //vector<int> pointsIndex;
		int  nFirstKeyPointIndex;
		int nLastKeyPointIndex;
		int nlastKeyPointStIndex;   //上一个关键点结构的索引
		int nkeyPointStIndex;    //关键点索引
		SpliteKeyPointSt() {
			nkeyPointStIndex = -1;
			nlastKeyPointStIndex = -1;
		}
	};
	SharedPtr(SpliteKeyPointSt);


	//class H_type
	//{
	//public:
	//	H_type();
	//	~H_type();

	//private:

	//};


	//struct H_vert
	//{

	//	orth::Point3d coord;

	//	H_edge* edge;  // one of the half-edges emantating from the vertex

	//};

	//struct H_face
	//{

	//	H_edge* edge;  // one of the half-edges bordering the face

	//};

	//struct H_edge
	//{

	//	H_vert* vert;   // vertex at the end of the half-edge
	//	H_edge* pair;   // oppositely oriented adjacent half-edge 
	//	H_face* face;   // face the half-edge borders
	//	H_edge* next;   // next half-edge around the face

	//};

	//***************************************//
	//			      ��������               //
	//**************************************//

	//三点计算平面
	inline Plane Point3CalculatePlane(Point3f &point_a, Point3f &point_b, Point3f &point_c)
	{
		Plane NewPlane;
		NewPlane.A = (point_c.y - point_a.y)*(point_c.z - point_a.z) - (point_b.z - point_a.z)*(point_c.y - point_a.y);

		NewPlane.B = (point_c.x - point_a.x)*(point_b.z - point_a.z) - (point_b.x - point_a.x)*(point_c.z - point_a.z);

		NewPlane.C = (point_b.x - point_a.x)*(point_c.y - point_a.y) - (point_c.x - point_a.x)*(point_b.y - point_a.y);

		NewPlane.D = -(NewPlane.A * point_a.x + NewPlane.B * point_a.y + NewPlane.C * point_a.z);

		NewPlane.Center = (point_a + point_b + point_c) / 3.0;

		return NewPlane;
	}

	//�����η������
	inline Normal TriangleNormal(Point3d &point_a, Point3d &point_b, Point3d &point_c)
	{
		return ((point_b - point_a).cross(point_c - point_a));
	}
	inline Normal TriangleNormal(Point3f &point_a, Point3f &point_b, Point3f &point_c)
	{
		return ((point_b - point_a).cross(point_c - point_a));
	}

	//�㵽�����
	inline double Point2PointDistance(Point3d &p1, Point3d &p2)
	{
		//double dis = (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z);
		double dis = (p1 - p2).dot(p1 - p2);
		return sqrt(dis);
	}
	inline float Point2PointDistance(Point3f &p1, Point3f &p2)
	{
		//float dis = (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z);
		float dis = (p1 - p2).dot(p1 - p2);
		return sqrt(dis);
	}

	//点到直线距离
	inline float Point2LineDistance(Point3f &p1, Point3f &line_p1, Point3f &line_p2)
	{
		Point3f vector_line = line_p2 - line_p1;
		Point3f vector_p1 = p1 - line_p1;
		float dis_p1 = Point2PointDistance(p1, line_p1);
		float dis_line = Point2PointDistance(line_p2, line_p1);
		float theta = acos((vector_line.dot(vector_p1) / dis_line));
		return sin(theta) * dis_p1;
	}
	inline float Point2LineDistance(Point3d &p, Point3d &line_p1, Point3d &line_p2)
	{
		Point3d vector_line = line_p2 - line_p1;
		Point3d vector_p1 = p - line_p1;
		float dis_p1 = Point2PointDistance(p, line_p1);
		float dis_line = Point2PointDistance(line_p2, line_p1);
		float theta = acos((vector_line.dot(vector_p1) / dis_line));
		return sin(theta) * dis_p1;
	}

	//�㵽ƽ�����
	inline double Point2PlaneDistance(Point3d &point, Plane &target_plane)
	{
		double a = target_plane.A, b = target_plane.B, c = target_plane.C, d = target_plane.D;
		double pqdot = a * point.x + b * point.y + c * point.z + d;
		double n = sqrt(a*a + b * b + c * c);
		return pqdot / n;
	}
	inline float Point2PlaneDistance(Point3f &point, Plane &target_plane)
	{
		float a = target_plane.A, b = target_plane.B, c = target_plane.C, d = target_plane.D;
		float pqdot = a * point.x + b * point.y + c * point.z + d;
		float n = sqrt(a*a + b * b + c * c);
		return pqdot / n;
	}

	//�㵽ƽ�����
	inline double Point2PlaneDistance(Point3d &point, Point3d &point_a, Point3d &point_b, Point3d &point_c)
	{
		Point3d ab = point_b - point_a;
		Point3d ac = point_c - point_a;
		Point3d normal_vector = ab.cross(ac);
		double a = normal_vector.x, b = normal_vector.y, c = normal_vector.z; double d = -a * point_a.x - b * point_a.y - c * point_a.z;
		double pqdot = a * point.x + b * point.y + c * point.z + d;
		double n = sqrt(a*a + b * b + c * c);
		return pqdot / n;
	}
	inline float Point2PlaneDistance(Point3f &point, Point3f &point_a, Point3f &point_b, Point3f &point_c)
	{
		Point3f ab = point_b - point_a;
		Point3f ac = point_c - point_a;
		Point3f normal_vector = ab.cross(ac);
		float a = normal_vector.x, b = normal_vector.y, c = normal_vector.z; float d = -a * point_a.x - b * point_a.y - c * point_a.z;
		float pqdot = a * point.x + b * point.y + c * point.z + d;
		float n = sqrt(a*a + b * b + c * c);
		return pqdot / n;
	}

	//点在直线上
	inline bool IsPointOnLine(Point3f &p, Point3f &line_p1, Point3f &line_p2, float min_distance)
	{
		//float distance = Point2LineDistance(p, line_p1, line_p2);
		//if (distance>min_distance)
		//{
		//	return false;
		//}

		Point3d vector_p1 = p - line_p1; //vector_p1.normalize();
		if (!vector_p1.normalize())
		{
			std::cout << "							vector_p1.normalize() = " << p << " -- " << line_p1 << std::endl;
			return true;
		}
		Point3d vector_p2 = p - line_p2; //vector_p2.normalize();
		if (!vector_p2.normalize())
		{
			std::cout << "							vector_p1.normalize() = " << p << " -- " << line_p2 << std::endl;
			return true;
		}
		std::cout << "							(vector_p1.dot(vector_p2) + 1) = " << abs(vector_p1.dot(vector_p2) + 1) << std::endl;
		return min_distance > abs(vector_p1.dot(vector_p2) + 1);
	}

	//plucker�������
	inline void plucker(Point3d &a, Point3d &b, double* l)
	{
		l[0] = a.x*b.y - b.x*a.y;
		l[1] = a.x*b.z - b.x*a.z;
		l[2] = a.x - b.x;
		l[3] = a.y*b.z - b.y*a.z;
		l[4] = a.z - b.z;
		l[5] = b.y - a.y;
	}
	inline void plucker(Point3f &a, Point3f &b, float* l)
	{
		l[0] = a.x*b.y - b.x*a.y;
		l[1] = a.x*b.z - b.x*a.z;
		l[2] = a.x - b.x;
		l[3] = a.y*b.z - b.y*a.z;
		l[4] = a.z - b.z;
		l[5] = b.y - a.y;
	}

	//plucker�������
	inline double sideOp(double *a, double *b)
	{
		double res = a[0] * b[4] + a[1] * b[5] + a[2] * b[3] + a[3] * b[2] + a[4] * b[0] + a[5] * b[1];
		return res;
	}
	inline float sideOp(float *a, float *b)
	{
		float res = a[0] * b[4] + a[1] * b[5] + a[2] * b[3] + a[3] * b[2] + a[4] * b[0] + a[5] * b[1];
		return res;
	}


	//�����ཻ�ж�
	inline int LineFaceIntersect(Point3d &l1, Point3d &l2, Point3d &a, Point3d &b, Point3d &c)
	{
		double e1[6] = { 0 }, e2[6] = { 0 }, e3[6] = { 0 }, L[6] = { 0 };
		plucker(b, a, e1);
		plucker(c, b, e2);
		plucker(a, c, e3);
		plucker(l1, l2, L);

		double s1 = sideOp(L, e1);
		double s2 = sideOp(L, e2);
		double s3 = sideOp(L, e3);

		//cout << s1<<" --- " << s2 << " --- " << s3 << endl;

		if (s1 == 0 && s2 == 0 && s3 == 0)
		{
			return COPLANE;
		}
		else if ((s1 > 0 && s2 > 0 && s3 > 0) || (s1 < 0 && s2 < 0 && s3 < 0))
		{
			return ACROSS;
		}
		else if ((s1 == 0 && s2*s3 > 0) || (s2 == 0 && s1*s3 > 0) || (s3 == 0 && s1*s2 > 0))
		{
			return AEDGE;
		}
		else if ((s1 == 0 && s2 == 0) || (s1 == 0 && s3 == 0) || (s3 == 0 && s2 == 0))
		{
			return AVERTEX;
		}
		else
		{
			return NONINTERSECT;
		}
	}
	inline int LineFaceIntersect(Point3f &l1, Point3f &l2, Point3f &a, Point3f &b, Point3f &c)
	{
		float e1[6] = { 0 }, e2[6] = { 0 }, e3[6] = { 0 }, L[6] = { 0 };
		plucker(b, a, e1);
		plucker(c, b, e2);
		plucker(a, c, e3);
		plucker(l1, l2, L);

		float s1 = sideOp(L, e1);
		float s2 = sideOp(L, e2);
		float s3 = sideOp(L, e3);

		//cout << s1<<" --- " << s2 << " --- " << s3 << endl;

		if (s1 == 0 && s2 == 0 && s3 == 0)
		{
			return COPLANE;
		}
		else if ((s1 > 0 && s2 > 0 && s3 > 0) || (s1 < 0 && s2 < 0 && s3 < 0))
		{
			return ACROSS;
		}
		else if ((s1 == 0 && s2*s3 > 0) || (s2 == 0 && s1*s3 > 0) || (s3 == 0 && s1*s2 > 0))
		{
			return AEDGE;
		}
		else if ((s1 == 0 && s2 == 0) || (s1 == 0 && s3 == 0) || (s3 == 0 && s2 == 0))
		{
			return AVERTEX;
		}
		else
		{
			return NONINTERSECT;
		}
	}

	//���ཻ
	inline bool FaceIntersect(Point3d &a1, Point3d &b1, Point3d &c1, Point3d &a2, Point3d &b2, Point3d &c2)
	{

		if (LineFaceIntersect(a1, b1, a2, b2, c2) == ACROSS)
		{
			double dis1 = Point2PlaneDistance(a1, a2, b2, c2);
			double dis2 = Point2PlaneDistance(b1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}

		}
		if (LineFaceIntersect(a1, c1, a2, b2, c2) == ACROSS)
		{
			double dis1 = Point2PlaneDistance(a1, a2, b2, c2);
			double dis2 = Point2PlaneDistance(c1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		if (LineFaceIntersect(b1, c1, a2, b2, c2) == ACROSS)
		{
			double dis1 = Point2PlaneDistance(b1, a2, b2, c2);
			double dis2 = Point2PlaneDistance(c1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		return false;
	}
	inline bool FaceIntersect(Point3f &a1, Point3f &b1, Point3f &c1, Point3f &a2, Point3f &b2, Point3f &c2)
	{

		if (LineFaceIntersect(a1, b1, a2, b2, c2) == ACROSS)
		{
			float dis1 = Point2PlaneDistance(a1, a2, b2, c2);
			float dis2 = Point2PlaneDistance(b1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}

		}
		if (LineFaceIntersect(a1, c1, a2, b2, c2) == ACROSS)
		{
			float dis1 = Point2PlaneDistance(a1, a2, b2, c2);
			float dis2 = Point2PlaneDistance(c1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		if (LineFaceIntersect(b1, c1, a2, b2, c2) == ACROSS)
		{
			float dis1 = Point2PlaneDistance(b1, a2, b2, c2);
			float dis2 = Point2PlaneDistance(c1, a2, b2, c2);
			if (dis1*dis2 < 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		return false;
	}

	//***************************************//
	//			    ����ģ������              //
	//***************************************//

	//model base class
	class MeshModel
	{
	public:
		bool MeshModel::HaveData()
		{
			return (P.size() > 0);
		}

		void MeshModel::Clear()
		{
			P.clear();
			F.clear();
			C.clear();
			N.clear();
			L.clear();
			Cur.clear();
			Selected.clear();


			Point3d original_center = orth::Point3d(0, 0, 0);
			//double* original_rt;

			Point3d current_center = orth::Point3d(0, 0, 0);
			//double* current_rt;

			motion_path.clear();
			rot_path.clear();

			Box box2;
			box = box2;


		}

		void MeshModel::resize(int s)
		{

			size_ = s;
		}

		void MeshModel::Rotation(double *rt_matrix)
		{
			//旋转顶点
			if (P.size())
			{

				for (int point_index = 0; point_index < P.size(); point_index++)
				{
					PointRot(rt_matrix, &P[point_index]);
				}
			}

			//旋转法向
			if (N.size())
			{

				for (int normal_index = 0; normal_index < N.size(); normal_index++)
				{
					NormalRot(rt_matrix, &N[normal_index]);
				}
			}

			////旋转包围盒
			//if (box.u_0.x)
			//{

			//	for (int normal_index = 0; normal_index < N.size(); normal_index++)
			//	{
			//		PointRot(rt_matrix, &N[normal_index]);
			//	}
			//}

		}

		bool MeshModel::NormalUpdate()
		{
			N.resize(P.size());
			FN.resize(F.size());
			for (unsigned int i = 0; i < F.size(); i++)
			{
				FN[i] = TriangleNormal(P[F[i].x], P[F[i].y], P[F[i].z])*-1;
			}
			vector<int> point_face_number(P.size());
			for (unsigned int i = 0; i < F.size(); i++)
			{
				N[F[i].x] += FN[i]; point_face_number[F[i].x]++;
				N[F[i].y] += FN[i]; point_face_number[F[i].y]++;
				N[F[i].z] += FN[i]; point_face_number[F[i].z]++;
			}
			for (unsigned int i = 0; i < P.size(); i++)
			{
				N[i] /= (float)point_face_number[i];

				double e = sqrt(N[i].x*N[i].x + N[i].y*N[i].y + N[i].z*N[i].z);

				N[i] /= e;
			}

			return true;
		}

		bool MeshModel::EdgeUpdate(const bool PSTypeChoes)
		{
			if (P.size() == 0)
			{
				return false;
			}

			if (PSTypeChoes)
			{
				P2Edge.clear();
				Edge_P.clear();

				P2Edge.resize(P.size());
				Edge_P.resize(this->F.size() * 3);
				//HalfEdge_Parallel edge_current;

				for (size_t face_index = 0; face_index < F.size(); face_index++)
				{
					vector<Index_ui> point_index(4);
					point_index[0] = F[face_index].x;
					point_index[1] = F[face_index].y;
					point_index[2] = F[face_index].z;
					point_index[3] = F[face_index].x;

					for (size_t point_ = 0; point_ < 3; point_++)
					{
						Edge_P[face_index * 3 + point_].CurrentPoint = point_index[point_];
						Edge_P[face_index * 3 + point_].EndPoint = point_index[point_ + 1];
						Edge_P[face_index * 3 + point_].CurrentFace = face_index;
						if (point_ == 2)
						{
							Edge_P[face_index * 3 + point_].NextEdge = face_index * 3;

						}
						else
						{
							Edge_P[face_index * 3 + point_].NextEdge = face_index * 3 + point_ + 1;

						}

						P2Edge[point_index[point_]].push_back(face_index * 3 + point_);
					}
				}

				for (size_t edge_index = 0; edge_index < Edge_P.size(); edge_index++)
				{
					Edge_P[edge_index].OppoEdge = -1;
					Index_ui current_point_index = Edge_P[edge_index].CurrentPoint;
					Index_ui End_point_index = Edge_P[edge_index].EndPoint;
					for (size_t p2edge_index = 0; p2edge_index < P2Edge[End_point_index].size(); p2edge_index++)
					{

						if (Edge_P[P2Edge[End_point_index][p2edge_index]].EndPoint == current_point_index)
						{
							Edge_P[edge_index].OppoEdge = P2Edge[End_point_index][p2edge_index];
							break;
						}
					}

				}

			}
			else
			{
				//P2Edge.clear();
				//Edge_S.reset(new HalfEdge_Serial);
				//P2Edge.resize(P.size());

				////HalfEdge_Parallel edge_current;

				//for (size_t face_index = 0; face_index < F.size(); face_index++)
				//{
				//	vector<Index_ui> point_index(4);
				//	point_index[0] = F[face_index].x;
				//	point_index[1] = F[face_index].y;
				//	point_index[2] = F[face_index].z;
				//	point_index[3] = F[face_index].x;

				//	for (size_t point_ = 0; point_ < 3; point_++)
				//	{
				//		*Edge_S[face_index * 3 + point_].CurrentPoint = P[point_index[point_]];
				//		*Edge_S[face_index * 3 + point_].EndPoint = P[point_index[point_ + 1]];
				//		*Edge_S[face_index * 3 + point_].CurrentFace = F[face_index];
				//		if (point_ == 2)
				//		{
				//			Edge_S[face_index * 3 + point_].NextEdge = face_index * 3;

				//		}
				//		else
				//		{
				//			Edge_S[face_index * 3 + point_].NextEdge = face_index * 3 + point_ + 1;

				//		}

				//		P2Edge[point_index[point_]].push_back(face_index * 3 + point_);
				//	}
				//}

				//for (size_t edge_index = 0; edge_index < Edge_S.size(); edge_index++)
				//{
				//	Edge_S[edge_index].OppoEdge = -1;
				//	Index_ui current_point_index = Edge_S[edge_index].CurrentPoint;
				//	Index_ui End_point_index = Edge_S[edge_index].EndPoint;
				//	for (size_t p2edge_index = 0; p2edge_index < P2Edge[End_point_index].size(); p2edge_index++)
				//	{

				//		if (Edge_S[P2Edge[End_point_index][p2edge_index]].EndPoint == current_point_index)
				//		{
				//			Edge_S[edge_index].OppoEdge = P2Edge[End_point_index][p2edge_index];
				//			break;
				//		}
				//	}

				//}
			}


		}

		bool MeshModel::ModelSplit(vector<orth::MeshModel> &models, const int small_mesh_filter)
		{
			if (P2Edge.size() == 0)
			{
				EdgeUpdate(1);
			}

			if (FN.size() == 0)
			{
				this->NormalUpdate();
			}

			L.clear();
			L.resize(P.size(), -1);

			vector<Index_ui> label_size;
			Index_ui label_index = 0;
			Index_ui candidate_scan_index = 0;
			vector<Index_ui> candidate_points_index;

			//从第一个面开始扫描，逐个对点进行标记
			for (size_t face_index = 0; face_index < F.size(); face_index++)
			{
				if (L[F[face_index].x] == -1)
				{
					candidate_points_index.push_back(F[face_index].x); L[F[face_index].x] = label_index; //std::cout << candidate_points_index[candidate_points_index.size()-1] << std::endl;
				}
				else
				{
					continue;
				}


				while (true)
				{
					if (candidate_points_index.size() == candidate_scan_index)
					{
						candidate_points_index.clear();
						label_size.push_back(candidate_scan_index);
						candidate_scan_index = 0;
						label_index++;
						break;
					}

					L[candidate_points_index[candidate_scan_index]] = label_index;

					//逐个加入候选点
					for (size_t p2e_index = 0; p2e_index < P2Edge[candidate_points_index[candidate_scan_index]].size(); p2e_index++)
					{
						if (L[Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] == -1)
						{
							candidate_points_index.push_back(Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint);
							L[Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] = label_index; //std::cout << candidate_points_index[candidate_points_index.size() - 1] << std::endl;
						}
					}

					//std::cout <<"									"<< candidate_scan_index << std::endl;
					candidate_scan_index++;

				}

			}

			int final_model_number = 0;
			vector<int> filter_flag(label_index);
			for (int filter_index = 0; filter_index < label_index; filter_index++)
			{

				if (label_size[filter_index] < small_mesh_filter)
				{
					filter_flag[filter_index] = -1;
				}
				else
				{
					filter_flag[filter_index] = final_model_number;
					final_model_number++;
				}

			}

			//将标记点按照索引分解成单独模型
			models.resize(final_model_number);
			vector<Index_ui> new_point_index(P.size());
			if (C.size() > 0)
			{
				for (size_t point_index = 0; point_index < P.size(); point_index++)
				{
					if (L[point_index] < 0 || L[point_index] >= label_index || filter_flag[L[point_index]] == -1)
					{
						continue;
					}

					models[filter_flag[L[point_index]]].P.push_back(P[point_index]);
					models[filter_flag[L[point_index]]].N.push_back(N[point_index]);
					models[filter_flag[L[point_index]]].C.push_back(C[point_index]);

					new_point_index[point_index] = (models[filter_flag[L[point_index]]].P.size() - 1);
				}
			}
			else
			{
				for (size_t point_index = 0; point_index < P.size(); point_index++)
				{
					if (L[point_index] < 0 || L[point_index] >= label_index || filter_flag[L[point_index]] == -1)
					{
						continue;
					}

					models[filter_flag[L[point_index]]].P.push_back(P[point_index]);
					models[filter_flag[L[point_index]]].N.push_back(N[point_index]);
					//models[filter_flag[L[point_index]]].C.push_back(C[point_index]);

					new_point_index[point_index] = (models[filter_flag[L[point_index]]].P.size() - 1);
				}
			}

			for (size_t face_index = 0; face_index < F.size(); face_index++)
			{

				Index_ui l_point1 = F[face_index].x;
				Index_ui l_point2 = F[face_index].y;
				Index_ui l_point3 = F[face_index].z;
				Label l_label = L[l_point1];

				if (L[l_point1] < 0 || L[l_point1] >= label_index || filter_flag[l_label] == -1)
				{
					continue;
				}

				orth::Face l_face(new_point_index[l_point1], new_point_index[l_point2], new_point_index[l_point3]);
				models[filter_flag[l_label]].F.push_back(l_face);
				//models[l_label].FN.push_back(FN[face_index]);
			}


			P2Edge.clear();
			Edge_P.clear();
			L.clear();
			return true;

		}

		bool MeshModel::SmallModelFilter(const int point_number_threshold)
		{
			if (P2Edge.size() == 0)
			{
				EdgeUpdate(1);
			}

			if (N.size() == 0)
			{
				this->NormalUpdate();
			}

			L.clear();
			L.resize(P.size(), -1);

			vector<Index_ui> label_size;
			Index_ui label_index = 0;
			Index_ui candidate_scan_index = 0;
			vector<Index_ui> candidate_points_index;

			//从第一个面开始扫描，逐个对点进行标记
			for (size_t face_index = 0; face_index < F.size(); face_index++)
			{
				if (L[F[face_index].x] == -1)
				{
					candidate_points_index.push_back(F[face_index].x); L[F[face_index].x] = label_index; //std::cout << candidate_points_index[candidate_points_index.size()-1] << std::endl;
				}
				else
				{
					continue;
				}


				while (true)
				{
					if (candidate_points_index.size() == candidate_scan_index)
					{
						candidate_points_index.clear();
						label_size.push_back(candidate_scan_index);
						candidate_scan_index = 0;
						label_index++;
						break;
					}

					L[candidate_points_index[candidate_scan_index]] = label_index;

					//逐个加入候选点
					for (size_t p2e_index = 0; p2e_index < P2Edge[candidate_points_index[candidate_scan_index]].size(); p2e_index++)
					{
						if (L[Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] == -1)
						{
							candidate_points_index.push_back(Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint);
							L[Edge_P[P2Edge[candidate_points_index[candidate_scan_index]][p2e_index]].EndPoint] = label_index; //std::cout << candidate_points_index[candidate_points_index.size() - 1] << std::endl;
						}
					}

					//std::cout <<"									"<< candidate_scan_index << std::endl;
					candidate_scan_index++;

				}

			}

			//搜索小于100个点的小mesh并记录其label
			label_size.push_back(candidate_scan_index);
			vector<int> index_filted;
			for (int label_index_ = 0; label_index_ < label_size.size(); label_index_++)
			{
				if (label_size[label_index_] < point_number_threshold)
				{
					index_filted.push_back(label_index_);
				}
			}

			//重新标记需要过滤掉的mesh
			PointLabel label_filter(P.size(), 1);
			for (int point_index = 0; point_index < P.size(); point_index++)
			{
				for (int label_index_ = 0; label_index_ < index_filted.size(); label_index_++)
				{

					if (index_filted[label_index_] == L[point_index])
					{
						label_filter[point_index] = -1;
						break;
					}

				}
			}

			//按照标记对模型进行过滤
			PointCloudF new_points;
			PointNormal new_normals;
			PointColor new_colors;
			vector<Index_ui> new_point_index(P.size());
			for (size_t point_index = 0; point_index < P.size(); point_index++)
			{
				if (label_filter[point_index] < 0)
				{
					continue;
				}

				new_points.push_back(P[point_index]);
				new_normals.push_back(N[point_index]);
				if (C.size() > 0)
				{
					new_colors.push_back(C[point_index]);
				}
				new_point_index[point_index] = (new_points.size() - 1);
			}
			Faces new_faces;
			FacesNormal new_face_normal;
			for (size_t face_index = 0; face_index < F.size(); face_index++)
			{

				Index_ui l_point1 = F[face_index].x;
				Index_ui l_point2 = F[face_index].y;
				Index_ui l_point3 = F[face_index].z;
				if (label_filter[l_point1] < 0)
				{
					continue;
				}
				orth::Face l_face(new_point_index[l_point1], new_point_index[l_point2], new_point_index[l_point3]);
				new_faces.push_back(l_face);
				new_face_normal.push_back(FN[face_index]);
			}

			//新数据替换
			F.swap(new_faces);
			P.swap(new_points);
			N.swap(new_normals);
			if (C.size() > 0)
			{
				C.swap(new_colors);
			}
			FN.swap(new_face_normal);
			L.clear();

			P2Edge.clear();
			Edge_P.clear();

			return true;
		}

		void MeshModel::NormalSmooth(const int iteration_times)
		{
			if (P2Edge.size() == 0)
			{
				EdgeUpdate(1);
			}

			for (int iter_times = 0; iter_times < iteration_times; iter_times++)
			{
				orth::PointNormal new_normal;
				for (int point_index = 0; point_index < P.size(); point_index++)
				{
					orth::Normal sum_normal(0, 0, 0);
					orth::Normal L_normal = N[point_index];
					for (int neighbour_index = 0; neighbour_index < P2Edge[point_index].size(); neighbour_index++)
					{
						orth::Normal C_normal = N[Edge_P[P2Edge[point_index][neighbour_index]].EndPoint];
						if (L_normal.dot(C_normal) > 0)
							sum_normal += C_normal;
						else
							sum_normal -= C_normal;
						//L_normal += N[Edge_P[P2Edge[point_index][neighbour_index]].EndPoint];
					}
					L_normal.normalize();
					new_normal.push_back(L_normal);
				}
				N.swap(new_normal);
				new_normal.clear();
			}

			P2Edge.clear();
			Edge_P.clear();
		}

		void MeshModel::ModelSample(const int rate)
		{
			S.clear();
			int jump_number = (int)(P.size() / rate);
			if (rate > P.size())
			{
				jump_number = 1;
			}

			for (int point_index = 0; point_index < P.size(); point_index += jump_number)
			{
				S.push_back(point_index);
			}
		}

		int MeshModel::RemoveDuplicateVertex(int Qctree_depth)
			//inline bool NearestPointSearch(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const int Qctree_depth, vector<unsigned int> &query_index, vector<double> &nearest_distance)
		{
			if (P.size() == 0)
			{
				return false;
			}



			//求外接矩形
			float x_min = 1000, x_max = -1000, y_min = 1000, y_max = -1000, z_min = 1000, z_max = -1000;
			for (size_t point_index = 0; point_index < P.size(); point_index++)
			{
				float x = P[point_index].x;
				float y = P[point_index].y;
				float z = P[point_index].z;
				if (x < x_min)
				{
					x_min = x;
				}
				if (x > x_max)
				{
					x_max = x;
				}
				if (y < y_min)
				{
					y_min = y;
				}
				if (y > y_max)
				{
					y_max = y;
				}
				if (z < z_min)
				{
					z_min = z;
				}
				if (z > z_max)
				{
					z_max = z;
				}
			}

			//cout << " x_min = " << x_min << " y_min = " << y_min << " z_min = " << z_min << " x_max = " << x_max << " y_max = " << y_max << " z_max = " << z_max << endl;

			x_min -= 5; y_min -= 5; z_min -= 5;
			x_max += 5; y_max += 5; z_max += 5;

			float size = (pow(2, Qctree_depth));

			//求目标点云的key
			vector<unsigned __int32> target_key(P.size());

			for (size_t points_index = 0; points_index < P.size(); points_index++)
			{
				unsigned __int32 key = 0;

				float cell_size_x = (x_max - x_min) / (size);
				float cell_size_y = (y_max - y_min) / (size);
				float cell_size_z = (z_max - z_min) / (size);

				float x = P[points_index].x - x_min;
				float y = P[points_index].y - y_min;
				float z = P[points_index].z - z_min;

				unsigned __int32 temp_x = floor(x / cell_size_x);
				unsigned __int32 temp_y = floor(y / cell_size_y);
				unsigned __int32 temp_z = floor(z / cell_size_z);

				key = temp_x * (int)size*(int)size + temp_y * (int)size + temp_z;

				target_key[points_index] = key;
			}

			//按照node来统计点
			vector<vector<unsigned __int32>> target_key_sort(size*size*size);
			for (size_t points_index = 0; points_index < P.size(); points_index++)
			{
				target_key_sort[target_key[points_index]].push_back(points_index);
			}

			vector<int> query_index(P.size(), -1);


			orth::PointCloudF final_pointcloud;
			//求冗余点并更新索引
			for (size_t points_index = 0; points_index < query_index.size(); points_index++)
			{
				if (query_index[points_index] != -1)
				{
					continue;
				}

				query_index[points_index] = final_pointcloud.size();
				final_pointcloud.push_back(P[points_index]);
				orth::Point3f p1 = P[points_index];

				if (target_key_sort[target_key[points_index]].size() > 0)
				{

					for (size_t i = 0; i < target_key_sort[target_key[points_index]].size(); i++)
					{
						orth::Point3f p2 = P[target_key_sort[target_key[points_index]][i]];

						float dis = orth::Point2PointDistance(p1, p2);
						if (dis < 0.00001)
						{
							query_index[target_key_sort[target_key[points_index]][i]] = query_index[points_index];
						}
					}
				}
			}

			for (int face_index = 0; face_index < F.size(); face_index++)
			{
				F[face_index].x = query_index[F[face_index].x];
				F[face_index].y = query_index[F[face_index].y];
				F[face_index].z = query_index[F[face_index].z];
			}

			P.swap(final_pointcloud);



			return true;
		}

		//void MeshModel::DateDownload(Eigen::MatrixXd &Verts, Eigen::MatrixXi &Faces)
		//{
		//	if (P.size()==0)
		//	{
		//		std::cout << " Wrong PointCloud Size !!" << std::endl;
		//		getchar();
		//		return;
		//	}
		//	if (F.size() == 0)
		//	{
		//		std::cout << " Wrong Faces Size !!" << std::endl;
		//		getchar();
		//		return;
		//	}
		//	Verts = Eigen::MatrixXd::Zero(P.size(), 3);
		//	for (size_t point_index = 0; point_index < P.size(); point_index++)
		//	{
		//		Verts(point_index, 0) = P[point_index].x;
		//		Verts(point_index, 1) = P[point_index].y;
		//		Verts(point_index, 2) = P[point_index].z;
		//	}
		//	Faces = Eigen::MatrixXi::Zero(F.size(), 3);
		//	for (size_t face_index = 0; face_index < F.size(); face_index++)
		//	{
		//		Faces(face_index, 0) = F[face_index].x;
		//		Faces(face_index, 1) = F[face_index].y;
		//		Faces(face_index, 2) = F[face_index].z;
		//	}
		//	std::cout << " DownLoad Done !!" << std::endl;
		//}


		void MeshModel::NormalRot(double *rt_matrix, Normal *normal)
		{
			float x = normal->x, y = normal->y, z = normal->z;
			float v_c_x = rt_matrix[0] * x + rt_matrix[1] * y + rt_matrix[2] * z;
			float v_c_y = rt_matrix[4] * x + rt_matrix[5] * y + rt_matrix[6] * z;
			float v_c_z = rt_matrix[8] * x + rt_matrix[9] * y + rt_matrix[10] * z;
			if (v_c_x == NULL || v_c_y == NULL || v_c_z == NULL)
			{
				*normal = Point3f(0, 0, 0);
				return;
			}
			normal->x = v_c_x;
			normal->y = v_c_y;
			normal->z = v_c_z;

		}

		void MeshModel::PointRot(double *rt_matrix, Point3d *point)
		{
			double x = point->x, y = point->y, z = point->z;
			double v_c_x = rt_matrix[0] * x + rt_matrix[1] * y + rt_matrix[2] * z + rt_matrix[3];
			double v_c_y = rt_matrix[4] * x + rt_matrix[5] * y + rt_matrix[6] * z + rt_matrix[7];
			double v_c_z = rt_matrix[8] * x + rt_matrix[9] * y + rt_matrix[10] * z + rt_matrix[11];
			if (v_c_x == NULL || v_c_y == NULL || v_c_z == NULL)
			{
				*point = Point3d(0, 0, 0);
				return;
			}
			point->x = v_c_x;
			point->y = v_c_y;
			point->z = v_c_z;

		}

		void MeshModel::PointRot(double *rt_matrix, Point3f *point)
		{
			float x = point->x, y = point->y, z = point->z;
			float v_c_x = rt_matrix[0] * x + rt_matrix[1] * y + rt_matrix[2] * z + rt_matrix[3];
			float v_c_y = rt_matrix[4] * x + rt_matrix[5] * y + rt_matrix[6] * z + rt_matrix[7];
			float v_c_z = rt_matrix[8] * x + rt_matrix[9] * y + rt_matrix[10] * z + rt_matrix[11];
			if (v_c_x == NULL || v_c_y == NULL || v_c_z == NULL)
			{
				*point = Point3f(0, 0, 0);
				return;
			}
			point->x = v_c_x;
			point->y = v_c_y;
			point->z = v_c_z;
		}



		inline int size() { return size_; }

		PointCloudF P;
		PointNormal N;
		PointColor C;
		Faces F;
		FacesNormal FN;
		PointLabel L;

		FaceMask FM;
		PointMask PM;
		EdgeMask EM;


		SamplePoints S;
		PointCurs Cur;
		PointLabel Selected;

		HalfEdgeCloud_P Edge_P;
		HalfEdgeCloud_S Edge_S;
		HalfPointCloud_P P2Edge;

		Point3d original_center;
		double* original_rt;

		Point3d current_center;
		double* current_rt;

		vector<Point3f> motion_path;
		vector<double*> rot_path;

		Box box;





	private:
		int size_ = 0;


	};

	//***************************************//
	//			    ģ�ͻ�������              //
	//***************************************//

	//���ڽ����ѯ
	inline bool NearestPointSearch(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const int Qctree_depth, vector<unsigned int> &query_index, vector<double> &nearest_distance)
	{
		if (mm_query->P.size() == 0)
		{
			return false;
		}
		query_index.resize(mm_query->P.size());
		nearest_distance.resize(mm_query->P.size());

		//����Ӿ���
		double x_min = 1000, x_max = -1000, y_min = 1000, y_max = -1000, z_min = 1000, z_max = -1000;
		for (size_t point_index = 0; point_index < mm_target->P.size(); point_index++)
		{
			double x = mm_target->P[point_index].x;
			double y = mm_target->P[point_index].y;
			double z = mm_target->P[point_index].z;
			if (x < x_min)
			{
				x_min = x;
			}
			if (x > x_max)
			{
				x_max = x;
			}
			if (y < y_min)
			{
				y_min = y;
			}
			if (y > y_max)
			{
				y_max = y;
			}
			if (z < z_min)
			{
				z_min = z;
			}
			if (z > z_max)
			{
				z_max = z;
			}
		}

		//cout << " x_min = " << x_min << " y_min = " << y_min << " z_min = " << z_min << " x_max = " << x_max << " y_max = " << y_max << " z_max = " << z_max << endl;

		x_min -= 5; y_min -= 5; z_min -= 5;
		x_max += 5; y_max += 5; z_max += 5;

		double size = (pow(2, Qctree_depth));

		//��Ŀ����Ƶ�key
		vector<unsigned __int32> target_key(mm_target->P.size());

		for (size_t points_index = 0; points_index < mm_target->P.size(); points_index++)
		{
			unsigned __int32 key = 0;

			//for (size_t tree_depth = 1; tree_depth <= Qctree_depth; ++tree_depth)
			//{
			//	double size = (pow(2, tree_depth - 1));
			//	double cell_size_x = (x_max - x_min) / (size);
			//	double cell_size_y = (y_max - y_min) / (size);
			//	double cell_size_z = (z_max - z_min) / (size);

			//	double x = mm_target->P[points_index].x - x_min;
			//	double y = mm_target->P[points_index].y - y_min;
			//	double z = mm_target->P[points_index].z - z_min;

			//double temp_x = x/cell_size_x;
			//double temp_y = y/cell_size_y;
			//double temp_z = z/cell_size_z;

			//double temp2_x = temp_x - floor(temp_x);
			//double temp2_y = temp_y - floor(temp_y);
			//double temp2_z = temp_z - floor(temp_z);

			//if (temp2_x>=0.5)
			//{
			//	key += 4;
			//}
			//if (temp2_y >= 0.5)
			//{
			//	key += 2;
			//}
			//if (temp2_z >= 0.5)
			//{
			//	key += 1;
			//}

			//cout << std::bitset<32>(key)  << endl;

			//if (tree_depth!= Qctree_depth)
			//{
			//	key = key << 3;
			//}

			//}


			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm_target->P[points_index].x - x_min;
			double y = mm_target->P[points_index].y - y_min;
			double z = mm_target->P[points_index].z - z_min;

			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			key = temp_x * (int)size*(int)size + temp_y * (int)size + temp_z;

			target_key[points_index] = key;
		}

		//����node��ͳ�Ƶ�
		vector<vector<unsigned __int32>> target_key_sort(size*size*size);
		for (size_t points_index = 0; points_index < mm_target->P.size(); points_index++)
		{
			target_key_sort[target_key[points_index]].push_back(points_index);
		}

		//���ѯ���Ƶ�key
		vector<unsigned int> Qctree_key(mm_query->P.size());

		for (size_t points_index = 0; points_index < mm_query->P.size(); points_index++)
		{
			unsigned __int32 key = 0;

			//for (size_t tree_depth = 1; tree_depth <= Qctree_depth; ++tree_depth)
			//{
			//	double size = (pow(2, tree_depth - 1));
			//	double cell_size_x = (x_max - x_min) / (size);
			//	double cell_size_y = (y_max - y_min) / (size);
			//	double cell_size_z = (z_max - z_min) / (size);

			//	double x = mm_query->P[points_index].x - x_min;
			//	double y = mm_query->P[points_index].y - y_min;
			//	double z = mm_query->P[points_index].z - z_min;

			//	double temp_x = x / cell_size_x;
			//	double temp_y = y / cell_size_y;
			//	double temp_z = z / cell_size_z;

			//	double temp2_x = temp_x - floor(temp_x);
			//	double temp2_y = temp_y - floor(temp_y);
			//	double temp2_z = temp_z - floor(temp_z);

			//	if (temp2_x >= 0.5)
			//	{
			//		key += 4;
			//	}
			//	if (temp2_y >= 0.5)
			//	{
			//		key += 2;
			//	}
			//	if (temp2_z >= 0.5)
			//	{
			//		key += 1;
			//	}

			//	cout << std::bitset<32>(key) << endl;

			//	if (tree_depth != Qctree_depth)
			//	{
			//		key = key << 3;
			//	}

			//}

			if (mm_query->P[points_index].x<x_min || mm_query->P[points_index].x>x_max || mm_query->P[points_index].y<y_min || mm_query->P[points_index].y>y_max || mm_query->P[points_index].z<z_min || mm_query->P[points_index].z>z_max)
			{
				nearest_distance[points_index] = -1;
				continue;
			}

			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm_query->P[points_index].x - x_min;
			double y = mm_query->P[points_index].y - y_min;
			double z = mm_query->P[points_index].z - z_min;



			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			key = temp_x * (int)size*(int)size + temp_y * (int)size + temp_z;

			Qctree_key[points_index] = key;
		}

		//���ѯ���Ƶ������

		for (size_t points_index = 0; points_index < mm_query->P.size(); points_index++)
		{
			if (nearest_distance[points_index] == -1)
			{
				continue;
			}

			orth::Point3f query_point = mm_query->P[points_index];
			vector<unsigned __int32> searched_points;
			int break_label = 2;

			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm_query->P[points_index].x - x_min;
			double y = mm_query->P[points_index].y - y_min;
			double z = mm_query->P[points_index].z - z_min;

			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			for (size_t cycle_index = 0; cycle_index < Qctree_depth - 1; ++cycle_index)
			{
				if (searched_points.size() > 0)
				{
					break_label--;
				}
				if (!break_label)
				{
					break;
				}

				int x_start = temp_x - cycle_index > 0 ? temp_x - cycle_index : 0;
				int y_start = temp_y - cycle_index > 0 ? temp_y - cycle_index : 0;
				int z_start = temp_z - cycle_index > 0 ? temp_z - cycle_index : 0;
				int x_end = temp_x + cycle_index < size ? temp_x + cycle_index : size - 1;
				int y_end = temp_y + cycle_index < size ? temp_y + cycle_index : size - 1;
				int z_end = temp_z + cycle_index < size ? temp_z + cycle_index : size - 1;

				for (size_t x_index = x_start; x_index <= x_end; ++x_index)
				{
					for (size_t y_index = y_start; y_index <= y_end; ++y_index)
					{

						for (size_t z_index = z_start; z_index <= z_end; ++z_index)
						{
							if (target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index].size() > 0)
							{

								for (size_t point_index = 0; point_index < target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index].size(); point_index++)
								{
									searched_points.push_back(target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index][point_index]);
								}

							}
						}
					}
				}

			}

			nearest_distance[points_index] = 100;
			for (size_t search_index = 0; search_index < searched_points.size(); search_index++)
			{
				double dis = orth::Point2PointDistance(query_point, mm_target->P[searched_points[search_index]]);
				if (dis < nearest_distance[points_index])
				{
					query_index[points_index] = searched_points[search_index];
					nearest_distance[points_index] = dis;
				}
			}
		}

		return true;
	}

	//���ڽ����ѯ
	inline bool RadiusPointSearch(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const float radius, const int Qctree_depth, vector<vector<unsigned int>> &query_index, vector<vector<double>> &nearest_distance)
	{
		if (mm_query->P.size() == 0)
		{
			return false;
		}
		query_index.resize(mm_query->P.size());
		nearest_distance.resize(mm_query->P.size());
		vector<bool> wrong_flag(mm_query->P.size(), false);

		//����Ӿ���
		double x_min = 1000, x_max = -1000, y_min = 1000, y_max = -1000, z_min = 1000, z_max = -1000;
		for (size_t point_index = 0; point_index < mm_target->P.size(); point_index++)
		{
			double x = mm_target->P[point_index].x;
			double y = mm_target->P[point_index].y;
			double z = mm_target->P[point_index].z;
			if (x < x_min)
			{
				x_min = x;
			}
			if (x > x_max)
			{
				x_max = x;
			}
			if (y < y_min)
			{
				y_min = y;
			}
			if (y > y_max)
			{
				y_max = y;
			}
			if (z < z_min)
			{
				z_min = z;
			}
			if (z > z_max)
			{
				z_max = z;
			}
		}

		//cout << " x_min = " << x_min << " y_min = " << y_min << " z_min = " << z_min << " x_max = " << x_max << " y_max = " << y_max << " z_max = " << z_max << endl;

		x_min -= 5; y_min -= 5; z_min -= 5;
		x_max += 5; y_max += 5; z_max += 5;

		double size = (pow(2, Qctree_depth));

		//��Ŀ����Ƶ�key
		vector<unsigned __int32> target_key(mm_target->P.size());

		for (size_t points_index = 0; points_index < mm_target->P.size(); points_index++)
		{
			unsigned __int32 key = 0;

			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm_target->P[points_index].x - x_min;
			double y = mm_target->P[points_index].y - y_min;
			double z = mm_target->P[points_index].z - z_min;

			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			key = temp_x * (int)size*(int)size + temp_y * (int)size + temp_z;

			target_key[points_index] = key;
		}

		//����node��ͳ�Ƶ�
		vector<vector<unsigned __int32>> target_key_sort(size*size*size);
		for (size_t points_index = 0; points_index < mm_target->P.size(); points_index++)
		{
			target_key_sort[target_key[points_index]].push_back(points_index);
		}

		//���ѯ���Ƶ�key
		vector<unsigned int> Qctree_key(mm_query->P.size());

		for (size_t points_index = 0; points_index < mm_query->P.size(); points_index++)
		{
			unsigned __int32 key = 0;

			if (mm_query->P[points_index].x<x_min || mm_query->P[points_index].x>x_max || mm_query->P[points_index].y<y_min || mm_query->P[points_index].y>y_max || mm_query->P[points_index].z<z_min || mm_query->P[points_index].z>z_max)
			{
				//nearest_distance[points_index] = -1;
				wrong_flag[points_index] = true;
				continue;
			}

			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm_query->P[points_index].x - x_min;
			double y = mm_query->P[points_index].y - y_min;
			double z = mm_query->P[points_index].z - z_min;



			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			key = temp_x * (int)size*(int)size + temp_y * (int)size + temp_z;

			Qctree_key[points_index] = key;
		}

		//���ѯ���Ƶ������

		for (size_t points_index = 0; points_index < mm_query->P.size(); points_index++)
		{
			if (wrong_flag[points_index])
			{
				continue;
			}

			orth::Point3f query_point = mm_query->P[points_index];
			vector<unsigned __int32> searched_points;
			int break_label = 2;

			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double cycle_radius_x = ceil(radius / cell_size_x);
			double cycle_radius_y = ceil(radius / cell_size_y);
			double cycle_radius_z = ceil(radius / cell_size_z);

			double x = mm_query->P[points_index].x - x_min;
			double y = mm_query->P[points_index].y - y_min;
			double z = mm_query->P[points_index].z - z_min;

			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			//for (size_t cycle_index = 0; cycle_index < Qctree_depth - 1; ++cycle_index)
			{
				if (searched_points.size() > 0)
				{
					break_label--;
				}
				if (!break_label)
				{
					break;
				}

				int x_start = temp_x - cycle_radius_x > 0 ? temp_x - cycle_radius_x : 0;
				int y_start = temp_y - cycle_radius_y > 0 ? temp_y - cycle_radius_y : 0;
				int z_start = temp_z - cycle_radius_z > 0 ? temp_z - cycle_radius_z : 0;
				int x_end = temp_x + cycle_radius_x < size ? temp_x + cycle_radius_x : size - 1;
				int y_end = temp_y + cycle_radius_y < size ? temp_y + cycle_radius_y : size - 1;
				int z_end = temp_z + cycle_radius_z < size ? temp_z + cycle_radius_z : size - 1;

				for (size_t x_index = x_start; x_index <= x_end; ++x_index)
				{
					for (size_t y_index = y_start; y_index <= y_end; ++y_index)
					{

						for (size_t z_index = z_start; z_index <= z_end; ++z_index)
						{
							if (target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index].size() > 0)
							{

								for (size_t point_index = 0; point_index < target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index].size(); point_index++)
								{
									int target_point_index = target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index][point_index];
									double dis = Point2PointDistance(query_point, mm_target->P[target_point_index]);
									if (dis < radius)
									{
										query_index[points_index].push_back(target_point_index);
										nearest_distance[points_index].push_back(dis);
									}
									//searched_points.push_back(target_key_sort[x_index * (int)size*(int)size + y_index * (int)size + z_index][point_index]);

								}

							}
						}
					}
				}

			}

			//nearest_distance[points_index] = 100;
			//for (size_t search_index = 0; search_index < searched_points.size(); search_index++)
			//{
			//	double dis = orth::Point2PointDistance(query_point, mm_target->P[searched_points[search_index]]);
			//	if (dis < nearest_distance[points_index])
			//	{
			//		query_index[points_index] = searched_points[search_index];
			//		nearest_distance[points_index] = dis;
			//	}
			//}
		}

		return true;
	}


	//mesh冗余点约减
	//mm:模型输入	distance_threashold:判断冗余采用的距离阈值	Qctree_depth:八叉树深度
	inline bool DuplicateVertexRemove(orth::MeshModel *mm, const float distance_threashold, int Qctree_depth)
		//inline bool NearestPointSearch(orth::MeshModel *mm_target, orth::MeshModel *mm_query, const int Qctree_depth, vector<unsigned int> &query_index, vector<double> &nearest_distance)
	{
		if (mm->P.size() == 0)
		{
			return false;
		}



		//求外接矩形
		double x_min = 1000, x_max = -1000, y_min = 1000, y_max = -1000, z_min = 1000, z_max = -1000;
		for (size_t point_index = 0; point_index < mm->P.size(); point_index++)
		{
			double x = mm->P[point_index].x;
			double y = mm->P[point_index].y;
			double z = mm->P[point_index].z;
			if (x < x_min)
			{
				x_min = x;
			}
			if (x > x_max)
			{
				x_max = x;
			}
			if (y < y_min)
			{
				y_min = y;
			}
			if (y > y_max)
			{
				y_max = y;
			}
			if (z < z_min)
			{
				z_min = z;
			}
			if (z > z_max)
			{
				z_max = z;
			}
		}

		//cout << " x_min = " << x_min << " y_min = " << y_min << " z_min = " << z_min << " x_max = " << x_max << " y_max = " << y_max << " z_max = " << z_max << endl;

		x_min -= 5; y_min -= 5; z_min -= 5;
		x_max += 5; y_max += 5; z_max += 5;

		double size = (pow(2, Qctree_depth));

		//求目标点云的key
		vector<unsigned __int32> target_key(mm->P.size());

		for (size_t points_index = 0; points_index < mm->P.size(); points_index++)
		{
			unsigned __int32 key = 0;

			double cell_size_x = (x_max - x_min) / (size);
			double cell_size_y = (y_max - y_min) / (size);
			double cell_size_z = (z_max - z_min) / (size);

			double x = mm->P[points_index].x - x_min;
			double y = mm->P[points_index].y - y_min;
			double z = mm->P[points_index].z - z_min;

			unsigned __int32 temp_x = floor(x / cell_size_x);
			unsigned __int32 temp_y = floor(y / cell_size_y);
			unsigned __int32 temp_z = floor(z / cell_size_z);

			key = temp_x * (int)size*(int)size + temp_y * (int)size + temp_z;

			target_key[points_index] = key;
		}

		//按照node来统计点
		vector<vector<unsigned __int32>> target_key_sort(size*size*size);
		for (size_t points_index = 0; points_index < mm->P.size(); points_index++)
		{
			target_key_sort[target_key[points_index]].push_back(points_index);
		}

		vector<int> query_index(mm->P.size(), -1);
		orth::PointCloudF final_pointcloud;


		//求冗余点并更新索引
		for (size_t points_index = 0; points_index < query_index.size(); points_index++)
		{
			if (query_index[points_index] != -1)
			{
				continue;
			}

			query_index[points_index] = final_pointcloud.size();
			final_pointcloud.push_back(mm->P[points_index]);
			orth::Point3f p1 = mm->P[points_index];

			if (target_key_sort[target_key[points_index]].size() > 0)
			{

				for (size_t i = 0; i < target_key_sort[target_key[points_index]].size(); i++)
				{
					orth::Point3f p2 = mm->P[target_key_sort[target_key[points_index]][i]];

					double dis = orth::Point2PointDistance(p1, p2);
					if (dis < distance_threashold)
					{
						query_index[target_key_sort[target_key[points_index]][i]] = query_index[points_index];
					}
				}
			}
		}

		for (int face_index = 0; face_index < mm->F.size(); face_index++)
		{
			mm->F[face_index].x = query_index[mm->F[face_index].x];
			mm->F[face_index].y = query_index[mm->F[face_index].y];
			mm->F[face_index].z = query_index[mm->F[face_index].z];
		}

		mm->P.swap(final_pointcloud);

		return true;
	}

	//ģ�ͺϲ�
	inline bool MergeModels(vector<orth::MeshModel> &mm_group_input, orth::MeshModel &mm_output)
	{
		if (mm_group_input.size() <= 0)
		{
			return false;
		}

		mm_output.Clear();

		for (size_t group_index = 0; group_index < mm_group_input.size(); group_index++)
		{
			vector<Index_ui> new_point_index(mm_group_input[group_index].P.size());
			if (mm_group_input[group_index].C.size() > 0)
			{
				for (size_t point_index = 0; point_index < mm_group_input[group_index].P.size(); point_index++)
				{
					mm_output.P.push_back(mm_group_input[group_index].P[point_index]);
					mm_output.N.push_back(mm_group_input[group_index].N[point_index]);
					mm_output.C.push_back(mm_group_input[group_index].C[point_index]);
					//mm_output.L.push_back(mm_group_input[group_index].L[point_index]);
					//models[L[point_index]].Cur.push_back(Cur[point_index]);
					new_point_index[point_index] = (mm_output.P.size() - 1);
				}
			}
			else
			{
				for (size_t point_index = 0; point_index < mm_group_input[group_index].P.size(); point_index++)
				{
					mm_output.P.push_back(mm_group_input[group_index].P[point_index]);
					mm_output.N.push_back(mm_group_input[group_index].N[point_index]);
					mm_output.C.push_back(orth::Color(128, 128, 128));
					//mm_output.L.push_back(mm_group_input[group_index].L[point_index]);
					//models[L[point_index]].Cur.push_back(Cur[point_index]);
					new_point_index[point_index] = (mm_output.P.size() - 1);
				}
			}

			for (size_t face_index = 0; face_index < (mm_group_input[group_index].F.size()); face_index++)
			{

				Index_ui l_point1 = mm_group_input[group_index].F[face_index].x;
				Index_ui l_point2 = mm_group_input[group_index].F[face_index].y;
				Index_ui l_point3 = mm_group_input[group_index].F[face_index].z;
				orth::Face l_face(new_point_index[l_point1], new_point_index[l_point2], new_point_index[l_point3]);
				mm_output.F.push_back(l_face);
				//mm_output.FN.push_back(mm_group_input[group_index].FN[face_index]);
			}
		}



		return true;
	}

	// teeth model
	class Teeth :public MeshModel
	{
	public:


		void Teeth::Rotation(double *rt_matrix)
		{
			//旋转顶点
			if (P.size())
			{

				for (int point_index = 0; point_index < P.size(); point_index++)
				{
					PointRot(rt_matrix, &P[point_index]);
				}
			}

			//旋转法向
			if (N.size())
			{

				for (int normal_index = 0; normal_index < N.size(); normal_index++)
				{
					NormalRot(rt_matrix, &N[normal_index]);
				}
			}

			//旋转中心点


			//if (motion_path.size())
			//{
			//	PointRot(rt_matrix, &current_center);
			//	motion_path.push_back(current_center);
			//	for (size_t i = 0; i < 16; i++)
			//	{
			//		current_rt[i] = rt_matrix[i];
			//	}
			//}
			//else
			//{

			//}

			////旋转包围盒
			//if (box.u_0.x)
			//{

			//	for (int normal_index = 0; normal_index < N.size(); normal_index++)
			//	{
			//		PointRot(rt_matrix, &N[normal_index]);
			//	}
			//}

			//旋转尖点

			if (cusp.size() > 0)
			{
				for (int cusp_index = 0; cusp_index < cusp.size(); cusp_index++)
				{
					PointRot(rt_matrix, &cusp[cusp_index]);
				}
			}
			if (cusp_b.size() > 0)
			{
				for (int cusp_index = 0; cusp_index < cusp_b.size(); cusp_index++)
				{
					PointRot(rt_matrix, &cusp_b[cusp_index]);
				}
			}
			if (cusp_l.size() > 0)
			{
				for (int cusp_index = 0; cusp_index < cusp_b.size(); cusp_index++)
				{
					PointRot(rt_matrix, &cusp_l[cusp_index]);
				}
			}
			if (incisal_edges.size() > 0)
			{
				for (int cusp_index = 0; cusp_index < incisal_edges.size(); cusp_index++)
				{
					PointRot(rt_matrix, &incisal_edges[cusp_index]);
				}
			}

		}



		Teeth::Teeth()
		{
		}

		Teeth::~Teeth()
		{
		}

		// cusp of a teeth
		vector<Point3f> cusp;

		// b_cusp of a teeth
		vector<Point3f> cusp_b;

		// l_cusp of a teeth
		vector<Point3f> cusp_l;

		// ridge of a teeth
		vector<Point3f> ridge;

		// groove of a teeth
		vector<Point3f> groove;

		// groove of a teeth
		vector<Point3f> incisal_edges;

		//buccolingual inc
		double premolar_theta;

		//buccolingual inc
		double molar_theta1;
		double molar_theta2;


	private:

	};

	class MaxillaryTeeth :public MeshModel
	{
	public:

		MaxillaryTeeth::MaxillaryTeeth()
		{
		}

		MaxillaryTeeth::~MaxillaryTeeth()
		{
		}

		vector<Point3f> arch;

		Plane occlusion;

		vector<Teeth> teeths;

		vector<Plane> division_plane;
		//
		//	add feature of the teeth
		//
		//

	private:

	};

	class InferiorTeeth :public MeshModel
	{
	public:

		InferiorTeeth::InferiorTeeth()
		{
		}

		InferiorTeeth::~InferiorTeeth()
		{
		}

		vector<Point3f> arch;

		Plane occlusion;

		vector<Teeth> teeths;

		vector<Plane> division_plane;

		//
		//	add feature of the teeth
		//
		//

	private:

	};





}

#endif // !BASETYPE_H

