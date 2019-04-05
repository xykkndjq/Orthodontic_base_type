#include "basetype.h"


namespace orth
{


	MeshModel::MeshModel()
	{
		//original_rt = (double*)malloc(16*sizeof(double));
		//current_rt = (double*)malloc(16 * sizeof(double));;
	}

	MeshModel::~MeshModel()
	{
		//free(original_rt);
		//original_rt = NULL;
		//free(current_rt);
		//current_rt = NULL;
	}

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


	void MeshModel::NormalUpdate()
	{
		N.resize(P.size());
		FN.resize(F.size());
		for (unsigned int i = 0; i < F.size(); i++)
		{
			FN[i] = TriangleNormal(P[F[i].x], P[F[i].y], P[F[i].z]);
		}
		vector<int> point_face_number(P.size());
		for (unsigned int i = 0; i < F.size(); i++)
		{
			N[F[i].x] += FN[i]; point_face_number[F[i].x]++;
			N[F[i].y] += FN[i]; point_face_number[F[i].y]++;
			N[F[i].z] += FN[i]; point_face_number[F[i].z]++;
		}
		for (unsigned int i = 0; i <P.size(); i++)
		{
			N[i] /= (float)point_face_number[i];

			double e = sqrt(N[i].x*N[i].x + N[i].y*N[i].y + N[i].z*N[i].z);

			N[i] /= e;
		}


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

	void Teeth::Rotation(double *rt_matrix)
	{
		//��ת����
		if (P.size())
		{

			for (int point_index = 0; point_index < P.size(); point_index++)
			{
				PointRot(rt_matrix, &P[point_index]);
			}
		}

		//��ת����
		if (N.size())
		{

			for (int normal_index = 0; normal_index < N.size(); normal_index++)
			{
				PointRot(rt_matrix, &N[normal_index]);
			}
		}

		//��ת���ĵ�


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

		////��ת��Χ��
		//if (box.u_0.x)
		//{

		//	for (int normal_index = 0; normal_index < N.size(); normal_index++)
		//	{
		//		PointRot(rt_matrix, &N[normal_index]);
		//	}
		//}

		//��ת���

		if (cusp.size()>0)
		{
			for (int cusp_index = 0; cusp_index < cusp.size(); cusp_index++)
			{
				PointRot(rt_matrix, &cusp[cusp_index]);
			}
		}
		if (cusp_b.size()>0)
		{
			for (int cusp_index = 0; cusp_index < cusp_b.size(); cusp_index++)
			{
				PointRot(rt_matrix, &cusp_b[cusp_index]);
			}
		}
		if (cusp_l.size()>0)
		{
			for (int cusp_index = 0; cusp_index < cusp_b.size(); cusp_index++)
			{
				PointRot(rt_matrix, &cusp_l[cusp_index]);
			}
		}
		if (incisal_edges.size()>0)
		{
			for (int cusp_index = 0; cusp_index < incisal_edges.size(); cusp_index++)
			{
				PointRot(rt_matrix, &incisal_edges[cusp_index]);
			}
		}

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
		double x = point->x, y = point->y, z = point->z;
		double v_c_x = rt_matrix[0] * x + rt_matrix[1] * y + rt_matrix[2] * z + rt_matrix[3];
		double v_c_y = rt_matrix[4] * x + rt_matrix[5] * y + rt_matrix[6] * z + rt_matrix[7];
		double v_c_z = rt_matrix[8] * x + rt_matrix[9] * y + rt_matrix[10] * z + rt_matrix[11];
		point->x = v_c_x;
		point->y = v_c_y;
		point->z = v_c_z;
	}

	Teeth::Teeth()
	{
	}

	Teeth::~Teeth()
	{
	}

	MaxillaryTeeth::MaxillaryTeeth()
	{
	}

	MaxillaryTeeth::~MaxillaryTeeth()
	{
	}

	InferiorTeeth::InferiorTeeth()
	{
	}

	InferiorTeeth::~InferiorTeeth()
	{
	}

}

