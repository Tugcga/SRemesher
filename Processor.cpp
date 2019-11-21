#include "dedge.h"
#include "subdivide.h"
#include "meshstats.h"
#include "hierarchy.h"
#include "field.h"
#include "normal.h"
#include "extract.h"
#include "bvh.h"

#include "config.hpp"
#include "field-math.hpp"
#include "optimizer.hpp"
#include "parametrizer.hpp"

#include <xsi_polygonmesh.h>
#include <xsi_geometryaccessor.h>
#include <xsi_application.h>
#include <xsi_comapihandler.h>
#include <xsi_kinematicstate.h>

void fill_in_vertices(MatrixXf &V, XSI::PolygonMesh &mesh, XSI::KinematicState &tfm, float object_scale)
{
	XSI::CGeometryAccessor ga = mesh.GetGeometryAccessor2();
	XSI::CDoubleArray vertices;
	ga.GetVertexPositions(vertices);
	size_t in_vertex_count = ga.GetVertexCount();
	
	V.resize(3, in_vertex_count);
	
	XSI::MATH::CMatrix4 tfm_matrix = tfm.GetTransform().GetMatrix4();
	for (size_t i = 0; i < in_vertex_count; i++)
	{
		float pos_x = vertices[3 * i] * tfm_matrix.GetValue(0, 0) + vertices[3 * i + 1] * tfm_matrix.GetValue(1, 0) + vertices[3 * i + 2] * tfm_matrix.GetValue(2, 0) + tfm_matrix.GetValue(3, 0);
		float pos_y = vertices[3 * i] * tfm_matrix.GetValue(0, 1) + vertices[3 * i + 1] * tfm_matrix.GetValue(1, 1) + vertices[3 * i + 2] * tfm_matrix.GetValue(2, 1) + tfm_matrix.GetValue(3, 1);
		float pos_z = vertices[3 * i] * tfm_matrix.GetValue(0, 2) + vertices[3 * i + 1] * tfm_matrix.GetValue(1, 2) + vertices[3 * i + 2] * tfm_matrix.GetValue(2, 2) + tfm_matrix.GetValue(3, 2);
		V.col(i) = Vector3f(pos_x * object_scale, pos_y * object_scale, pos_z * object_scale);
	}
}

void fill_in_vertices(Eigen::MatrixXd &V, XSI::PolygonMesh &mesh, XSI::KinematicState &tfm, float object_scale)
{
	XSI::CGeometryAccessor ga = mesh.GetGeometryAccessor2();
	XSI::CDoubleArray vertices;
	ga.GetVertexPositions(vertices);
	size_t in_vertex_count = ga.GetVertexCount();

	V.resize(3, in_vertex_count);

	XSI::MATH::CMatrix4 tfm_matrix = tfm.GetTransform().GetMatrix4();
	for (size_t i = 0; i < in_vertex_count; i++)
	{
		double pos_x = vertices[3 * i] * tfm_matrix.GetValue(0, 0) + vertices[3 * i + 1] * tfm_matrix.GetValue(1, 0) + vertices[3 * i + 2] * tfm_matrix.GetValue(2, 0) + tfm_matrix.GetValue(3, 0);
		double pos_y = vertices[3 * i] * tfm_matrix.GetValue(0, 1) + vertices[3 * i + 1] * tfm_matrix.GetValue(1, 1) + vertices[3 * i + 2] * tfm_matrix.GetValue(2, 1) + tfm_matrix.GetValue(3, 1);
		double pos_z = vertices[3 * i] * tfm_matrix.GetValue(0, 2) + vertices[3 * i + 1] * tfm_matrix.GetValue(1, 2) + vertices[3 * i + 2] * tfm_matrix.GetValue(2, 2) + tfm_matrix.GetValue(3, 2);
		V.col(i) = Eigen::Vector3d(pos_x * object_scale, pos_y * object_scale, pos_z * object_scale);
	}
}

void fill_in_faces(MatrixXu &F, XSI::PolygonMesh &mesh)
{
	XSI::CGeometryAccessor ga = mesh.GetGeometryAccessor2();
	XSI::CLongArray triangles;
	ga.GetTriangleVertexIndices(triangles);
	size_t in_triangles_count = ga.GetTriangleCount();

	F.resize(3, in_triangles_count);
	for (size_t i = 0; i < in_triangles_count; i++)
	{
		F.col(i) = Vector3u(triangles[3 * i], triangles[3 * i + 1], triangles[3 * i + 2]);
	}
}

void fill_in_faces(MatrixXi &F, XSI::PolygonMesh &mesh)
{
	XSI::CGeometryAccessor ga = mesh.GetGeometryAccessor2();
	XSI::CLongArray triangles;
	ga.GetTriangleVertexIndices(triangles);
	size_t in_triangles_count = ga.GetTriangleCount();

	F.resize(3, in_triangles_count);
	for (size_t i = 0; i < in_triangles_count; i++)
	{
		F.col(i) = Vector3i(triangles[3 * i], triangles[3 * i + 1], triangles[3 * i + 2]);
	}
}

void fill_out(XSI::MATH::CVector3Array &out_vertices, XSI::CLongArray &out_faces, MatrixXf &O_extr, MatrixXu &F_extr, float object_scale)
{
	for (size_t i = 0; i < O_extr.cols(); i++)
	{
		out_vertices.Add(XSI::MATH::CVector3(O_extr(0, i) / object_scale, O_extr(1, i) / object_scale, O_extr(2, i) / object_scale));
	}
	for (size_t i = 0; i < F_extr.cols(); i++)
	{
		out_faces.Add(F_extr.rows());
		for (size_t r = 0; r < F_extr.rows(); r++)
		{
			out_faces.Add(F_extr(r, i));
		}
	}
}

void fill_out(XSI::MATH::CVector3Array &out_vertices, XSI::CLongArray &out_faces, std::vector<Eigen::Vector3d> &verts, std::vector<Vector4i> &faces, float normalize_scale, Eigen::Vector3d &normalize_offset, float object_scale)
{
	for (int i = 0; i < verts.size(); ++i) 
	{
		auto t = verts[i] * normalize_scale + normalize_offset;
		out_vertices.Add(XSI::MATH::CVector3(t[0] / object_scale, t[1] / object_scale, t[2] / object_scale));
	}
	for (int i = 0; i < faces.size(); ++i)
	{
		out_faces.Add(4);
		out_faces.Add(faces[i][0]);
		out_faces.Add(faces[i][1]);
		out_faces.Add(faces[i][2]);
		out_faces.Add(faces[i][3]);
	}
}

void instant_mesh_build(XSI::PolygonMesh &mesh, XSI::KinematicState &tfm, XSI::MATH::CVector3Array &out_vertices, XSI::CLongArray &out_faces,
	bool log_process, float object_scale,
	bool deterministic, bool extrinsic, float edge_length, int face_count, int vertex_count, int posy, int rosy, float crease_angle, bool align_to_boundaries, int smooth_iter, bool pure_quad, int n_procs)
{
	tbb::task_scheduler_init init(n_procs == -1 ? tbb::task_scheduler_init::automatic : n_procs);
	XSI::CComAPIHandler xsiuitoolkit;
	xsiuitoolkit.CreateInstance("XSI.UIToolkit");
	XSI::CValue rtn = xsiuitoolkit.GetProperty("ProgressBar");
	XSI::CComAPIHandler progressbar(rtn);
	progressbar.PutProperty("Visible", true);
	progressbar.PutProperty("Minimum", (LONG)0);
	progressbar.PutProperty("Maximum", (LONG)7);
	progressbar.PutProperty("Value", (LONG)0);
	progressbar.PutProperty("Caption", XSI::CValue("Read geometry..."));

	Float scale = edge_length;

	MatrixXu F;
	MatrixXf V, N;
	VectorXf A;

	std::set<uint32_t> crease_in, crease_out;
	BVH *bvh = nullptr;
	AdjacencyMatrix adj = nullptr;
	
	fill_in_vertices(V, mesh, tfm, object_scale);
	fill_in_faces(F, mesh);

	progressbar.PutProperty("Value", (LONG)1);
	progressbar.PutProperty("Caption", XSI::CValue("Gather statistics..."));

	Timer<> timer;
	MeshStats stats = compute_mesh_stats(F, V, deterministic);

	if (scale < 0 && vertex_count < 0 && face_count < 0) 
	{
		if(log_process)
		{
			XSI::Application().LogMessage(XSI::CString("No target vertex count/face count/scale argument provided. Setting to the default of 1 / 16 * input vertex count."));
		}
		vertex_count = V.cols() / 16;
		if(vertex_count <= 0)
		{
			vertex_count = 4;
		}
	}

	if (scale > 0)
	{
		Float face_area = posy == 4 ? (scale*scale) : (std::sqrt(3.f) / 4.f*scale*scale);
		face_count = stats.mSurfaceArea / face_area;
		vertex_count = posy == 4 ? face_count : (face_count / 2);
	}
	else if (face_count > 0)
	{
		Float face_area = stats.mSurfaceArea / face_count;
		vertex_count = posy == 4 ? face_count : (face_count / 2);
		scale = posy == 4 ? std::sqrt(face_area) : (2 * std::sqrt(face_area * std::sqrt(1.f / 3.f)));
	}
	else if (vertex_count > 0)
	{
		face_count = posy == 4 ? vertex_count : (vertex_count * 2);
		Float face_area = stats.mSurfaceArea / face_count;
		scale = posy == 4 ? std::sqrt(face_area) : (2 * std::sqrt(face_area * std::sqrt(1.f / 3.f)));
	}

	if(log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Output mesh goals (approximate)"));
		XSI::Application().LogMessage(XSI::CString("   Vertex count           = ") +  XSI::CString(vertex_count));
		XSI::Application().LogMessage(XSI::CString("   Face count             = ") + XSI::CString(face_count));
		XSI::Application().LogMessage(XSI::CString("   Edge count             = ") + XSI::CString(scale));
	}

	MultiResolutionHierarchy mRes;

	progressbar.PutProperty("Value", (LONG)2);
	progressbar.PutProperty("Caption", XSI::CValue("Subdivide..."));

	VectorXu V2E, E2E;
	VectorXb boundary, nonManifold;
	if (stats.mMaximumEdgeLength * 2 > scale || stats.mMaximumEdgeLength > stats.mAverageEdgeLength * 2) 
	{
		if(log_process)
		{
			XSI::Application().LogMessage(XSI::CString("Input mesh is too coarse for the desired output edge length (max input mesh edge length=") + XSI::CString(stats.mMaximumEdgeLength) + XSI::CString("), subdividing .."));
		}
		build_dedge(F, V, V2E, E2E, boundary, nonManifold);
		subdivide(F, V, V2E, E2E, boundary, nonManifold, std::min(scale / 2, (Float)stats.mAverageEdgeLength * 2), deterministic);
	}

	progressbar.PutProperty("Value", (LONG)3);
	progressbar.PutProperty("Caption", XSI::CValue("Building..."));

	build_dedge(F, V, V2E, E2E, boundary, nonManifold);

	adj = generate_adjacency_matrix_uniform(F, V2E, E2E, nonManifold);

	if (crease_angle >= 0)
	{
		generate_crease_normals(F, V, V2E, E2E, boundary, nonManifold, crease_angle, N, crease_in);
	}
	else
	{
		generate_smooth_normals(F, V, V2E, E2E, nonManifold, N);
	}

	compute_dual_vertex_areas(F, V, V2E, E2E, nonManifold, A);

	mRes.setE2E(std::move(E2E));

	mRes.setAdj(std::move(adj));
	mRes.setF(std::move(F));
	mRes.setV(std::move(V));
	mRes.setA(std::move(A));
	mRes.setN(std::move(N));
	mRes.setScale(scale);
	mRes.build(deterministic);
	mRes.resetSolution();

	progressbar.PutProperty("Value", (LONG)4);
	progressbar.PutProperty("Caption", XSI::CValue("Align to boundary..."));

	if (align_to_boundaries)
	{
		mRes.clearConstraints();
		for (uint32_t i = 0; i<3 * mRes.F().cols(); ++i)
		{
			if (mRes.E2E()[i] == INVALID)
			{
				uint32_t i0 = mRes.F()(i % 3, i / 3);
				uint32_t i1 = mRes.F()((i + 1) % 3, i / 3);
				Eigen::Vector3f p0 = mRes.V().col(i0), p1 = mRes.V().col(i1);
				Eigen::Vector3f edge = p1 - p0;
				if (edge.squaredNorm() > 0)
				{
					edge.normalize();
					mRes.CO().col(i0) = p0;
					mRes.CO().col(i1) = p1;
					mRes.CQ().col(i0) = mRes.CQ().col(i1) = edge;
					mRes.CQw()[i0] = mRes.CQw()[i1] = mRes.COw()[i0] = mRes.COw()[i1] = 1.0f;
				}
			}
		}
		mRes.propagateConstraints(rosy, posy);
	}

	progressbar.PutProperty("Value", (LONG)5);
	progressbar.PutProperty("Caption", XSI::CValue("Build mesh data..."));

	if (bvh)
	{
		bvh->setData(&mRes.F(), &mRes.V(), &mRes.N());
	}
	else if (smooth_iter > 0)
	{
		bvh = new BVH(&mRes.F(), &mRes.V(), &mRes.N(), stats.mAABB);
		bvh->build();
	}

	if(log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Preprocessing is done. (total time excluding file I/O: ") + XSI::CString(timeString(timer.reset()).c_str()) + ")");
	}

	progressbar.PutProperty("Value", (LONG)6);
	progressbar.PutProperty("Caption", XSI::CValue("Optimization..."));

	Optimizer optimizer(mRes, false);
	optimizer.setRoSy(rosy);
	optimizer.setPoSy(posy);
	optimizer.setExtrinsic(extrinsic);

	if(log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Optimizing orientation field .. "));
	}
	cout.flush();
	optimizer.optimizeOrientations(-1);
	optimizer.notify();
	optimizer.wait();

	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Done. (took ") + XSI::CString(timeString(timer.reset()).c_str()) + ")");
	}

	std::map<uint32_t, uint32_t> sing;
	compute_orientation_singularities(mRes, sing, extrinsic, rosy);

	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Orientation field has ") + XSI::CString(sing.size()) + " singularities.");
	}
	timer.reset();

	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Optimizing position field .. "));
	}
	cout.flush();
	optimizer.optimizePositions(-1);
	optimizer.notify();
	optimizer.wait();

	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Done. (took ") + XSI::CString(timeString(timer.reset()).c_str()) + ")");
	}
	
	optimizer.shutdown();

	progressbar.PutProperty("Value", (LONG)7);
	progressbar.PutProperty("Caption", XSI::CValue("Extract geometry data..."));

	MatrixXf O_extr, N_extr, Nf_extr;
	std::vector<std::vector<TaggedLink>> adj_extr;
	extract_graph(mRes, extrinsic, rosy, posy, adj_extr, O_extr, N_extr, crease_in, crease_out, deterministic);

	MatrixXu F_extr;
	extract_faces(adj_extr, O_extr, N_extr, Nf_extr, F_extr, posy, mRes.scale(), crease_out, true, pure_quad, bvh, smooth_iter);

	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Extraction is done. (total time: ") + XSI::CString(timeString(timer.reset()).c_str()) + ")");
	}
	
	fill_out(out_vertices, out_faces, O_extr, F_extr, object_scale);

	if (bvh)
	{
		delete bvh;
	}

	progressbar.PutProperty("Visible", false);
}

void quadri_flow_build(XSI::PolygonMesh &mesh, XSI::KinematicState &tfm, XSI::MATH::CVector3Array &out_vertices, XSI::CLongArray &out_faces, bool log_process, float object_scale,
	int faces, bool sharp, bool boundary, bool adaptive_scale, bool min_flow, bool sat, int seed)
{
	XSI::CComAPIHandler xsiuitoolkit;
	xsiuitoolkit.CreateInstance("XSI.UIToolkit");
	XSI::CValue rtn = xsiuitoolkit.GetProperty("ProgressBar");
	XSI::CComAPIHandler progressbar(rtn);
	progressbar.PutProperty("Visible", true);
	progressbar.PutProperty("Minimum", (LONG)0);
	progressbar.PutProperty("Maximum", (LONG)6);
	progressbar.PutProperty("Value", (LONG)0);
	progressbar.PutProperty("Caption", XSI::CValue("Read geometry..."));

	int t1, t2;

	qflow::Parametrizer field;
	field.flag_preserve_sharp = sharp ? 1 : 0;
	field.flag_preserve_boundary = boundary ? 1 : 0;
	field.flag_adaptive_scale = adaptive_scale ? 1 : 0;
	field.flag_minimum_cost_flow = min_flow ? 1 : 0;
	field.flag_aggresive_sat = sat ? 1 : 0;
	field.hierarchy.rng_seed = seed;

	fill_in_vertices(field.V, mesh, tfm, object_scale);
	fill_in_faces(field.F, mesh);
	
	field.NormalizeMesh();

	progressbar.PutProperty("Value", (LONG)1);
	progressbar.PutProperty("Caption", XSI::CValue("Initialize..."));

	t1 = qflow::GetCurrentTime64();
	field.Initialize(faces);
	t2 = qflow::GetCurrentTime64();
	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Initialize: ") + XSI::CString((t2 - t1) * 1e-3) + " seconds");
	}

	if (field.flag_preserve_boundary) 
	{
		qflow::Hierarchy& mRes = field.hierarchy;
		mRes.clearConstraints();
		for (uint32_t i = 0; i < 3 * mRes.mF.cols(); ++i) 
		{
			if (mRes.mE2E[i] == -1) 
			{
				uint32_t i0 = mRes.mF(i % 3, i / 3);
				uint32_t i1 = mRes.mF((i + 1) % 3, i / 3);
				Eigen::Vector3d p0 = mRes.mV[0].col(i0), p1 = mRes.mV[0].col(i1);
				Eigen::Vector3d edge = p1 - p0;
				if (edge.squaredNorm() > 0) {
					edge.normalize();
					mRes.mCO[0].col(i0) = p0;
					mRes.mCO[0].col(i1) = p1;
					mRes.mCQ[0].col(i0) = mRes.mCQ[0].col(i1) = edge;
					mRes.mCQw[0][i0] = mRes.mCQw[0][i1] = mRes.mCOw[0][i0] = mRes.mCOw[0][i1] = 1.0;
				}
			}
		}
		mRes.propagateConstraints();
	}

	progressbar.PutProperty("Value", (LONG)2);
	progressbar.PutProperty("Caption", XSI::CValue("Solve orientation field..."));
	t1 = qflow::GetCurrentTime64();
	qflow::Optimizer::optimize_orientations(field.hierarchy);
	field.ComputeOrientationSingularities();
	t2 = qflow::GetCurrentTime64();
	if(log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Solve orientation field: ") + XSI::CString((t2 - t1) * 1e-3) + " seconds");
	}

	if (field.flag_adaptive_scale == 1) 
	{
		t1 = qflow::GetCurrentTime64();
		field.EstimateSlope();
		t2 = qflow::GetCurrentTime64();
		if (log_process)
		{
			XSI::Application().LogMessage(XSI::CString("Estimate slope: ") + XSI::CString((t2 - t1) * 1e-3) + " seconds");
		}
	}

	progressbar.PutProperty("Value", (LONG)3);
	progressbar.PutProperty("Caption", XSI::CValue("Solve for scale..."));

	t1 = qflow::GetCurrentTime64();
	qflow::Optimizer::optimize_scale(field.hierarchy, field.rho, field.flag_adaptive_scale);
	field.flag_adaptive_scale = 1;
	t2 = qflow::GetCurrentTime64();
	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Solve for scale: ") + XSI::CString((t2 - t1) * 1e-3) + " seconds");
	}

	progressbar.PutProperty("Value", (LONG)4);
	progressbar.PutProperty("Caption", XSI::CValue("Solve for position field..."));
	t1 = qflow::GetCurrentTime64();
	qflow::Optimizer::optimize_positions(field.hierarchy, field.flag_adaptive_scale);

	field.ComputePositionSingularities();
	t2 = qflow::GetCurrentTime64();
	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Solve for position field: ") + XSI::CString((t2 - t1) * 1e-3) + " seconds");
	}

	progressbar.PutProperty("Value", (LONG)5);
	progressbar.PutProperty("Caption", XSI::CValue("Solve index map..."));
	t1 = qflow::GetCurrentTime64();
	field.ComputeIndexMap();
	t2 = qflow::GetCurrentTime64();
	if (log_process)
	{
		XSI::Application().LogMessage(XSI::CString("Solve index map: ") + XSI::CString((t2 - t1) * 1e-3) + " seconds");
	}

	progressbar.PutProperty("Value", (LONG)6);
	progressbar.PutProperty("Caption", XSI::CValue("Extract geometry data..."));

	fill_out(out_vertices, out_faces, field.O_compact, field.F_compact, field.normalize_scale, field.normalize_offset, object_scale);
	
	progressbar.PutProperty("Visible", false);
}