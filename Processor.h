#pragma once
void instant_mesh_build(XSI::PolygonMesh &mesh, XSI::KinematicState &tfm, XSI::MATH::CVector3Array &out_vertices, XSI::CLongArray &out_faces, bool log_process, float object_scale,//default parameters
	bool deterministic, bool extrinsic, float edge_length, int face_count, int vertex_count, int posy, int rosy, float crease_angle, bool align_to_boundaries, int smooth_iter, bool pure_quad, int n_procs);

void quadri_flow_build(XSI::PolygonMesh &mesh, XSI::KinematicState &tfm, XSI::MATH::CVector3Array &out_vertices, XSI::CLongArray &out_faces, bool log_process, float object_scale,
	int faces, bool sharp, bool boundary, bool adaptive_scale, bool min_flow, bool sat, int seed);