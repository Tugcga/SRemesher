#include "SRemesherHelpers.h"
#include "Processor.h"

using namespace XSI; 

SICALLBACK XSILoadPlugin(PluginRegistrar& in_reg)
{
	in_reg.PutAuthor("Shekn");
	in_reg.PutName("SRemesherPlugin");
	in_reg.PutVersion(1,0);
	in_reg.RegisterOperator("SRemesherInstantMeshes");
	in_reg.RegisterOperator("SRemesherQuadriFlow");
	in_reg.RegisterCommand("ApplySRemesherInstantMeshes","ApplySRemesherInstantMeshes");
	in_reg.RegisterCommand("ApplySRemesherQuadriFlow", "ApplySRemesherQuadriFlow");
	in_reg.RegisterMenu(siMenuTbModelCreatePolygonMeshID, "SRemesher", true);
	//RegistrationInsertionPoint - do not remove this line

	return CStatus::OK;
}

SICALLBACK XSIUnloadPlugin(const PluginRegistrar& in_reg)
{
	CString strPluginName;
	strPluginName = in_reg.GetName();

	return CStatus::OK;
}

SICALLBACK SRemesher_Init(XSI::CRef& in_ref)
{
	Context ctxt = in_ref;
	Menu menu = ctxt.GetSource();

	MenuItem item;
	menu.AddCommandItem("Instant Meshes", "ApplySRemesherInstantMeshes", item);
	menu.AddCommandItem("QuadriFlow", "ApplySRemesherQuadriFlow", item);

	return CStatus::OK;
}

SICALLBACK ApplySRemesherInstantMeshes_Init(CRef& in_ctxt)
{
	Context ctxt(in_ctxt);
	Command oCmd;
	oCmd = ctxt.GetSource();
	oCmd.PutDescription(L"Create an instance of SRemesherInstantMeshes operator");
	oCmd.SetFlag(siNoLogging, false);

	return CStatus::OK;
}

SICALLBACK ApplySRemesherQuadriFlow_Init(CRef& in_ctxt)
{
	Context ctxt(in_ctxt);
	Command oCmd;
	oCmd = ctxt.GetSource();
	oCmd.PutDescription(L"Create an instance of SRemesherQuadriFlow operator");
	oCmd.SetFlag(siNoLogging, false);

	return CStatus::OK;
}

CStatus apply_operator(CString &op_name, CString &property_name)
{
	Selection selection = Application().GetSelection();
	if (selection.GetCount() == 0)
	{
		Log(CString("Select at least one polygonmesh object"), siWarningMsg);
		return CStatus::Abort;
	}

	Model root = Application().GetActiveSceneRoot();

	for (LONG i = 0; i < selection.GetCount(); i++)
	{
		X3DObject obj = selection[i];
		CString obj_type = obj.GetType();
		if (obj_type == "polymsh")
		{//add operator only to the polymesh object
		 //create object for the output
			X3DObject new_object;
			CMeshBuilder mesh_builder;
			root.AddPolygonMesh(obj.GetName() + "_remesh", new_object, mesh_builder);

			//create operator
			CustomOperator remesh_op = Application().GetFactory().CreateObject(op_name);
			//add ports
			remesh_op.AddInputPort(obj.GetActivePrimitive());
			remesh_op.AddInputPort(obj.GetKinematics().GetGlobal());
			remesh_op.AddOutputPort(new_object.GetActivePrimitive());

			remesh_op.Connect();

			//select output object
			selection.Clear();
			selection.Add(new_object);

			//inspect PPG
			CValueArray inspect_args(5);
			inspect_args[0] = remesh_op;
			inspect_args[1] = "";
			inspect_args[2] = property_name;
			inspect_args[3] = siRecycle;
			inspect_args[4] = false;
			CValue retval;
			Application().ExecuteCommand("InspectObj", inspect_args, retval);

			return CStatus::OK;
		}
	}

	Log(CString("Select at least one polygonmesh object"), siWarningMsg);
	return CStatus::Abort;
}

SICALLBACK ApplySRemesherInstantMeshes_Execute(CRef& in_ctxt)
{
	Context ctxt(in_ctxt);
	CValueArray args = ctxt.GetAttribute("Arguments");

	return apply_operator(CString("SRemesherInstantMeshes"), CString("Instant Meshes"));
}

SICALLBACK ApplySRemesherQuadriFlow_Execute(CRef& in_ctxt)
{
	Context ctxt(in_ctxt);
	CValueArray args = ctxt.GetAttribute("Arguments");

	return apply_operator(CString("SRemesherQuadriFlow"), CString("QuadiFlow"));
}

#define kParamCaps	(siAnimatable | siPersistable | siKeyable)

SICALLBACK SRemesherInstantMeshes_Define(CRef& in_ctxt)
{
	Context ctxt( in_ctxt );
	Factory factory = Application().GetFactory();
	CustomOperator custom_op = ctxt.GetSource();

	custom_op.PutAlwaysEvaluate(false);
	custom_op.PutDebug(0);

	CRef def_log = factory.CreateParamDef("log", CValue::siBool, kParamCaps, "log", "", false, 0, 1, 0, 1);
	CRef def_deterministic = factory.CreateParamDef("deterministic", CValue::siBool, kParamCaps, "deterministic", "", true, 0, 1, 0, 1);
	CRef def_dominant = factory.CreateParamDef("dominant", CValue::siBool, kParamCaps, "dominant", "", true, 0, 1, 0, 1);
	CRef def_extrinsic = factory.CreateParamDef("extrinsic", CValue::siBool, kParamCaps, "extrinsic", "", true, 0, 1, 0, 1);
	CRef def_boundaries = factory.CreateParamDef("boundaries", CValue::siBool, kParamCaps, "boundaries", "", true, 0, 1, 0, 1);
	CRef def_threads = factory.CreateParamDef("threads", CValue::siInt2, kParamCaps, "threads", "", -1, -1, 32767, -1, 12);
	CRef def_smooth = factory.CreateParamDef("smooth", CValue::siUInt1, kParamCaps, "smooth", "", 0, 0, 255, 0, 8);
	CRef def_crease = factory.CreateParamDef("crease", CValue::siFloat, kParamCaps, "crease", "", 0.0f, 0.0, 6.28f, 0.0f, 1.57f);
	CRef def_rosy = factory.CreateParamDef("rosy", CValue::siUInt1, kParamCaps, "rosy", "", 1, 0, 2, 0, 2);
	CRef def_posy = factory.CreateParamDef("posy", CValue::siUInt1, kParamCaps, "posy", "", 0, 0, 1, 0, 1);
	CRef def_edge_length = factory.CreateParamDef("edge_length", CValue::siFloat, kParamCaps, "edge_length", "", -1.0f, -1.0f, FLT_MAX, -1.0f, 4.0f);
	CRef def_faces = factory.CreateParamDef("faces", CValue::siInt4, kParamCaps, "faces", "", -1, -1, INT_MAX, -1, 2048);
	CRef def_scale = factory.CreateParamDef("scale", CValue::siFloat, kParamCaps, "scale", "", 1.0f, 0.01f, FLT_MAX, 0.01f, 2.0f);

	Parameter param_log;
	Parameter param_deterministic;
	Parameter param_dominant;
	Parameter param_extrinsic;
	Parameter param_boundaries;
	Parameter param_threads;
	Parameter param_smooth;
	Parameter param_crease;
	Parameter param_rosy;
	Parameter param_posy;
	Parameter param_edge_length;
	Parameter param_faces;
	Parameter param_scale;

	custom_op.AddParameter(def_log, param_log);
	custom_op.AddParameter(def_deterministic, param_deterministic);
	custom_op.AddParameter(def_dominant, param_dominant);
	custom_op.AddParameter(def_extrinsic, param_extrinsic);
	custom_op.AddParameter(def_boundaries, param_boundaries);
	custom_op.AddParameter(def_threads, param_threads);
	custom_op.AddParameter(def_smooth, param_smooth);
	custom_op.AddParameter(def_crease, param_crease);
	custom_op.AddParameter(def_rosy, param_rosy);
	custom_op.AddParameter(def_posy, param_posy);
	custom_op.AddParameter(def_edge_length, param_edge_length);
	custom_op.AddParameter(def_faces, param_faces);
	custom_op.AddParameter(def_scale, param_scale);

	return CStatus::OK;
}

SICALLBACK SRemesherQuadriFlow_Define(CRef& in_ctxt)
{
	Context ctxt(in_ctxt);
	Factory factory = Application().GetFactory();
	CustomOperator custom_op = ctxt.GetSource();

	custom_op.PutAlwaysEvaluate(false);
	custom_op.PutDebug(0);

	CRef def_log = factory.CreateParamDef("log", CValue::siBool, kParamCaps, "log", "", false, 0, 1, 0, 1);
	CRef def_scale = factory.CreateParamDef("scale", CValue::siFloat, kParamCaps, "scale", "", 1.0f, 0.01f, FLT_MAX, 0.01f, 2.0f);
	CRef def_faces = factory.CreateParamDef("faces", CValue::siInt4, kParamCaps, "faces", "", -1, -1, INT_MAX, -1, 2048);
	CRef def_sharp = factory.CreateParamDef("sharp", CValue::siBool, kParamCaps, "sharp", "", false, 0, 1, 0, 1);
	CRef def_boundary = factory.CreateParamDef("boundary", CValue::siBool, kParamCaps, "boundary", "", false, 0, 1, 0, 1);
	CRef def_adaptive_scale = factory.CreateParamDef("adaptive_scale", CValue::siBool, kParamCaps, "adaptive_scale", "", false, 0, 1, 0, 1);
	CRef def_min_flow = factory.CreateParamDef("min_flow", CValue::siBool, kParamCaps, "min_flow", "", false, 0, 1, 0, 1);
	CRef def_sat = factory.CreateParamDef("sat", CValue::siBool, kParamCaps, "sat", "", false, 0, 1, 0, 1);
	CRef def_seed = factory.CreateParamDef("seed", CValue::siInt4, kParamCaps, "seed", "", 0, INT_MIN, INT_MAX, -10, 10);

	Parameter param_log;
	Parameter param_scale;
	Parameter param_faces;
	Parameter param_sharp;
	Parameter param_boundary;
	Parameter param_adaptive_scale;
	Parameter param_min_flow;
	Parameter param_sat;
	Parameter param_seed;

	custom_op.AddParameter(def_log, param_log);
	custom_op.AddParameter(def_scale, param_scale);
	custom_op.AddParameter(def_faces, param_faces);
	custom_op.AddParameter(def_sharp, param_sharp);
	custom_op.AddParameter(def_boundary, param_boundary);
	custom_op.AddParameter(def_adaptive_scale, param_adaptive_scale);
	custom_op.AddParameter(def_min_flow, param_min_flow);
	custom_op.AddParameter(def_sat, param_sat);
	custom_op.AddParameter(def_seed, param_seed);

	return CStatus::OK;
}

void build_im_ui(PPGLayout &layout)
{
	CValueArray enum_rosy;
	enum_rosy.Add(CValue("2"));
	enum_rosy.Add(CValue(0));
	enum_rosy.Add(CValue("4"));
	enum_rosy.Add(CValue(1));
	enum_rosy.Add(CValue("6"));
	enum_rosy.Add(CValue(2));

	CValueArray enum_posy;
	enum_posy.Add(CValue("4"));
	enum_posy.Add(CValue(0));
	enum_posy.Add(CValue("6"));
	enum_posy.Add(CValue(1));

	layout.Clear();

	layout.AddGroup("Instant Mesh Options");
	layout.AddGroup("Desired Geometry Density");
	layout.AddItem("edge_length", "Edge Length");
	layout.AddItem("faces", "Faces Count");
	layout.EndGroup();

	layout.AddItem("scale", "Object Scale");
	layout.AddItem("crease", "Angle Crease");
	layout.AddItem("smooth", "Smooth Iterations");
	layout.AddItem("boundaries", "Align to Boundaries");
	layout.AddItem("dominant", "Pure Quad Mesh");
	layout.AddItem("deterministic", "Deterministic");
	layout.AddItem("extrinsic", "Extrinsic");

	layout.AddEnumControl("rosy", enum_rosy, "Orientation Symmetry");
	layout.AddEnumControl("posy", enum_posy, "Position Symmetry");
	layout.EndGroup();

	layout.AddGroup("Logging");
	layout.AddItem("log", "Log Process");
	layout.EndGroup();
}

void build_qf_ui(PPGLayout &layout)
{
	layout.Clear();

	layout.AddGroup("Quadri Flow Options");

	layout.AddItem("faces", "Faces Count");
	layout.AddItem("sharp", "Preserve Sharp");
	layout.AddItem("boundary", "Preserve Boundary");
	layout.AddItem("adaptive_scale", "Adaptive Scale");
	layout.AddItem("min_flow", "Minimum Cost Flow");
	layout.AddItem("sat", "Aggressive SAT");
	layout.AddItem("seed", "Seed");

	//layout.AddItem("scale", "Object Scale");

	layout.EndGroup();

	layout.AddGroup("Logging");
	layout.AddItem("log", "Log Process");
	layout.EndGroup();
}

XSIPLUGINCALLBACK CStatus SRemesherInstantMeshes_DefineLayout(CRef& in_ctxt)
{
	XSI::Context ctxt(in_ctxt);

	PPGLayout layout = ctxt.GetSource();

	build_im_ui(layout);

	return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus SRemesherQuadriFlow_DefineLayout(CRef& in_ctxt)
{
	XSI::Context ctxt(in_ctxt);

	PPGLayout layout = ctxt.GetSource();

	build_qf_ui(layout);

	return CStatus::OK;
}

SICALLBACK SRemesherInstantMeshes_Init(CRef& in_ctxt)
{
	OperatorContext ctxt(in_ctxt);

	return CStatus::OK;
}

SICALLBACK SRemesherQuadriFlow_Init(CRef& in_ctxt)
{
	OperatorContext ctxt(in_ctxt);

	return CStatus::OK;
}

SICALLBACK SRemesherInstantMeshes_Term(CRef& in_ctxt)
{
	OperatorContext ctxt(in_ctxt);

	return CStatus::OK;
}

SICALLBACK SRemesherQuadriFlow_Term(CRef& in_ctxt)
{
	OperatorContext ctxt(in_ctxt);

	return CStatus::OK;
}

SICALLBACK SRemesherInstantMeshes_Update(CRef& in_ctxt)
{
	OperatorContext ctxt(in_ctxt);

	Primitive in_primitive(ctxt.GetInputValue(0));
	KinematicState in_tfm(ctxt.GetInputValue(1));

	PolygonMesh in_polymesh(in_primitive.GetGeometry());

	//process
	MATH::CVector3Array vertices;
	CLongArray faces;

	int posy = ctxt.GetParameterValue("posy");
	int rosy = ctxt.GetParameterValue("rosy");
	int mode = ctxt.GetParameterValue("mode");
	float target_edge_legth = ctxt.GetParameterValue("edge_length");
	int target_face_count = ctxt.GetParameterValue("faces");
	int target_vertex_count = -1;

	instant_mesh_build(in_polymesh, in_tfm, vertices, faces, ctxt.GetParameterValue("log"), ctxt.GetParameterValue("scale"),
		ctxt.GetParameterValue("deterministic"), ctxt.GetParameterValue("extrinsic"), target_edge_legth, target_face_count, target_vertex_count,
		posy == 0 ? 4 : 3,
		rosy == 0 ? 2 : (rosy == 1 ? 4 : 6),
		ctxt.GetParameterValue("crease"), ctxt.GetParameterValue("boundaries"), ctxt.GetParameterValue("smooth"), ctxt.GetParameterValue("dominant"), ctxt.GetParameterValue("threads"));

	//output
	Operator source = ctxt.GetSource();
	CRefArray out_ports = source.GetOutputPorts();
	OutputPort out_port(out_ports[0]);
	Primitive out_primitive(out_port.GetTarget());
	PolygonMesh out_polygonmesh(out_primitive.GetGeometry());

	out_polygonmesh.Set(vertices, faces);

	return CStatus::OK;
}

SICALLBACK SRemesherQuadriFlow_Update(CRef& in_ctxt)
{
	OperatorContext ctxt(in_ctxt);
	Primitive in_primitive(ctxt.GetInputValue(0));
	KinematicState in_tfm(ctxt.GetInputValue(1));

	PolygonMesh in_polymesh(in_primitive.GetGeometry());

	MATH::CVector3Array out_vertices;
	CLongArray out_faces;

	bool is_log = ctxt.GetParameterValue("log");
	int faces = ctxt.GetParameterValue("faces");
	if(faces < 0)
	{
		CPolygonFaceRefArray in_faces = in_polymesh.GetPolygons();
		faces = in_faces.GetCount() / 16;
		if(is_log)
		{
			Log(CString("Set destination polygons count to ") + CString(faces));
		}
	}

	quadri_flow_build(in_polymesh, in_tfm, out_vertices, out_faces, is_log, 1.0f,
		faces, ctxt.GetParameterValue("sharp"), ctxt.GetParameterValue("boundary"), ctxt.GetParameterValue("adaptive_scale"), ctxt.GetParameterValue("min_flow"), ctxt.GetParameterValue("sat"), ctxt.GetParameterValue("seed"));

	Operator source = ctxt.GetSource();
	CRefArray out_ports = source.GetOutputPorts();
	OutputPort out_port(out_ports[0]);
	Primitive out_primitive(out_port.GetTarget());
	PolygonMesh out_polygonmesh(out_primitive.GetGeometry());

	out_polygonmesh.Set(out_vertices, out_faces);

	return CStatus::OK;
}

