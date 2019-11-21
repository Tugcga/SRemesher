#pragma once
#include <xsi_application.h>
#include <xsi_context.h>
#include <xsi_pluginregistrar.h>
#include <xsi_status.h>
#include <xsi_customoperator.h>
#include <xsi_operatorcontext.h>
#include <xsi_ppglayout.h>
#include <xsi_ppgeventcontext.h>
#include <xsi_selection.h>
#include <xsi_command.h>
#include <xsi_factory.h>
#include <xsi_primitive.h>
#include <xsi_kinematics.h>
#include <xsi_outputport.h>
#include <xsi_x3dobject.h>
#include <xsi_polygonmesh.h>
#include <xsi_geometryaccessor.h>
#include <xsi_menu.h>
#include <xsi_model.h>
#include <xsi_meshbuilder.h>
#include <xsi_polygonface.h>

void Log(XSI::CString &message, XSI::siSeverityType message_type = XSI::siInfoMsg)
{
	XSI::Application().LogMessage(message, message_type);
}

void Log(XSI::MATH::CVector3Array &array)
{
	XSI::CString to_pint;
	for(size_t i = 0; i < array.GetCount(); i++)
	{
		XSI::MATH::CVector3 vector = array[i];
		to_pint += "(" + XSI::CString(vector.GetX()) + ", " + XSI::CString(vector.GetY()) + ", " + XSI::CString(vector.GetZ()) + (i == array.GetCount() - 1 ? ")" : "), ");
	}
	Log(to_pint);
}

void Log(XSI::CLongArray &array)
{
	XSI::CString to_print;
	for(size_t i = 0; i < array.GetCount(); i++)
	{
		to_print += XSI::CString(array[i]) + (i == array.GetCount() - 1 ? "" : ", ");
	}
	Log(to_print);
}