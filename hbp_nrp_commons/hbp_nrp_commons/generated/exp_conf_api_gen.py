# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END

# ./exp_conf_api_gen.py
# -*- coding: utf-8 -*-
# PyXB bindings for NM:e455c55bde4037492ee2ed5fe3b59ed8c44bb0d3
# Generated 2017-12-12 10:11:11.584984 by PyXB version 1.2.4 using Python 2.7.6.final.0
# Namespace http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig

from __future__ import unicode_literals
import pyxb
import pyxb.binding
import pyxb.binding.saxer
import io
import pyxb.utils.utility
import pyxb.utils.domutils
import sys
import pyxb.utils.six as _six

# Unique identifier for bindings created at the same time
_GenerationUID = pyxb.utils.utility.UniqueIdentifier('urn:uuid:6594d9b0-df1c-11e7-be31-b8ca3a8b0340')

# Version of PyXB used to generate the bindings
_PyXBVersion = '1.2.4'
# Generated bindings are not compatible across PyXB versions
if pyxb.__version__ != _PyXBVersion:
    raise pyxb.PyXBVersionError(_PyXBVersion)

# Import bindings for namespaces imported into schema
import pyxb.binding.datatypes
import _sc as _ImportedBinding__sc

# NOTE: All namespace declarations are reserved within the binding
Namespace = pyxb.namespace.NamespaceForURI('http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig', create_if_missing=True)
Namespace.configureCategories(['typeBinding', 'elementBinding'])
_Namespace_sc = _ImportedBinding__sc.Namespace
_Namespace_sc.configureCategories(['typeBinding', 'elementBinding'])

def CreateFromDocument (xml_text, default_namespace=None, location_base=None):
    """Parse the given XML and use the document element to create a
    Python instance.

    @param xml_text An XML document.  This should be data (Python 2
    str or Python 3 bytes), or a text (Python 2 unicode or Python 3
    str) in the L{pyxb._InputEncoding} encoding.

    @keyword default_namespace The L{pyxb.Namespace} instance to use as the
    default namespace where there is no default namespace in scope.
    If unspecified or C{None}, the namespace of the module containing
    this function will be used.

    @keyword location_base: An object to be recorded as the base of all
    L{pyxb.utils.utility.Location} instances associated with events and
    objects handled by the parser.  You might pass the URI from which
    the document was obtained.
    """

    if pyxb.XMLStyle_saxer != pyxb._XMLStyle:
        dom = pyxb.utils.domutils.StringToDOM(xml_text)
        return CreateFromDOM(dom.documentElement, default_namespace=default_namespace)
    if default_namespace is None:
        default_namespace = Namespace.fallbackNamespace()
    saxer = pyxb.binding.saxer.make_parser(fallback_namespace=default_namespace, location_base=location_base)
    handler = saxer.getContentHandler()
    xmld = xml_text
    if isinstance(xmld, _six.text_type):
        xmld = xmld.encode(pyxb._InputEncoding)
    saxer.parse(io.BytesIO(xmld))
    instance = handler.rootObject()
    return instance

def CreateFromDOM (node, default_namespace=None):
    """Create a Python instance from the given DOM node.
    The node tag must correspond to an element declaration in this module.

    @deprecated: Forcing use of DOM interface is unnecessary; use L{CreateFromDocument}."""
    if default_namespace is None:
        default_namespace = Namespace.fallbackNamespace()
    return pyxb.binding.basis.element.AnyCreateFromDOM(node, default_namespace)


# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ThumbnailFile
class ThumbnailFile (pyxb.binding.datatypes.string):

    """This type denotes a path to an image file. The supported extensions are .png, .jpg, .jpeg and .gif. The file name must not contain whitespaces."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ThumbnailFile')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 18, 2)
    _Documentation = 'This type denotes a path to an image file. The supported extensions are .png, .jpg, .jpeg and .gif. The file name must not contain whitespaces.'
ThumbnailFile._CF_pattern = pyxb.binding.facets.CF_pattern()
ThumbnailFile._CF_pattern.addPattern(pattern='[a-zA-Z0-9\\._\\-/]*\\.(png|gif|jp[e]?g)')
ThumbnailFile._InitializeFacetMap(ThumbnailFile._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'ThumbnailFile', ThumbnailFile)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}MaturityType
class MaturityType (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """This type denotes a maturity of an experiment. It can either be development or production."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'MaturityType')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 27, 2)
    _Documentation = 'This type denotes a maturity of an experiment. It can either be development or production.'
MaturityType._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=MaturityType, enum_prefix=None)
MaturityType.development = MaturityType._CF_enumeration.addEnumeration(unicode_value='development', tag='development')
MaturityType.production = MaturityType._CF_enumeration.addEnumeration(unicode_value='production', tag='production')
MaturityType._InitializeFacetMap(MaturityType._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'MaturityType', MaturityType)

# List simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}Tags
# superclasses pyxb.binding.datatypes.anySimpleType
class Tags (pyxb.binding.basis.STD_list):

    """Simple type that is a list of pyxb.binding.datatypes.string."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Tags')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 81, 2)
    _Documentation = None

    _ItemType = pyxb.binding.datatypes.string
Tags._InitializeFacetMap()
Namespace.addCategoryObject('typeBinding', 'Tags', Tags)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ConfTypeEnumeration
class ConfTypeEnumeration (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """This enumeration lists the standard configuration types used in the NRP."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ConfTypeEnumeration')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 92, 2)
    _Documentation = 'This enumeration lists the standard configuration types used in the NRP.'
ConfTypeEnumeration._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=ConfTypeEnumeration, enum_prefix=None)
ConfTypeEnumeration.n3d_settings = ConfTypeEnumeration._CF_enumeration.addEnumeration(unicode_value='3d-settings', tag='n3d_settings')
ConfTypeEnumeration._InitializeFacetMap(ConfTypeEnumeration._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'ConfTypeEnumeration', ConfTypeEnumeration)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}PhysicsEngine
class PhysicsEngine (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """This enumeration contains the physics engines supported by the NRP. This includes the standard physics engine ODE and OpenSim."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'PhysicsEngine')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 203, 2)
    _Documentation = 'This enumeration contains the physics engines supported by the NRP. This includes the standard physics engine ODE and OpenSim.'
PhysicsEngine._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=PhysicsEngine, enum_prefix=None)
PhysicsEngine.ode = PhysicsEngine._CF_enumeration.addEnumeration(unicode_value='ode', tag='ode')
PhysicsEngine.opensim = PhysicsEngine._CF_enumeration.addEnumeration(unicode_value='opensim', tag='opensim')
PhysicsEngine._InitializeFacetMap(PhysicsEngine._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'PhysicsEngine', PhysicsEngine)

# Union simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ConfType
# superclasses pyxb.binding.datatypes.anySimpleType
class ConfType (pyxb.binding.basis.STD_union):

    """This type denotes a configuration type which can be a standard configuration type or a custom type. The latter is just any string."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ConfType')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 85, 2)
    _Documentation = 'This type denotes a configuration type which can be a standard configuration type or a custom type. The latter is just any string.'

    _MemberTypes = ( ConfTypeEnumeration, pyxb.binding.datatypes.string, )
ConfType._CF_pattern = pyxb.binding.facets.CF_pattern()
ConfType._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=ConfType)
ConfType.n3d_settings = '3d-settings'             # originally ConfTypeEnumeration.n3d_settings
ConfType._InitializeFacetMap(ConfType._CF_pattern,
   ConfType._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'ConfType', ConfType)

# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}RobotPose with content type EMPTY
class RobotPose (pyxb.binding.basis.complexTypeDefinition):
    """This type represents a robot pose. It consists of a position part (x, y and z coordinates) and a rotation part (ux, uy, uz and theta). All fields are double precision values."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RobotPose')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 38, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute x uses Python identifier x
    __x = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'x'), 'x', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_x', pyxb.binding.datatypes.double, required=True)
    __x._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 42, 4)
    __x._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 42, 4)
    
    x = property(__x.value, __x.set, None, 'The x coordinate of the robot position')

    
    # Attribute y uses Python identifier y
    __y = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'y'), 'y', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_y', pyxb.binding.datatypes.double, required=True)
    __y._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 47, 4)
    __y._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 47, 4)
    
    y = property(__y.value, __y.set, None, 'The y coordinate of the robot position')

    
    # Attribute z uses Python identifier z
    __z = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'z'), 'z', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_z', pyxb.binding.datatypes.double, required=True)
    __z._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 52, 4)
    __z._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 52, 4)
    
    z = property(__z.value, __z.set, None, 'The z coordinate of the robot position')

    
    # Attribute ux uses Python identifier ux
    __ux = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'ux'), 'ux', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_ux', pyxb.binding.datatypes.double, required=True)
    __ux._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 57, 4)
    __ux._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 57, 4)
    
    ux = property(__ux.value, __ux.set, None, None)

    
    # Attribute uy uses Python identifier uy
    __uy = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'uy'), 'uy', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_uy', pyxb.binding.datatypes.double, required=True)
    __uy._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 58, 4)
    __uy._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 58, 4)
    
    uy = property(__uy.value, __uy.set, None, None)

    
    # Attribute uz uses Python identifier uz
    __uz = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'uz'), 'uz', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_uz', pyxb.binding.datatypes.double, required=True)
    __uz._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 59, 4)
    __uz._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 59, 4)
    
    uz = property(__uz.value, __uz.set, None, None)

    
    # Attribute theta uses Python identifier theta
    __theta = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'theta'), 'theta', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_theta', pyxb.binding.datatypes.double, required=True)
    __theta._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 60, 4)
    __theta._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 60, 4)
    
    theta = property(__theta.value, __theta.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __x.name() : __x,
        __y.name() : __y,
        __z.name() : __z,
        __ux.name() : __ux,
        __uy.name() : __uy,
        __uz.name() : __uz,
        __theta.name() : __theta
    })
Namespace.addCategoryObject('typeBinding', 'RobotPose', RobotPose)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}Position with content type EMPTY
class Position (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes a position with x, y and z coordinates."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Position')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 102, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute x uses Python identifier x
    __x = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'x'), 'x', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_x', pyxb.binding.datatypes.double, required=True)
    __x._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 106, 4)
    __x._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 106, 4)
    
    x = property(__x.value, __x.set, None, 'The x coordinate of the position')

    
    # Attribute y uses Python identifier y
    __y = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'y'), 'y', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_y', pyxb.binding.datatypes.double, required=True)
    __y._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 111, 4)
    __y._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 111, 4)
    
    y = property(__y.value, __y.set, None, 'The y coordinate of the position')

    
    # Attribute z uses Python identifier z
    __z = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'z'), 'z', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_z', pyxb.binding.datatypes.double, required=True)
    __z._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 116, 4)
    __z._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 116, 4)
    
    z = property(__z.value, __z.set, None, 'The z coordinate of the position')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __x.name() : __x,
        __y.name() : __y,
        __z.name() : __z
    })
Namespace.addCategoryObject('typeBinding', 'Position', Position)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}CameraPose with content type ELEMENT_ONLY
class CameraPose (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes a camera pose. Unlike the robot pose, a camera pose is specified using a position of the camera and a point to which the camera looks at. The camera is always rotated with the up vector z (0,0,1)."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'CameraPose')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 124, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraPosition uses Python identifier cameraPosition
    __cameraPosition = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition'), 'cameraPosition', '__httpschemas_humanbrainproject_euSP102014ExDConfig_CameraPose_httpschemas_humanbrainproject_euSP102014ExDConfigcameraPosition', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 129, 6), )

    
    cameraPosition = property(__cameraPosition.value, __cameraPosition.set, None, 'The position of the camera')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraLookAt uses Python identifier cameraLookAt
    __cameraLookAt = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt'), 'cameraLookAt', '__httpschemas_humanbrainproject_euSP102014ExDConfig_CameraPose_httpschemas_humanbrainproject_euSP102014ExDConfigcameraLookAt', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 134, 6), )

    
    cameraLookAt = property(__cameraLookAt.value, __cameraLookAt.set, None, 'The position to which the camera should look at')

    _ElementMap.update({
        __cameraPosition.name() : __cameraPosition,
        __cameraLookAt.name() : __cameraLookAt
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'CameraPose', CameraPose)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}EnvironmentModel with content type ELEMENT_ONLY
class EnvironmentModel (pyxb.binding.basis.complexTypeDefinition):
    """This type defines the necessary configuration for an environment. It combines the specification of an environment model through the src attribute and a robot pose using the element robotPose."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'EnvironmentModel')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 143, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}robotPose uses Python identifier robotPose
    __robotPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'robotPose'), 'robotPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_EnvironmentModel_httpschemas_humanbrainproject_euSP102014ExDConfigrobotPose', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 148, 6), )

    
    robotPose = property(__robotPose.value, __robotPose.set, None, 'The position of the robot')

    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_EnvironmentModel_src', pyxb.binding.datatypes.string, required=True)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 154, 4)
    __src._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 154, 4)
    
    src = property(__src.value, __src.set, None, 'A path to an SDF file that specifies the scene')

    _ElementMap.update({
        __robotPose.name() : __robotPose
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'EnvironmentModel', EnvironmentModel)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}VisualModel with content type ELEMENT_ONLY
class VisualModel (pyxb.binding.basis.complexTypeDefinition):
    """This type defines a visual model (for example for the robot) as used in the frontend."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'VisualModel')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 162, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}visualPose uses Python identifier visualPose
    __visualPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'visualPose'), 'visualPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_httpschemas_humanbrainproject_euSP102014ExDConfigvisualPose', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 167, 6), )

    
    visualPose = property(__visualPose.value, __visualPose.set, None, None)

    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_src', pyxb.binding.datatypes.string, required=True)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 169, 4)
    __src._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 169, 4)
    
    src = property(__src.value, __src.set, None, None)

    
    # Attribute scale uses Python identifier scale
    __scale = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'scale'), 'scale', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_scale', pyxb.binding.datatypes.double)
    __scale._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 170, 4)
    __scale._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 170, 4)
    
    scale = property(__scale.value, __scale.set, None, None)

    _ElementMap.update({
        __visualPose.name() : __visualPose
    })
    _AttributeMap.update({
        __src.name() : __src,
        __scale.name() : __scale
    })
Namespace.addCategoryObject('typeBinding', 'VisualModel', VisualModel)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}BibiConf with content type EMPTY
class BibiConf (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes the BIBI configuration used for this experiment. It is described using a reference to the BIBI model in the src attribute and an attribute processes to specify the number of processes that should be used to run the experiment. The default value for processes is 1."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'BibiConf')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 174, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_BibiConf_src', pyxb.binding.datatypes.string, required=True)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 178, 4)
    __src._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 178, 4)
    
    src = property(__src.value, __src.set, None, 'The path to the BIBI configuration that specifies the model, the neural network and the connection between those.')

    
    # Attribute processes uses Python identifier processes
    __processes = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'processes'), 'processes', '__httpschemas_humanbrainproject_euSP102014ExDConfig_BibiConf_processes', pyxb.binding.datatypes.positiveInteger, unicode_default='1')
    __processes._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 183, 4)
    __processes._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 183, 4)
    
    processes = property(__processes.value, __processes.set, None, 'The number of processes that should be used to run the neural network simulation. If this value is larger than 1, a dedicated simulation setup for distributed simulation of the neural network is used.')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src,
        __processes.name() : __processes
    })
Namespace.addCategoryObject('typeBinding', 'BibiConf', BibiConf)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}RosLaunch with content type EMPTY
class RosLaunch (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes a Ros Launchfile configuration."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RosLaunch')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 191, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RosLaunch_src', pyxb.binding.datatypes.string, required=True)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 195, 4)
    __src._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 195, 4)
    
    src = property(__src.value, __src.set, None, 'The path to a ROSLaunch file')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'RosLaunch', RosLaunch)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ExD with content type ELEMENT_ONLY
class ExD_ (pyxb.binding.basis.complexTypeDefinition):
    """This type is the root type for an experiment configuration."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ExD')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 214, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}name uses Python identifier name
    __name = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigname', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 219, 6), )

    
    name = property(__name.value, __name.set, None, 'This element denotes the name of the experiment as it appears in the experiment list.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}thumbnail uses Python identifier thumbnail
    __thumbnail = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'thumbnail'), 'thumbnail', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigthumbnail', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 224, 6), )

    
    thumbnail = property(__thumbnail.value, __thumbnail.set, None, 'This element references a path to a thumbnail that is used to give the user a forecast to the experiment.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}description uses Python identifier description
    __description = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'description'), 'description', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigdescription', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 229, 6), )

    
    description = property(__description.value, __description.set, None, 'This description will appear in the experiment description and provide a short description explaining what the experiment is all about.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}tags uses Python identifier tags
    __tags = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'tags'), 'tags', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigtags', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 234, 6), )

    
    tags = property(__tags.value, __tags.set, None, 'List of space separated tags that describe the experiment.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}timeout uses Python identifier timeout
    __timeout = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'timeout'), 'timeout', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigtimeout', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 239, 6), )

    
    timeout = property(__timeout.value, __timeout.set, None, 'The timeout of an experiment is the time an experiment is allowed to run by default, specified in seconds. If that time has elapsed, the users are asked whether they want to extend the runtime of the simulation. On the servers, this will only be allowed if the timeout fits within the cluster allocation.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}configuration uses Python identifier configuration
    __configuration = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'configuration'), 'configuration', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigconfiguration', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 244, 6), )

    
    configuration = property(__configuration.value, __configuration.set, None, 'An experiment may have multiple configuration entries. Despite configuration entries can be specified in anywhere in the ExD element, they must appear together.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}maturity uses Python identifier maturity
    __maturity = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'maturity'), 'maturity', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigmaturity', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 249, 6), )

    
    maturity = property(__maturity.value, __maturity.set, None, 'The maturity of an experiment determines whether it is shown by default to the user or only browsable in dev mode.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}environmentModel uses Python identifier environmentModel
    __environmentModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'environmentModel'), 'environmentModel', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigenvironmentModel', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 254, 6), )

    
    environmentModel = property(__environmentModel.value, __environmentModel.set, None, 'The environment model of an experiment specifies the used world file for a simulation and the pose where the robot should be spawned.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}visualModel uses Python identifier visualModel
    __visualModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'visualModel'), 'visualModel', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigvisualModel', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 259, 6), )

    
    visualModel = property(__visualModel.value, __visualModel.set, None, 'With the visual model, an experiment can specify an alternatively used model for the frontend visualization. This is helpful in case the robot model used in gazebo is very detailed and thus hard to visualize on the client. On the server, there may be more resources available to simulate more complex models.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}bibiConf uses Python identifier bibiConf
    __bibiConf = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'bibiConf'), 'bibiConf', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigbibiConf', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 264, 6), )

    
    bibiConf = property(__bibiConf.value, __bibiConf.set, None, 'The bibiConf element of an experiment configuration specifies the ')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}experimentControl uses Python identifier experimentControl
    __experimentControl = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'experimentControl'), 'experimentControl', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigexperimentControl', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 269, 6), )

    
    experimentControl = property(__experimentControl.value, __experimentControl.set, None, 'The experiment control lists all state machines that control the experiment.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}experimentEvaluation uses Python identifier experimentEvaluation
    __experimentEvaluation = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation'), 'experimentEvaluation', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigexperimentEvaluation', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 278, 6), )

    
    experimentEvaluation = property(__experimentEvaluation.value, __experimentEvaluation.set, None, 'The experiment evaluation element lists all state machines that evaluate the success of a simulated experiment.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraPose uses Python identifier cameraPose
    __cameraPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraPose'), 'cameraPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigcameraPose', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 287, 6), )

    
    cameraPose = property(__cameraPose.value, __cameraPose.set, None, 'The camera pose specifies the initial position of the camera when a simulation is started.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}rosLaunch uses Python identifier rosLaunch
    __rosLaunch = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'rosLaunch'), 'rosLaunch', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigrosLaunch', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 292, 6), )

    
    rosLaunch = property(__rosLaunch.value, __rosLaunch.set, None, 'The roslaunch element species the path to a ROSLaunch file that is executed when the experiment is simulated. If no file is specified, no ROSLaunch file is executed at the beginning of an experiment.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}rngSeed uses Python identifier rngSeed
    __rngSeed = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'rngSeed'), 'rngSeed', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigrngSeed', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 297, 6), )

    
    rngSeed = property(__rngSeed.value, __rngSeed.set, None, 'If specified, this element specifies the random number generator seed. If this field is left blank, a seed is generated and therefore, the simulation is not 100% deterministic. If a seed is specified here, this seed is used for the robot and neural simulation, making the simulation much more deterministic.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}physicsEngine uses Python identifier physicsEngine
    __physicsEngine = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'physicsEngine'), 'physicsEngine', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigphysicsEngine', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 302, 6), )

    
    physicsEngine = property(__physicsEngine.value, __physicsEngine.set, None, 'If specified, this element denotes the physics simulator that should be used. We currently support either ODE or OpenSim.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}gzbridgesettings uses Python identifier gzbridgesettings
    __gzbridgesettings = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'gzbridgesettings'), 'gzbridgesettings', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfiggzbridgesettings', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 307, 6), )

    
    gzbridgesettings = property(__gzbridgesettings.value, __gzbridgesettings.set, None, 'Settings for the relay of the component relaying information from the simulation backend to the visualization client.')

    _ElementMap.update({
        __name.name() : __name,
        __thumbnail.name() : __thumbnail,
        __description.name() : __description,
        __tags.name() : __tags,
        __timeout.name() : __timeout,
        __configuration.name() : __configuration,
        __maturity.name() : __maturity,
        __environmentModel.name() : __environmentModel,
        __visualModel.name() : __visualModel,
        __bibiConf.name() : __bibiConf,
        __experimentControl.name() : __experimentControl,
        __experimentEvaluation.name() : __experimentEvaluation,
        __cameraPose.name() : __cameraPose,
        __rosLaunch.name() : __rosLaunch,
        __rngSeed.name() : __rngSeed,
        __physicsEngine.name() : __physicsEngine,
        __gzbridgesettings.name() : __gzbridgesettings
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'ExD', ExD_)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ExperimentControl with content type ELEMENT_ONLY
class ExperimentControl (pyxb.binding.basis.complexTypeDefinition):
    """This type depicts a list of state machines"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ExperimentControl')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 315, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}stateMachine uses Python identifier stateMachine
    __stateMachine = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'stateMachine'), 'stateMachine', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExperimentControl_httpschemas_humanbrainproject_euSP102014ExDConfigstateMachine', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 320, 6), )

    
    stateMachine = property(__stateMachine.value, __stateMachine.set, None, 'The actual state machines of this list of state machines')

    _ElementMap.update({
        __stateMachine.name() : __stateMachine
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'ExperimentControl', ExperimentControl)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}GzBridgeSettings with content type ELEMENT_ONLY
class GzBridgeSettings (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}GzBridgeSettings with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'GzBridgeSettings')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 328, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}pose_update_delta_translation uses Python identifier pose_update_delta_translation
    __pose_update_delta_translation = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'pose_update_delta_translation'), 'pose_update_delta_translation', '__httpschemas_humanbrainproject_euSP102014ExDConfig_GzBridgeSettings_httpschemas_humanbrainproject_euSP102014ExDConfigpose_update_delta_translation', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 330, 12), )

    
    pose_update_delta_translation = property(__pose_update_delta_translation.value, __pose_update_delta_translation.set, None, '\n                  The magnitude of translation delta by which a pose must change for it to be relayed to the frontend.\n                ')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}pose_update_delta_rotation uses Python identifier pose_update_delta_rotation
    __pose_update_delta_rotation = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'pose_update_delta_rotation'), 'pose_update_delta_rotation', '__httpschemas_humanbrainproject_euSP102014ExDConfig_GzBridgeSettings_httpschemas_humanbrainproject_euSP102014ExDConfigpose_update_delta_rotation', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 337, 12), )

    
    pose_update_delta_rotation = property(__pose_update_delta_rotation.value, __pose_update_delta_rotation.set, None, '\n                  The angle delta by which a pose must change for it to be relayed to the frontend.\n                ')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}pose_update_early_threshold uses Python identifier pose_update_early_threshold
    __pose_update_early_threshold = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'pose_update_early_threshold'), 'pose_update_early_threshold', '__httpschemas_humanbrainproject_euSP102014ExDConfig_GzBridgeSettings_httpschemas_humanbrainproject_euSP102014ExDConfigpose_update_early_threshold', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 344, 12), )

    
    pose_update_early_threshold = property(__pose_update_early_threshold.value, __pose_update_early_threshold.set, None, '\n                  Maximal period during which larger thresholds are used rather than those defined in gzbridgesettings.\n                ')

    _ElementMap.update({
        __pose_update_delta_translation.name() : __pose_update_delta_translation,
        __pose_update_delta_rotation.name() : __pose_update_delta_rotation,
        __pose_update_early_threshold.name() : __pose_update_early_threshold
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'GzBridgeSettings', GzBridgeSettings)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine with content type EMPTY
class StateMachine (pyxb.binding.basis.complexTypeDefinition):
    """This abstract type depicts a state machine. Currently, State Machines in SMACH or SCXML are supported, though state machines in SCXML are currently ignored."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'StateMachine')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 354, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102014ExDConfig_StateMachine_id', pyxb.binding.datatypes.string, required=True)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 358, 4)
    __id._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 358, 4)
    
    id = property(__id.value, __id.set, None, 'Any state machine must have an identifier. This identifier is used to communicate with the state machine and therefore must be an identifier.')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __id.name() : __id
    })
Namespace.addCategoryObject('typeBinding', 'StateMachine', StateMachine)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}SMACHStateMachine with content type EMPTY
class SMACHStateMachine (StateMachine):
    """This type depicts a SMACH state machine. It is specified using a path to the source code of the state machine."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SMACHStateMachine')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 365, 2)
    _ElementMap = StateMachine._ElementMap.copy()
    _AttributeMap = StateMachine._AttributeMap.copy()
    # Base type is StateMachine
    
    # Attribute id inherited from {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SMACHStateMachine_src', pyxb.binding.datatypes.string, required=True)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 371, 8)
    __src._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 371, 8)
    
    src = property(__src.value, __src.set, None, 'The path to an Python script that describes the state machine. This script has to have a variable with global scope that must have the name sm or stateMachine.')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'SMACHStateMachine', SMACHStateMachine)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}SCXMLStateMachine with content type ELEMENT_ONLY
class SCXMLStateMachine (StateMachine):
    """This type denotes an SCXML state machine. SCXML is a W3C standard for state charts. However, state machines in this format are currently not run. State machines in SCXML are currently not interpreted."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SCXMLStateMachine')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 380, 2)
    _ElementMap = StateMachine._ElementMap.copy()
    _AttributeMap = StateMachine._AttributeMap.copy()
    # Base type is StateMachine
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml uses Python identifier scxml
    __scxml = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml'), 'scxml', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SCXMLStateMachine_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscxml', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 86, 1), )

    
    scxml = property(__scxml.value, __scxml.set, None, None)

    
    # Attribute id inherited from {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SCXMLStateMachine_src', pyxb.binding.datatypes.string)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 389, 8)
    __src._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 389, 8)
    
    src = property(__src.value, __src.set, None, None)

    _ElementMap.update({
        __scxml.name() : __scxml
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'SCXMLStateMachine', SCXMLStateMachine)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ConfFile with content type EMPTY
class ConfFile (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes a configuration entry. Configuration entries are used for multiple purposes, therefore the type of the configuration entry is set explicitly in an attribute called type. The actual configuration is referenced as a file through the src attribute."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ConfFile')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 64, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ConfFile_src', pyxb.binding.datatypes.string, required=True)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 68, 4)
    __src._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 68, 4)
    
    src = property(__src.value, __src.set, None, 'The path to the file that acts as configuration. Files specified as configuration are automatically considered whe an experiment is deployed.')

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ConfFile_type', ConfType, required=True)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 73, 4)
    __type._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 73, 4)
    
    type = property(__type.value, __type.set, None, 'The type of the configuration entry describes what this entry is used for. The NRP allows both predefined and custom entries.')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src,
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'ConfFile', ConfFile)


ExD = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'ExD'), ExD_, documentation='The root element of a experiment configuration model must be an ExD object.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 394, 2))
Namespace.addCategoryObject('elementBinding', ExD.name().localName(), ExD)



CameraPose._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition'), Position, scope=CameraPose, documentation='The position of the camera', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 129, 6)))

CameraPose._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt'), Position, scope=CameraPose, documentation='The position to which the camera should look at', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 134, 6)))

def _BuildAutomaton ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton
    del _BuildAutomaton
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = None
    symbol = pyxb.binding.content.ElementUse(CameraPose._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 129, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(CameraPose._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 134, 6))
    st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    transitions = []
    transitions.append(fac.Transition(st_1, [
         ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    st_1._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
CameraPose._Automaton = _BuildAutomaton()




EnvironmentModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'robotPose'), RobotPose, scope=EnvironmentModel, documentation='The position of the robot', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 148, 6)))

def _BuildAutomaton_ ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_
    del _BuildAutomaton_
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(EnvironmentModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'robotPose')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 148, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
EnvironmentModel._Automaton = _BuildAutomaton_()




VisualModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'visualPose'), RobotPose, scope=VisualModel, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 167, 6)))

def _BuildAutomaton_2 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_2
    del _BuildAutomaton_2
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(VisualModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'visualPose')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 167, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
VisualModel._Automaton = _BuildAutomaton_2()




ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'name'), pyxb.binding.datatypes.string, scope=ExD_, documentation='This element denotes the name of the experiment as it appears in the experiment list.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 219, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'thumbnail'), ThumbnailFile, scope=ExD_, documentation='This element references a path to a thumbnail that is used to give the user a forecast to the experiment.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 224, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'description'), pyxb.binding.datatypes.string, scope=ExD_, documentation='This description will appear in the experiment description and provide a short description explaining what the experiment is all about.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 229, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'tags'), Tags, scope=ExD_, documentation='List of space separated tags that describe the experiment.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 234, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'timeout'), pyxb.binding.datatypes.double, scope=ExD_, documentation='The timeout of an experiment is the time an experiment is allowed to run by default, specified in seconds. If that time has elapsed, the users are asked whether they want to extend the runtime of the simulation. On the servers, this will only be allowed if the timeout fits within the cluster allocation.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 239, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'configuration'), ConfFile, scope=ExD_, documentation='An experiment may have multiple configuration entries. Despite configuration entries can be specified in anywhere in the ExD element, they must appear together.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 244, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'maturity'), MaturityType, scope=ExD_, documentation='The maturity of an experiment determines whether it is shown by default to the user or only browsable in dev mode.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 249, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'environmentModel'), EnvironmentModel, scope=ExD_, documentation='The environment model of an experiment specifies the used world file for a simulation and the pose where the robot should be spawned.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 254, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'visualModel'), VisualModel, scope=ExD_, documentation='With the visual model, an experiment can specify an alternatively used model for the frontend visualization. This is helpful in case the robot model used in gazebo is very detailed and thus hard to visualize on the client. On the server, there may be more resources available to simulate more complex models.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 259, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'bibiConf'), BibiConf, scope=ExD_, documentation='The bibiConf element of an experiment configuration specifies the ', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 264, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'experimentControl'), ExperimentControl, scope=ExD_, documentation='The experiment control lists all state machines that control the experiment.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 269, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation'), ExperimentControl, scope=ExD_, documentation='The experiment evaluation element lists all state machines that evaluate the success of a simulated experiment.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 278, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraPose'), CameraPose, scope=ExD_, documentation='The camera pose specifies the initial position of the camera when a simulation is started.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 287, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'rosLaunch'), RosLaunch, scope=ExD_, documentation='The roslaunch element species the path to a ROSLaunch file that is executed when the experiment is simulated. If no file is specified, no ROSLaunch file is executed at the beginning of an experiment.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 292, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'rngSeed'), pyxb.binding.datatypes.positiveInteger, scope=ExD_, documentation='If specified, this element specifies the random number generator seed. If this field is left blank, a seed is generated and therefore, the simulation is not 100% deterministic. If a seed is specified here, this seed is used for the robot and neural simulation, making the simulation much more deterministic.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 297, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'physicsEngine'), PhysicsEngine, scope=ExD_, documentation='If specified, this element denotes the physics simulator that should be used. We currently support either ODE or OpenSim.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 302, 6)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'gzbridgesettings'), GzBridgeSettings, scope=ExD_, documentation='Settings for the relay of the component relaying information from the simulation backend to the visualization client.', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 307, 6)))

def _BuildAutomaton_3 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_3
    del _BuildAutomaton_3
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 234, 6))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 239, 6))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 244, 6))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 249, 6))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 259, 6))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 269, 6))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 278, 6))
    counters.add(cc_6)
    cc_7 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 287, 6))
    counters.add(cc_7)
    cc_8 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 292, 6))
    counters.add(cc_8)
    cc_9 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 297, 6))
    counters.add(cc_9)
    cc_10 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 302, 6))
    counters.add(cc_10)
    cc_11 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 307, 6))
    counters.add(cc_11)
    states = []
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'name')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 219, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'thumbnail')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 224, 6))
    st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'description')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 229, 6))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'tags')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 234, 6))
    st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'timeout')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 239, 6))
    st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'configuration')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 244, 6))
    st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'maturity')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 249, 6))
    st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'environmentModel')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 254, 6))
    st_7 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'visualModel')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 259, 6))
    st_8 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'bibiConf')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 264, 6))
    st_9 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_9)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'experimentControl')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 269, 6))
    st_10 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_10)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 278, 6))
    st_11 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_11)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraPose')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 287, 6))
    st_12 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_12)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'rosLaunch')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 292, 6))
    st_13 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_13)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_9, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'rngSeed')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 297, 6))
    st_14 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_14)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_10, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'physicsEngine')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 302, 6))
    st_15 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_15)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_11, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'gzbridgesettings')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 307, 6))
    st_16 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_16)
    transitions = []
    transitions.append(fac.Transition(st_1, [
         ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_2, [
         ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
         ]))
    transitions.append(fac.Transition(st_4, [
         ]))
    transitions.append(fac.Transition(st_5, [
         ]))
    transitions.append(fac.Transition(st_6, [
         ]))
    transitions.append(fac.Transition(st_7, [
         ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_3, False) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_8, [
         ]))
    transitions.append(fac.Transition(st_9, [
         ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_4, False) ]))
    st_8._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_10, [
         ]))
    transitions.append(fac.Transition(st_11, [
         ]))
    transitions.append(fac.Transition(st_12, [
         ]))
    transitions.append(fac.Transition(st_13, [
         ]))
    transitions.append(fac.Transition(st_14, [
         ]))
    transitions.append(fac.Transition(st_15, [
         ]))
    transitions.append(fac.Transition(st_16, [
         ]))
    st_9._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_5, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_5, False) ]))
    st_10._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_6, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_6, False) ]))
    st_11._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_7, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_7, False) ]))
    st_12._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_8, False) ]))
    st_13._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_9, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_9, False) ]))
    st_14._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_10, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_10, False) ]))
    st_15._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_11, True) ]))
    st_16._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
ExD_._Automaton = _BuildAutomaton_3()




ExperimentControl._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'stateMachine'), StateMachine, scope=ExperimentControl, documentation='The actual state machines of this list of state machines', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 320, 6)))

def _BuildAutomaton_4 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_4
    del _BuildAutomaton_4
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(ExperimentControl._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'stateMachine')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 320, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
         ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
ExperimentControl._Automaton = _BuildAutomaton_4()




GzBridgeSettings._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'pose_update_delta_translation'), pyxb.binding.datatypes.float, scope=GzBridgeSettings, documentation='\n                  The magnitude of translation delta by which a pose must change for it to be relayed to the frontend.\n                ', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 330, 12)))

GzBridgeSettings._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'pose_update_delta_rotation'), pyxb.binding.datatypes.float, scope=GzBridgeSettings, documentation='\n                  The angle delta by which a pose must change for it to be relayed to the frontend.\n                ', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 337, 12)))

GzBridgeSettings._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'pose_update_early_threshold'), pyxb.binding.datatypes.float, scope=GzBridgeSettings, documentation='\n                  Maximal period during which larger thresholds are used rather than those defined in gzbridgesettings.\n                ', location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 344, 12)))

def _BuildAutomaton_5 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_5
    del _BuildAutomaton_5
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 330, 12))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 337, 12))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 344, 12))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(GzBridgeSettings._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'pose_update_delta_translation')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 330, 12))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(GzBridgeSettings._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'pose_update_delta_rotation')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 337, 12))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(GzBridgeSettings._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'pose_update_early_threshold')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 344, 12))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_2, True) ]))
    st_2._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
GzBridgeSettings._Automaton = _BuildAutomaton_5()




SCXMLStateMachine._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml'), _ImportedBinding__sc.scxml_scxml_type, scope=SCXMLStateMachine, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 86, 1)))

def _BuildAutomaton_6 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_6
    del _BuildAutomaton_6
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 387, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(SCXMLStateMachine._UseForTag(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Experiments/ExDConfFile.xsd', 387, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
SCXMLStateMachine._Automaton = _BuildAutomaton_6()
