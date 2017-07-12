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
# ./binding.py
# -*- coding: utf-8 -*-
# PyXB bindings for NM:e455c55bde4037492ee2ed5fe3b59ed8c44bb0d3
# Generated 2017-06-14 13:45:12.523856 by PyXB version 1.2.4 using Python 2.7.12.final.0
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
_GenerationUID = pyxb.utils.utility.UniqueIdentifier('urn:uuid:ecbf16d0-50f6-11e7-bf13-704d7b2cc7b3')

# Version of PyXB used to generate the bindings
_PyXBVersion = '1.2.4'
# Generated bindings are not compatible across PyXB versions
if pyxb.__version__ != _PyXBVersion:
    raise pyxb.PyXBVersionError(_PyXBVersion)

# Import bindings for namespaces imported into schema
import _sc as _ImportedBinding__sc
import pyxb.binding.datatypes

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

    """An atomic simple type."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ThumbnailFile')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 18, 4)
    _Documentation = None
ThumbnailFile._CF_pattern = pyxb.binding.facets.CF_pattern()
ThumbnailFile._CF_pattern.addPattern(pattern='[a-zA-Z0-9\\._\\-/]*\\.(png|gif|jp[e]?g)')
ThumbnailFile._InitializeFacetMap(ThumbnailFile._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'ThumbnailFile', ThumbnailFile)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}MaturityType
class MaturityType (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """An atomic simple type."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'MaturityType')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 24, 4)
    _Documentation = None
MaturityType._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=MaturityType, enum_prefix=None)
MaturityType.development = MaturityType._CF_enumeration.addEnumeration(unicode_value='development', tag='development')
MaturityType.production = MaturityType._CF_enumeration.addEnumeration(unicode_value='production', tag='production')
MaturityType._InitializeFacetMap(MaturityType._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'MaturityType', MaturityType)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}Conf_Type_Enumeration
class Conf_Type_Enumeration (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """An atomic simple type."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Conf_Type_Enumeration')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 54, 4)
    _Documentation = None
Conf_Type_Enumeration._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=Conf_Type_Enumeration, enum_prefix=None)
Conf_Type_Enumeration.n3d_settings = Conf_Type_Enumeration._CF_enumeration.addEnumeration(unicode_value='3d-settings', tag='n3d_settings')
Conf_Type_Enumeration._InitializeFacetMap(Conf_Type_Enumeration._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'Conf_Type_Enumeration', Conf_Type_Enumeration)

# Union simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}Conf_Type
# superclasses pyxb.binding.datatypes.anySimpleType
class Conf_Type (pyxb.binding.basis.STD_union):

    """Simple type that is a union of Conf_Type_Enumeration, pyxb.binding.datatypes.string."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Conf_Type')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 50, 4)
    _Documentation = None

    _MemberTypes = ( Conf_Type_Enumeration, pyxb.binding.datatypes.string, )
Conf_Type._CF_pattern = pyxb.binding.facets.CF_pattern()
Conf_Type._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=Conf_Type)
Conf_Type.n3d_settings = '3d-settings'            # originally Conf_Type_Enumeration.n3d_settings
Conf_Type._InitializeFacetMap(Conf_Type._CF_pattern,
   Conf_Type._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'Conf_Type', Conf_Type)

# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}RobotPose with content type EMPTY
class RobotPose (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}RobotPose with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RobotPose')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 41, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute x uses Python identifier x
    __x = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'x'), 'x', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_x', pyxb.binding.datatypes.double)
    __x._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 33, 8)
    __x._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 33, 8)
    
    x = property(__x.value, __x.set, None, None)

    
    # Attribute y uses Python identifier y
    __y = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'y'), 'y', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_y', pyxb.binding.datatypes.double)
    __y._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 34, 8)
    __y._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 34, 8)
    
    y = property(__y.value, __y.set, None, None)

    
    # Attribute z uses Python identifier z
    __z = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'z'), 'z', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_z', pyxb.binding.datatypes.double)
    __z._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 35, 8)
    __z._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 35, 8)
    
    z = property(__z.value, __z.set, None, None)

    
    # Attribute ux uses Python identifier ux
    __ux = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'ux'), 'ux', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_ux', pyxb.binding.datatypes.double)
    __ux._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 36, 8)
    __ux._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 36, 8)
    
    ux = property(__ux.value, __ux.set, None, None)

    
    # Attribute uy uses Python identifier uy
    __uy = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'uy'), 'uy', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_uy', pyxb.binding.datatypes.double)
    __uy._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 37, 8)
    __uy._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 37, 8)
    
    uy = property(__uy.value, __uy.set, None, None)

    
    # Attribute uz uses Python identifier uz
    __uz = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'uz'), 'uz', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_uz', pyxb.binding.datatypes.double)
    __uz._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 38, 8)
    __uz._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 38, 8)
    
    uz = property(__uz.value, __uz.set, None, None)

    
    # Attribute theta uses Python identifier theta
    __theta = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'theta'), 'theta', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_theta', pyxb.binding.datatypes.double)
    __theta._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 39, 8)
    __theta._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 39, 8)
    
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
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}Position with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Position')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 66, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute x uses Python identifier x
    __x = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'x'), 'x', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_x', pyxb.binding.datatypes.double)
    __x._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 62, 8)
    __x._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 62, 8)
    
    x = property(__x.value, __x.set, None, None)

    
    # Attribute y uses Python identifier y
    __y = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'y'), 'y', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_y', pyxb.binding.datatypes.double)
    __y._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 63, 8)
    __y._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 63, 8)
    
    y = property(__y.value, __y.set, None, None)

    
    # Attribute z uses Python identifier z
    __z = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'z'), 'z', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_z', pyxb.binding.datatypes.double)
    __z._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 64, 8)
    __z._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 64, 8)
    
    z = property(__z.value, __z.set, None, None)

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
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}CameraPose with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'CameraPose')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 71, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraPosition uses Python identifier cameraPosition
    __cameraPosition = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition'), 'cameraPosition', '__httpschemas_humanbrainproject_euSP102014ExDConfig_CameraPose_httpschemas_humanbrainproject_euSP102014ExDConfigcameraPosition', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 73, 12), )

    
    cameraPosition = property(__cameraPosition.value, __cameraPosition.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraLookAt uses Python identifier cameraLookAt
    __cameraLookAt = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt'), 'cameraLookAt', '__httpschemas_humanbrainproject_euSP102014ExDConfig_CameraPose_httpschemas_humanbrainproject_euSP102014ExDConfigcameraLookAt', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 74, 12), )

    
    cameraLookAt = property(__cameraLookAt.value, __cameraLookAt.set, None, None)

    _ElementMap.update({
        __cameraPosition.name() : __cameraPosition,
        __cameraLookAt.name() : __cameraLookAt
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'CameraPose', CameraPose)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}EnvironmentModel with content type ELEMENT_ONLY
class EnvironmentModel (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}EnvironmentModel with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'EnvironmentModel')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 81, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}robotPose uses Python identifier robotPose
    __robotPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'robotPose'), 'robotPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_EnvironmentModel_httpschemas_humanbrainproject_euSP102014ExDConfigrobotPose', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 83, 12), )

    
    robotPose = property(__robotPose.value, __robotPose.set, None, None)

    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_EnvironmentModel_src', pyxb.binding.datatypes.string)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 85, 8)
    __src._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 85, 8)
    
    src = property(__src.value, __src.set, None, None)

    _ElementMap.update({
        __robotPose.name() : __robotPose
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'EnvironmentModel', EnvironmentModel)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}VisualModel with content type ELEMENT_ONLY
class VisualModel (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}VisualModel with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'VisualModel')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 89, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}visualPose uses Python identifier visualPose
    __visualPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'visualPose'), 'visualPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_httpschemas_humanbrainproject_euSP102014ExDConfigvisualPose', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 91, 12), )

    
    visualPose = property(__visualPose.value, __visualPose.set, None, None)

    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_src', pyxb.binding.datatypes.string)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 93, 8)
    __src._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 93, 8)
    
    src = property(__src.value, __src.set, None, None)

    
    # Attribute scale uses Python identifier scale
    __scale = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'scale'), 'scale', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_scale', pyxb.binding.datatypes.double)
    __scale._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 94, 8)
    __scale._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 94, 8)
    
    scale = property(__scale.value, __scale.set, None, None)

    _ElementMap.update({
        __visualPose.name() : __visualPose
    })
    _AttributeMap.update({
        __src.name() : __src,
        __scale.name() : __scale
    })
Namespace.addCategoryObject('typeBinding', 'VisualModel', VisualModel)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}bibiConf with content type EMPTY
class bibiConf (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}bibiConf with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'bibiConf')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 98, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_bibiConf_src', pyxb.binding.datatypes.string)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 99, 8)
    __src._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 99, 8)
    
    src = property(__src.value, __src.set, None, None)

    
    # Attribute processes uses Python identifier processes
    __processes = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'processes'), 'processes', '__httpschemas_humanbrainproject_euSP102014ExDConfig_bibiConf_processes', pyxb.binding.datatypes.positiveInteger, unicode_default='1')
    __processes._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 100, 8)
    __processes._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 100, 8)
    
    processes = property(__processes.value, __processes.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src,
        __processes.name() : __processes
    })
Namespace.addCategoryObject('typeBinding', 'bibiConf', bibiConf)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}RosLaunch with content type EMPTY
class RosLaunch (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}RosLaunch with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RosLaunch')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 104, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RosLaunch_src', pyxb.binding.datatypes.string)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 105, 8)
    __src._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 105, 8)
    
    src = property(__src.value, __src.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'RosLaunch', RosLaunch)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ExD with content type ELEMENT_ONLY
class ExD_ (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ExD with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ExD')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 109, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}name uses Python identifier name
    __name = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigname', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 111, 12), )

    
    name = property(__name.value, __name.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}thumbnail uses Python identifier thumbnail
    __thumbnail = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'thumbnail'), 'thumbnail', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigthumbnail', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 112, 12), )

    
    thumbnail = property(__thumbnail.value, __thumbnail.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}description uses Python identifier description
    __description = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'description'), 'description', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigdescription', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 113, 12), )

    
    description = property(__description.value, __description.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}timeout uses Python identifier timeout
    __timeout = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'timeout'), 'timeout', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigtimeout', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 114, 12), )

    
    timeout = property(__timeout.value, __timeout.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}configuration uses Python identifier configuration
    __configuration = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'configuration'), 'configuration', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigconfiguration', True, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 115, 12), )

    
    configuration = property(__configuration.value, __configuration.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}maturity uses Python identifier maturity
    __maturity = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'maturity'), 'maturity', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigmaturity', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 116, 12), )

    
    maturity = property(__maturity.value, __maturity.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}environmentModel uses Python identifier environmentModel
    __environmentModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'environmentModel'), 'environmentModel', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigenvironmentModel', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 117, 12), )

    
    environmentModel = property(__environmentModel.value, __environmentModel.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}visualModel uses Python identifier visualModel
    __visualModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'visualModel'), 'visualModel', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigvisualModel', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 118, 12), )

    
    visualModel = property(__visualModel.value, __visualModel.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}bibiConf uses Python identifier bibiConf
    __bibiConf = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'bibiConf'), 'bibiConf', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigbibiConf', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 119, 12), )

    
    bibiConf = property(__bibiConf.value, __bibiConf.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}experimentControl uses Python identifier experimentControl
    __experimentControl = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'experimentControl'), 'experimentControl', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigexperimentControl', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 120, 12), )

    
    experimentControl = property(__experimentControl.value, __experimentControl.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}experimentEvaluation uses Python identifier experimentEvaluation
    __experimentEvaluation = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation'), 'experimentEvaluation', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigexperimentEvaluation', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 126, 12), )

    
    experimentEvaluation = property(__experimentEvaluation.value, __experimentEvaluation.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraPose uses Python identifier cameraPose
    __cameraPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraPose'), 'cameraPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigcameraPose', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 132, 12), )

    
    cameraPose = property(__cameraPose.value, __cameraPose.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}rosLaunch uses Python identifier rosLaunch
    __rosLaunch = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'rosLaunch'), 'rosLaunch', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigrosLaunch', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 133, 12), )

    
    rosLaunch = property(__rosLaunch.value, __rosLaunch.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}rngSeed uses Python identifier rngSeed
    __rngSeed = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'rngSeed'), 'rngSeed', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigrngSeed', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 134, 12), )

    
    rngSeed = property(__rngSeed.value, __rngSeed.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}physicsEngine uses Python identifier physicsEngine
    __physicsEngine = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'physicsEngine'), 'physicsEngine', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigphysicsEngine', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 136, 12), )

    
    physicsEngine = property(__physicsEngine.value, __physicsEngine.set, None, None)

    _ElementMap.update({
        __name.name() : __name,
        __thumbnail.name() : __thumbnail,
        __description.name() : __description,
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
        __physicsEngine.name() : __physicsEngine
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'ExD', ExD_)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ExperimentControl with content type ELEMENT_ONLY
class ExperimentControl (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ExperimentControl with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ExperimentControl')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 145, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}stateMachine uses Python identifier stateMachine
    __stateMachine = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'stateMachine'), 'stateMachine', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExperimentControl_httpschemas_humanbrainproject_euSP102014ExDConfigstateMachine', True, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 142, 12), )

    
    stateMachine = property(__stateMachine.value, __stateMachine.set, None, None)

    _ElementMap.update({
        __stateMachine.name() : __stateMachine
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'ExperimentControl', ExperimentControl)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine with content type EMPTY
class StateMachine (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'StateMachine')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 149, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102014ExDConfig_StateMachine_id', pyxb.binding.datatypes.string, required=True)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 150, 8)
    __id._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 150, 8)
    
    id = property(__id.value, __id.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __id.name() : __id
    })
Namespace.addCategoryObject('typeBinding', 'StateMachine', StateMachine)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}SMACHStateMachine with content type EMPTY
class SMACHStateMachine (StateMachine):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}SMACHStateMachine with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SMACHStateMachine')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 153, 4)
    _ElementMap = StateMachine._ElementMap.copy()
    _AttributeMap = StateMachine._AttributeMap.copy()
    # Base type is StateMachine
    
    # Attribute id inherited from {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SMACHStateMachine_src', pyxb.binding.datatypes.string)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 156, 16)
    __src._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 156, 16)
    
    src = property(__src.value, __src.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'SMACHStateMachine', SMACHStateMachine)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}SCXMLStateMachine with content type ELEMENT_ONLY
class SCXMLStateMachine (StateMachine):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}SCXMLStateMachine with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SCXMLStateMachine')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 161, 4)
    _ElementMap = StateMachine._ElementMap.copy()
    _AttributeMap = StateMachine._AttributeMap.copy()
    # Base type is StateMachine
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml uses Python identifier scxml
    __scxml = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml'), 'scxml', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SCXMLStateMachine_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscxml', False, pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 86, 1), )

    
    scxml = property(__scxml.value, __scxml.set, None, None)

    
    # Attribute id inherited from {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SCXMLStateMachine_src', pyxb.binding.datatypes.string)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 167, 14)
    __src._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 167, 14)
    
    src = property(__src.value, __src.set, None, None)

    _ElementMap.update({
        __scxml.name() : __scxml
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'SCXMLStateMachine', SCXMLStateMachine)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}Conf_File with content type EMPTY
class Conf_File (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}Conf_File with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Conf_File')
    _XSDLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 45, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Conf_File_src', pyxb.binding.datatypes.string)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 46, 8)
    __src._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 46, 8)
    
    src = property(__src.value, __src.set, None, None)

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Conf_File_type', Conf_Type)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 47, 8)
    __type._UseLocation = pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 47, 8)
    
    type = property(__type.value, __type.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src,
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'Conf_File', Conf_File)


ExD = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'ExD'), ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 173, 4))
Namespace.addCategoryObject('elementBinding', ExD.name().localName(), ExD)



CameraPose._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition'), Position, scope=CameraPose, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 73, 12)))

CameraPose._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt'), Position, scope=CameraPose, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 74, 12)))

def _BuildAutomaton ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton
    del _BuildAutomaton
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = None
    symbol = pyxb.binding.content.ElementUse(CameraPose._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 73, 12))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(CameraPose._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 74, 12))
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




EnvironmentModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'robotPose'), RobotPose, scope=EnvironmentModel, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 83, 12)))

def _BuildAutomaton_ ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_
    del _BuildAutomaton_
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(EnvironmentModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'robotPose')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 83, 12))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
EnvironmentModel._Automaton = _BuildAutomaton_()




VisualModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'visualPose'), RobotPose, scope=VisualModel, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 91, 12)))

def _BuildAutomaton_2 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_2
    del _BuildAutomaton_2
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(VisualModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'visualPose')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 91, 12))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
VisualModel._Automaton = _BuildAutomaton_2()




ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'name'), pyxb.binding.datatypes.string, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 111, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'thumbnail'), ThumbnailFile, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 112, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'description'), pyxb.binding.datatypes.string, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 113, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'timeout'), pyxb.binding.datatypes.double, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 114, 12), unicode_default='5'))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'configuration'), Conf_File, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 115, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'maturity'), MaturityType, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 116, 12), unicode_default='development'))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'environmentModel'), EnvironmentModel, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 117, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'visualModel'), VisualModel, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 118, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'bibiConf'), bibiConf, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 119, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'experimentControl'), ExperimentControl, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 120, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation'), ExperimentControl, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 126, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraPose'), CameraPose, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 132, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'rosLaunch'), RosLaunch, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 133, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'rngSeed'), pyxb.binding.datatypes.positiveInteger, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 134, 12)))

ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'physicsEngine'), pyxb.binding.datatypes.string, scope=ExD_, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 136, 12), unicode_default='ode'))

def _BuildAutomaton_3 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_3
    del _BuildAutomaton_3
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 114, 12))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 115, 12))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 116, 12))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 118, 12))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 120, 12))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 126, 12))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 132, 12))
    counters.add(cc_6)
    cc_7 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 133, 12))
    counters.add(cc_7)
    cc_8 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 134, 12))
    counters.add(cc_8)
    cc_9 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 136, 12))
    counters.add(cc_9)
    states = []
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'name')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 111, 12))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'thumbnail')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 112, 12))
    st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'description')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 113, 12))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'timeout')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 114, 12))
    st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'configuration')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 115, 12))
    st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'maturity')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 116, 12))
    st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'environmentModel')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 117, 12))
    st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'visualModel')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 118, 12))
    st_7 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'bibiConf')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 119, 12))
    st_8 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'experimentControl')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 120, 12))
    st_9 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_9)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 126, 12))
    st_10 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_10)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraPose')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 132, 12))
    st_11 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_11)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'rosLaunch')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 133, 12))
    st_12 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_12)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'rngSeed')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 134, 12))
    st_13 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_13)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_9, False))
    symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'physicsEngine')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 136, 12))
    st_14 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_14)
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
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_7, [
         ]))
    transitions.append(fac.Transition(st_8, [
         ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_3, False) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
         ]))
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
    st_8._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, False) ]))
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
    st_11._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_7, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_7, False) ]))
    st_12._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_8, False) ]))
    st_13._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_9, True) ]))
    st_14._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
ExD_._Automaton = _BuildAutomaton_3()




ExperimentControl._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'stateMachine'), StateMachine, scope=ExperimentControl, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 142, 12)))

def _BuildAutomaton_4 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_4
    del _BuildAutomaton_4
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 146, 8))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(ExperimentControl._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'stateMachine')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 142, 12))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
         ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
ExperimentControl._Automaton = _BuildAutomaton_4()




SCXMLStateMachine._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml'), _ImportedBinding__sc.scxml_scxml_type, scope=SCXMLStateMachine, location=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 86, 1)))

def _BuildAutomaton_5 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_5
    del _BuildAutomaton_5
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 165, 16))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(SCXMLStateMachine._UseForTag(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml')), pyxb.utils.utility.Location('/home/faichele/src/HBP/Experiments/ExDConfFile.xsd', 165, 16))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
SCXMLStateMachine._Automaton = _BuildAutomaton_5()

