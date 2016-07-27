# ./test.py
# -*- coding: utf-8 -*-
# PyXB bindings for NM:e455c55bde4037492ee2ed5fe3b59ed8c44bb0d3
# Generated 2016-07-26 16:42:34.801908 by PyXB version 1.2.4 using Python 2.7.6.final.0
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
_GenerationUID = pyxb.utils.utility.UniqueIdentifier('urn:uuid:30b5956e-533f-11e6-9fff-847beb4693fd')

# Version of PyXB used to generate the bindings
_PyXBVersion = '1.2.4'
# Generated bindings are not compatible across PyXB versions
if pyxb.__version__ != _PyXBVersion:
    raise pyxb.PyXBVersionError(_PyXBVersion)

# Import bindings for namespaces imported into schema
import pyxb.binding.datatypes
import hbp_scxml_gen as _ImportedBinding__sc

# todo: remove hack: check if namespace already exists
ns = pyxb.namespace.NamespaceForURI('http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig',
                                      create_if_missing=False)

if ns is not None:
    print "Namespace: for bibi already exists."
    from hbp_nrp_backend.exd_config.generated.exp_conf_api_gen import *
    pass

else:
    # NOTE: All namespace declarations are reserved within the binding
    Namespace = pyxb.namespace.NamespaceForURI(
        'http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig', create_if_missing=True)
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


    # Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}MaturityType
    class MaturityType (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

        """An atomic simple type."""

        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'MaturityType')
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 18, 4)
        _Documentation = None
    MaturityType._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=MaturityType, enum_prefix=None)
    MaturityType.development = MaturityType._CF_enumeration.addEnumeration(unicode_value='development', tag='development')
    MaturityType.production = MaturityType._CF_enumeration.addEnumeration(unicode_value='production', tag='production')
    MaturityType._InitializeFacetMap(MaturityType._CF_enumeration)
    Namespace.addCategoryObject('typeBinding', 'MaturityType', MaturityType)

    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}RobotPose with content type EMPTY
    class RobotPose (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}RobotPose with content type EMPTY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RobotPose')
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 35, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute x uses Python identifier x
        __x = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'x'), 'x', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_x', pyxb.binding.datatypes.double)
        __x._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 27, 8)
        __x._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 27, 8)

        x = property(__x.value, __x.set, None, None)


        # Attribute y uses Python identifier y
        __y = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'y'), 'y', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_y', pyxb.binding.datatypes.double)
        __y._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 28, 8)
        __y._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 28, 8)

        y = property(__y.value, __y.set, None, None)


        # Attribute z uses Python identifier z
        __z = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'z'), 'z', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_z', pyxb.binding.datatypes.double)
        __z._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 29, 8)
        __z._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 29, 8)

        z = property(__z.value, __z.set, None, None)


        # Attribute ux uses Python identifier ux
        __ux = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'ux'), 'ux', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_ux', pyxb.binding.datatypes.double)
        __ux._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 30, 8)
        __ux._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 30, 8)

        ux = property(__ux.value, __ux.set, None, None)


        # Attribute uy uses Python identifier uy
        __uy = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'uy'), 'uy', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_uy', pyxb.binding.datatypes.double)
        __uy._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 31, 8)
        __uy._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 31, 8)

        uy = property(__uy.value, __uy.set, None, None)


        # Attribute uz uses Python identifier uz
        __uz = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'uz'), 'uz', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_uz', pyxb.binding.datatypes.double)
        __uz._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 32, 8)
        __uz._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 32, 8)

        uz = property(__uz.value, __uz.set, None, None)


        # Attribute theta uses Python identifier theta
        __theta = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'theta'), 'theta', '__httpschemas_humanbrainproject_euSP102014ExDConfig_RobotPose_theta', pyxb.binding.datatypes.double)
        __theta._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 33, 8)
        __theta._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 33, 8)

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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 46, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute x uses Python identifier x
        __x = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'x'), 'x', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_x', pyxb.binding.datatypes.double)
        __x._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 42, 8)
        __x._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 42, 8)

        x = property(__x.value, __x.set, None, None)


        # Attribute y uses Python identifier y
        __y = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'y'), 'y', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_y', pyxb.binding.datatypes.double)
        __y._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 43, 8)
        __y._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 43, 8)

        y = property(__y.value, __y.set, None, None)


        # Attribute z uses Python identifier z
        __z = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'z'), 'z', '__httpschemas_humanbrainproject_euSP102014ExDConfig_Position_z', pyxb.binding.datatypes.double)
        __z._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 44, 8)
        __z._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 44, 8)

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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 51, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraPosition uses Python identifier cameraPosition
        __cameraPosition = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition'), 'cameraPosition', '__httpschemas_humanbrainproject_euSP102014ExDConfig_CameraPose_httpschemas_humanbrainproject_euSP102014ExDConfigcameraPosition', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 53, 12), )


        cameraPosition = property(__cameraPosition.value, __cameraPosition.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraLookAt uses Python identifier cameraLookAt
        __cameraLookAt = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt'), 'cameraLookAt', '__httpschemas_humanbrainproject_euSP102014ExDConfig_CameraPose_httpschemas_humanbrainproject_euSP102014ExDConfigcameraLookAt', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 54, 12), )


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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 61, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}robotPose uses Python identifier robotPose
        __robotPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'robotPose'), 'robotPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_EnvironmentModel_httpschemas_humanbrainproject_euSP102014ExDConfigrobotPose', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 63, 12), )


        robotPose = property(__robotPose.value, __robotPose.set, None, None)


        # Attribute src uses Python identifier src
        __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_EnvironmentModel_src', pyxb.binding.datatypes.string)
        __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 65, 8)
        __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 65, 8)

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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 69, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}visualPose uses Python identifier visualPose
        __visualPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'visualPose'), 'visualPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_httpschemas_humanbrainproject_euSP102014ExDConfigvisualPose', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 71, 12), )


        visualPose = property(__visualPose.value, __visualPose.set, None, None)


        # Attribute src uses Python identifier src
        __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_src', pyxb.binding.datatypes.string)
        __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 73, 8)
        __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 73, 8)

        src = property(__src.value, __src.set, None, None)


        # Attribute scale uses Python identifier scale
        __scale = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'scale'), 'scale', '__httpschemas_humanbrainproject_euSP102014ExDConfig_VisualModel_scale', pyxb.binding.datatypes.double)
        __scale._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 74, 8)
        __scale._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 74, 8)

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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 77, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute src uses Python identifier src
        __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_bibiConf_src', pyxb.binding.datatypes.string)
        __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 78, 8)
        __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 78, 8)

        src = property(__src.value, __src.set, None, None)

        _ElementMap.update({

        })
        _AttributeMap.update({
            __src.name() : __src
        })
    Namespace.addCategoryObject('typeBinding', 'bibiConf', bibiConf)


    # Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ExD with content type ELEMENT_ONLY
    class ExD_ (pyxb.binding.basis.complexTypeDefinition):
        """Complex type {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}ExD with content type ELEMENT_ONLY"""
        _TypeDefinition = None
        _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
        _Abstract = False
        _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ExD')
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 82, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}name uses Python identifier name
        __name = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigname', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 84, 12), )


        name = property(__name.value, __name.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}description uses Python identifier description
        __description = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'description'), 'description', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigdescription', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 85, 12), )


        description = property(__description.value, __description.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}timeout uses Python identifier timeout
        __timeout = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'timeout'), 'timeout', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigtimeout', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 86, 12), )


        timeout = property(__timeout.value, __timeout.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}maturity uses Python identifier maturity
        __maturity = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'maturity'), 'maturity', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigmaturity', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 87, 12), )


        maturity = property(__maturity.value, __maturity.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}environmentModel uses Python identifier environmentModel
        __environmentModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'environmentModel'), 'environmentModel', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigenvironmentModel', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 88, 12), )


        environmentModel = property(__environmentModel.value, __environmentModel.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}visualModel uses Python identifier visualModel
        __visualModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'visualModel'), 'visualModel', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigvisualModel', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 89, 12), )


        visualModel = property(__visualModel.value, __visualModel.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}bibiConf uses Python identifier bibiConf
        __bibiConf = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'bibiConf'), 'bibiConf', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigbibiConf', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 90, 12), )


        bibiConf = property(__bibiConf.value, __bibiConf.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}experimentControl uses Python identifier experimentControl
        __experimentControl = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'experimentControl'), 'experimentControl', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigexperimentControl', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 91, 12), )


        experimentControl = property(__experimentControl.value, __experimentControl.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}experimentEvaluation uses Python identifier experimentEvaluation
        __experimentEvaluation = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation'), 'experimentEvaluation', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigexperimentEvaluation', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 97, 12), )


        experimentEvaluation = property(__experimentEvaluation.value, __experimentEvaluation.set, None, None)


        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}cameraPose uses Python identifier cameraPose
        __cameraPose = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cameraPose'), 'cameraPose', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExD__httpschemas_humanbrainproject_euSP102014ExDConfigcameraPose', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 103, 12), )


        cameraPose = property(__cameraPose.value, __cameraPose.set, None, None)

        _ElementMap.update({
            __name.name() : __name,
            __description.name() : __description,
            __timeout.name() : __timeout,
            __maturity.name() : __maturity,
            __environmentModel.name() : __environmentModel,
            __visualModel.name() : __visualModel,
            __bibiConf.name() : __bibiConf,
            __experimentControl.name() : __experimentControl,
            __experimentEvaluation.name() : __experimentEvaluation,
            __cameraPose.name() : __cameraPose
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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 112, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Element {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}stateMachine uses Python identifier stateMachine
        __stateMachine = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'stateMachine'), 'stateMachine', '__httpschemas_humanbrainproject_euSP102014ExDConfig_ExperimentControl_httpschemas_humanbrainproject_euSP102014ExDConfigstateMachine', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 109, 12), )


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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 116, 4)
        _ElementMap = {}
        _AttributeMap = {}
        # Base type is pyxb.binding.datatypes.anyType

        # Attribute id uses Python identifier id
        __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102014ExDConfig_StateMachine_id', pyxb.binding.datatypes.string, required=True)
        __id._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 117, 8)
        __id._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 117, 8)

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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 120, 4)
        _ElementMap = StateMachine._ElementMap.copy()
        _AttributeMap = StateMachine._AttributeMap.copy()
        # Base type is StateMachine

        # Attribute id inherited from {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine

        # Attribute src uses Python identifier src
        __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SMACHStateMachine_src', pyxb.binding.datatypes.string)
        __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 123, 16)
        __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 123, 16)

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
        _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 128, 4)
        _ElementMap = StateMachine._ElementMap.copy()
        _AttributeMap = StateMachine._AttributeMap.copy()
        # Base type is StateMachine

        # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml uses Python identifier scxml
        __scxml = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml'), 'scxml', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SCXMLStateMachine_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscxml', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/StateMachines/hbp-scxml/hbp-scxml-module-core.xsd', 86, 1), )


        scxml = property(__scxml.value, __scxml.set, None, None)


        # Attribute id inherited from {http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}StateMachine

        # Attribute src uses Python identifier src
        __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014ExDConfig_SCXMLStateMachine_src', pyxb.binding.datatypes.string)
        __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 134, 4)
        __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 134, 4)

        src = property(__src.value, __src.set, None, None)

        _ElementMap.update({
            __scxml.name() : __scxml
        })
        _AttributeMap.update({
            __src.name() : __src
        })
    Namespace.addCategoryObject('typeBinding', 'SCXMLStateMachine', SCXMLStateMachine)


    ExD = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'ExD'), ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 140, 4))
    Namespace.addCategoryObject('elementBinding', ExD.name().localName(), ExD)



    CameraPose._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition'), Position, scope=CameraPose, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 53, 12)))

    CameraPose._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt'), Position, scope=CameraPose, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 54, 12)))

    def _BuildAutomaton ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton
        del _BuildAutomaton
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = None
        symbol = pyxb.binding.content.ElementUse(CameraPose._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraPosition')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 53, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(CameraPose._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraLookAt')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 54, 12))
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




    EnvironmentModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'robotPose'), RobotPose, scope=EnvironmentModel, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 63, 12)))

    def _BuildAutomaton_ ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_
        del _BuildAutomaton_
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(EnvironmentModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'robotPose')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 63, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    EnvironmentModel._Automaton = _BuildAutomaton_()




    VisualModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'visualPose'), RobotPose, scope=VisualModel, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 71, 12)))

    def _BuildAutomaton_2 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_2
        del _BuildAutomaton_2
        import pyxb.utils.fac as fac

        counters = set()
        states = []
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(VisualModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'visualPose')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 71, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    VisualModel._Automaton = _BuildAutomaton_2()




    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'name'), pyxb.binding.datatypes.string, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 84, 12)))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'description'), pyxb.binding.datatypes.string, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 85, 12)))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'timeout'), pyxb.binding.datatypes.double, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 86, 12), unicode_default='5'))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'maturity'), MaturityType, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 87, 12), unicode_default='development'))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'environmentModel'), EnvironmentModel, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 88, 12)))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'visualModel'), VisualModel, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 89, 12)))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'bibiConf'), bibiConf, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 90, 12)))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'experimentControl'), ExperimentControl, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 91, 12)))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation'), ExperimentControl, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 97, 12)))

    ExD_._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cameraPose'), CameraPose, scope=ExD_, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 103, 12)))

    def _BuildAutomaton_3 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_3
        del _BuildAutomaton_3
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 86, 12))
        counters.add(cc_0)
        cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 87, 12))
        counters.add(cc_1)
        cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 89, 12))
        counters.add(cc_2)
        cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 91, 12))
        counters.add(cc_3)
        cc_4 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 97, 12))
        counters.add(cc_4)
        cc_5 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 103, 12))
        counters.add(cc_5)
        states = []
        final_update = None
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'name')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 84, 12))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        final_update = None
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'description')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 85, 12))
        st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_1)
        final_update = None
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'timeout')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 86, 12))
        st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_2)
        final_update = None
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'maturity')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 87, 12))
        st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_3)
        final_update = None
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'environmentModel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 88, 12))
        st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_4)
        final_update = None
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'visualModel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 89, 12))
        st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_5)
        final_update = set()
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'bibiConf')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 90, 12))
        st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_6)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_3, False))
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'experimentControl')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 91, 12))
        st_7 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_7)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_4, False))
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'experimentEvaluation')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 97, 12))
        st_8 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_8)
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_5, False))
        symbol = pyxb.binding.content.ElementUse(ExD_._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cameraPose')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 103, 12))
        st_9 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
        states.append(st_9)
        transitions = []
        transitions.append(fac.Transition(st_1, [
             ]))
        st_0._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_2, [
             ]))
        transitions.append(fac.Transition(st_3, [
             ]))
        transitions.append(fac.Transition(st_4, [
             ]))
        st_1._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_2, [
            fac.UpdateInstruction(cc_0, True) ]))
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_0, False) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_0, False) ]))
        st_2._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_3, [
            fac.UpdateInstruction(cc_1, True) ]))
        transitions.append(fac.Transition(st_4, [
            fac.UpdateInstruction(cc_1, False) ]))
        st_3._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_5, [
             ]))
        transitions.append(fac.Transition(st_6, [
             ]))
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
        transitions.append(fac.Transition(st_9, [
             ]))
        st_6._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_7, [
            fac.UpdateInstruction(cc_3, True) ]))
        transitions.append(fac.Transition(st_8, [
            fac.UpdateInstruction(cc_3, False) ]))
        transitions.append(fac.Transition(st_9, [
            fac.UpdateInstruction(cc_3, False) ]))
        st_7._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_8, [
            fac.UpdateInstruction(cc_4, True) ]))
        transitions.append(fac.Transition(st_9, [
            fac.UpdateInstruction(cc_4, False) ]))
        st_8._set_transitionSet(transitions)
        transitions = []
        transitions.append(fac.Transition(st_9, [
            fac.UpdateInstruction(cc_5, True) ]))
        st_9._set_transitionSet(transitions)
        return fac.Automaton(states, counters, False, containing_state=None)
    ExD_._Automaton = _BuildAutomaton_3()




    ExperimentControl._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'stateMachine'), StateMachine, scope=ExperimentControl, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 109, 12)))

    def _BuildAutomaton_4 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_4
        del _BuildAutomaton_4
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 113, 8))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(ExperimentControl._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'stateMachine')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 109, 12))
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




    SCXMLStateMachine._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml'), _ImportedBinding__sc.scxml_scxml_type, scope=SCXMLStateMachine, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/StateMachines/hbp-scxml/hbp-scxml-module-core.xsd', 86, 1)))

    def _BuildAutomaton_5 ():
        # Remove this helper function from the namespace after it is invoked
        global _BuildAutomaton_5
        del _BuildAutomaton_5
        import pyxb.utils.fac as fac

        counters = set()
        cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 132, 5))
        counters.add(cc_0)
        states = []
        final_update = set()
        final_update.add(fac.UpdateInstruction(cc_0, False))
        symbol = pyxb.binding.content.ElementUse(SCXMLStateMachine._UseForTag(pyxb.namespace.ExpandedName(_Namespace_sc, 'scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Models/ExDConf/ExDConfFile.xsd', 132, 5))
        st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
        states.append(st_0)
        transitions = []
        transitions.append(fac.Transition(st_0, [
            fac.UpdateInstruction(cc_0, True) ]))
        st_0._set_transitionSet(transitions)
        return fac.Automaton(states, counters, True, containing_state=None)
    SCXMLStateMachine._Automaton = _BuildAutomaton_5()

