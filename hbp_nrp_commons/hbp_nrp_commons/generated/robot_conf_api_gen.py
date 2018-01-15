# ./robot_conf_api_gen.py
# -*- coding: utf-8 -*-
# PyXB bindings for NM:21e0df4f43fd2ef466c22083dc298aa5044036b7
# Generated 2018-01-15 09:25:59.486981 by PyXB version 1.2.4 using Python 2.7.12.final.0
# Namespace http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config

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
_GenerationUID = pyxb.utils.utility.UniqueIdentifier('urn:uuid:b728d724-f9cd-11e7-b414-d89ef3016f63')

# Version of PyXB used to generate the bindings
_PyXBVersion = '1.2.4'
# Generated bindings are not compatible across PyXB versions
if pyxb.__version__ != _PyXBVersion:
    raise pyxb.PyXBVersionError(_PyXBVersion)

# Import bindings for namespaces imported into schema
import pyxb.binding.datatypes

# NOTE: All namespace declarations are reserved within the binding
Namespace = pyxb.namespace.NamespaceForURI('http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config', create_if_missing=True)
Namespace.configureCategories(['typeBinding', 'elementBinding'])

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


# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}sensor_type
class sensor_type (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """An atomic simple type."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'sensor_type')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 56, 4)
    _Documentation = None
sensor_type._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=sensor_type, enum_prefix=None)
sensor_type.camera = sensor_type._CF_enumeration.addEnumeration(unicode_value='camera', tag='camera')
sensor_type.audio = sensor_type._CF_enumeration.addEnumeration(unicode_value='audio', tag='audio')
sensor_type.contact = sensor_type._CF_enumeration.addEnumeration(unicode_value='contact', tag='contact')
sensor_type.laser = sensor_type._CF_enumeration.addEnumeration(unicode_value='laser', tag='laser')
sensor_type.ultrasound = sensor_type._CF_enumeration.addEnumeration(unicode_value='ultrasound', tag='ultrasound')
sensor_type.radar = sensor_type._CF_enumeration.addEnumeration(unicode_value='radar', tag='radar')
sensor_type.gps = sensor_type._CF_enumeration.addEnumeration(unicode_value='gps', tag='gps')
sensor_type.olfaction = sensor_type._CF_enumeration.addEnumeration(unicode_value='olfaction', tag='olfaction')
sensor_type.other = sensor_type._CF_enumeration.addEnumeration(unicode_value='other', tag='other')
sensor_type._InitializeFacetMap(sensor_type._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'sensor_type', sensor_type)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}actuator_type
class actuator_type (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """An atomic simple type."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'actuator_type')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 82, 4)
    _Documentation = None
actuator_type._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=actuator_type, enum_prefix=None)
actuator_type.motor = actuator_type._CF_enumeration.addEnumeration(unicode_value='motor', tag='motor')
actuator_type.muscle = actuator_type._CF_enumeration.addEnumeration(unicode_value='muscle', tag='muscle')
actuator_type.linear = actuator_type._CF_enumeration.addEnumeration(unicode_value='linear', tag='linear')
actuator_type.pneumatic = actuator_type._CF_enumeration.addEnumeration(unicode_value='pneumatic', tag='pneumatic')
actuator_type.hydraulic = actuator_type._CF_enumeration.addEnumeration(unicode_value='hydraulic', tag='hydraulic')
actuator_type.other = actuator_type._CF_enumeration.addEnumeration(unicode_value='other', tag='other')
actuator_type._InitializeFacetMap(actuator_type._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'actuator_type', actuator_type)

# Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}ModelConfiguration with content type ELEMENT_ONLY
class ModelConfiguration (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}ModelConfiguration with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ModelConfiguration')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 9, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}name uses Python identifier name
    __name = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configname', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 11, 12), )

    
    name = property(__name.value, __name.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}version uses Python identifier version
    __version = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'version'), 'version', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configversion', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 12, 12), )

    
    version = property(__version.value, __version.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}license uses Python identifier license
    __license = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'license'), 'license', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configlicense', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 13, 12), )

    
    license = property(__license.value, __license.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}thumbnail uses Python identifier thumbnail
    __thumbnail = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'thumbnail'), 'thumbnail', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configthumbnail', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 14, 12), )

    
    thumbnail = property(__thumbnail.value, __thumbnail.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}sdf uses Python identifier sdf
    __sdf = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'sdf'), 'sdf', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configsdf', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 15, 12), )

    
    sdf = property(__sdf.value, __sdf.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}author uses Python identifier author
    __author = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'author'), 'author', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configauthor', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 16, 12), )

    
    author = property(__author.value, __author.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}description uses Python identifier description
    __description = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'description'), 'description', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configdescription', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 17, 12), )

    
    description = property(__description.value, __description.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}website uses Python identifier website
    __website = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'website'), 'website', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configwebsite', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 18, 12), )

    
    website = property(__website.value, __website.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}sensors uses Python identifier sensors
    __sensors = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'sensors'), 'sensors', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configsensors', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 19, 12), )

    
    sensors = property(__sensors.value, __sensors.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}actuators uses Python identifier actuators
    __actuators = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'actuators'), 'actuators', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configactuators', False, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 20, 12), )

    
    actuators = property(__actuators.value, __actuators.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}publication uses Python identifier publication
    __publication = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'publication'), 'publication', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configpublication', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 21, 12), )

    
    publication = property(__publication.value, __publication.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}youtube uses Python identifier youtube
    __youtube = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'youtube'), 'youtube', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configyoutube', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 22, 12), )

    
    youtube = property(__youtube.value, __youtube.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}picture uses Python identifier picture
    __picture = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'picture'), 'picture', '__httpschemas_humanbrainproject_euSP102017robot_model_config_ModelConfiguration_httpschemas_humanbrainproject_euSP102017robot_model_configpicture', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 23, 12), )

    
    picture = property(__picture.value, __picture.set, None, None)

    _ElementMap.update({
        __name.name() : __name,
        __version.name() : __version,
        __license.name() : __license,
        __thumbnail.name() : __thumbnail,
        __sdf.name() : __sdf,
        __author.name() : __author,
        __description.name() : __description,
        __website.name() : __website,
        __sensors.name() : __sensors,
        __actuators.name() : __actuators,
        __publication.name() : __publication,
        __youtube.name() : __youtube,
        __picture.name() : __picture
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'ModelConfiguration', ModelConfiguration)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}sdf_versioned with content type SIMPLE
class sdf_versioned (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}sdf_versioned with content type SIMPLE"""
    _TypeDefinition = pyxb.binding.datatypes.string
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_SIMPLE
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'sdf_versioned')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 27, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.string
    
    # Attribute version uses Python identifier version
    __version = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'version'), 'version', '__httpschemas_humanbrainproject_euSP102017robot_model_config_sdf_versioned_version', pyxb.binding.datatypes.decimal)
    __version._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 30, 16)
    __version._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 30, 16)
    
    version = property(__version.value, __version.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __version.name() : __version
    })
Namespace.addCategoryObject('typeBinding', 'sdf_versioned', sdf_versioned)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}author_type with content type ELEMENT_ONLY
class author_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}author_type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'author_type')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 36, 5)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}name uses Python identifier name
    __name = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102017robot_model_config_author_type_httpschemas_humanbrainproject_euSP102017robot_model_configname', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 38, 13), )

    
    name = property(__name.value, __name.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}email uses Python identifier email
    __email = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'email'), 'email', '__httpschemas_humanbrainproject_euSP102017robot_model_config_author_type_httpschemas_humanbrainproject_euSP102017robot_model_configemail', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 39, 13), )

    
    email = property(__email.value, __email.set, None, None)

    _ElementMap.update({
        __name.name() : __name,
        __email.name() : __email
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'author_type', author_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}sensors_type with content type ELEMENT_ONLY
class sensors_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}sensors_type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'sensors_type')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 44, 5)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}sensor uses Python identifier sensor
    __sensor = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'sensor'), 'sensor', '__httpschemas_humanbrainproject_euSP102017robot_model_config_sensors_type_httpschemas_humanbrainproject_euSP102017robot_model_configsensor', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 46, 13), )

    
    sensor = property(__sensor.value, __sensor.set, None, None)

    _ElementMap.update({
        __sensor.name() : __sensor
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'sensors_type', sensors_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}actuators_type with content type ELEMENT_ONLY
class actuators_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}actuators_type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'actuators_type')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 71, 5)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}actuator uses Python identifier actuator
    __actuator = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'actuator'), 'actuator', '__httpschemas_humanbrainproject_euSP102017robot_model_config_actuators_type_httpschemas_humanbrainproject_euSP102017robot_model_configactuator', True, pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 73, 13), )

    
    actuator = property(__actuator.value, __actuator.set, None, None)

    _ElementMap.update({
        __actuator.name() : __actuator
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'actuators_type', actuators_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}youtube_resource with content type EMPTY
class youtube_resource (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}youtube_resource with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'youtube_resource')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 94, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute title uses Python identifier title
    __title = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'title'), 'title', '__httpschemas_humanbrainproject_euSP102017robot_model_config_youtube_resource_title', pyxb.binding.datatypes.string, required=True)
    __title._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 95, 8)
    __title._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 95, 8)
    
    title = property(__title.value, __title.set, None, None)

    
    # Attribute youtube-id uses Python identifier youtube_id
    __youtube_id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'youtube-id'), 'youtube_id', '__httpschemas_humanbrainproject_euSP102017robot_model_config_youtube_resource_youtube_id', pyxb.binding.datatypes.string, required=True)
    __youtube_id._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 96, 8)
    __youtube_id._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 96, 8)
    
    youtube_id = property(__youtube_id.value, __youtube_id.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __title.name() : __title,
        __youtube_id.name() : __youtube_id
    })
Namespace.addCategoryObject('typeBinding', 'youtube_resource', youtube_resource)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}url_resource with content type EMPTY
class url_resource (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}url_resource with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'url_resource')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 99, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute title uses Python identifier title
    __title = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'title'), 'title', '__httpschemas_humanbrainproject_euSP102017robot_model_config_url_resource_title', pyxb.binding.datatypes.string, required=True)
    __title._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 100, 8)
    __title._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 100, 8)
    
    title = property(__title.value, __title.set, None, None)

    
    # Attribute url uses Python identifier url
    __url = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'url'), 'url', '__httpschemas_humanbrainproject_euSP102017robot_model_config_url_resource_url', pyxb.binding.datatypes.string, required=True)
    __url._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 101, 8)
    __url._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 101, 8)
    
    url = property(__url.value, __url.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __title.name() : __title,
        __url.name() : __url
    })
Namespace.addCategoryObject('typeBinding', 'url_resource', url_resource)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}publication_type with content type EMPTY
class publication_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2017/robot_model_config}publication_type with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'publication_type')
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 105, 4)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute title uses Python identifier title
    __title = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'title'), 'title', '__httpschemas_humanbrainproject_euSP102017robot_model_config_publication_type_title', pyxb.binding.datatypes.string, required=True)
    __title._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 106, 8)
    __title._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 106, 8)
    
    title = property(__title.value, __title.set, None, None)

    
    # Attribute url uses Python identifier url
    __url = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'url'), 'url', '__httpschemas_humanbrainproject_euSP102017robot_model_config_publication_type_url', pyxb.binding.datatypes.string, required=True)
    __url._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 107, 8)
    __url._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 107, 8)
    
    url = property(__url.value, __url.set, None, None)

    
    # Attribute authors uses Python identifier authors
    __authors = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'authors'), 'authors', '__httpschemas_humanbrainproject_euSP102017robot_model_config_publication_type_authors', pyxb.binding.datatypes.string, required=True)
    __authors._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 108, 8)
    __authors._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 108, 8)
    
    authors = property(__authors.value, __authors.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __title.name() : __title,
        __url.name() : __url,
        __authors.name() : __authors
    })
Namespace.addCategoryObject('typeBinding', 'publication_type', publication_type)


# Complex type [anonymous] with content type EMPTY
class CTD_ANON (pyxb.binding.basis.complexTypeDefinition):
    """Complex type [anonymous] with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = None
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 47, 16)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102017robot_model_config_CTD_ANON_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 48, 20)
    __name._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 48, 20)
    
    name = property(__name.value, __name.set, None, None)

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102017robot_model_config_CTD_ANON_type', sensor_type, required=True)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 49, 20)
    __type._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 49, 20)
    
    type = property(__type.value, __type.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __name.name() : __name,
        __type.name() : __type
    })



# Complex type [anonymous] with content type EMPTY
class CTD_ANON_ (pyxb.binding.basis.complexTypeDefinition):
    """Complex type [anonymous] with content type EMPTY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = None
    _XSDLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 74, 16)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102017robot_model_config_CTD_ANON__name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 75, 20)
    __name._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 75, 20)
    
    name = property(__name.value, __name.set, None, None)

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102017robot_model_config_CTD_ANON__type', actuator_type, required=True)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 76, 20)
    __type._UseLocation = pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 76, 20)
    
    type = property(__type.value, __type.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __name.name() : __name,
        __type.name() : __type
    })



model = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'model'), ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 8, 4))
Namespace.addCategoryObject('elementBinding', model.name().localName(), model)



ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'name'), pyxb.binding.datatypes.string, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 11, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'version'), pyxb.binding.datatypes.decimal, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 12, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'license'), pyxb.binding.datatypes.string, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 13, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'thumbnail'), pyxb.binding.datatypes.string, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 14, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'sdf'), sdf_versioned, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 15, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'author'), author_type, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 16, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'description'), pyxb.binding.datatypes.string, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 17, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'website'), pyxb.binding.datatypes.string, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 18, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'sensors'), sensors_type, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 19, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'actuators'), actuators_type, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 20, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'publication'), publication_type, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 21, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'youtube'), youtube_resource, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 22, 12)))

ModelConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'picture'), url_resource, scope=ModelConfiguration, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 23, 12)))

def _BuildAutomaton ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton
    del _BuildAutomaton
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 12, 12))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 14, 12))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 16, 12))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 21, 12))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 22, 12))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 23, 12))
    counters.add(cc_5)
    states = []
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'name')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 11, 12))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'version')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 12, 12))
    st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'license')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 13, 12))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'thumbnail')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 14, 12))
    st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'sdf')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 15, 12))
    st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'author')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 16, 12))
    st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'description')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 17, 12))
    st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'website')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 18, 12))
    st_7 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'sensors')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 19, 12))
    st_8 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'actuators')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 20, 12))
    st_9 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_9)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'publication')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 21, 12))
    st_10 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_10)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'youtube')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 22, 12))
    st_11 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_11)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.ElementUse(ModelConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'picture')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 23, 12))
    st_12 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_12)
    transitions = []
    transitions.append(fac.Transition(st_1, [
         ]))
    transitions.append(fac.Transition(st_2, [
         ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
         ]))
    transitions.append(fac.Transition(st_4, [
         ]))
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
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_8, [
         ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
         ]))
    st_8._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_10, [
         ]))
    transitions.append(fac.Transition(st_11, [
         ]))
    transitions.append(fac.Transition(st_12, [
         ]))
    st_9._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_3, False) ]))
    st_10._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, False) ]))
    st_11._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_5, True) ]))
    st_12._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
ModelConfiguration._Automaton = _BuildAutomaton()




author_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'name'), pyxb.binding.datatypes.string, scope=author_type, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 38, 13)))

author_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'email'), pyxb.binding.datatypes.string, scope=author_type, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 39, 13)))

def _BuildAutomaton_ ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_
    del _BuildAutomaton_
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 37, 9))
    counters.add(cc_0)
    states = []
    final_update = None
    symbol = pyxb.binding.content.ElementUse(author_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'name')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 38, 13))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(author_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'email')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 39, 13))
    st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    transitions = []
    transitions.append(fac.Transition(st_1, [
         ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_1._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
author_type._Automaton = _BuildAutomaton_()




sensors_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'sensor'), CTD_ANON, scope=sensors_type, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 46, 13)))

def _BuildAutomaton_2 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_2
    del _BuildAutomaton_2
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 45, 9))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(sensors_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'sensor')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 46, 13))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
sensors_type._Automaton = _BuildAutomaton_2()




actuators_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'actuator'), CTD_ANON_, scope=actuators_type, location=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 73, 13)))

def _BuildAutomaton_3 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_3
    del _BuildAutomaton_3
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 72, 9))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(actuators_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'actuator')), pyxb.utils.utility.Location('/home/cmartins/Documents/NRP/Models/robot_model_configuration.xsd', 73, 13))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
actuators_type._Automaton = _BuildAutomaton_3()

