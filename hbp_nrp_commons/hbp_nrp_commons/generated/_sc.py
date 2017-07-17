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
# ./_sc.py
# -*- coding: utf-8 -*-
# PyXB bindings for NM:757c0b070fc96b1e70d6f186e818dd670f5c7db9
# Generated 2017-07-11 08:30:13.526639 by PyXB version 1.2.4 using Python 2.7.12.final.0
# Namespace http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml [xmlns:sc]

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
_GenerationUID = pyxb.utils.utility.UniqueIdentifier('urn:uuid:6549d3e0-6602-11e7-af07-847beb4693fd')

# Version of PyXB used to generate the bindings
_PyXBVersion = '1.2.4'
# Generated bindings are not compatible across PyXB versions
if pyxb.__version__ != _PyXBVersion:
    raise pyxb.PyXBVersionError(_PyXBVersion)

# Import bindings for namespaces imported into schema
import pyxb.binding.datatypes

# NOTE: All namespace declarations are reserved within the binding
Namespace = pyxb.namespace.NamespaceForURI('http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml', create_if_missing=True)
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


# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}Exmode.datatype
class Exmode_datatype (pyxb.binding.datatypes.NMTOKEN, pyxb.binding.basis.enumeration_mixin):

    """
            Describes the processor execution mode for this document, being
            either "lax" or "strict".
			"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Exmode.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 18, 1)
    _Documentation = '\n            Describes the processor execution mode for this document, being\n            either "lax" or "strict".\n\t\t\t'
Exmode_datatype._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=Exmode_datatype, enum_prefix=None)
Exmode_datatype.lax = Exmode_datatype._CF_enumeration.addEnumeration(unicode_value='lax', tag='lax')
Exmode_datatype.strict = Exmode_datatype._CF_enumeration.addEnumeration(unicode_value='strict', tag='strict')
Exmode_datatype._InitializeFacetMap(Exmode_datatype._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'Exmode.datatype', Exmode_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}Binding.datatype
class Binding_datatype (pyxb.binding.datatypes.NMTOKEN, pyxb.binding.basis.enumeration_mixin):

    """
            The binding type in use for the SCXML document.
			"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Binding.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 31, 1)
    _Documentation = '\n            The binding type in use for the SCXML document.\n\t\t\t'
Binding_datatype._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=Binding_datatype, enum_prefix=None)
Binding_datatype.early = Binding_datatype._CF_enumeration.addEnumeration(unicode_value='early', tag='early')
Binding_datatype.late = Binding_datatype._CF_enumeration.addEnumeration(unicode_value='late', tag='late')
Binding_datatype._InitializeFacetMap(Binding_datatype._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'Binding.datatype', Binding_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}HistoryType.datatype
class HistoryType_datatype (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """An atomic simple type."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'HistoryType.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 44, 1)
    _Documentation = None
HistoryType_datatype._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=HistoryType_datatype, enum_prefix=None)
HistoryType_datatype.shallow = HistoryType_datatype._CF_enumeration.addEnumeration(unicode_value='shallow', tag='shallow')
HistoryType_datatype.deep = HistoryType_datatype._CF_enumeration.addEnumeration(unicode_value='deep', tag='deep')
HistoryType_datatype._InitializeFacetMap(HistoryType_datatype._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'HistoryType.datatype', HistoryType_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}TransitionType.datatype
class TransitionType_datatype (pyxb.binding.datatypes.NMTOKEN, pyxb.binding.basis.enumeration_mixin):

    """
            The type of the transition i.e. internal or external.
			"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TransitionType.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 51, 1)
    _Documentation = '\n            The type of the transition i.e. internal or external.\n\t\t\t'
TransitionType_datatype._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=TransitionType_datatype, enum_prefix=None)
TransitionType_datatype.internal = TransitionType_datatype._CF_enumeration.addEnumeration(unicode_value='internal', tag='internal')
TransitionType_datatype.external = TransitionType_datatype._CF_enumeration.addEnumeration(unicode_value='external', tag='external')
TransitionType_datatype._InitializeFacetMap(TransitionType_datatype._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'TransitionType.datatype', TransitionType_datatype)

# List simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}Boolean.datatype
# superclasses pyxb.binding.datatypes.NMTOKENS, pyxb.binding.basis.enumeration_mixin
class Boolean_datatype (pyxb.binding.basis.STD_list):

    """
            Boolean: true or false only
			"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Boolean.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 63, 1)
    _Documentation = '\n            Boolean: true or false only\n\t\t\t'

    _ItemType = pyxb.binding.datatypes.NMTOKEN
Boolean_datatype._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=Boolean_datatype, enum_prefix=None)
Boolean_datatype._CF_enumeration.addEnumeration(unicode_value='true', tag=None)
Boolean_datatype._CF_enumeration.addEnumeration(unicode_value='false', tag=None)
Boolean_datatype._InitializeFacetMap(Boolean_datatype._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'Boolean.datatype', Boolean_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}AssignType.datatype
class AssignType_datatype (pyxb.binding.datatypes.NMTOKEN, pyxb.binding.basis.enumeration_mixin):

    """
            The assign type that allows for precise manipulation of the
            datamodel location. Types are:
                 replacechildren (default),
                 firstchild, lastchild,
                 previoussibling, nextsibling,
                 replace, delete,
                 addattribute
			"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'AssignType.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 75, 1)
    _Documentation = '\n            The assign type that allows for precise manipulation of the\n            datamodel location. Types are:\n                 replacechildren (default),\n                 firstchild, lastchild,\n                 previoussibling, nextsibling,\n                 replace, delete,\n                 addattribute\n\t\t\t'
AssignType_datatype._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=AssignType_datatype, enum_prefix=None)
AssignType_datatype.replacechildren = AssignType_datatype._CF_enumeration.addEnumeration(unicode_value='replacechildren', tag='replacechildren')
AssignType_datatype.firstchild = AssignType_datatype._CF_enumeration.addEnumeration(unicode_value='firstchild', tag='firstchild')
AssignType_datatype.lastchild = AssignType_datatype._CF_enumeration.addEnumeration(unicode_value='lastchild', tag='lastchild')
AssignType_datatype.previoussibling = AssignType_datatype._CF_enumeration.addEnumeration(unicode_value='previoussibling', tag='previoussibling')
AssignType_datatype.nextsibling = AssignType_datatype._CF_enumeration.addEnumeration(unicode_value='nextsibling', tag='nextsibling')
AssignType_datatype.replace = AssignType_datatype._CF_enumeration.addEnumeration(unicode_value='replace', tag='replace')
AssignType_datatype.delete = AssignType_datatype._CF_enumeration.addEnumeration(unicode_value='delete', tag='delete')
AssignType_datatype.addattribute = AssignType_datatype._CF_enumeration.addEnumeration(unicode_value='addattribute', tag='addattribute')
AssignType_datatype._InitializeFacetMap(AssignType_datatype._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'AssignType.datatype', AssignType_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}URI.datatype
class URI_datatype (pyxb.binding.datatypes.anyURI):

    """
            The xsd:anyURI type and thus URI references in SCXML
            documents may contain a wide array of international
            characters. Implementers should reference RFC 3987 and
            the "Character Model for the World Wide Web 1.0:
            Resource Identifiers" in order to provide appropriate
            support for these characters in VoiceXML documents and
            when processing values of this type or mapping them to
            URIs.
			"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'URI.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 99, 1)
    _Documentation = '\n            The xsd:anyURI type and thus URI references in SCXML\n            documents may contain a wide array of international\n            characters. Implementers should reference RFC 3987 and\n            the "Character Model for the World Wide Web 1.0:\n            Resource Identifiers" in order to provide appropriate\n            support for these characters in VoiceXML documents and\n            when processing values of this type or mapping them to\n            URIs.\n\t\t\t'
URI_datatype._InitializeFacetMap()
Namespace.addCategoryObject('typeBinding', 'URI.datatype', URI_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}Integer.datatype
class Integer_datatype (pyxb.binding.datatypes.nonNegativeInteger):

    """Non-negative integer"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Integer.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 115, 1)
    _Documentation = 'Non-negative integer'
Integer_datatype._InitializeFacetMap()
Namespace.addCategoryObject('typeBinding', 'Integer.datatype', Integer_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}Duration.datatype
class Duration_datatype (pyxb.binding.datatypes.string):

    """
            Duration allowing positive values ranging from milliseconds
            to days.
			"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Duration.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 122, 1)
    _Documentation = '\n            Duration allowing positive values ranging from milliseconds\n            to days.\n\t\t\t'
Duration_datatype._CF_pattern = pyxb.binding.facets.CF_pattern()
Duration_datatype._CF_pattern.addPattern(pattern='\\d*(\\.\\d+)?(ms|s|m|h|d)')
Duration_datatype._InitializeFacetMap(Duration_datatype._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'Duration.datatype', Duration_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}EventType.datatype
class EventType_datatype (pyxb.binding.datatypes.token):

    """
            EventType is the name of an event.
            Example legal values:
            	foo
            	foo.bar
            	foo.bar.baz
            """

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'EventType.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 135, 1)
    _Documentation = '\n            EventType is the name of an event.\n            Example legal values:\n            \tfoo\n            \tfoo.bar\n            \tfoo.bar.baz\n            '
EventType_datatype._CF_pattern = pyxb.binding.facets.CF_pattern()
EventType_datatype._CF_pattern.addPattern(pattern='(\\i|\\d|\\-)+(\\.(\\i|\\d|\\-)+)*')
EventType_datatype._InitializeFacetMap(EventType_datatype._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'EventType.datatype', EventType_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}EventTypes.datatype
class EventTypes_datatype (pyxb.binding.datatypes.token):

    """
			Custom datatype for the event attribute in SCXML based on xsd:token.
			Example legal values:
				*
				foo
				foo.bar
				foo.*
				foo.bar.*
				foo bar baz
				foo.bar bar.* baz.foo.*
            """

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'EventTypes.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 150, 1)
    _Documentation = '\n\t\t\tCustom datatype for the event attribute in SCXML based on xsd:token.\n\t\t\tExample legal values:\n\t\t\t\t*\n\t\t\t\tfoo\n\t\t\t\tfoo.bar\n\t\t\t\tfoo.*\n\t\t\t\tfoo.bar.*\n\t\t\t\tfoo bar baz\n\t\t\t\tfoo.bar bar.* baz.foo.*\n            '
EventTypes_datatype._CF_pattern = pyxb.binding.facets.CF_pattern()
EventTypes_datatype._CF_pattern.addPattern(pattern='\\.?\\*|(\\i|\\d|\\-)+(\\.(\\i|\\d|\\-)+)*(\\.\\*)?(\\s(\\i|\\d|\\-)+(\\.(\\i|\\d|\\-)+)*(\\.\\*)?)*')
EventTypes_datatype._InitializeFacetMap(EventTypes_datatype._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'EventTypes.datatype', EventTypes_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}CondLang.datatype
class CondLang_datatype (pyxb.binding.datatypes.string):

    """
		    Conditional language is expression
		    which must evaluate to Boolean True or False.
		    The expression language must define In(stateID)
		    as a valid expression.
            """

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'CondLang.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 170, 1)
    _Documentation = '\n\t\t    Conditional language is expression\n\t\t    which must evaluate to Boolean True or False.\n\t\t    The expression language must define In(stateID)\n\t\t    as a valid expression.\n            '
CondLang_datatype._InitializeFacetMap()
Namespace.addCategoryObject('typeBinding', 'CondLang.datatype', CondLang_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}LocLang.datatype
class LocLang_datatype (pyxb.binding.datatypes.string):

    """
	        Location language is expression
		    identifying a location in the datamodel.
	        """

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'LocLang.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 183, 1)
    _Documentation = '\n\t        Location language is expression\n\t\t    identifying a location in the datamodel.\n\t        '
LocLang_datatype._InitializeFacetMap()
Namespace.addCategoryObject('typeBinding', 'LocLang.datatype', LocLang_datatype)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}ValueLang.datatype
class ValueLang_datatype (pyxb.binding.datatypes.string):

    """
		    Value language is expression
		    return a value.
            """

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ValueLang.datatype')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-datatypes.xsd', 194, 1)
    _Documentation = '\n\t\t    Value language is expression\n\t\t    return a value.\n            '
ValueLang_datatype._InitializeFacetMap()
Namespace.addCategoryObject('typeBinding', 'ValueLang.datatype', ValueLang_datatype)

# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.state.type with content type ELEMENT_ONLY
class scxml_state_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.state.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.state.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 114, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}state uses Python identifier state
    __state = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'state'), 'state', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlstate', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 120, 1), )

    
    state = property(__state.value, __state.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}initial uses Python identifier initial
    __initial = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'initial'), 'initial', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlinitial', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 137, 1), )

    
    initial = property(__initial.value, __initial.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}onentry uses Python identifier onentry
    __onentry = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'onentry'), 'onentry', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlonentry', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 152, 1), )

    
    onentry = property(__onentry.value, __onentry.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}onexit uses Python identifier onexit
    __onexit = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'onexit'), 'onexit', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlonexit', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 167, 1), )

    
    onexit = property(__onexit.value, __onexit.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}transition uses Python identifier transition
    __transition = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'transition'), 'transition', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmltransition', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1), )

    
    transition = property(__transition.value, __transition.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}parallel uses Python identifier parallel
    __parallel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'parallel'), 'parallel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlparallel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 215, 1), )

    
    parallel = property(__parallel.value, __parallel.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}final uses Python identifier final
    __final = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'final'), 'final', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlfinal', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 240, 1), )

    
    final = property(__final.value, __final.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}history uses Python identifier history
    __history = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'history'), 'history', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlhistory', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 259, 1), )

    
    history = property(__history.value, __history.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}datamodel uses Python identifier datamodel
    __datamodel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'datamodel'), 'datamodel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmldatamodel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 59, 1), )

    
    datamodel = property(__datamodel.value, __datamodel.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}invoke uses Python identifier invoke
    __invoke = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'invoke'), 'invoke', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlinvoke', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 129, 1), )

    
    invoke = property(__invoke.value, __invoke.set, None, None)

    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_id', pyxb.binding.datatypes.ID)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 90, 2)
    __id._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 90, 2)
    
    id = property(__id.value, __id.set, None, None)

    
    # Attribute initial uses Python identifier initial_
    __initial_ = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'initial'), 'initial_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_state_type_initial', pyxb.binding.datatypes.IDREFS)
    __initial_._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 91, 2)
    __initial_._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 91, 2)
    
    initial_ = property(__initial_.value, __initial_.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __state.name() : __state,
        __initial.name() : __initial,
        __onentry.name() : __onentry,
        __onexit.name() : __onexit,
        __transition.name() : __transition,
        __parallel.name() : __parallel,
        __final.name() : __final,
        __history.name() : __history,
        __datamodel.name() : __datamodel,
        __invoke.name() : __invoke
    })
    _AttributeMap.update({
        __id.name() : __id,
        __initial_.name() : __initial_
    })
Namespace.addCategoryObject('typeBinding', 'scxml.state.type', scxml_state_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.initial.type with content type ELEMENT_ONLY
class scxml_initial_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.initial.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.initial.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 133, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}transition uses Python identifier transition
    __transition = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'transition'), 'transition', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_initial_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmltransition', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1), )

    
    transition = property(__transition.value, __transition.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __transition.name() : __transition
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'scxml.initial.type', scxml_initial_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.onentry.type with content type ELEMENT_ONLY
class scxml_onentry_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.onentry.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.onentry.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 148, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}if uses Python identifier if_
    __if = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'if'), 'if_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onentry_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlif', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1), )

    
    if_ = property(__if.value, __if.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}foreach uses Python identifier foreach
    __foreach = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'foreach'), 'foreach', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onentry_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlforeach', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1), )

    
    foreach = property(__foreach.value, __foreach.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}raise uses Python identifier raise_
    __raise = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'raise'), 'raise_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onentry_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlraise', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1), )

    
    raise_ = property(__raise.value, __raise.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}log uses Python identifier log
    __log = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'log'), 'log', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onentry_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmllog', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1), )

    
    log = property(__log.value, __log.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}assign uses Python identifier assign
    __assign = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'assign'), 'assign', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onentry_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlassign', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1), )

    
    assign = property(__assign.value, __assign.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}script uses Python identifier script
    __script = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'script'), 'script', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onentry_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscript', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1), )

    
    script = property(__script.value, __script.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}send uses Python identifier send
    __send = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'send'), 'send', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onentry_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlsend', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1), )

    
    send = property(__send.value, __send.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}cancel uses Python identifier cancel
    __cancel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cancel'), 'cancel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onentry_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcancel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1), )

    
    cancel = property(__cancel.value, __cancel.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __if.name() : __if,
        __foreach.name() : __foreach,
        __raise.name() : __raise,
        __log.name() : __log,
        __assign.name() : __assign,
        __script.name() : __script,
        __send.name() : __send,
        __cancel.name() : __cancel
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'scxml.onentry.type', scxml_onentry_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.onexit.type with content type ELEMENT_ONLY
class scxml_onexit_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.onexit.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.onexit.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 163, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}if uses Python identifier if_
    __if = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'if'), 'if_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onexit_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlif', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1), )

    
    if_ = property(__if.value, __if.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}foreach uses Python identifier foreach
    __foreach = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'foreach'), 'foreach', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onexit_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlforeach', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1), )

    
    foreach = property(__foreach.value, __foreach.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}raise uses Python identifier raise_
    __raise = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'raise'), 'raise_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onexit_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlraise', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1), )

    
    raise_ = property(__raise.value, __raise.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}log uses Python identifier log
    __log = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'log'), 'log', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onexit_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmllog', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1), )

    
    log = property(__log.value, __log.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}assign uses Python identifier assign
    __assign = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'assign'), 'assign', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onexit_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlassign', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1), )

    
    assign = property(__assign.value, __assign.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}script uses Python identifier script
    __script = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'script'), 'script', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onexit_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscript', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1), )

    
    script = property(__script.value, __script.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}send uses Python identifier send
    __send = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'send'), 'send', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onexit_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlsend', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1), )

    
    send = property(__send.value, __send.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}cancel uses Python identifier cancel
    __cancel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cancel'), 'cancel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_onexit_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcancel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1), )

    
    cancel = property(__cancel.value, __cancel.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __if.name() : __if,
        __foreach.name() : __foreach,
        __raise.name() : __raise,
        __log.name() : __log,
        __assign.name() : __assign,
        __script.name() : __script,
        __send.name() : __send,
        __cancel.name() : __cancel
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'scxml.onexit.type', scxml_onexit_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.parallel.type with content type ELEMENT_ONLY
class scxml_parallel_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.parallel.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.parallel.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 211, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}state uses Python identifier state
    __state = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'state'), 'state', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlstate', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 120, 1), )

    
    state = property(__state.value, __state.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}onentry uses Python identifier onentry
    __onentry = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'onentry'), 'onentry', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlonentry', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 152, 1), )

    
    onentry = property(__onentry.value, __onentry.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}onexit uses Python identifier onexit
    __onexit = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'onexit'), 'onexit', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlonexit', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 167, 1), )

    
    onexit = property(__onexit.value, __onexit.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}transition uses Python identifier transition
    __transition = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'transition'), 'transition', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmltransition', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1), )

    
    transition = property(__transition.value, __transition.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}parallel uses Python identifier parallel
    __parallel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'parallel'), 'parallel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlparallel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 215, 1), )

    
    parallel = property(__parallel.value, __parallel.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}history uses Python identifier history
    __history = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'history'), 'history', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlhistory', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 259, 1), )

    
    history = property(__history.value, __history.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}datamodel uses Python identifier datamodel
    __datamodel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'datamodel'), 'datamodel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmldatamodel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 59, 1), )

    
    datamodel = property(__datamodel.value, __datamodel.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}invoke uses Python identifier invoke
    __invoke = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'invoke'), 'invoke', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlinvoke', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 129, 1), )

    
    invoke = property(__invoke.value, __invoke.set, None, None)

    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_parallel_type_id', pyxb.binding.datatypes.ID)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 190, 2)
    __id._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 190, 2)
    
    id = property(__id.value, __id.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __state.name() : __state,
        __onentry.name() : __onentry,
        __onexit.name() : __onexit,
        __transition.name() : __transition,
        __parallel.name() : __parallel,
        __history.name() : __history,
        __datamodel.name() : __datamodel,
        __invoke.name() : __invoke
    })
    _AttributeMap.update({
        __id.name() : __id
    })
Namespace.addCategoryObject('typeBinding', 'scxml.parallel.type', scxml_parallel_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.final.type with content type ELEMENT_ONLY
class scxml_final_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.final.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.final.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 236, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}onentry uses Python identifier onentry
    __onentry = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'onentry'), 'onentry', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_final_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlonentry', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 152, 1), )

    
    onentry = property(__onentry.value, __onentry.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}onexit uses Python identifier onexit
    __onexit = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'onexit'), 'onexit', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_final_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlonexit', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 167, 1), )

    
    onexit = property(__onexit.value, __onexit.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}donedata uses Python identifier donedata
    __donedata = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'donedata'), 'donedata', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_final_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmldonedata', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 277, 1), )

    
    donedata = property(__donedata.value, __donedata.set, None, None)

    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_final_type_id', pyxb.binding.datatypes.ID)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 219, 2)
    __id._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 219, 2)
    
    id = property(__id.value, __id.set, None, None)

    
    # Attribute outcome uses Python identifier outcome
    __outcome = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'outcome'), 'outcome', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_final_type_outcome', pyxb.binding.datatypes.string)
    __outcome._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 220, 2)
    __outcome._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 220, 2)
    
    outcome = property(__outcome.value, __outcome.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __onentry.name() : __onentry,
        __onexit.name() : __onexit,
        __donedata.name() : __donedata
    })
    _AttributeMap.update({
        __id.name() : __id,
        __outcome.name() : __outcome
    })
Namespace.addCategoryObject('typeBinding', 'scxml.final.type', scxml_final_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.donedata.type with content type ELEMENT_ONLY
class scxml_donedata_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.donedata.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.donedata.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 273, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}param uses Python identifier param
    __param = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'param'), 'param', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_donedata_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlparam', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 97, 1), )

    
    param = property(__param.value, __param.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}content uses Python identifier content_
    __content = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'content'), 'content_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_donedata_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcontent', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 149, 1), )

    
    content_ = property(__content.value, __content.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _ElementMap.update({
        __param.name() : __param,
        __content.name() : __content
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'scxml.donedata.type', scxml_donedata_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.else.type with content type ELEMENT_ONLY
class scxml_else_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.else.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.else.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 344, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'scxml.else.type', scxml_else_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.raise.type with content type ELEMENT_ONLY
class scxml_raise_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.raise.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.raise.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 383, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute event uses Python identifier event
    __event = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'event'), 'event', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_raise_type_event', pyxb.binding.datatypes.NMTOKEN, required=True)
    __event._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 370, 2)
    __event._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 370, 2)
    
    event = property(__event.value, __event.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __event.name() : __event
    })
Namespace.addCategoryObject('typeBinding', 'scxml.raise.type', scxml_raise_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.datamodel.type with content type ELEMENT_ONLY
class scxml_datamodel_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.datamodel.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.datamodel.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 55, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}data uses Python identifier data
    __data = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'data'), 'data', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_datamodel_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmldata', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 77, 1), )

    
    data = property(__data.value, __data.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __data.name() : __data
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'scxml.datamodel.type', scxml_datamodel_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.finalize.type with content type ELEMENT_ONLY
class scxml_finalize_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.finalize.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.finalize.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 145, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}if uses Python identifier if_
    __if = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'if'), 'if_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_finalize_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlif', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1), )

    
    if_ = property(__if.value, __if.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}foreach uses Python identifier foreach
    __foreach = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'foreach'), 'foreach', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_finalize_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlforeach', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1), )

    
    foreach = property(__foreach.value, __foreach.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}raise uses Python identifier raise_
    __raise = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'raise'), 'raise_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_finalize_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlraise', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1), )

    
    raise_ = property(__raise.value, __raise.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}log uses Python identifier log
    __log = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'log'), 'log', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_finalize_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmllog', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1), )

    
    log = property(__log.value, __log.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}assign uses Python identifier assign
    __assign = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'assign'), 'assign', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_finalize_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlassign', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1), )

    
    assign = property(__assign.value, __assign.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}script uses Python identifier script
    __script = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'script'), 'script', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_finalize_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscript', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1), )

    
    script = property(__script.value, __script.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}send uses Python identifier send
    __send = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'send'), 'send', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_finalize_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlsend', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1), )

    
    send = property(__send.value, __send.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}cancel uses Python identifier cancel
    __cancel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cancel'), 'cancel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_finalize_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcancel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1), )

    
    cancel = property(__cancel.value, __cancel.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __if.name() : __if,
        __foreach.name() : __foreach,
        __raise.name() : __raise,
        __log.name() : __log,
        __assign.name() : __assign,
        __script.name() : __script,
        __send.name() : __send,
        __cancel.name() : __cancel
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'scxml.finalize.type', scxml_finalize_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.scxml.type with content type ELEMENT_ONLY
class scxml_scxml_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.scxml.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.scxml.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 82, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}state uses Python identifier state
    __state = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'state'), 'state', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlstate', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 120, 1), )

    
    state = property(__state.value, __state.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}parallel uses Python identifier parallel
    __parallel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'parallel'), 'parallel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlparallel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 215, 1), )

    
    parallel = property(__parallel.value, __parallel.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}final uses Python identifier final
    __final = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'final'), 'final', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlfinal', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 240, 1), )

    
    final = property(__final.value, __final.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}datamodel uses Python identifier datamodel
    __datamodel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'datamodel'), 'datamodel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmldatamodel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 59, 1), )

    
    datamodel = property(__datamodel.value, __datamodel.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}script uses Python identifier script
    __script = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'script'), 'script', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscript', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1), )

    
    script = property(__script.value, __script.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}invoke uses Python identifier invoke
    __invoke = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'invoke'), 'invoke', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlinvoke', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 129, 1), )

    
    invoke = property(__invoke.value, __invoke.set, None, None)

    
    # Attribute initial uses Python identifier initial
    __initial = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'initial'), 'initial', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_initial', pyxb.binding.datatypes.IDREFS)
    __initial._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 58, 2)
    __initial._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 58, 2)
    
    initial = property(__initial.value, __initial.set, None, None)

    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_name', pyxb.binding.datatypes.NMTOKEN)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 59, 2)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 59, 2)
    
    name = property(__name.value, __name.set, None, None)

    
    # Attribute version uses Python identifier version
    __version = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'version'), 'version', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_version', pyxb.binding.datatypes.decimal, fixed=True, unicode_default='1.0', required=True)
    __version._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 60, 2)
    __version._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 60, 2)
    
    version = property(__version.value, __version.set, None, None)

    
    # Attribute datamodel uses Python identifier datamodel_
    __datamodel_ = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'datamodel'), 'datamodel_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_datamodel', pyxb.binding.datatypes.NMTOKEN, unicode_default='null')
    __datamodel_._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 61, 2)
    __datamodel_._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 61, 2)
    
    datamodel_ = property(__datamodel_.value, __datamodel_.set, None, None)

    
    # Attribute binding uses Python identifier binding
    __binding = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'binding'), 'binding', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_binding', Binding_datatype)
    __binding._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 62, 2)
    __binding._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 62, 2)
    
    binding = property(__binding.value, __binding.set, None, None)

    
    # Attribute exmode uses Python identifier exmode
    __exmode = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'exmode'), 'exmode', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_scxml_type_exmode', Exmode_datatype)
    __exmode._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 63, 2)
    __exmode._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 63, 2)
    
    exmode = property(__exmode.value, __exmode.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __state.name() : __state,
        __parallel.name() : __parallel,
        __final.name() : __final,
        __datamodel.name() : __datamodel,
        __script.name() : __script,
        __invoke.name() : __invoke
    })
    _AttributeMap.update({
        __initial.name() : __initial,
        __name.name() : __name,
        __version.name() : __version,
        __datamodel_.name() : __datamodel_,
        __binding.name() : __binding,
        __exmode.name() : __exmode
    })
Namespace.addCategoryObject('typeBinding', 'scxml.scxml.type', scxml_scxml_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.transition.type with content type ELEMENT_ONLY
class scxml_transition_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.transition.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.transition.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 182, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}if uses Python identifier if_
    __if = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'if'), 'if_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlif', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1), )

    
    if_ = property(__if.value, __if.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}foreach uses Python identifier foreach
    __foreach = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'foreach'), 'foreach', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlforeach', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1), )

    
    foreach = property(__foreach.value, __foreach.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}raise uses Python identifier raise_
    __raise = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'raise'), 'raise_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlraise', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1), )

    
    raise_ = property(__raise.value, __raise.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}log uses Python identifier log
    __log = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'log'), 'log', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmllog', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1), )

    
    log = property(__log.value, __log.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}assign uses Python identifier assign
    __assign = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'assign'), 'assign', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlassign', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1), )

    
    assign = property(__assign.value, __assign.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}script uses Python identifier script
    __script = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'script'), 'script', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscript', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1), )

    
    script = property(__script.value, __script.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}send uses Python identifier send
    __send = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'send'), 'send', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlsend', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1), )

    
    send = property(__send.value, __send.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}cancel uses Python identifier cancel
    __cancel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cancel'), 'cancel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcancel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1), )

    
    cancel = property(__cancel.value, __cancel.set, None, None)

    
    # Attribute event uses Python identifier event
    __event = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'event'), 'event', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_event', EventTypes_datatype)
    __event._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 171, 2)
    __event._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 171, 2)
    
    event = property(__event.value, __event.set, None, None)

    
    # Attribute cond uses Python identifier cond
    __cond = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'cond'), 'cond', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_cond', CondLang_datatype)
    __cond._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 172, 2)
    __cond._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 172, 2)
    
    cond = property(__cond.value, __cond.set, None, None)

    
    # Attribute target uses Python identifier target
    __target = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'target'), 'target', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_target', pyxb.binding.datatypes.IDREFS)
    __target._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 173, 2)
    __target._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 173, 2)
    
    target = property(__target.value, __target.set, None, None)

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_transition_type_type', TransitionType_datatype)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 174, 2)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 174, 2)
    
    type = property(__type.value, __type.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __if.name() : __if,
        __foreach.name() : __foreach,
        __raise.name() : __raise,
        __log.name() : __log,
        __assign.name() : __assign,
        __script.name() : __script,
        __send.name() : __send,
        __cancel.name() : __cancel
    })
    _AttributeMap.update({
        __event.name() : __event,
        __cond.name() : __cond,
        __target.name() : __target,
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'scxml.transition.type', scxml_transition_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.history.type with content type ELEMENT_ONLY
class scxml_history_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.history.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.history.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 255, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}transition uses Python identifier transition
    __transition = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'transition'), 'transition', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_history_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmltransition', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1), )

    
    transition = property(__transition.value, __transition.set, None, None)

    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_history_type_id', pyxb.binding.datatypes.ID)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 244, 2)
    __id._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 244, 2)
    
    id = property(__id.value, __id.set, None, None)

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_history_type_type', HistoryType_datatype)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 245, 2)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 245, 2)
    
    type = property(__type.value, __type.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __transition.name() : __transition
    })
    _AttributeMap.update({
        __id.name() : __id,
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'scxml.history.type', scxml_history_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.if.type with content type ELEMENT_ONLY
class scxml_if_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.if.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.if.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 303, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}if uses Python identifier if_
    __if = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'if'), 'if_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlif', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1), )

    
    if_ = property(__if.value, __if.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}elseif uses Python identifier elseif
    __elseif = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'elseif'), 'elseif', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlelseif', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 328, 1), )

    
    elseif = property(__elseif.value, __elseif.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}else uses Python identifier else_
    __else = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'else'), 'else_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlelse', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 348, 1), )

    
    else_ = property(__else.value, __else.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}foreach uses Python identifier foreach
    __foreach = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'foreach'), 'foreach', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlforeach', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1), )

    
    foreach = property(__foreach.value, __foreach.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}raise uses Python identifier raise_
    __raise = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'raise'), 'raise_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlraise', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1), )

    
    raise_ = property(__raise.value, __raise.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}log uses Python identifier log
    __log = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'log'), 'log', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmllog', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1), )

    
    log = property(__log.value, __log.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}assign uses Python identifier assign
    __assign = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'assign'), 'assign', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlassign', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1), )

    
    assign = property(__assign.value, __assign.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}script uses Python identifier script
    __script = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'script'), 'script', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscript', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1), )

    
    script = property(__script.value, __script.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}send uses Python identifier send
    __send = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'send'), 'send', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlsend', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1), )

    
    send = property(__send.value, __send.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}cancel uses Python identifier cancel
    __cancel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cancel'), 'cancel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcancel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1), )

    
    cancel = property(__cancel.value, __cancel.set, None, None)

    
    # Attribute cond uses Python identifier cond
    __cond = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'cond'), 'cond', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_if_type_cond', CondLang_datatype, required=True)
    __cond._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 281, 2)
    __cond._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 281, 2)
    
    cond = property(__cond.value, __cond.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __if.name() : __if,
        __elseif.name() : __elseif,
        __else.name() : __else,
        __foreach.name() : __foreach,
        __raise.name() : __raise,
        __log.name() : __log,
        __assign.name() : __assign,
        __script.name() : __script,
        __send.name() : __send,
        __cancel.name() : __cancel
    })
    _AttributeMap.update({
        __cond.name() : __cond
    })
Namespace.addCategoryObject('typeBinding', 'scxml.if.type', scxml_if_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.elseif.type with content type ELEMENT_ONLY
class scxml_elseif_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.elseif.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.elseif.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 324, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute cond uses Python identifier cond
    __cond = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'cond'), 'cond', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_elseif_type_cond', CondLang_datatype, required=True)
    __cond._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 311, 2)
    __cond._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 311, 2)
    
    cond = property(__cond.value, __cond.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __cond.name() : __cond
    })
Namespace.addCategoryObject('typeBinding', 'scxml.elseif.type', scxml_elseif_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.foreach.type with content type ELEMENT_ONLY
class scxml_foreach_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.foreach.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.foreach.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 362, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}if uses Python identifier if_
    __if = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'if'), 'if_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlif', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1), )

    
    if_ = property(__if.value, __if.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}foreach uses Python identifier foreach
    __foreach = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'foreach'), 'foreach', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlforeach', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1), )

    
    foreach = property(__foreach.value, __foreach.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}raise uses Python identifier raise_
    __raise = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'raise'), 'raise_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlraise', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1), )

    
    raise_ = property(__raise.value, __raise.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}log uses Python identifier log
    __log = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'log'), 'log', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmllog', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1), )

    
    log = property(__log.value, __log.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}assign uses Python identifier assign
    __assign = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'assign'), 'assign', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlassign', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1), )

    
    assign = property(__assign.value, __assign.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}script uses Python identifier script
    __script = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'script'), 'script', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlscript', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1), )

    
    script = property(__script.value, __script.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}send uses Python identifier send
    __send = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'send'), 'send', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlsend', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1), )

    
    send = property(__send.value, __send.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}cancel uses Python identifier cancel
    __cancel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'cancel'), 'cancel', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcancel', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1), )

    
    cancel = property(__cancel.value, __cancel.set, None, None)

    
    # Attribute array uses Python identifier array
    __array = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'array'), 'array', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_array', ValueLang_datatype, required=True)
    __array._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 352, 2)
    __array._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 352, 2)
    
    array = property(__array.value, __array.set, None, None)

    
    # Attribute item uses Python identifier item
    __item = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'item'), 'item', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_item', pyxb.binding.datatypes.string, required=True)
    __item._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 353, 2)
    __item._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 353, 2)
    
    item = property(__item.value, __item.set, None, None)

    
    # Attribute index uses Python identifier index
    __index = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'index'), 'index', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_foreach_type_index', pyxb.binding.datatypes.string)
    __index._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 354, 2)
    __index._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 354, 2)
    
    index = property(__index.value, __index.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __if.name() : __if,
        __foreach.name() : __foreach,
        __raise.name() : __raise,
        __log.name() : __log,
        __assign.name() : __assign,
        __script.name() : __script,
        __send.name() : __send,
        __cancel.name() : __cancel
    })
    _AttributeMap.update({
        __array.name() : __array,
        __item.name() : __item,
        __index.name() : __index
    })
Namespace.addCategoryObject('typeBinding', 'scxml.foreach.type', scxml_foreach_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.log.type with content type ELEMENT_ONLY
class scxml_log_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.log.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.log.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 400, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute label uses Python identifier label
    __label = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'label'), 'label', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_log_type_label', pyxb.binding.datatypes.string)
    __label._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 391, 2)
    __label._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 391, 2)
    
    label = property(__label.value, __label.set, None, None)

    
    # Attribute expr uses Python identifier expr
    __expr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'expr'), 'expr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_log_type_expr', ValueLang_datatype)
    __expr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 392, 2)
    __expr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 392, 2)
    
    expr = property(__expr.value, __expr.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __label.name() : __label,
        __expr.name() : __expr
    })
Namespace.addCategoryObject('typeBinding', 'scxml.log.type', scxml_log_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.data.type with content type MIXED
class scxml_data_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.data.type with content type MIXED"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_MIXED
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.data.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 73, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_data_type_id', pyxb.binding.datatypes.ID, required=True)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 63, 2)
    __id._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 63, 2)
    
    id = property(__id.value, __id.set, None, None)

    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_data_type_src', URI_datatype)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 64, 2)
    __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 64, 2)
    
    src = property(__src.value, __src.set, None, None)

    
    # Attribute expr uses Python identifier expr
    __expr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'expr'), 'expr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_data_type_expr', ValueLang_datatype)
    __expr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 65, 2)
    __expr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 65, 2)
    
    expr = property(__expr.value, __expr.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __id.name() : __id,
        __src.name() : __src,
        __expr.name() : __expr
    })
Namespace.addCategoryObject('typeBinding', 'scxml.data.type', scxml_data_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.param.type with content type ELEMENT_ONLY
class scxml_param_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.param.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.param.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 93, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_param_type_name', pyxb.binding.datatypes.NMTOKEN, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 83, 2)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 83, 2)
    
    name = property(__name.value, __name.set, None, None)

    
    # Attribute expr uses Python identifier expr
    __expr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'expr'), 'expr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_param_type_expr', ValueLang_datatype)
    __expr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 84, 2)
    __expr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 84, 2)
    
    expr = property(__expr.value, __expr.set, None, None)

    
    # Attribute location uses Python identifier location
    __location = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'location'), 'location', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_param_type_location', LocLang_datatype)
    __location._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 85, 2)
    __location._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 85, 2)
    
    location = property(__location.value, __location.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __name.name() : __name,
        __expr.name() : __expr,
        __location.name() : __location
    })
Namespace.addCategoryObject('typeBinding', 'scxml.param.type', scxml_param_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.assign.type with content type MIXED
class scxml_assign_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.assign.type with content type MIXED"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_MIXED
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.assign.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 112, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute location uses Python identifier location
    __location = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'location'), 'location', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_assign_type_location', LocLang_datatype, required=True)
    __location._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 101, 2)
    __location._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 101, 2)
    
    location = property(__location.value, __location.set, None, None)

    
    # Attribute expr uses Python identifier expr
    __expr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'expr'), 'expr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_assign_type_expr', ValueLang_datatype)
    __expr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 102, 2)
    __expr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 102, 2)
    
    expr = property(__expr.value, __expr.set, None, None)

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_assign_type_type', AssignType_datatype, unicode_default='replacechildren')
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 103, 2)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 103, 2)
    
    type = property(__type.value, __type.set, None, None)

    
    # Attribute attr uses Python identifier attr
    __attr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'attr'), 'attr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_assign_type_attr', pyxb.binding.datatypes.NMTOKEN)
    __attr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 104, 2)
    __attr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 104, 2)
    
    attr = property(__attr.value, __attr.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __location.name() : __location,
        __expr.name() : __expr,
        __type.name() : __type,
        __attr.name() : __attr
    })
Namespace.addCategoryObject('typeBinding', 'scxml.assign.type', scxml_assign_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.script.type with content type MIXED
class scxml_script_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.script.type with content type MIXED"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_MIXED
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.script.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 129, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_script_type_src', URI_datatype)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 121, 2)
    __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 121, 2)
    
    src = property(__src.value, __src.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'scxml.script.type', scxml_script_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.content.type with content type MIXED
class scxml_content_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.content.type with content type MIXED"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_MIXED
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.content.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 145, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute expr uses Python identifier expr
    __expr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'expr'), 'expr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_content_type_expr', ValueLang_datatype)
    __expr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 138, 2)
    __expr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 138, 2)
    
    expr = property(__expr.value, __expr.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __expr.name() : __expr
    })
Namespace.addCategoryObject('typeBinding', 'scxml.content.type', scxml_content_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.send.type with content type ELEMENT_ONLY
class scxml_send_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.send.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.send.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 70, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}param uses Python identifier param
    __param = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'param'), 'param', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlparam', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 97, 1), )

    
    param = property(__param.value, __param.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}content uses Python identifier content_
    __content = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'content'), 'content_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcontent', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 149, 1), )

    
    content_ = property(__content.value, __content.set, None, None)

    
    # Attribute event uses Python identifier event
    __event = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'event'), 'event', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_event', EventType_datatype)
    __event._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 45, 2)
    __event._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 45, 2)
    
    event = property(__event.value, __event.set, None, None)

    
    # Attribute eventexpr uses Python identifier eventexpr
    __eventexpr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'eventexpr'), 'eventexpr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_eventexpr', ValueLang_datatype)
    __eventexpr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 46, 2)
    __eventexpr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 46, 2)
    
    eventexpr = property(__eventexpr.value, __eventexpr.set, None, None)

    
    # Attribute target uses Python identifier target
    __target = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'target'), 'target', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_target', URI_datatype)
    __target._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 47, 2)
    __target._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 47, 2)
    
    target = property(__target.value, __target.set, None, None)

    
    # Attribute targetexpr uses Python identifier targetexpr
    __targetexpr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'targetexpr'), 'targetexpr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_targetexpr', ValueLang_datatype)
    __targetexpr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 48, 2)
    __targetexpr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 48, 2)
    
    targetexpr = property(__targetexpr.value, __targetexpr.set, None, None)

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_type', pyxb.binding.datatypes.string, unicode_default='scxml')
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 49, 2)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 49, 2)
    
    type = property(__type.value, __type.set, None, None)

    
    # Attribute typeexpr uses Python identifier typeexpr
    __typeexpr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'typeexpr'), 'typeexpr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_typeexpr', ValueLang_datatype)
    __typeexpr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 50, 2)
    __typeexpr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 50, 2)
    
    typeexpr = property(__typeexpr.value, __typeexpr.set, None, None)

    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_id', pyxb.binding.datatypes.ID)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 51, 2)
    __id._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 51, 2)
    
    id = property(__id.value, __id.set, None, None)

    
    # Attribute idlocation uses Python identifier idlocation
    __idlocation = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'idlocation'), 'idlocation', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_idlocation', LocLang_datatype)
    __idlocation._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 52, 2)
    __idlocation._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 52, 2)
    
    idlocation = property(__idlocation.value, __idlocation.set, None, None)

    
    # Attribute delay uses Python identifier delay
    __delay = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'delay'), 'delay', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_delay', Duration_datatype, unicode_default='0s')
    __delay._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 53, 2)
    __delay._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 53, 2)
    
    delay = property(__delay.value, __delay.set, None, None)

    
    # Attribute delayexpr uses Python identifier delayexpr
    __delayexpr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'delayexpr'), 'delayexpr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_delayexpr', ValueLang_datatype)
    __delayexpr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 54, 2)
    __delayexpr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 54, 2)
    
    delayexpr = property(__delayexpr.value, __delayexpr.set, None, None)

    
    # Attribute namelist uses Python identifier namelist
    __namelist = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'namelist'), 'namelist', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_send_type_namelist', pyxb.binding.datatypes.string)
    __namelist._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 55, 2)
    __namelist._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 55, 2)
    
    namelist = property(__namelist.value, __namelist.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __param.name() : __param,
        __content.name() : __content
    })
    _AttributeMap.update({
        __event.name() : __event,
        __eventexpr.name() : __eventexpr,
        __target.name() : __target,
        __targetexpr.name() : __targetexpr,
        __type.name() : __type,
        __typeexpr.name() : __typeexpr,
        __id.name() : __id,
        __idlocation.name() : __idlocation,
        __delay.name() : __delay,
        __delayexpr.name() : __delayexpr,
        __namelist.name() : __namelist
    })
Namespace.addCategoryObject('typeBinding', 'scxml.send.type', scxml_send_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.cancel.type with content type ELEMENT_ONLY
class scxml_cancel_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.cancel.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.cancel.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 92, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute sendid uses Python identifier sendid
    __sendid = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'sendid'), 'sendid', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_cancel_type_sendid', pyxb.binding.datatypes.IDREF)
    __sendid._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 78, 2)
    __sendid._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 78, 2)
    
    sendid = property(__sendid.value, __sendid.set, None, None)

    
    # Attribute sendidexpr uses Python identifier sendidexpr
    __sendidexpr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'sendidexpr'), 'sendidexpr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_cancel_type_sendidexpr', ValueLang_datatype)
    __sendidexpr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 79, 2)
    __sendidexpr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 79, 2)
    
    sendidexpr = property(__sendidexpr.value, __sendidexpr.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __sendid.name() : __sendid,
        __sendidexpr.name() : __sendidexpr
    })
Namespace.addCategoryObject('typeBinding', 'scxml.cancel.type', scxml_cancel_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type with content type ELEMENT_ONLY
class scxml_invoke_type (pyxb.binding.basis.complexTypeDefinition):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.invoke.type')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 125, 1)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}param uses Python identifier param
    __param = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'param'), 'param', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlparam', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 97, 1), )

    
    param = property(__param.value, __param.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}content uses Python identifier content_
    __content = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'content'), 'content_', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlcontent', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 149, 1), )

    
    content_ = property(__content.value, __content.set, None, None)

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}finalize uses Python identifier finalize
    __finalize = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'finalize'), 'finalize', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_httpschemas_humanbrainproject_euSP102015ExDConfigscxmlfinalize', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 149, 1), )

    
    finalize = property(__finalize.value, __finalize.set, None, None)

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_type', pyxb.binding.datatypes.string, unicode_default='scxml')
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 102, 2)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 102, 2)
    
    type = property(__type.value, __type.set, None, None)

    
    # Attribute typeexpr uses Python identifier typeexpr
    __typeexpr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'typeexpr'), 'typeexpr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_typeexpr', ValueLang_datatype)
    __typeexpr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 103, 2)
    __typeexpr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 103, 2)
    
    typeexpr = property(__typeexpr.value, __typeexpr.set, None, None)

    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_src', URI_datatype)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 104, 2)
    __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 104, 2)
    
    src = property(__src.value, __src.set, None, None)

    
    # Attribute srcexpr uses Python identifier srcexpr
    __srcexpr = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'srcexpr'), 'srcexpr', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_srcexpr', ValueLang_datatype)
    __srcexpr._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 105, 2)
    __srcexpr._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 105, 2)
    
    srcexpr = property(__srcexpr.value, __srcexpr.set, None, None)

    
    # Attribute id uses Python identifier id
    __id = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'id'), 'id', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_id', pyxb.binding.datatypes.ID)
    __id._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 106, 2)
    __id._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 106, 2)
    
    id = property(__id.value, __id.set, None, None)

    
    # Attribute idlocation uses Python identifier idlocation
    __idlocation = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'idlocation'), 'idlocation', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_idlocation', LocLang_datatype)
    __idlocation._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 107, 2)
    __idlocation._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 107, 2)
    
    idlocation = property(__idlocation.value, __idlocation.set, None, None)

    
    # Attribute namelist uses Python identifier namelist
    __namelist = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'namelist'), 'namelist', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_namelist', pyxb.binding.datatypes.string)
    __namelist._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 108, 2)
    __namelist._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 108, 2)
    
    namelist = property(__namelist.value, __namelist.set, None, None)

    
    # Attribute autoforward uses Python identifier autoforward
    __autoforward = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'autoforward'), 'autoforward', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_invoke_type_autoforward', Boolean_datatype, unicode_default='false')
    __autoforward._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 109, 2)
    __autoforward._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 109, 2)
    
    autoforward = property(__autoforward.value, __autoforward.set, None, None)

    _AttributeWildcard = pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml'))
    _HasWildcardElement = True
    _ElementMap.update({
        __param.name() : __param,
        __content.name() : __content,
        __finalize.name() : __finalize
    })
    _AttributeMap.update({
        __type.name() : __type,
        __typeexpr.name() : __typeexpr,
        __src.name() : __src,
        __srcexpr.name() : __srcexpr,
        __id.name() : __id,
        __idlocation.name() : __idlocation,
        __namelist.name() : __namelist,
        __autoforward.name() : __autoforward
    })
Namespace.addCategoryObject('typeBinding', 'scxml.invoke.type', scxml_invoke_type)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.eventmapper.restrict with content type ELEMENT_ONLY
class scxml_eventmapper_restrict (scxml_invoke_type):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.eventmapper.restrict with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.eventmapper.restrict')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-event-mapping.xsd', 6, 4)
    _ElementMap = scxml_invoke_type._ElementMap.copy()
    _AttributeMap = scxml_invoke_type._AttributeMap.copy()
    # Base type is scxml_invoke_type
    
    # Attribute type inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute typeexpr inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute src inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute srcexpr inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute id inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute idlocation inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute namelist inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute autoforward inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'scxml.eventmapper.restrict', scxml_eventmapper_restrict)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.eventmapper with content type ELEMENT_ONLY
class scxml_eventmapper (scxml_eventmapper_restrict):
    """Complex type {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.eventmapper with content type ELEMENT_ONLY"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'scxml.eventmapper')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-event-mapping.xsd', 14, 4)
    _ElementMap = scxml_eventmapper_restrict._ElementMap.copy()
    _AttributeMap = scxml_eventmapper_restrict._AttributeMap.copy()
    # Base type is scxml_eventmapper_restrict
    
    # Attribute event uses Python identifier event
    __event = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'event'), 'event', '__httpschemas_humanbrainproject_euSP102015ExDConfigscxml_scxml_eventmapper_event', pyxb.binding.datatypes.string)
    __event._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-event-mapping.xsd', 17, 16)
    __event._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-event-mapping.xsd', 17, 16)
    
    event = property(__event.value, __event.set, None, None)

    
    # Attribute type inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute typeexpr inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute src inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute srcexpr inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute id inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute idlocation inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute namelist inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    
    # Attribute autoforward inherited from {http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml}scxml.invoke.type
    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __event.name() : __event
    })
Namespace.addCategoryObject('typeBinding', 'scxml.eventmapper', scxml_eventmapper)


state = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'state'), scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 120, 1))
Namespace.addCategoryObject('elementBinding', state.name().localName(), state)

initial = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'initial'), scxml_initial_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 137, 1))
Namespace.addCategoryObject('elementBinding', initial.name().localName(), initial)

onentry = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'onentry'), scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 152, 1))
Namespace.addCategoryObject('elementBinding', onentry.name().localName(), onentry)

onexit = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'onexit'), scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 167, 1))
Namespace.addCategoryObject('elementBinding', onexit.name().localName(), onexit)

parallel = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'parallel'), scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 215, 1))
Namespace.addCategoryObject('elementBinding', parallel.name().localName(), parallel)

final = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'final'), scxml_final_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 240, 1))
Namespace.addCategoryObject('elementBinding', final.name().localName(), final)

donedata = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'donedata'), scxml_donedata_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 277, 1))
Namespace.addCategoryObject('elementBinding', donedata.name().localName(), donedata)

else_ = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'else'), scxml_else_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 348, 1))
Namespace.addCategoryObject('elementBinding', else_.name().localName(), else_)

raise_ = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'raise'), scxml_raise_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1))
Namespace.addCategoryObject('elementBinding', raise_.name().localName(), raise_)

datamodel = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'datamodel'), scxml_datamodel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 59, 1))
Namespace.addCategoryObject('elementBinding', datamodel.name().localName(), datamodel)

finalize = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'finalize'), scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 149, 1))
Namespace.addCategoryObject('elementBinding', finalize.name().localName(), finalize)

scxml = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'scxml'), scxml_scxml_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 86, 1))
Namespace.addCategoryObject('elementBinding', scxml.name().localName(), scxml)

transition = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'transition'), scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1))
Namespace.addCategoryObject('elementBinding', transition.name().localName(), transition)

history = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'history'), scxml_history_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 259, 1))
Namespace.addCategoryObject('elementBinding', history.name().localName(), history)

if_ = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'if'), scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1))
Namespace.addCategoryObject('elementBinding', if_.name().localName(), if_)

elseif = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'elseif'), scxml_elseif_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 328, 1))
Namespace.addCategoryObject('elementBinding', elseif.name().localName(), elseif)

foreach = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'foreach'), scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1))
Namespace.addCategoryObject('elementBinding', foreach.name().localName(), foreach)

log = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'log'), scxml_log_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1))
Namespace.addCategoryObject('elementBinding', log.name().localName(), log)

data = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'data'), scxml_data_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 77, 1))
Namespace.addCategoryObject('elementBinding', data.name().localName(), data)

param = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'param'), scxml_param_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 97, 1))
Namespace.addCategoryObject('elementBinding', param.name().localName(), param)

assign = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'assign'), scxml_assign_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1))
Namespace.addCategoryObject('elementBinding', assign.name().localName(), assign)

script = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'script'), scxml_script_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1))
Namespace.addCategoryObject('elementBinding', script.name().localName(), script)

content = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'content'), scxml_content_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 149, 1))
Namespace.addCategoryObject('elementBinding', content.name().localName(), content)

send = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'send'), scxml_send_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1))
Namespace.addCategoryObject('elementBinding', send.name().localName(), send)

cancel = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cancel'), scxml_cancel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1))
Namespace.addCategoryObject('elementBinding', cancel.name().localName(), cancel)

invoke = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'invoke'), scxml_invoke_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 129, 1))
Namespace.addCategoryObject('elementBinding', invoke.name().localName(), invoke)

registerEvent = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'registerEvent'), scxml_eventmapper, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-event-mapping.xsd', 21, 4))
Namespace.addCategoryObject('elementBinding', registerEvent.name().localName(), registerEvent)



scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'state'), scxml_state_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 120, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'initial'), scxml_initial_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 137, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'onentry'), scxml_onentry_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 152, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'onexit'), scxml_onexit_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 167, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'transition'), scxml_transition_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'parallel'), scxml_parallel_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 215, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'final'), scxml_final_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 240, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'history'), scxml_history_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 259, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'datamodel'), scxml_datamodel_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 59, 1)))

scxml_state_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'invoke'), scxml_invoke_type, scope=scxml_state_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 129, 1)))

def _BuildAutomaton ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton
    del _BuildAutomaton
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 111, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 96, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 97, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 98, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 99, 3))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 100, 3))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 101, 3))
    counters.add(cc_6)
    cc_7 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 102, 3))
    counters.add(cc_7)
    cc_8 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 103, 3))
    counters.add(cc_8)
    cc_9 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 104, 3))
    counters.add(cc_9)
    cc_10 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 105, 3))
    counters.add(cc_10)
    cc_11 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 106, 3))
    counters.add(cc_11)
    cc_12 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_12)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'onentry')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 96, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'onexit')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 97, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'transition')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 98, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'initial')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 99, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'state')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 100, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'parallel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 101, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_7, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'final')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 102, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'history')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 103, 3))
    st_7 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_9, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'datamodel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 104, 3))
    st_8 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_10, False))
    symbol = pyxb.binding.content.ElementUse(scxml_state_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'invoke')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 105, 3))
    st_9 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_9)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_11, False))
    final_update.add(fac.UpdateInstruction(cc_12, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_10 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_10)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_5, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_6, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_7, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_9, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False) ]))
    st_8._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_10, True) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_10, False) ]))
    st_9._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_11, False),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_11, True),
        fac.UpdateInstruction(cc_12, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_12, True) ]))
    st_10._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_state_type._Automaton = _BuildAutomaton()




scxml_initial_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'transition'), scxml_transition_type, scope=scxml_initial_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1)))

def _BuildAutomaton_ ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_
    del _BuildAutomaton_
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 128, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 130, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_3)
    states = []
    final_update = None
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(scxml_initial_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'transition')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 129, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False),
        fac.UpdateInstruction(cc_1, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_2, [
         ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_2, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, True) ]))
    st_2._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
scxml_initial_type._Automaton = _BuildAutomaton_()




scxml_onentry_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'if'), scxml_if_type, scope=scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1)))

scxml_onentry_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'foreach'), scxml_foreach_type, scope=scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1)))

scxml_onentry_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'raise'), scxml_raise_type, scope=scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1)))

scxml_onentry_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'log'), scxml_log_type, scope=scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1)))

scxml_onentry_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'assign'), scxml_assign_type, scope=scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1)))

scxml_onentry_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'script'), scxml_script_type, scope=scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1)))

scxml_onentry_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'send'), scxml_send_type, scope=scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1)))

scxml_onentry_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cancel'), scxml_cancel_type, scope=scxml_onentry_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1)))

def _BuildAutomaton_2 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_2
    del _BuildAutomaton_2
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 145, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 84, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onentry_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'raise')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 85, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onentry_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'if')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 86, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onentry_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'foreach')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 87, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onentry_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'send')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 88, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onentry_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 89, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onentry_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'assign')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 90, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onentry_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'log')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 91, 3))
    st_7 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onentry_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cancel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 92, 3))
    st_8 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_8._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_onentry_type._Automaton = _BuildAutomaton_2()




scxml_onexit_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'if'), scxml_if_type, scope=scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1)))

scxml_onexit_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'foreach'), scxml_foreach_type, scope=scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1)))

scxml_onexit_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'raise'), scxml_raise_type, scope=scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1)))

scxml_onexit_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'log'), scxml_log_type, scope=scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1)))

scxml_onexit_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'assign'), scxml_assign_type, scope=scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1)))

scxml_onexit_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'script'), scxml_script_type, scope=scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1)))

scxml_onexit_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'send'), scxml_send_type, scope=scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1)))

scxml_onexit_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cancel'), scxml_cancel_type, scope=scxml_onexit_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1)))

def _BuildAutomaton_3 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_3
    del _BuildAutomaton_3
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 160, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 84, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onexit_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'raise')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 85, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onexit_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'if')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 86, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onexit_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'foreach')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 87, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onexit_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'send')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 88, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onexit_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 89, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onexit_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'assign')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 90, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onexit_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'log')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 91, 3))
    st_7 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_onexit_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cancel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 92, 3))
    st_8 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_8._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_onexit_type._Automaton = _BuildAutomaton_3()




scxml_parallel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'state'), scxml_state_type, scope=scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 120, 1)))

scxml_parallel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'onentry'), scxml_onentry_type, scope=scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 152, 1)))

scxml_parallel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'onexit'), scxml_onexit_type, scope=scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 167, 1)))

scxml_parallel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'transition'), scxml_transition_type, scope=scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1)))

scxml_parallel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'parallel'), scxml_parallel_type, scope=scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 215, 1)))

scxml_parallel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'history'), scxml_history_type, scope=scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 259, 1)))

scxml_parallel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'datamodel'), scxml_datamodel_type, scope=scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 59, 1)))

scxml_parallel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'invoke'), scxml_invoke_type, scope=scxml_parallel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 129, 1)))

def _BuildAutomaton_4 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_4
    del _BuildAutomaton_4
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 208, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 195, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 196, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 197, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 198, 3))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 199, 3))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 200, 3))
    counters.add(cc_6)
    cc_7 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 201, 3))
    counters.add(cc_7)
    cc_8 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 202, 3))
    counters.add(cc_8)
    cc_9 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 203, 3))
    counters.add(cc_9)
    cc_10 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_10)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(scxml_parallel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'onentry')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 195, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(scxml_parallel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'onexit')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 196, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(scxml_parallel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'transition')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 197, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_parallel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'state')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 198, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.ElementUse(scxml_parallel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'parallel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 199, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.ElementUse(scxml_parallel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'history')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 200, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_7, False))
    symbol = pyxb.binding.content.ElementUse(scxml_parallel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'datamodel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 201, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_parallel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'invoke')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 202, 3))
    st_7 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_9, False))
    final_update.add(fac.UpdateInstruction(cc_10, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_8 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_5, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_6, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_7, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_8, False) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_9, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_10, True) ]))
    st_8._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_parallel_type._Automaton = _BuildAutomaton_4()




scxml_final_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'onentry'), scxml_onentry_type, scope=scxml_final_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 152, 1)))

scxml_final_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'onexit'), scxml_onexit_type, scope=scxml_final_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 167, 1)))

scxml_final_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'donedata'), scxml_donedata_type, scope=scxml_final_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 277, 1)))

def _BuildAutomaton_5 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_5
    del _BuildAutomaton_5
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 233, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 225, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 226, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 227, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 228, 3))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_5)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(scxml_final_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'onentry')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 225, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(scxml_final_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'onexit')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 226, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(scxml_final_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'donedata')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 227, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_5, True) ]))
    st_3._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_final_type._Automaton = _BuildAutomaton_5()




scxml_donedata_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'param'), scxml_param_type, scope=scxml_donedata_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 97, 1)))

scxml_donedata_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'content'), scxml_content_type, scope=scxml_donedata_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 149, 1)))

def _BuildAutomaton_6 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_6
    del _BuildAutomaton_6
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 269, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 270, 3))
    counters.add(cc_1)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_donedata_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'content')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 269, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(scxml_donedata_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'param')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 270, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    st_1._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_donedata_type._Automaton = _BuildAutomaton_6()




def _BuildAutomaton_7 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_7
    del _BuildAutomaton_7
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 341, 3))
    counters.add(cc_0)
    states = []
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_else_type._Automaton = _BuildAutomaton_7()




def _BuildAutomaton_8 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_8
    del _BuildAutomaton_8
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 380, 3))
    counters.add(cc_0)
    states = []
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_raise_type._Automaton = _BuildAutomaton_8()




scxml_datamodel_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'data'), scxml_data_type, scope=scxml_datamodel_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 77, 1)))

def _BuildAutomaton_9 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_9
    del _BuildAutomaton_9
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 51, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 52, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_datamodel_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'data')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 51, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, True) ]))
    st_1._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_datamodel_type._Automaton = _BuildAutomaton_9()




scxml_finalize_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'if'), scxml_if_type, scope=scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1)))

scxml_finalize_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'foreach'), scxml_foreach_type, scope=scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1)))

scxml_finalize_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'raise'), scxml_raise_type, scope=scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1)))

scxml_finalize_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'log'), scxml_log_type, scope=scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1)))

scxml_finalize_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'assign'), scxml_assign_type, scope=scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1)))

scxml_finalize_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'script'), scxml_script_type, scope=scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1)))

scxml_finalize_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'send'), scxml_send_type, scope=scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1)))

scxml_finalize_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cancel'), scxml_cancel_type, scope=scxml_finalize_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1)))

def _BuildAutomaton_10 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_10
    del _BuildAutomaton_10
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 142, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 84, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_finalize_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'raise')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 85, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_finalize_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'if')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 86, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_finalize_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'foreach')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 87, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_finalize_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'send')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 88, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_finalize_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 89, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_finalize_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'assign')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 90, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_finalize_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'log')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 91, 3))
    st_7 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_finalize_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cancel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 92, 3))
    st_8 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_8._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_finalize_type._Automaton = _BuildAutomaton_10()




scxml_scxml_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'state'), scxml_state_type, scope=scxml_scxml_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 120, 1)))

scxml_scxml_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'parallel'), scxml_parallel_type, scope=scxml_scxml_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 215, 1)))

scxml_scxml_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'final'), scxml_final_type, scope=scxml_scxml_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 240, 1)))

scxml_scxml_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'datamodel'), scxml_datamodel_type, scope=scxml_scxml_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 59, 1)))

scxml_scxml_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'script'), scxml_script_type, scope=scxml_scxml_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1)))

scxml_scxml_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'invoke'), scxml_invoke_type, scope=scxml_scxml_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 129, 1)))

def _BuildAutomaton_11 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_11
    del _BuildAutomaton_11
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 79, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 68, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 69, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 70, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 71, 3))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 72, 3))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 73, 3))
    counters.add(cc_6)
    cc_7 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 74, 3))
    counters.add(cc_7)
    cc_8 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_8)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(scxml_scxml_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'state')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 68, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(scxml_scxml_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'parallel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 69, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(scxml_scxml_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'final')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 70, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_scxml_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'invoke')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 71, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.ElementUse(scxml_scxml_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'datamodel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 72, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.ElementUse(scxml_scxml_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 73, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_5, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_5, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_6, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_6, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_7, False),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_6._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_scxml_type._Automaton = _BuildAutomaton_11()




scxml_transition_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'if'), scxml_if_type, scope=scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1)))

scxml_transition_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'foreach'), scxml_foreach_type, scope=scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1)))

scxml_transition_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'raise'), scxml_raise_type, scope=scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1)))

scxml_transition_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'log'), scxml_log_type, scope=scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1)))

scxml_transition_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'assign'), scxml_assign_type, scope=scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1)))

scxml_transition_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'script'), scxml_script_type, scope=scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1)))

scxml_transition_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'send'), scxml_send_type, scope=scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1)))

scxml_transition_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cancel'), scxml_cancel_type, scope=scxml_transition_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1)))

def _BuildAutomaton_12 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_12
    del _BuildAutomaton_12
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 179, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 84, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_transition_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'raise')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 85, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_transition_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'if')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 86, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_transition_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'foreach')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 87, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_transition_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'send')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 88, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_transition_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 89, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_transition_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'assign')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 90, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_transition_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'log')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 91, 3))
    st_7 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_transition_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cancel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 92, 3))
    st_8 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_8._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_transition_type._Automaton = _BuildAutomaton_12()




scxml_history_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'transition'), scxml_transition_type, scope=scxml_history_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 186, 1)))

def _BuildAutomaton_13 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_13
    del _BuildAutomaton_13
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 250, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 252, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_3)
    states = []
    final_update = None
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(scxml_history_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'transition')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 251, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False),
        fac.UpdateInstruction(cc_1, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_2, [
         ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_2, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, True) ]))
    st_2._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
scxml_history_type._Automaton = _BuildAutomaton_13()




scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'if'), scxml_if_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'elseif'), scxml_elseif_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 328, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'else'), scxml_else_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 348, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'foreach'), scxml_foreach_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'raise'), scxml_raise_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'log'), scxml_log_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'assign'), scxml_assign_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'script'), scxml_script_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'send'), scxml_send_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1)))

scxml_if_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cancel'), scxml_cancel_type, scope=scxml_if_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1)))

def _BuildAutomaton_14 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_14
    del _BuildAutomaton_14
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 298, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 84, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 299, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 287, 3))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 84, 3))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_6)
    cc_7 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 300, 3))
    counters.add(cc_7)
    cc_8 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 293, 3))
    counters.add(cc_8)
    cc_9 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 84, 3))
    counters.add(cc_9)
    cc_10 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_10)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'raise')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 85, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'if')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 86, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'foreach')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 87, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'send')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 88, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 89, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'assign')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 90, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'log')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 91, 3))
    st_7 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cancel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 92, 3))
    st_8 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'elseif')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 286, 3))
    st_9 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_9)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    final_update.add(fac.UpdateInstruction(cc_5, False))
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_10 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_10)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'raise')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 85, 3))
    st_11 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_11)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'if')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 86, 3))
    st_12 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_12)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'foreach')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 87, 3))
    st_13 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_13)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'send')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 88, 3))
    st_14 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_14)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 89, 3))
    st_15 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_15)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'assign')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 90, 3))
    st_16 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_16)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'log')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 91, 3))
    st_17 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_17)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cancel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 92, 3))
    st_18 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_18)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'else')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 292, 3))
    st_19 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_19)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    final_update.add(fac.UpdateInstruction(cc_9, False))
    final_update.add(fac.UpdateInstruction(cc_10, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_20 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_20)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'raise')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 85, 3))
    st_21 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_21)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'if')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 86, 3))
    st_22 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_22)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'foreach')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 87, 3))
    st_23 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_23)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'send')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 88, 3))
    st_24 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_24)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 89, 3))
    st_25 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_25)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'assign')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 90, 3))
    st_26 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_26)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'log')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 91, 3))
    st_27 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_27)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_7, False))
    final_update.add(fac.UpdateInstruction(cc_8, False))
    symbol = pyxb.binding.content.ElementUse(scxml_if_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cancel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 92, 3))
    st_28 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_28)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_8._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True) ]))
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
    transitions.append(fac.Transition(st_17, [
         ]))
    transitions.append(fac.Transition(st_18, [
         ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False) ]))
    st_9._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_5, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_6, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    st_10._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    st_11._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    st_12._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    st_13._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    st_14._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    st_15._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    st_16._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    st_17._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_9, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_10, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_11, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_12, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_13, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_14, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_15, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_16, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_17, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_18, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    st_18._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True) ]))
    transitions.append(fac.Transition(st_20, [
         ]))
    transitions.append(fac.Transition(st_21, [
         ]))
    transitions.append(fac.Transition(st_22, [
         ]))
    transitions.append(fac.Transition(st_23, [
         ]))
    transitions.append(fac.Transition(st_24, [
         ]))
    transitions.append(fac.Transition(st_25, [
         ]))
    transitions.append(fac.Transition(st_26, [
         ]))
    transitions.append(fac.Transition(st_27, [
         ]))
    transitions.append(fac.Transition(st_28, [
         ]))
    st_19._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_9, True),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_10, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True),
        fac.UpdateInstruction(cc_9, False),
        fac.UpdateInstruction(cc_10, False) ]))
    st_20._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_21._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_22._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_23._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_24._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_25._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_26._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_27._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_19, [
        fac.UpdateInstruction(cc_7, True),
        fac.UpdateInstruction(cc_8, False) ]))
    transitions.append(fac.Transition(st_20, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_21, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_22, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_23, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_24, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_25, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_26, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_27, [
        fac.UpdateInstruction(cc_8, True) ]))
    transitions.append(fac.Transition(st_28, [
        fac.UpdateInstruction(cc_8, True) ]))
    st_28._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_if_type._Automaton = _BuildAutomaton_14()




def _BuildAutomaton_15 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_15
    del _BuildAutomaton_15
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 321, 3))
    counters.add(cc_0)
    states = []
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_elseif_type._Automaton = _BuildAutomaton_15()




scxml_foreach_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'if'), scxml_if_type, scope=scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 307, 1)))

scxml_foreach_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'foreach'), scxml_foreach_type, scope=scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 366, 1)))

scxml_foreach_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'raise'), scxml_raise_type, scope=scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 387, 1)))

scxml_foreach_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'log'), scxml_log_type, scope=scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 404, 1)))

scxml_foreach_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'assign'), scxml_assign_type, scope=scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 116, 1)))

scxml_foreach_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'script'), scxml_script_type, scope=scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 133, 1)))

scxml_foreach_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'send'), scxml_send_type, scope=scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 74, 1)))

scxml_foreach_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'cancel'), scxml_cancel_type, scope=scxml_foreach_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 96, 1)))

def _BuildAutomaton_16 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_16
    del _BuildAutomaton_16
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 359, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 84, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_foreach_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'raise')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 85, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_foreach_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'if')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 86, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_foreach_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'foreach')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 87, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_foreach_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'send')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 88, 3))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_foreach_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'script')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 89, 3))
    st_5 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_foreach_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'assign')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 90, 3))
    st_6 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_foreach_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'log')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 91, 3))
    st_7 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(scxml_foreach_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'cancel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml.xsd', 92, 3))
    st_8 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_8._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_foreach_type._Automaton = _BuildAutomaton_16()




def _BuildAutomaton_17 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_17
    del _BuildAutomaton_17
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-core.xsd', 397, 2))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_1)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_log_type._Automaton = _BuildAutomaton_17()




def _BuildAutomaton_18 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_18
    del _BuildAutomaton_18
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 70, 3))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=pyxb.binding.content.Wildcard.NC_any), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 70, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_data_type._Automaton = _BuildAutomaton_18()




def _BuildAutomaton_19 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_19
    del _BuildAutomaton_19
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 90, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_1)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_param_type._Automaton = _BuildAutomaton_19()




def _BuildAutomaton_20 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_20
    del _BuildAutomaton_20
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 109, 3))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=pyxb.binding.content.Wildcard.NC_any), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 109, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_assign_type._Automaton = _BuildAutomaton_20()




def _BuildAutomaton_21 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_21
    del _BuildAutomaton_21
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 126, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_1)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_script_type._Automaton = _BuildAutomaton_21()




def _BuildAutomaton_22 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_22
    del _BuildAutomaton_22
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 142, 2))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=pyxb.binding.content.Wildcard.NC_any), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 142, 2))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_content_type._Automaton = _BuildAutomaton_22()




scxml_send_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'param'), scxml_param_type, scope=scxml_send_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 97, 1)))

scxml_send_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'content'), scxml_content_type, scope=scxml_send_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 149, 1)))

def _BuildAutomaton_23 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_23
    del _BuildAutomaton_23
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 67, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 60, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 61, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 62, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_4)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(scxml_send_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'content')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 60, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(scxml_send_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'param')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 61, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_4, True) ]))
    st_2._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_send_type._Automaton = _BuildAutomaton_23()




def _BuildAutomaton_24 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_24
    del _BuildAutomaton_24
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 89, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 84, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_cancel_type._Automaton = _BuildAutomaton_24()




scxml_invoke_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'param'), scxml_param_type, scope=scxml_invoke_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 97, 1)))

scxml_invoke_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'content'), scxml_content_type, scope=scxml_invoke_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-data.xsd', 149, 1)))

scxml_invoke_type._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'finalize'), scxml_finalize_type, scope=scxml_invoke_type, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 149, 1)))

def _BuildAutomaton_25 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_25
    del _BuildAutomaton_25
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 122, 3))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 114, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 115, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 116, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 117, 3))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_5)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(scxml_invoke_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'content')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 114, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(scxml_invoke_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'param')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 115, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(scxml_invoke_type._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'finalize')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 116, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_3, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_4, False),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_4, True),
        fac.UpdateInstruction(cc_5, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_5, True) ]))
    st_3._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_invoke_type._Automaton = _BuildAutomaton_25()




def _BuildAutomaton_26 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_26
    del _BuildAutomaton_26
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=0, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-event-mapping.xsd', 9, 16))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 122, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 114, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 115, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 116, 3))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 117, 3))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_6)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(scxml_eventmapper_restrict._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'content')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 114, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(scxml_eventmapper_restrict._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'param')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 115, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_eventmapper_restrict._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'finalize')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 116, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_5, False))
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_3, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_4, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_5, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_6, True) ]))
    st_3._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_eventmapper_restrict._Automaton = _BuildAutomaton_26()




def _BuildAutomaton_27 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_27
    del _BuildAutomaton_27
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=0, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-event-mapping.xsd', 9, 16))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 122, 3))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 114, 3))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 115, 3))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 116, 3))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 117, 3))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    counters.add(cc_6)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(scxml_eventmapper._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'content')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 114, 3))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(scxml_eventmapper._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'param')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 115, 3))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(scxml_eventmapper._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'finalize')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-module-external.xsd', 116, 3))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    final_update.add(fac.UpdateInstruction(cc_1, False))
    final_update.add(fac.UpdateInstruction(cc_5, False))
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_lax, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/hbp-scxml/hbp-scxml-contentmodels.xsd', 31, 3))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_3, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_4, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, True),
        fac.UpdateInstruction(cc_1, False),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True),
        fac.UpdateInstruction(cc_5, False),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_5, True),
        fac.UpdateInstruction(cc_6, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_6, True) ]))
    st_3._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
scxml_eventmapper._Automaton = _BuildAutomaton_27()


registerEvent._setSubstitutionGroup(invoke)
